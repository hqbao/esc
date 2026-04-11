// 6-Step Trapezoidal BLDC — sensorless using BEMF zero-crossing
//
// Classic trapezoidal (120° block) commutation:
//   - Only 2 of 3 phases driven at any time, 3rd is floating
//   - BEMF zero-crossing on the floating phase triggers next commutation step
//   - Hardware comparators (COMP1/2/4) detect zero-crossing against Vbus/2
//
// Startup: IDLE → ALIGN → RAMP (forced open-loop) → CLOSEDLOOP (BEMF sync)
//
// Commutation table (6 sectors per electrical revolution):
//
//   Step | High  | Low   | Float | ZC detect
//   ─────┼───────┼───────┼───────┼──────────
//     0  |  U    |  W    |  V    | COMP2 falling
//     1  |  V    |  W    |  U    | COMP1 rising
//     2  |  V    |  U    |  W    | COMP4 falling
//     3  |  W    |  U    |  V    | COMP2 rising
//     4  |  W    |  V    |  U    | COMP1 falling
//     5  |  U    |  V    |  W    | COMP4 rising
//
// Log classes:
//   COMMUTATION (3) — state, step, speed, duty, Vbus, zc_cnt

#include "foc.h"
#include <pubsub.h>
#include <platform.h>
#include <messages.h>
#include <macro.h>
#include <string.h>
#include <math.h>

// HAL comparator + DAC handles
#include "platform_hw.h"

// ── States ─────────────────────────────────────────────────────────────────

typedef enum {
    STATE_IDLE,         // 0 — motor stopped
    STATE_ALIGN,        // 1 — lock rotor to known position
    STATE_RAMP,         // 2 — forced (blind) commutation ramp-up
    STATE_CLOSEDLOOP,   // 3 — BEMF zero-crossing synced
} state_e;

// ── Constants ──────────────────────────────────────────────────────────────

// Startup
#define ALIGN_TICKS          8000       // 200ms at 40kHz
#define ALIGN_DUTY           0.12f      // 12% alignment pulse
#define RAMP_INITIAL_PERIOD  1600       // Initial step (40ms)
#define RAMP_FINAL_PERIOD    320        // Target step (8ms, handoff speed)
#define RAMP_ACCEL           10         // (1600-320)/10 = 128 steps, ~4s ramp
#define RAMP_DUTY            0.20f      // 20% duty — needs enough torque for rotor sync
#define RAMP_MIN_STEPS       48         // 8 electrical revolutions minimum
#define RAMP_MAX_STEPS       250        // Safety limit (need ~128 steps)

// Closed-loop
#define DUTY_MAX         0.95f
#define DUTY_MIN         0.06f          // Keep enough torque after handoff
#define THROTTLE_START   0.01f
#define ZC_BLANKING_PCT  0.10f          // Ignore ZC for first 10% of step (PWM noise)
#define COMM_DELAY_PCT   0.50f          // Commutate at 50% after ZC (30° electrical delay)
#define SPEED_LPF        0.05f          // Speed estimate filter coefficient

// BEMF zero-crossing threshold
// B-G431B-ESC1: Empirical BEMF divider ratio ~0.50 (from ALIGN data: Vbus/2 → ~3600 ADC).
// Both RAMP and CLOSEDLOOP use level detection with this threshold.
// Speed control in CLOSEDLOOP uses ZC health (sliding window) + throttle mapping.
#define BEMF_DIVIDER_RATIO  0.50f       // Vbus/2 threshold for ZC detection
#define VDDA                3.3f        // Analog reference voltage
#define ZC_MISS_MAX         6           // Consecutive ZC misses → stall

// ── Commutation table ──────────────────────────────────────────────────────
// For each step: which phase is HIGH (PWM), which is LOW (ground), which floats
// Encoding: high_phase, low_phase, float_phase

typedef struct {
    pwm_port_t high;
    pwm_port_t low;
    pwm_port_t floating;
} step_pattern_t;

static const step_pattern_t STEP_TABLE[6] = {
    { PWM_PORT1, PWM_PORT3, PWM_PORT2 },  // Step 0: U+, W-, V float
    { PWM_PORT2, PWM_PORT3, PWM_PORT1 },  // Step 1: V+, W-, U float
    { PWM_PORT2, PWM_PORT1, PWM_PORT3 },  // Step 2: V+, U-, W float
    { PWM_PORT3, PWM_PORT1, PWM_PORT2 },  // Step 3: W+, U-, V float
    { PWM_PORT3, PWM_PORT2, PWM_PORT1 },  // Step 4: W+, V-, U float
    { PWM_PORT1, PWM_PORT2, PWM_PORT3 },  // Step 5: U+, V-, W float
};

// Expected comparator output AFTER zero-crossing for each step.
// Derived from which phase was previously driven:
//   Step 0: V was LOW (step 5: V−) → V rises through Vbus/2 → RISING  → expect 0
//   Step 1: U was HIGH (step 0: U+) → U falls through Vbus/2 → FALLING → expect 1
//   Step 2: W was LOW (step 1: W−) → W rises                 → RISING  → expect 0
//   Step 3: V was HIGH (step 2: V+) → V falls                 → FALLING → expect 1
//   Step 4: U was LOW (step 3: U−) → U rises                 → RISING  → expect 0
//   Step 5: W was HIGH (step 4: W+) → W falls                 → FALLING → expect 1
//
static const uint32_t ZC_POLARITY[6] = {
    1, 0, 1, 0, 1, 0,
};

// ── State variables ────────────────────────────────────────────────────────

static state_e   g_state;
static float     g_throttle;
static uint8_t   g_log_class;

// Commutation
static uint8_t   g_step;               // Current commutation step (0-5)
static uint32_t  g_step_ticks;         // Ticks elapsed in current step
static uint32_t  g_step_period;        // Ticks per step (determines speed)
static uint32_t  g_align_cnt;

// Ramp
static uint32_t  g_ramp_period;        // Current forced commutation period
static uint32_t  g_ramp_steps;         // Total forced steps completed
static uint32_t  g_ramp_zc_consec;     // Consecutive steps with valid ZC during RAMP

// Zero-crossing
static uint8_t   g_zc_detected;        // ZC event seen this step
static uint32_t  g_zc_tick;            // Tick at which ZC was detected
static uint32_t  g_zc_count;           // Cumulative good ZC detections
static uint32_t  g_zc_miss;            // Consecutive missed ZC events
static uint8_t   g_zc_debounce;        // Consecutive matching comparator samples
static uint8_t   g_zc_pre_cross;       // Seen opposite polarity (pre-crossing state)

// Sliding window ZC tracking (replaces strict consecutive for transition)
static uint8_t   g_zc_window[12];      // Last 12 steps: 1=detected, 0=missed
static uint8_t   g_zc_window_idx;
static uint8_t   g_zc_window_sum;      // Count of detections in window
static uint8_t   g_step_zc_map;        // 6-bit bitmask: bit N = step N detected ZC

// Speed
static float     g_speed_erps;         // Electrical rev/s estimate

// Bus voltage
static float     g_vbus = 12.0f;
static float     g_duty;              // Current duty cycle (for threshold tracking)
static uint32_t  g_dac_val;           // Current DAC threshold value (for logging)

// Track which step is currently applied (0xFF = none)
static uint8_t   g_active_step = 0xFF;

// Comparator values captured at PWM valley (center of ON window) in ISR
static uint8_t   g_comp_cache[3];     // [0]=COMP1(U), [1]=COMP2(V), [2]=COMP4(W)

// ── Helpers ────────────────────────────────────────────────────────────────

static void reset_state(void) {
    g_state       = STATE_IDLE;
    g_step        = 0;
    g_step_ticks  = 0;
    g_step_period = 0;
    g_align_cnt   = 0;
    g_ramp_period = RAMP_INITIAL_PERIOD;
    g_ramp_steps  = 0;
    g_ramp_zc_consec = 0;
    g_zc_detected = 0;
    g_zc_tick     = 0;
    g_zc_count    = 0;
    g_zc_miss     = 0;
    g_speed_erps  = 0.0f;
    g_active_step = 0xFF;
    g_duty        = 0.0f;
    memset(g_zc_window, 0, sizeof(g_zc_window));
    g_zc_window_idx = 0;
    g_zc_window_sum = 0;
    g_step_zc_map   = 0;
}

static void apply_step(uint8_t step, float duty) {
    const step_pattern_t *p = &STEP_TABLE[step];
    uint32_t pwm_val = (uint32_t)(duty * (float)PWM_PERIOD);
    if (pwm_val > (uint32_t)(DUTY_MAX * (float)PWM_PERIOD))
        pwm_val = (uint32_t)(DUTY_MAX * (float)PWM_PERIOD);

    // Only reconfigure channels on step change (float/drive calls HAL Stop/Start)
    if (step != g_active_step) {
        // Float the undriven phase FIRST (prevent shoot-through)
        platform_pwm_float(p->floating);

        // Low-side: CCR=0 → high-side OFF, complementary (low-side MOSFET) ON = ground path
        platform_pwm_drive(p->low);
        platform_pwm_send(p->low, 0);

        // High-side: enable PWM on this phase
        platform_pwm_drive(p->high);

        g_active_step = step;
    }

    // Update duty cycle every tick (just a register write, no HAL stop/start)
    platform_pwm_send(p->high, pwm_val);
}

static void advance_step(void) {
    g_step = (g_step + 1) % 6;
    g_step_ticks = 0;
    g_zc_detected = 0;
    g_zc_debounce = 0;
    g_zc_pre_cross = 0;
}

static uint8_t read_comparator(uint8_t step) {
    // Use ISR-cached values captured at PWM valley (center of ON window).
    // COMP_TABLE maps: steps 0,3→COMP2(idx1), steps 1,4→COMP1(idx0), steps 2,5→COMP4(idx2)
    static const uint8_t COMP_CACHE_IDX[6] = { 1, 0, 2, 1, 0, 2 };
    return g_comp_cache[COMP_CACHE_IDX[step]];
}

static uint8_t detect_zc_edge(uint8_t step, uint8_t comp_val) {
    uint8_t expected = (uint8_t)ZC_POLARITY[step];

    // Require an opposite-polarity pre-state before accepting the crossing.
    if (!g_zc_pre_cross) {
        if (comp_val != expected) {
            g_zc_pre_cross = 1;
        }
        return 0;
    }

    // Single-sample confirm after pre-cross (hardware comparator output
    // is already captured at PWM valley — extra debounce causes missed ZCs).
    if (comp_val == expected) {
        if (++g_zc_debounce >= 1) {
            return 1;
        }
    } else {
        g_zc_debounce = 0;
    }

    return 0;
}

static void update_bemf_threshold(float duty) {
    (void)duty;
    float v_threshold = g_vbus * 0.5f * BEMF_DIVIDER_RATIO;
    uint32_t threshold = (uint32_t)(v_threshold / VDDA * 4095.0f);
    if (threshold < 100) threshold = 100;
    if (threshold > 4000) threshold = 4000;
    g_bemf_threshold = (uint16_t)threshold;
    g_dac_val = threshold;  // Reuse for logging
    g_duty = duty;
}

// ── Sensor callbacks ───────────────────────────────────────────────────────

static void on_bus_voltage(uint8_t *data, size_t size) {
    if (size < sizeof(bus_measurement_t)) return;
    bus_measurement_t m;
    memcpy(&m, data, sizeof(m));
    g_vbus = m.bus_voltage;
    if (g_state != STATE_IDLE) update_bemf_threshold(g_duty);
}

// ── Throttle ───────────────────────────────────────────────────────────────

static void on_motor_throttle(uint8_t *data, size_t size) {
    if (size < sizeof(motor_throttle_t)) return;
    motor_throttle_t cmd;
    memcpy(&cmd, data, sizeof(cmd));
    g_throttle = LIMIT(cmd.throttle, 0.0f, 1.0f);

    if (g_throttle < THROTTLE_START && g_state != STATE_IDLE) {
        platform_pwm_stop();
        reset_state();
    }
    else if (g_throttle > THROTTLE_START && g_state == STATE_IDLE) {
        reset_state();
        g_state = STATE_ALIGN;
        platform_pwm_start();
        // Ground all motor terminals to discharge BEMF divider capacitances.
        // After MCU reset, foc_setup → platform_pwm_init does this implicitly
        // (CCR=0 on all channels). Without reset, residual charge from the
        // previous run biases the comparators. apply_step will override on
        // the first ALIGN tick.
        platform_pwm_drive(PWM_PORT1); platform_pwm_send(PWM_PORT1, 0);
        platform_pwm_drive(PWM_PORT2); platform_pwm_send(PWM_PORT2, 0);
        platform_pwm_drive(PWM_PORT3); platform_pwm_send(PWM_PORT3, 0);
        g_active_step = 0xFF;  // Force apply_step to reconfigure
    }
}

static void on_notify_log_class(uint8_t *data, size_t size) {
    if (size < 1) return;
    g_log_class = data[0];
}

// ── 40 kHz commutation tick ────────────────────────────────────────────────

static void on_tick(uint8_t *data, size_t size) {
    // Cache comparator values captured at PWM valley by ISR
    if (size >= 3) {
        g_comp_cache[0] = data[0];
        g_comp_cache[1] = data[1];
        g_comp_cache[2] = data[2];
    }
    if (g_state == STATE_IDLE) return;

    g_step_ticks++;

    switch (g_state) {

    // ── ALIGN: Lock rotor to step 0 position ───────────────────────────
    case STATE_ALIGN:
        apply_step(0, ALIGN_DUTY);
        if (g_align_cnt == 0) update_bemf_threshold(ALIGN_DUTY);
        if (++g_align_cnt >= ALIGN_TICKS) {
            g_state = STATE_RAMP;
            g_step = 0;
            g_step_ticks = 0;
            g_ramp_period = RAMP_INITIAL_PERIOD;
            g_ramp_steps = 0;
            update_bemf_threshold(RAMP_DUTY);
        }
        break;

    // ── RAMP: Forced commutation with ZC validation ────────────────────
    case STATE_RAMP: {
        apply_step(g_step, RAMP_DUTY);

        // BEMF zero-crossing: after blanking, check if comp matches expected polarity.
        // Simple level check in RAMP — false positives are harmless since timing
        // is forced, and the 10/12 sliding window filters random matches.
        uint32_t blanking = (uint32_t)((float)g_ramp_period * ZC_BLANKING_PCT);
        if (!g_zc_detected && g_step_ticks > blanking) {
            uint8_t comp_val = read_comparator(g_step);
            if (comp_val == (uint8_t)ZC_POLARITY[g_step]) {
                g_zc_detected = 1;
                g_zc_tick = g_step_ticks;
                g_zc_count++;
            }
        }

        if (g_step_ticks >= g_ramp_period) {
            // Per-step ZC bitmask (which of 6 steps detect ZC)
            if (g_zc_detected) {
                g_step_zc_map |= (1 << g_step);
                g_ramp_zc_consec++;
            } else {
                g_step_zc_map &= ~(1 << g_step);
                g_ramp_zc_consec = 0;
            }

            // Sliding window: track last 12 step results
            g_zc_window_sum -= g_zc_window[g_zc_window_idx];
            g_zc_window[g_zc_window_idx] = g_zc_detected ? 1 : 0;
            g_zc_window_sum += g_zc_window[g_zc_window_idx];
            g_zc_window_idx = (g_zc_window_idx + 1) % 12;

            advance_step();
            g_ramp_steps++;

            // Gradually decrease step period (= increase speed)
            if (g_ramp_period > RAMP_FINAL_PERIOD + RAMP_ACCEL) {
                g_ramp_period -= RAMP_ACCEL;
            } else {
                g_ramp_period = RAMP_FINAL_PERIOD;
            }

            // Transition: fast enough AND 8/12 recent steps have valid ZC
            // (at startup speed, BEMF is weak and some steps may miss ZC —
            // CLOSEDLOOP uses forced timing + ZC feedback, so partial detection is OK)
            if (g_ramp_steps >= RAMP_MIN_STEPS &&
                g_ramp_period <= RAMP_FINAL_PERIOD &&
                g_zc_window_sum >= 8) {
                g_state = STATE_CLOSEDLOOP;
                g_step_period = g_ramp_period;
                g_zc_miss = 0;
            }

            // Safety: abort ramp if it takes too long (motor stalled or no sync)
            if (g_ramp_steps >= RAMP_MAX_STEPS) {
                platform_pwm_stop();
                reset_state();
            }
        }
        break;
    }

    // ── CLOSEDLOOP: Forced timing with ZC-health speed control ────────
    // Commutation at forced step_period. ZC level detection validates sync.
    // Period adjusts based on ZC reliability (sliding window) + throttle demand.
    case STATE_CLOSEDLOOP: {
        float duty = LIMIT(g_throttle, DUTY_MIN, DUTY_MAX);
        apply_step(g_step, duty);
        // Update threshold when duty changes significantly
        if (duty > g_duty * 1.05f || duty < g_duty * 0.95f) {
            update_bemf_threshold(duty);
        }

        // Blanking: 10% of step period
        uint32_t blanking = (uint32_t)((float)g_step_period * ZC_BLANKING_PCT);
        if (blanking < 4) blanking = 4;

        // BEMF zero-crossing: level detection (same as RAMP)
        if (!g_zc_detected && g_step_ticks > blanking) {
            uint8_t comp_val = read_comparator(g_step);
            if (comp_val == (uint8_t)ZC_POLARITY[g_step]) {
                g_zc_detected = 1;
                g_zc_tick = g_step_ticks;
                g_zc_count++;
            }
        }

        // Commutate at forced period
        if (g_step_ticks >= g_step_period) {
            // Track ZC in sliding window (reuse RAMP's mechanism)
            g_zc_window_sum -= g_zc_window[g_zc_window_idx];
            g_zc_window[g_zc_window_idx] = g_zc_detected ? 1 : 0;
            g_zc_window_sum += g_zc_window[g_zc_window_idx];
            g_zc_window_idx = (g_zc_window_idx + 1) % 12;

            if (g_zc_detected) {
                g_zc_miss = 0;
            } else {
                g_zc_miss++;
                if (g_zc_miss >= ZC_MISS_MAX) {
                    platform_pwm_stop();
                    reset_state();
                    break;
                }
            }

            // Speed control: target period from throttle
            // Map throttle 0.06..1.0 → period 320..20 (20.8..333 ERPS)
            float t_norm = (duty - DUTY_MIN) / (DUTY_MAX - DUTY_MIN);
            uint32_t target_period = 320 - (uint32_t)(t_norm * 300.0f);
            if (target_period < 20) target_period = 20;
            if (target_period > 320) target_period = 320;

            // Adjust period toward target, gated by ZC health
            if (g_zc_window_sum >= 8 && g_step_period > target_period) {
                // Reliable ZC + below target speed → accelerate
                g_step_period -= 1;
            } else if (g_zc_window_sum < 6 || g_step_period < target_period) {
                // Poor ZC or above target speed → decelerate
                g_step_period += 2;
                if (g_step_period > 10000) g_step_period = 10000;
            }

            float erps_raw = (float)PWM_FREQ / ((float)g_step_period * 6.0f);
            g_speed_erps += SPEED_LPF * (erps_raw - g_speed_erps);

            advance_step();
        }
        break;
    }

    default:
        break;
    }
}

// ── 25 Hz logging ──────────────────────────────────────────────────────────

static void on_scheduler_25hz(uint8_t *data, size_t size) {
    if (g_log_class == 0) return;

    float log_data[12];
    int ok = 0;

    switch (g_log_class) {
    case LOG_CLASS_COMMUTATION: {
        // [0]state [1]step [2]speed_erps [3]duty [4]Vbus [5]step_period
        // [6]zc_count [7]ramp_period [8]ramp_zc_consec [9]zc_miss|MOE|CCER
        // [10]comp raw: c1|(c2<<1)|(c4<<2) | (threshold << 4)
        // [11]step_zc_map | (zc_window_sum << 8)
        // For ADC-based BEMF: also log raw ADC values of all 3 phases
        uint8_t c1 = g_comp_cache[0];
        uint8_t c2 = g_comp_cache[1];
        uint8_t c4 = g_comp_cache[2];
        // Pack: [7:0]=zc_miss, [8]=MOE, [19:9]=CCER lower 11 bits
        uint32_t moe = (TIM1->BDTR & TIM_BDTR_MOE) ? 1 : 0;
        uint32_t ccer = TIM1->CCER & 0x7FF; // CH1/1N/CH2/2N/CH3/3N enable bits
        // Pack all 3 BEMF ADC values for diagnostic
        uint32_t bu = g_bemf_raw[0];  // Phase U (PA4)
        uint32_t bv = g_bemf_raw[1];  // Phase V (PC4)
        uint32_t bw = g_bemf_raw[2];  // Phase W (PB11)
        log_data[0]  = (float)g_state;
        log_data[1]  = (float)g_step;
        log_data[2]  = g_speed_erps;
        log_data[3]  = g_throttle;
        log_data[4]  = g_vbus;
        log_data[5]  = (float)g_step_period;
        log_data[6]  = (float)g_zc_count;
        log_data[7]  = (float)g_ramp_period;
        log_data[8]  = (float)g_ramp_zc_consec;
        log_data[9]  = (float)((g_zc_miss & 0xFF) | (moe << 8) | (ccer << 9));
        // f[10]: bemf_u[11:0] | bemf_v[23:12]
        log_data[10] = (float)(bu | (bv << 12));
        // f[11]: bemf_w[11:0] | zc_map[17:12] | zc_win[22:18] | threshold_hi[23]
        log_data[11] = (float)(bw | ((g_step_zc_map & 0x3F) << 12) | ((g_zc_window_sum & 0x1F) << 18));
        ok = 1;
        break;
    }
    }

    if (ok) {
        publish(SEND_LOG, (uint8_t *)log_data, sizeof(log_data));
    }
}

// ── Setup ──────────────────────────────────────────────────────────────────

static void on_foc_release(uint8_t *data, size_t size) {
    platform_pwm_stop();
    reset_state();
    g_throttle  = 0.0f;
    g_log_class = 0;
}

void foc_setup(void) {
    // Init PWM channels and set CCR4=1 for ADC trigger at PWM valley.
    // CubeMX sets PulseNoDither_4=1, but enforce it here in case of reset.
    platform_pwm_init(PWM_PORT1);
    platform_pwm_init(PWM_PORT2);
    platform_pwm_init(PWM_PORT3);
    htim1.Instance->CCR4 = 1;

    // BEMF sensing via ADC2 on PA4/PC4/PB11 (NOT comparators on PA1/PA7/PB0).
    // The hardware COMPs read current-sense OPAmp inputs, not BEMF dividers.
    bemf_adc_init();
    update_bemf_threshold(ALIGN_DUTY);

    subscribe(FOC_RELEASE, on_foc_release);
    subscribe(MOTOR_THROTTLE, on_motor_throttle);
    subscribe(ADC_INJECTED_COMPLETE, on_tick);
    subscribe(SENSOR_BUS_VOLTAGE, on_bus_voltage);
    subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
    subscribe(SCHEDULER_25HZ, on_scheduler_25hz);
}
