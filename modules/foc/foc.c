// 6-Step Trapezoidal BLDC — sensorless using BEMF zero-crossing
//
// Classic trapezoidal (120° block) commutation:
//   - Only 2 of 3 phases driven at any time, 3rd is floating
//   - BEMF zero-crossing on the floating phase triggers next commutation step
//   - Zero-crossing detected by software "virtual comparators": ADC2 samples
//     BEMF voltage on the floating phase at each PWM valley (40 kHz), and
//     platform_isr.c compares against a Vbus/2 threshold to produce a 1-bit
//     above/below result per phase. This replaces hardware COMP1/2/4 which
//     on B-G431B-ESC1 are wired to current-sense OPAmp outputs, NOT BEMF.
//
// Startup: IDLE → ALIGN → RAMP (forced open-loop) → CLOSEDLOOP (BEMF sync)
//
// Commutation table (6 sectors per electrical revolution):
//
//   Step | High  | Low   | Float | ZC detect (virtual comparator)
//   ─────┼───────┼───────┼───────┼────────────────────────────────
//     0  |  U    |  W    |  V    | V falling (ADC2 ch5  < threshold)
//     1  |  V    |  W    |  U    | U rising  (ADC2 ch17 > threshold)
//     2  |  V    |  U    |  W    | W falling (ADC2 ch14 < threshold)
//     3  |  W    |  U    |  V    | V rising  (ADC2 ch5  > threshold)
//     4  |  W    |  V    |  U    | U falling (ADC2 ch17 < threshold)
//     5  |  U    |  V    |  W    | W rising  (ADC2 ch14 > threshold)
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

// HAL handles + BEMF ADC declarations
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
#define ALIGN_TICKS          16000      // 400ms at 40kHz
#define ALIGN_DUTY           0.15f      // 15% alignment pulse
#define RAMP_INITIAL_PERIOD  2400       // Initial step (60ms)
#define RAMP_FINAL_PERIOD    400        // Target step (10ms) — enough BEMF for ZC
#define RAMP_ACCEL           30         // (2400-400)/30 ≈ 67 steps, ~2s ramp
#define RAMP_DUTY            0.18f      // 18% duty — enough torque, minimize heating
#define RAMP_MIN_STEPS       24         // 4 electrical revolutions minimum
#define RAMP_MAX_STEPS       500        // Safety limit

// Closed-loop
#define DUTY_MAX         0.95f
#define DUTY_MIN         0.03f
#define THROTTLE_START   0.01f
#define ZC_BLANKING_PCT  0.15f          // Ignore ZC for first 15% of step (flyback ringing)
#define SPEED_LPF        0.05f          // Speed estimate filter coefficient
#define COMM_DEAD_TICKS  1              // Dead-time between commutations (25µs at 40kHz)

// Zero-crossing: sign-change detection with self-referencing neutral point.
// Neutral = (BEMF_U + BEMF_V + BEMF_W) / 3 — virtual star point from ADC readings.
// ZC detected when signed BEMF on the floating phase crosses zero (sign flip).
// No fixed threshold needed — adapts automatically to speed, load, and Vbus.
#define ZC_MISS_MAX         36          // Consecutive ZC misses → stall
#define MIN_STEP_PERIOD     120         // Floor: 3ms/step → max ~55 ERPS (anti-runaway)

// Burst capture for BEMF waveform visualization at full 40kHz
#define LOG_CLASS_BEMF_BURST  4
#define LOG_CLASS_ADC_LIVE    5
#define BURST_SAMPLES         720       // ~1 elec revolution at 55 ERPS
#define BURST_CHUNK_SAMPLES   6         // Samples per log frame (36 bytes in 9 floats)

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

// Expected zero-crossing state AFTER zero-crossing for each step.
//   Step 0: V was LOW (step 5: V−) → V rises through neutral → RISING  → expect 1
//   Step 1: U was HIGH (step 0: U+) → U falls through neutral → FALLING → expect 0
//   Step 2: W was LOW (step 1: W−) → W rises                  → RISING  → expect 1
//   Step 3: V was HIGH (step 2: V+) → V falls                  → FALLING → expect 0
//   Step 4: U was LOW (step 3: U−) → U rises                  → RISING  → expect 1
//   Step 5: W was HIGH (step 4: W+) → W falls                  → FALLING → expect 0
//
static const uint8_t ZC_RISING[6] = {
    1, 0, 1, 0, 1, 0,
};

// Floating phase index for each step: which g_bemf_signed[] index to check
// U=0, V=1, W=2
static const uint8_t FLOAT_PHASE[6] = {
    1, 0, 2, 1, 0, 2,  // V, U, W, V, U, W
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

// Signed BEMF for zero-crossing sign-change detection
// Computed each tick: raw ADC minus virtual neutral (average of all 3 phases)
static int16_t   g_bemf_signed[3];     // Current tick: [U, V, W]
static int16_t   g_bemf_prev[3];       // Previous tick: [U, V, W]

// Sliding window ZC tracking (replaces strict consecutive for transition)
static uint8_t   g_zc_window[12];      // Last 12 steps: 1=detected, 0=missed
static uint8_t   g_zc_window_idx;
static uint8_t   g_zc_window_sum;      // Count of detections in window
static uint8_t   g_step_zc_map;        // 6-bit bitmask: bit N = step N detected ZC

// Speed
static float     g_speed_erps;         // Electrical rev/s estimate

// Bus voltage
static float     g_vbus = 12.0f;

// Commutation dead-time counter (0 = not in dead-time)
static uint8_t   g_comm_dead;

// Track which step is currently applied (0xFF = none)
static uint8_t   g_active_step = 0xFF;

// Burst capture state
static uint16_t g_burst_buf[BURST_SAMPLES * 3];  // Raw ADC: [U,V,W] × BURST_SAMPLES
static uint16_t g_burst_pos;
static uint16_t g_burst_send_pos;
static uint8_t  g_burst_state;          // 0=idle, 1=capturing, 2=sending

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
    g_comm_dead   = 0;
    memset(g_zc_window, 0, sizeof(g_zc_window));
    g_zc_window_idx = 0;
    g_zc_window_sum = 0;
    g_step_zc_map   = 0;
    memset(g_bemf_signed, 0, sizeof(g_bemf_signed));
    memset(g_bemf_prev, 0, sizeof(g_bemf_prev));
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
    g_step = (g_step + 1) % 6;  // Forward: 0→1→2→3→4→5→0
    g_step_ticks = 0;
    g_zc_detected = 0;
}

// ── Sign-change ZC detection ───────────────────────────────────────────────
// Detect zero-crossing by checking if the signed BEMF on the floating phase
// changes sign. This replaces the threshold-based virtual comparator.
// Returns 1 if ZC detected on the floating phase of the given step.
static uint8_t detect_zc_sign_change(uint8_t step) {
    uint8_t fp = FLOAT_PHASE[step];
    int16_t bemf = g_bemf_signed[fp];
    int16_t prev = g_bemf_prev[fp];
    uint8_t rising = ZC_RISING[step];

    if (rising) {
        // Rising ZC: previous was negative (or zero), current is positive
        if (prev <= 0 && bemf > 0) return 1;
    } else {
        // Falling ZC: previous was positive (or zero), current is negative
        if (prev >= 0 && bemf < 0) return 1;
    }
    return 0;
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
        // previous run biases the BEMF ADC readings. apply_step will override on
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
    if (g_log_class == LOG_CLASS_BEMF_BURST) {
        g_burst_pos = 0;
        g_burst_send_pos = 0;
        g_burst_state = 1;
    }
}

// ── 40 kHz commutation tick ────────────────────────────────────────────────

static void on_tick(uint8_t *data, size_t size) {
    (void)data; (void)size;

    // Burst capture runs regardless of motor state (hand-rotate visualization)
    if (g_burst_state == 1) {
        if (g_burst_pos < BURST_SAMPLES) {
            uint16_t base = g_burst_pos * 3;
            g_burst_buf[base]     = (g_bemf_raw[0] & 0x3FF)
                                  | ((uint16_t)g_step << 10)
                                  | ((uint16_t)g_zc_detected << 13);
            g_burst_buf[base + 1] = g_bemf_raw[1];
            g_burst_buf[base + 2] = g_bemf_raw[2];
            g_burst_pos++;
        } else {
            g_burst_state = 2;
            g_burst_send_pos = 0;
        }
    }

    if (g_state == STATE_IDLE) return;

    // ── Compute signed BEMF relative to virtual neutral point ──────────
    // Neutral = (U + V + W) / 3 — self-referencing virtual star point.
    // In Y-connected BLDC, the average of all 3 terminal voltages equals
    // the neutral regardless of which phases are driven. This adapts
    // automatically to speed, load, and Vbus — no fixed threshold needed.
    int16_t raw_u = (int16_t)g_bemf_raw[0];
    int16_t raw_v = (int16_t)g_bemf_raw[1];
    int16_t raw_w = (int16_t)g_bemf_raw[2];
    int16_t neutral = (raw_u + raw_v + raw_w) / 3;

    g_bemf_signed[0] = raw_u - neutral;  // Phase U
    g_bemf_signed[1] = raw_v - neutral;  // Phase V
    g_bemf_signed[2] = raw_w - neutral;  // Phase W

    g_step_ticks++;

    switch (g_state) {

    // ── ALIGN: Lock rotor to step 0 position ───────────────────────────
    case STATE_ALIGN:
        apply_step(0, ALIGN_DUTY);
        if (++g_align_cnt >= ALIGN_TICKS) {
            g_state = STATE_RAMP;
            g_step = 1;  // First step after align at 0 (60° pull)
            g_step_ticks = 0;
            g_ramp_period = RAMP_INITIAL_PERIOD;
            g_ramp_steps = 0;
        }
        break;

    // ── RAMP: Forced commutation with sign-change ZC validation ────────
    case STATE_RAMP: {
        apply_step(g_step, RAMP_DUTY);

        // Sign-change ZC detection on the floating phase.
        // Only check after blanking (20% of step) to avoid flyback ringing.
        uint32_t blanking = (uint32_t)((float)g_ramp_period * 0.20f);
        if (!g_zc_detected && g_step_ticks > blanking) {
            if (detect_zc_sign_change(g_step)) {
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

            // Transition: fast enough AND 6/12 recent steps have valid ZC
            if (g_ramp_steps >= RAMP_MIN_STEPS &&
                g_ramp_period <= RAMP_FINAL_PERIOD &&
                g_zc_window_sum >= 6) {
                g_state = STATE_CLOSEDLOOP;
                g_step_period = g_ramp_period;
                g_zc_miss = 0;
                g_comm_dead = 0;
            }

            // Safety: abort ramp if it takes too long (motor stalled or no sync)
            if (g_ramp_steps >= RAMP_MAX_STEPS) {
                platform_pwm_stop();
                reset_state();
            }
        }
        break;
    }

    // ── CLOSEDLOOP: Sign-change ZC with dead-time commutation ──────────
    // Detect BEMF zero-crossing via sign change on the floating phase,
    // wait 30° electrical delay, then commutate with brief dead-time.
    // No fixed threshold — self-referencing neutral adapts automatically.
    case STATE_CLOSEDLOOP: {
        float duty = LIMIT(g_throttle, DUTY_MIN, DUTY_MAX);

        // Dead-time: all phases floating for COMM_DEAD_TICKS after commutation.
        // Allows BEMF to settle cleanly for the next step's ZC detection.
        if (g_comm_dead > 0) {
            g_comm_dead--;
            if (g_comm_dead == 0) {
                apply_step(g_step, duty);
            }
            break;
        }

        apply_step(g_step, duty);

        // Blanking: ignore ZC for first 15% of step (flyback ringing)
        uint32_t blanking = (uint32_t)((float)g_step_period * ZC_BLANKING_PCT);
        if (blanking < 20) blanking = 20;

        // Sign-change zero-crossing detection on floating phase
        if (!g_zc_detected && g_step_ticks > blanking) {
            if (detect_zc_sign_change(g_step)) {
                g_zc_detected = 1;
                g_zc_tick = g_step_ticks;
                g_zc_count++;
            }
        }

        // 30° commutation delay: ZC occurs at midpoint of 60° step,
        // so commutate at 2×T_zc (T_zc ticks after ZC = 30° electrical).
        uint8_t do_commutate = 0;
        if (g_zc_detected && g_step_ticks >= g_zc_tick * 2) {
            do_commutate = 1;
        }
        // Safety timeout: if no ZC detected, force commutate to maintain timing.
        if (!g_zc_detected && g_step_ticks >= g_step_period * 2) {
            do_commutate = 1;
        }

        if (do_commutate) {
            // Track ZC in sliding window
            g_zc_window_sum -= g_zc_window[g_zc_window_idx];
            g_zc_window[g_zc_window_idx] = g_zc_detected ? 1 : 0;
            g_zc_window_sum += g_zc_window[g_zc_window_idx];
            g_zc_window_idx = (g_zc_window_idx + 1) % 12;

            if (g_zc_detected) {
                // Update step period directly from ZC timing.
                // Sign-change detection is inherently accurate — no IIR filter
                // needed. Just clamp to prevent runaway.
                uint32_t new_period = g_zc_tick * 2;
                if (new_period < MIN_STEP_PERIOD) new_period = MIN_STEP_PERIOD;
                g_step_period = new_period;
                g_zc_miss = 0;
            } else {
                g_zc_miss++;
                if (g_zc_miss >= ZC_MISS_MAX) {
                    platform_pwm_stop();
                    reset_state();
                    break;
                }
            }

            float erps_raw = (float)PWM_FREQ / ((float)g_step_period * 6.0f);
            g_speed_erps += SPEED_LPF * (erps_raw - g_speed_erps);

            // Float all phases for dead-time, then advance step
            platform_pwm_float(PWM_PORT1);
            platform_pwm_float(PWM_PORT2);
            platform_pwm_float(PWM_PORT3);
            g_active_step = 0xFF;  // Force apply_step to reconfigure
            g_comm_dead = COMM_DEAD_TICKS;

            advance_step();
        }
        break;
    }

    default:
        break;
    }

    // Update previous signed BEMF for next tick's sign-change detection
    g_bemf_prev[0] = g_bemf_signed[0];
    g_bemf_prev[1] = g_bemf_signed[1];
    g_bemf_prev[2] = g_bemf_signed[2];
}

// ── 25 Hz burst streaming ──────────────────────────────────────────────────

static void on_scheduler_25hz(uint8_t *data, size_t size) {
    (void)data; (void)size;
    if (g_burst_state != 2) return;

    float log_data[12];
    log_data[0] = -1.0f;  // Burst marker
    log_data[1] = (float)g_burst_send_pos;
    log_data[2] = (float)BURST_SAMPLES;
    uint16_t remaining = BURST_SAMPLES - g_burst_send_pos;
    uint16_t count = remaining < BURST_CHUNK_SAMPLES ? remaining : BURST_CHUNK_SAMPLES;
    memcpy(&log_data[3], &g_burst_buf[g_burst_send_pos * 3],
           count * 3 * sizeof(uint16_t));
    if (count < BURST_CHUNK_SAMPLES) {
        memset((uint8_t *)&log_data[3] + count * 3 * sizeof(uint16_t), 0,
               (BURST_CHUNK_SAMPLES - count) * 3 * sizeof(uint16_t));
    }
    g_burst_send_pos += count;
    if (g_burst_send_pos >= BURST_SAMPLES) {
        g_burst_state = 0;
    }
    publish(SEND_LOG, (uint8_t *)log_data, sizeof(log_data));
}

// ── 100 Hz ADC live ───────────────────────────────────────────────────────
// Compact 8-byte frame: 4 × uint16_t [U, V, W, step]
// At 19200 baud: 16 bytes/frame × 100 Hz = 1600 B/s (fits in 1920 B/s limit)
// Reads g_bemf_dma directly — always updated by ADC2 DMA, even when TIM1/motor idle.

static void on_scheduler_100hz(uint8_t *data, size_t size) {
    (void)data; (void)size;
    if (g_log_class != LOG_CLASS_ADC_LIVE) return;

    uint16_t adc_frame[4];
    adc_frame[0] = g_bemf_dma[0];
    adc_frame[1] = g_bemf_dma[1];
    adc_frame[2] = g_bemf_dma[2];
    adc_frame[3] = (uint16_t)g_step;
    publish(SEND_LOG, (uint8_t *)adc_frame, sizeof(adc_frame));
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
    // In PWM Mode 1 center-aligned, the active pulse (high-side ON) occurs
    // when CNT < CCR, so the ON-time is centered around CNT=0 (the valley).
    // CCR4=1 triggers the injected ADC at the center of the ON pulse, where
    // the active high-side FET places the neutral point at ~Vbus/2 — correct
    // for BEMF threshold comparison. (CCR4=ARR-1 would sample at the center
    // of the OFF-time where neutral collapses to 0V.)
    platform_pwm_init(PWM_PORT1);
    platform_pwm_init(PWM_PORT2);
    platform_pwm_init(PWM_PORT3);
    htim1.Instance->CCR4 = 1;

    subscribe(FOC_RELEASE, on_foc_release);
    subscribe(MOTOR_THROTTLE, on_motor_throttle);
    subscribe(ADC_INJECTED_COMPLETE, on_tick);
    subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
    subscribe(SCHEDULER_25HZ, on_scheduler_25hz);
    subscribe(SCHEDULER_100HZ, on_scheduler_100hz);
}
