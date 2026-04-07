// Sensorless FOC for B-G431B-ESC1 — BEMF Sliding Mode Observer + PLL
//
// Combines two modes:
//   1. Open-loop V/f sinusoidal commutation for startup (ALIGN → RAMP)
//   2. Closed-loop using BEMF observer angle estimation (RUN)
//
// The Sliding Mode Observer (SMO) estimates back-EMF from measured phase
// currents and reconstructed applied voltages. A Phase-Locked Loop (PLL)
// tracks the BEMF vector angle to extract the rotor electrical angle and
// speed. During startup, BEMF is too weak for tracking, so forced open-loop
// commutation is used. Once the observer converges, it takes over.
//
// Motor model in α-β (Clarke) frame:
//   di_α/dt = (1/L) · (v_α − R·i_α − e_α)
//   di_β/dt = (1/L) · (v_β − R·i_β − e_β)
//
// BEMF: e_α = −λ·ω·sin(θ), e_β = λ·ω·cos(θ)
// PLL error: e_α·cos(θ_est) + e_β·sin(θ_est) = λ·ω·sin(θ_est − θ)

#include "foc.h"
#include <pubsub.h>
#include <platform.h>
#include <messages.h>
#include <macro.h>
#include <string.h>
#include <math.h>

// ── 3-phase sine lookup table ──────────────────────────────────────────────
static float g_phases[SINE_TABLE_SIZE][3];

static void create_lookup_table(void) {
    float step = TWO_PI / (float)SINE_TABLE_SIZE;
    float angle = 0.0f;
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        g_phases[i][0] = sinf(angle);
        g_phases[i][1] = sinf(angle + TWO_PI / 3.0f);
        g_phases[i][2] = sinf(angle + TWO_PI * 2.0f / 3.0f);
        angle += step;
    }
}

// ── Observer constants ─────────────────────────────────────────────────────
#define DT_40KHZ           (1.0f / 40000.0f)

// SMO tuning — adjust per motor
#define SMO_GAIN           50.0f    // must exceed max BEMF voltage
#define SMO_BANDWIDTH      0.5f     // saturation function bandwidth (Amps)
#define BEMF_LPF_COEFF     0.02f    // BEMF low-pass filter (0–1, higher = faster)
#define PLL_KP             200.0f   // PLL proportional gain
#define PLL_KI             5000.0f  // PLL integral gain
#define OBSERVER_MIN_SPEED 50.0f    // min ω (rad/s elec) to trust observer
#define BEMF_MIN_MAG       0.5f     // min |BEMF| (V) to trust observer
#define OMEGA_MAX          10000.0f // PLL speed clamp (rad/s)

// Saturation function: clamped linear ≈ sign() without chattering
static inline float sat(float x, float h) {
    float y = x / h;
    if (y > 1.0f) return 1.0f;
    if (y < -1.0f) return -1.0f;
    return y;
}

// Clarke transform: 3-phase → α-β
static inline void clarke(float a, float b, float c,
                           float *alpha, float *beta) {
    *alpha = (2.0f * a - b - c) / 3.0f;
    *beta  = (b - c) / 1.7320508f;  // √3
}

// ── Observer state ─────────────────────────────────────────────────────────
static float g_i_alpha_hat;    // estimated α current (A)
static float g_i_beta_hat;     // estimated β current (A)
static float g_bemf_alpha;     // filtered BEMF α (V)
static float g_bemf_beta;      // filtered BEMF β (V)
static float g_theta_est;      // estimated electrical angle (rad, 0–2π)
static float g_omega_est;      // estimated electrical speed (rad/s)
static float g_bemf_mag;       // |BEMF| for convergence check
static uint8_t g_obs_valid;    // 1 when observer has converged

static void observer_reset(void) {
    g_i_alpha_hat = 0.0f;
    g_i_beta_hat  = 0.0f;
    g_bemf_alpha  = 0.0f;
    g_bemf_beta   = 0.0f;
    g_theta_est   = 0.0f;
    g_omega_est   = 0.0f;
    g_bemf_mag    = 0.0f;
    g_obs_valid   = 0;
}

static void observer_update(float i_u, float i_v, float i_w,
                             float duty_u, float duty_v, float duty_w,
                             float vbus) {
    // Clarke transform on measured currents
    float i_alpha, i_beta;
    clarke(i_u, i_v, i_w, &i_alpha, &i_beta);

    // Reconstruct applied phase voltages from duty cycles
    float v_u = (duty_u / (float)PWM_PERIOD) * vbus;
    float v_v = (duty_v / (float)PWM_PERIOD) * vbus;
    float v_w = (duty_w / (float)PWM_PERIOD) * vbus;
    float v_alpha, v_beta;
    clarke(v_u, v_v, v_w, &v_alpha, &v_beta);

    // Sliding mode: current estimation error → switching function
    float err_alpha = g_i_alpha_hat - i_alpha;
    float err_beta  = g_i_beta_hat  - i_beta;
    float z_alpha = SMO_GAIN * sat(err_alpha, SMO_BANDWIDTH);
    float z_beta  = SMO_GAIN * sat(err_beta,  SMO_BANDWIDTH);

    // Observer current prediction (Euler forward)
    float inv_L = 1.0f / MOTOR_INDUCTANCE;
    g_i_alpha_hat += DT_40KHZ * inv_L *
                     (v_alpha - MOTOR_RESISTANCE * g_i_alpha_hat - z_alpha);
    g_i_beta_hat  += DT_40KHZ * inv_L *
                     (v_beta  - MOTOR_RESISTANCE * g_i_beta_hat  - z_beta);

    // Low-pass filter sliding mode output → estimated BEMF
    g_bemf_alpha += BEMF_LPF_COEFF * (z_alpha - g_bemf_alpha);
    g_bemf_beta  += BEMF_LPF_COEFF * (z_beta  - g_bemf_beta);

    // BEMF magnitude
    g_bemf_mag = sqrtf(g_bemf_alpha * g_bemf_alpha +
                       g_bemf_beta  * g_bemf_beta);

    // PLL: error = e_α·cos(θ̂) + e_β·sin(θ̂) = λω·sin(θ̂ − θ)
    float sin_est = sinf(g_theta_est);
    float cos_est = cosf(g_theta_est);
    float pll_err = g_bemf_alpha * cos_est + g_bemf_beta * sin_est;

    // PI controller on PLL error → speed and angle
    g_omega_est += PLL_KI * pll_err * DT_40KHZ;
    g_theta_est += (g_omega_est + PLL_KP * pll_err) * DT_40KHZ;

    // Clamp speed (prevent runaway)
    if (g_omega_est < 0.0f) g_omega_est = 0.0f;
    if (g_omega_est > OMEGA_MAX) g_omega_est = OMEGA_MAX;

    // Wrap angle to [0, 2π]
    if (g_theta_est >= TWO_PI) g_theta_est -= TWO_PI;
    if (g_theta_est < 0.0f)   g_theta_est += TWO_PI;

    // Convergence: BEMF strong enough and speed above minimum
    g_obs_valid = (g_bemf_mag > BEMF_MIN_MAG &&
                   g_omega_est > OBSERVER_MIN_SPEED) ? 1 : 0;
}

// ── FOC state ──────────────────────────────────────────────────────────────
typedef enum {
    STATE_IDLE,
    STATE_ALIGN,
    STATE_RAMP,
    STATE_RUN,
} foc_state_e;

static foc_state_e g_state;
static float       g_ol_angle;       // open-loop angle (fractional table index)
static float       g_ol_speed;       // open-loop speed (index/tick)
static float       g_throttle;       // 0.0–1.0
static float       g_amplitude;      // PWM amplitude (timer ticks)
static uint32_t    g_align_counter;
static uint32_t    g_ramp_counter;
static uint8_t     g_active_log_class;

// Last applied duty cycles (for observer voltage reconstruction)
static float       g_duty_u;
static float       g_duty_v;
static float       g_duty_w;

// Latest sensor readings (updated by PubSub callbacks)
static float       g_i_u, g_i_v, g_i_w;   // phase currents (A)
static float       g_vbus = 12.0f;         // bus voltage (V)

// Encoder angle (primary source, updated at 1 kHz)
static float       g_encoder_angle;        // electrical angle (rad, 0–2π)
static uint8_t     g_encoder_valid;

#define MAX_AMPLITUDE  ((float)PWM_PERIOD * 0.45f)

// Open-loop speed: table index increments per 40kHz tick at full throttle
// ω_elec = SPEED * 40000 / TABLE_SIZE * 2π  (rad/s)
// Example: 2.0 → 2·40000/256·2π ≈ 1963 rad/s → 312 Hz → 2676 RPM (7 poles)
#define SPEED_AT_FULL_THROTTLE  2.0f

// Startup parameters
#define ALIGN_TICKS     8000       // 200 ms at 40 kHz
#define RAMP_SPEED_INIT 0.05f
#define RAMP_SPEED_INC  0.000005f
#define RAMP_AMP_INIT   0.05f
#define RAMP_AMP_INC    0.000002f

// ── Apply 3-phase PWM ──────────────────────────────────────────────────────

// From table index (open-loop path)
static void apply_pwm_table(float step, float amplitude) {
    int idx = (int)step;
    if (idx >= SINE_TABLE_SIZE) idx -= SINE_TABLE_SIZE;
    if (idx < 0) idx += SINE_TABLE_SIZE;

    g_duty_u = (1.0f + g_phases[idx][0]) * amplitude;
    g_duty_v = (1.0f + g_phases[idx][1]) * amplitude;
    g_duty_w = (1.0f + g_phases[idx][2]) * amplitude;

    platform_pwm_set_duty(PWM_PHASE_U, (uint32_t)g_duty_u);
    platform_pwm_set_duty(PWM_PHASE_V, (uint32_t)g_duty_v);
    platform_pwm_set_duty(PWM_PHASE_W, (uint32_t)g_duty_w);
}

// From angle in radians (observer path)
static void apply_pwm_angle(float angle_rad, float amplitude) {
    float step = angle_rad * ((float)SINE_TABLE_SIZE / TWO_PI);
    if (step >= (float)SINE_TABLE_SIZE) step -= (float)SINE_TABLE_SIZE;
    if (step < 0.0f) step += (float)SINE_TABLE_SIZE;
    apply_pwm_table(step, amplitude);
}

// ── Callbacks ──────────────────────────────────────────────────────────────

static void on_phase_current(uint8_t *data, size_t size) {
    if (size < sizeof(phase_current_t)) return;
    phase_current_t c;
    memcpy(&c, data, sizeof(c));
    g_i_u = c.phase_u;
    g_i_v = c.phase_v;
    g_i_w = c.phase_w;
}

static void on_bus_voltage(uint8_t *data, size_t size) {
    if (size < sizeof(bus_measurement_t)) return;
    bus_measurement_t m;
    memcpy(&m, data, sizeof(m));
    if (m.bus_voltage > 1.0f) g_vbus = m.bus_voltage;
}

static void on_encoder(uint8_t *data, size_t size) {
    if (size < sizeof(encoder_data_t)) return;
    encoder_data_t enc;
    memcpy(&enc, data, sizeof(enc));
    g_encoder_angle = enc.electrical_angle;
    g_encoder_valid = enc.valid;
}

static void on_motor_throttle(uint8_t *data, size_t size) {
    if (size < sizeof(motor_throttle_t)) return;
    motor_throttle_t cmd;
    memcpy(&cmd, data, sizeof(cmd));
    g_throttle = LIMIT(cmd.throttle, 0.0f, 1.0f);

    if (g_throttle < 0.01f && g_state != STATE_IDLE) {
        platform_pwm_stop();
        g_state = STATE_IDLE;
        g_ol_angle = 0.0f;
        g_ol_speed = 0.0f;
        g_amplitude = 0.0f;
        observer_reset();
    } else if (g_throttle > 0.01f && g_state == STATE_IDLE) {
        g_state = STATE_ALIGN;
        g_align_counter = 0;
        g_ramp_counter = 0;
        g_ol_angle = 0.0f;
        g_ol_speed = RAMP_SPEED_INIT;
        g_amplitude = RAMP_AMP_INIT * MAX_AMPLITUDE;
        observer_reset();
        platform_pwm_start();
    }
}

// 40 kHz commutation tick (triggered by ADC injected → TIM1_CC4)
static void on_adc_injected_complete(uint8_t *data, size_t size) {
    if (g_state == STATE_IDLE) return;

    switch (g_state) {
    case STATE_ALIGN:
        // Hold at fixed angle to lock rotor
        apply_pwm_table(0.0f, 0.08f * MAX_AMPLITUDE);
        if (++g_align_counter >= ALIGN_TICKS) {
            g_state = STATE_RAMP;
            g_ramp_counter = 0;
        }
        break;

    case STATE_RAMP: {
        // Open-loop forced commutation with increasing speed + voltage
        g_ol_angle += g_ol_speed;
        if (g_ol_angle >= (float)SINE_TABLE_SIZE)
            g_ol_angle -= (float)SINE_TABLE_SIZE;

        g_ol_speed  += RAMP_SPEED_INC;
        g_amplitude += RAMP_AMP_INC * MAX_AMPLITUDE;

        float target_speed = g_throttle * SPEED_AT_FULL_THROTTLE;
        float target_amp   = g_throttle * MAX_AMPLITUDE;
        if (g_ol_speed > target_speed) g_ol_speed = target_speed;
        if (g_amplitude > target_amp)  g_amplitude = target_amp;

        apply_pwm_table(g_ol_angle, g_amplitude);

        // Run observer in background so it can converge before handoff
        observer_update(g_i_u, g_i_v, g_i_w,
                        g_duty_u, g_duty_v, g_duty_w, g_vbus);

        if (++g_ramp_counter > 4000 && g_ol_speed >= target_speed) {
            g_state = STATE_RUN;
        }
        break;
    }

    case STATE_RUN:
        g_amplitude = g_throttle * MAX_AMPLITUDE;

        // Always run observer (fallback angle source)
        observer_update(g_i_u, g_i_v, g_i_w,
                        g_duty_u, g_duty_v, g_duty_w, g_vbus);

        if (g_encoder_valid) {
            // Primary: commutate at encoder angle
            apply_pwm_angle(g_encoder_angle, g_amplitude);
        } else if (g_obs_valid) {
            // Fallback: BEMF observer angle
            apply_pwm_angle(g_theta_est, g_amplitude);
        } else {
            // Last resort: open-loop
            g_ol_speed = g_throttle * SPEED_AT_FULL_THROTTLE;
            g_ol_angle += g_ol_speed;
            if (g_ol_angle >= (float)SINE_TABLE_SIZE)
                g_ol_angle -= (float)SINE_TABLE_SIZE;
            apply_pwm_table(g_ol_angle, g_amplitude);
        }
        break;

    default:
        break;
    }
}

static void on_foc_setup(uint8_t *data, size_t size) {
    platform_pwm_enable_phase(PWM_PHASE_U);
    platform_pwm_enable_phase(PWM_PHASE_V);
    platform_pwm_enable_phase(PWM_PHASE_W);
}

static void on_foc_release(uint8_t *data, size_t size) {
    platform_pwm_stop();
    g_state = STATE_IDLE;
    g_throttle = 0.0f;
    g_ol_angle = 0.0f;
    g_ol_speed = 0.0f;
    g_amplitude = 0.0f;
    observer_reset();
}

static void on_notify_log_class(uint8_t *data, size_t size) {
    if (size < 1) return;
    g_active_log_class = (data[0] == LOG_CLASS_FOC) ? LOG_CLASS_FOC : 0;
}

static void on_scheduler_25hz(uint8_t *data, size_t size) {
    if (g_active_log_class == 0) return;

    float log_data[8] = {
        (float)g_state,
        g_encoder_angle,            // encoder electrical angle (rad)
        g_theta_est,                // observer angle (rad)
        g_omega_est,                // observer speed (rad/s electrical)
        g_amplitude,                // PWM amplitude (ticks)
        (float)g_encoder_valid,     // encoder valid flag
        (float)g_obs_valid,         // observer converged flag
        g_bemf_mag                  // |BEMF| (V)
    };
    publish(SEND_LOG, (uint8_t *)log_data, sizeof(log_data));
}

// ── Setup ──────────────────────────────────────────────────────────────────
void foc_setup(void) {
    create_lookup_table();
    observer_reset();

    subscribe(FOC_SETUP, on_foc_setup);
    subscribe(FOC_RELEASE, on_foc_release);
    subscribe(MOTOR_THROTTLE, on_motor_throttle);
    subscribe(ADC_INJECTED_COMPLETE, on_adc_injected_complete);
    subscribe(SENSOR_PHASE_CURRENT, on_phase_current);
    subscribe(SENSOR_BUS_VOLTAGE, on_bus_voltage);
    subscribe(SENSOR_ENCODER, on_encoder);
    subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
    subscribe(SCHEDULER_25HZ, on_scheduler_25hz);
}
