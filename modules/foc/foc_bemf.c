// FOC BEMF — sensorless closed-loop using flux observer
//
// Startup: IDLE → ALIGN → RAMP → OPENLOOP → CLOSEDLOOP
//
// Observer estimates rotor electrical angle from back-EMF using a leaky
// flux integrator (voltage model):
//   dpsi/dt = (v_applied - R * i) - wc * psi
//   theta_e = atan2(psi_beta, psi_alpha)
//
// The leaky term (wc) prevents DC drift.  Phase error atan(wc/w_e) is
// small when w_e >> wc.
//
// Encoder is read for comparison logging only — NOT used for commutation.
//
// Build: bash build.sh bemf
//
// Log classes:
//   VOLTAGE  (3) — Park Id/Iq + commutation theta + bemf_theta + enc_elec
//   CURRENT  (2) — Clarke + phase currents + Vbus
//   BEMF     (8) — Observer: bemf_theta, ol_theta, enc_elec, flux, mag, speed

#include "foc.h"
#ifdef FOC_MODE_BEMF
#include "foc_math.h"
#include <pubsub.h>
#include <platform.h>
#include <messages.h>
#include <macro.h>
#include <string.h>
#include <math.h>

// ── Log classes ────────────────────────────────────────────────────────────

#define LOG_CLASS_CURRENT  2
#define LOG_CLASS_VOLTAGE  3
#define LOG_CLASS_BEMF     8

// ── States ─────────────────────────────────────────────────────────────────

typedef enum {
    STATE_IDLE,         // 0
    STATE_ALIGN,        // 1
    STATE_RAMP,         // 2
    STATE_OPENLOOP,     // 3
    STATE_BLENDING,     // 4  — gradual OL→CL transition
    STATE_CLOSEDLOOP,   // 5
} state_e;

// ── Constants ──────────────────────────────────────────────────────────────

#define MAX_DUTY        ((float)PWM_PERIOD * 0.45f)

#define OL_SPEED        0.15f               // Max OL speed (23.4 Hz electrical)
#define OL_AMPLITUDE    (0.15f * MAX_DUTY)  // Strong enough to kick it

#define ALIGN_TICKS     4000                // 100 ms align
#define ALIGN_AMPLITUDE (0.15f * MAX_DUTY)
#define RAMP_MIN_TICKS  2000                // minimum 50 ms in ramp
#define RAMP_SPEED_INIT 0.01f
#define RAMP_SPEED_INC  0.00005f            // reaches max in ~70 ms
#define RAMP_AMP_INIT   (0.15f * MAX_DUTY)

// Current limiter (voltage-mode with Iq clamp)
#define V_MAX_SCALAR    0.577f              // 1/sqrt(3) max inverter voltage scalar
#define VQ_RAMP_RATE    0.001f              // ~40 V/s ramp response at 40kHz
#define IQ_LIMIT        3.0f                // Max Iq before Vq reduction (amps)
#define IQ_LIMIT_GAIN   0.5f                // Vq reduction per excess amp
#define VQ_MIN          (0.15f * MAX_DUTY * 12.0f / PWM_PERIOD) // Min Vq to sustain BEMF (0.81V)

// Observer tuning
#define DT_S            (1.0f / 40000.0f)   // 25 µs per commutation tick
#define OBSERVER_R      0.05f               // Observer resistance (low for high-KV tolerance)
#define OBSERVER_WC     20.0f               // Leaky integrator cutoff (rad/s)
#define BEMF_MAG_THRESH 0.005f              // Flux magnitude to allow blend start
#define BEMF_SPEED_LPF  0.005f              // Speed low-pass filter coefficient
#define OBS_MIN_SPEED   1.0f                // Min rad/s to apply phase lead comp
#define TIMING_ADVANCE  0.00003f            // Phase advance per rad/s (~30us)
#define TIMING_ADV_MAX  0.26f               // Cap timing advance to ~15 degrees
#define OL_HOLDOFF_TICKS 2000               // 50 ms steady OL holdoff
#define BLEND_TICKS      4000               // 100 ms blend duration (OL→CL)

// PWM & General control
#define DUTY_MAX_CLAMP  0.95f               // 95% max PWM
#define DUTY_MIN_CLAMP  0.05f               // 5% min PWM (bootstrap cap charge)
#define TIMING_OFFSET   0.5f                // SVPWM zero-sequence modulation center
#define THROTTLE_START  0.01f               // Throttle deadband
#define OL_SPEED_LPF    0.001f              // Open-loop speed transition smoothing

#define WRAP_ANGLE(val, max_val) do { while((val) >= (max_val)) (val) -= (max_val); while((val) < 0.0f) (val) += (max_val); } while(0)

// ── State ──────────────────────────────────────────────────────────────────

static state_e  g_state;
static float    g_throttle;
static uint8_t  g_log_class;

// Open-loop (ALIGN / RAMP / OL)
static float    g_ol_angle;
static float    g_ol_speed;
static float    g_ol_speed_smooth;
static float    g_amplitude;
static uint32_t g_align_cnt;
static uint32_t g_ramp_cnt;
static uint32_t g_ol_cnt;

// Encoder (read-only, for comparison logging)
static float    g_enc_elec;

// Measurements
static float    g_iu, g_iv, g_iw;
static float    g_vbus = 12.0f;

// Clarke / Park
static float    g_i_alpha, g_i_beta;
static float    g_id, g_iq;
static float    g_theta;           // angle used for commutation

// Flux observer
static float    g_flux_alpha, g_flux_beta;
static float    g_bemf_theta;      // estimated electrical angle (rad)
static float    g_bemf_theta_prev;
static float    g_bemf_speed;      // estimated electrical angular velocity (rad/s)
static float    g_bemf_mag;        // flux magnitude

// Blending (OL → CL)
static float    g_blend;            // 0.0 = pure OL, 1.0 = pure BEMF
static uint32_t g_blend_cnt;

// CL voltage
static float    g_cl_vq;

// FOC outputs
static float    g_vd, g_vq;
static float    g_v_alpha, g_v_beta;

// Duty cycles
static float    g_duty_a, g_duty_b, g_duty_c;

// ── Reset Helper ───────────────────────────────────────────────────────────

static void reset_foc_state(void) {
    g_state     = STATE_IDLE;
    g_ol_angle  = 0.0f;
    g_ol_speed  = 0.0f;
    g_ol_speed_smooth = 0.0f;
    g_amplitude = 0.0f;
    g_align_cnt = 0;
    g_ramp_cnt  = 0;
    g_ol_cnt    = 0;
    g_cl_vq     = 0.0f;
    g_flux_alpha = 0.0f;
    g_flux_beta  = 0.0f;
    g_bemf_theta = 0.0f;
    g_bemf_theta_prev = 0.0f;
    g_bemf_speed = 0.0f;
    g_bemf_mag   = 0.0f;
    g_blend      = 0.0f;
    g_blend_cnt  = 0;
}

// ── PWM helpers ────────────────────────────────────────────────────────────

static void apply_svpwm(float va, float vb, float vc) {
    float vmin = va, vmax = va;
    if (vb < vmin) vmin = vb;
    if (vb > vmax) vmax = vb;
    if (vc < vmin) vmin = vc;
    if (vc > vmax) vmax = vc;
    float voff = -(vmin + vmax) * TIMING_OFFSET;
    va += voff;  vb += voff;  vc += voff;

    float scale = (float)PWM_PERIOD / g_vbus;
    float half  = (float)PWM_PERIOD * TIMING_OFFSET;
    g_duty_a = half + va * scale;
    g_duty_b = half + vb * scale;
    g_duty_c = half + vc * scale;

    float max_d = (float)PWM_PERIOD * DUTY_MAX_CLAMP;
    float min_d = (float)PWM_PERIOD * DUTY_MIN_CLAMP;
    g_duty_a = LIMIT(g_duty_a, min_d, max_d);
    g_duty_b = LIMIT(g_duty_b, min_d, max_d);
    g_duty_c = LIMIT(g_duty_c, min_d, max_d);

    platform_pwm_send(PWM_PORT1, (uint32_t)g_duty_a);
    platform_pwm_send(PWM_PORT2, (uint32_t)g_duty_b);
    platform_pwm_send(PWM_PORT3, (uint32_t)g_duty_c);
}

static inline float ol_theta_from_angle(float ol_angle) {
    float theta = ol_angle * (TWO_PI / (float)SINE_TABLE_SIZE);
    WRAP_ANGLE(theta, TWO_PI);
    return theta;
}

static void apply_openloop(float step, float amplitude) {
    float theta = ol_theta_from_angle(step);
    float v_amp = amplitude * g_vbus / (float)PWM_PERIOD;

    // Apply Vd = amplitude along d-axis (same approach as foc_encoder)
    inv_park(v_amp, 0.0f, theta, &g_v_alpha, &g_v_beta);
    float va, vb, vc;
    inv_clarke(g_v_alpha, g_v_beta, &va, &vb, &vc);
    apply_svpwm(va, vb, vc);
}

// ── Flux observer ──────────────────────────────────────────────────────────

static void observer_update(void) {
    // Reconstruct applied phase voltages from duty cycles (previous tick)
    float va = (g_duty_a / (float)PWM_PERIOD) * g_vbus;
    float vb = (g_duty_b / (float)PWM_PERIOD) * g_vbus;
    float vc = (g_duty_c / (float)PWM_PERIOD) * g_vbus;

    float v_alpha_app, v_beta_app;
    clarke(va, vb, vc, &v_alpha_app, &v_beta_app);

    // Leaky flux integrator:  dpsi/dt = (v - R*i) - wc * psi
    g_flux_alpha += DT_S * ((v_alpha_app - OBSERVER_R * g_i_alpha)
                            - OBSERVER_WC * g_flux_alpha);
    g_flux_beta  += DT_S * ((v_beta_app  - OBSERVER_R * g_i_beta)
                            - OBSERVER_WC * g_flux_beta);

    // Electrical angle from flux linkage
    float theta_est = atan2f(g_flux_beta, g_flux_alpha);
    if (theta_est < 0.0f) theta_est += TWO_PI;

    // Electrical speed estimate (moderate LPF — balance tracking vs noise)
    float dtheta = theta_est - g_bemf_theta_prev;
    if (dtheta >  (float)M_PI) dtheta -= TWO_PI;
    if (dtheta < -(float)M_PI) dtheta += TWO_PI;
    float speed_raw = dtheta / DT_S;
    g_bemf_speed += BEMF_SPEED_LPF * (speed_raw - g_bemf_speed);

    g_bemf_theta_prev = theta_est;
    g_bemf_theta = theta_est;

    // Compensate observer phase lead AND add timing advance for RL lag/software delay
    float abs_spd = fabsf(g_bemf_speed);
    if (abs_spd > OBS_MIN_SPEED) {
        float phase_lead = atanf(OBSERVER_WC / abs_spd);
        float timing_advance = abs_spd * TIMING_ADVANCE;
        if (timing_advance > TIMING_ADV_MAX) timing_advance = TIMING_ADV_MAX;
        
        float correction = phase_lead - timing_advance;
        if (g_bemf_speed > 0.0f)
            g_bemf_theta -= correction;
        else
            g_bemf_theta += correction;
            
        WRAP_ANGLE(g_bemf_theta, TWO_PI);
    }

    // Flux magnitude (transition criterion)
    g_bemf_mag = sqrtf(g_flux_alpha * g_flux_alpha + g_flux_beta * g_flux_beta);
}

// ── Sensor callbacks ───────────────────────────────────────────────────────

static void on_bus_voltage(uint8_t *data, size_t size) {
    if (size < sizeof(bus_measurement_t)) return;
    bus_measurement_t m;
    memcpy(&m, data, sizeof(m));
    g_vbus = m.bus_voltage;
}

static void on_encoder(uint8_t *data, size_t size) {
    if (size < sizeof(encoder_data_t)) return;
    encoder_data_t enc;
    memcpy(&enc, data, sizeof(enc));
    g_enc_elec = enc.electrical_angle;
}

// ── Throttle ───────────────────────────────────────────────────────────────

static void on_motor_throttle(uint8_t *data, size_t size) {
    if (size < sizeof(motor_throttle_t)) return;
    motor_throttle_t cmd;
    memcpy(&cmd, data, sizeof(cmd));
    g_throttle = LIMIT(cmd.throttle, 0.0f, 1.0f);

    if (g_throttle < THROTTLE_START && g_state != STATE_IDLE) {
        platform_pwm_stop();
        reset_foc_state();
    }
    else if (g_throttle > THROTTLE_START && g_state == STATE_IDLE) {
        reset_foc_state();
        g_state     = STATE_ALIGN;
        g_ol_speed  = RAMP_SPEED_INIT;
        g_ol_speed_smooth = OL_SPEED;
        g_amplitude = RAMP_AMP_INIT;
        platform_pwm_start();
    }
}

static void on_notify_log_class(uint8_t *data, size_t size) {
    if (size < 1) return;
    g_log_class = data[0];
}

// ── CL commutation (voltage-mode + Iq current limiter) ─────────────────

static void cl_commutate(float theta) {
    float v_max = g_vbus * V_MAX_SCALAR;
    float vq_target = g_throttle * v_max;
    if (vq_target < VQ_MIN) vq_target = VQ_MIN;   // Prevent stall at low throttle
    if (vq_target > v_max)  vq_target = v_max;

    if (g_cl_vq < vq_target) {
        g_cl_vq += VQ_RAMP_RATE;
        if (g_cl_vq > vq_target) g_cl_vq = vq_target;
    } else if (g_cl_vq > vq_target) {
        g_cl_vq -= VQ_RAMP_RATE;
        if (g_cl_vq < vq_target) g_cl_vq = vq_target;
    }

    // Current limiter: active limit for BOTH motoring (positive Iq) and braking (negative Iq)
    float vq_out = g_cl_vq;
    if (g_iq > IQ_LIMIT) {
        // Motoring too hard -> reduce voltage to lower current
        vq_out -= (g_iq - IQ_LIMIT) * IQ_LIMIT_GAIN;
        if (vq_out < 0.0f) vq_out = 0.0f;
    } else if (g_iq < -IQ_LIMIT) {
        // Braking too hard -> INCREASE voltage to catch up to BEMF and prevent stall/desync
        vq_out += (-g_iq - IQ_LIMIT) * IQ_LIMIT_GAIN;
    }

    g_vd = 0.0f;
    g_vq = vq_out;

    if (g_vq > v_max)  g_vq = v_max;
    if (g_vq < -v_max) g_vq = -v_max;

    inv_park(g_vd, g_vq, theta, &g_v_alpha, &g_v_beta);
    float va, vb, vc;
    inv_clarke(g_v_alpha, g_v_beta, &va, &vb, &vc);
    apply_svpwm(va, vb, vc);
}

// ── 40 kHz commutation ─────────────────────────────────────────────────────

static void on_commutate(uint8_t *data, size_t size) {
    if (size < sizeof(phase_current_t)) return;
    if (g_state == STATE_IDLE) return;

    phase_current_t c;
    memcpy(&c, data, sizeof(c));
    g_iu = c.phase_u;
    g_iv = c.phase_v;
    g_iw = c.phase_w;

    clarke(g_iu, g_iv, g_iw, &g_i_alpha, &g_i_beta);

    // Run observer in ALL active states (background estimation)
    observer_update();

    // Select angle for commutation
    float ol_theta = ol_theta_from_angle(g_ol_angle);
    float theta;
    if (g_state <= STATE_OPENLOOP) {
        theta = ol_theta;
    } else if (g_state == STATE_BLENDING) {
        // Blend OL and BEMF angles
        float diff = g_bemf_theta - ol_theta;
        if (diff >  (float)M_PI) diff -= TWO_PI;
        if (diff < -(float)M_PI) diff += TWO_PI;
        theta = ol_theta + g_blend * diff;
        WRAP_ANGLE(theta, TWO_PI);
    } else {
        theta = g_bemf_theta;
    }
    g_theta = theta;
    park(g_i_alpha, g_i_beta, theta, &g_id, &g_iq);

    switch (g_state) {

    case STATE_ALIGN:
        apply_openloop(0.0f, ALIGN_AMPLITUDE);
        if (++g_align_cnt >= ALIGN_TICKS) {
            g_state = STATE_RAMP;
            g_ramp_cnt = 0;
        }
        break;

    case STATE_RAMP:
    case STATE_OPENLOOP:
    case STATE_BLENDING: {
        g_ol_speed += RAMP_SPEED_INC;
        if (g_ol_speed > OL_SPEED) g_ol_speed = OL_SPEED;

        g_ol_speed_smooth += (OL_SPEED - g_ol_speed_smooth) * OL_SPEED_LPF;
        g_amplitude = OL_AMPLITUDE + g_throttle * (MAX_DUTY - OL_AMPLITUDE);
        
        g_ol_angle += (g_state == STATE_RAMP) ? g_ol_speed : g_ol_speed_smooth;
        WRAP_ANGLE(g_ol_angle, (float)SINE_TABLE_SIZE);

        if (g_state == STATE_RAMP) {
            apply_openloop(g_ol_angle, g_amplitude);
            if (++g_ramp_cnt > RAMP_MIN_TICKS && g_ol_speed >= OL_SPEED) {
                g_state  = STATE_OPENLOOP;
                g_ol_cnt = 0;
            }
        } 
        else if (g_state == STATE_OPENLOOP) {
            apply_openloop(g_ol_angle, g_amplitude);
            if (++g_ol_cnt > OL_HOLDOFF_TICKS && g_bemf_mag > BEMF_MAG_THRESH) {
                g_state     = STATE_BLENDING;
                g_blend     = 0.0f;
                g_blend_cnt = 0;
            }
        }
        else if (g_state == STATE_BLENDING) {
            // Ramp blend 0→1
            g_blend = (float)(++g_blend_cnt) / (float)BLEND_TICKS;
            if (g_blend >= 1.0f) g_blend = 1.0f;

            float vd_ol = g_amplitude * g_vbus / (float)PWM_PERIOD;
            float v_max = g_vbus * V_MAX_SCALAR;
            float vq_target = g_throttle * v_max;
            if (vq_target < VQ_MIN) vq_target = VQ_MIN;

            g_vd = (1.0f - g_blend) * vd_ol;
            g_vq = g_blend * vq_target;

            inv_park(g_vd, g_vq, theta, &g_v_alpha, &g_v_beta);
            float va, vb, vc;
            inv_clarke(g_v_alpha, g_v_beta, &va, &vb, &vc);
            apply_svpwm(va, vb, vc);

            if (g_blend >= 1.0f) {
                g_cl_vq = g_vq;
                g_state = STATE_CLOSEDLOOP;
            }
        }
        break;
    }

    case STATE_CLOSEDLOOP:
        cl_commutate(theta);
        break;

    default:
        break;
    }
}

// ── 25 Hz logging ──────────────────────────────────────────────────────────

static void on_scheduler_25hz(uint8_t *data, size_t size) {
    if (g_log_class == 0) return;

    float log_data[8];
    int ok = 0;

    switch (g_log_class) {
    case LOG_CLASS_VOLTAGE:
        // [0]state [1]Id [2]Iq [3]Vd [4]Vq [5]Iq_limit [6]theta [7]bemf_theta
        log_data[0] = (float)g_state;
        log_data[1] = g_id;
        log_data[2] = g_iq;
        log_data[3] = g_vd;
        log_data[4] = g_vq;
        log_data[5] = IQ_LIMIT;
        log_data[6] = g_theta;
        log_data[7] = g_bemf_theta;
        ok = 1;
        break;

    case LOG_CLASS_CURRENT:
        // [0]state [1]Ia [2]Ib [3]enc_elec [4]Iu [5]Iv [6]Iw [7]Vbus
        log_data[0] = (float)g_state;
        log_data[1] = g_i_alpha;
        log_data[2] = g_i_beta;
        log_data[3] = g_enc_elec;
        log_data[4] = g_iu;
        log_data[5] = g_iv;
        log_data[6] = g_iw;
        log_data[7] = g_vbus;
        ok = 1;
        break;

    case LOG_CLASS_BEMF:
        // [0]state [1]bemf_theta [2]ol_theta [3]enc_elec
        // [4]flux_a [5]flux_b [6]bemf_mag [7]bemf_speed
        log_data[0] = (float)g_state;
        log_data[1] = g_bemf_theta;
        log_data[2] = ol_theta_from_angle(g_ol_angle);
        log_data[3] = g_enc_elec;
        log_data[4] = g_flux_alpha;
        log_data[5] = g_flux_beta;
        log_data[6] = g_bemf_mag;
        log_data[7] = g_bemf_speed;
        ok = 1;
        break;

    default:
        break;
    }

    if (ok)
        publish(SEND_LOG, (uint8_t *)log_data, sizeof(log_data));
}

// ── PWM enable/release ─────────────────────────────────────────────────────

static void on_foc_setup(uint8_t *data, size_t size) {
    platform_pwm_init(PWM_PORT1);
    platform_pwm_init(PWM_PORT2);
    platform_pwm_init(PWM_PORT3);
}

static void on_foc_release(uint8_t *data, size_t size) {
    platform_pwm_stop();
    g_state     = STATE_IDLE;
    g_throttle  = 0.0f;
    g_ol_angle  = 0.0f;
    g_ol_speed  = 0.0f;
    g_ol_speed_smooth = 0.0f;
    g_amplitude = 0.0f;
    g_cl_vq     = 0.0f;
    g_flux_alpha = 0.0f;
    g_flux_beta  = 0.0f;
    g_bemf_theta = 0.0f;
    g_bemf_speed = 0.0f;
    g_bemf_mag   = 0.0f;
    g_ol_cnt     = 0;
    g_log_class  = 0;
}

// ── Setup ──────────────────────────────────────────────────────────────────

void foc_bemf_setup(void) {
    subscribe(FOC_SETUP, on_foc_setup);
    subscribe(FOC_RELEASE, on_foc_release);
    subscribe(MOTOR_THROTTLE, on_motor_throttle);
    subscribe(SENSOR_PHASE_CURRENT, on_commutate);
    subscribe(SENSOR_BUS_VOLTAGE, on_bus_voltage);
    subscribe(SENSOR_ENCODER, on_encoder);
    subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
    subscribe(SCHEDULER_25HZ, on_scheduler_25hz);
}
#endif // FOC_MODE_BEMF
