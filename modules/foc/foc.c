// ═══════════════════════════════════════════════════════════════════════════
// Standard Field-Oriented Control with AS5048A Encoder
//
// Checkpoint-based development:
//   CP1: Open-loop + encoder direction/pole pairs     [PASSED]
//   CP2: Encoder offset calibration
//   CP3: Clarke transform verification
//   CP4: Park transform verification
//   CP5: Closed-loop current control
//   CP6: Speed control
//
// Pipeline (closed-loop):
//   Measure:  Iu,Iv,Iw (ADC) + theta_enc (encoder) + Vbus (ADC)
//   Clarke:   Iu,Iv,Iw -> Ia, Ib
//   Park:     Ia,Ib,theta -> Id,Iq
//   PI:       Id_ref=0, Iq_ref -> Vd,Vq
//   InvPark:  Vd,Vq,theta -> Va,Vb
//   SVM:      Va,Vb -> Da,Db,Dc -> PWM
// ═══════════════════════════════════════════════════════════════════════════

#include "foc.h"
#include "foc_math.h"
#include <pubsub.h>
#include <platform.h>
#include <messages.h>
#include <macro.h>
#include <pi_controller.h>
#include <string.h>
#include <math.h>

// ── Sine table (open-loop commutation) ─────────────────────────────────────

static float g_sine[SINE_TABLE_SIZE][3];

static void create_sine_table(void) {
    float step = TWO_PI / (float)SINE_TABLE_SIZE;
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        float a = step * (float)i;
        g_sine[i][0] = sinf(a);
        g_sine[i][1] = sinf(a + TWO_PI / 3.0f);
        g_sine[i][2] = sinf(a + TWO_PI * 2.0f / 3.0f);
    }
}

// ── Constants ──────────────────────────────────────────────────────────────

#define DT              (1.0f / 40000.0f)
#define MAX_DUTY        ((float)PWM_PERIOD * 0.45f)

// Open-loop
#define OL_SPEED        0.005f  // 0.78 Hz electrical (slow for CP3/CP4 visualization)
#define OL_AMPLITUDE    (0.35f * MAX_DUTY)

// Startup
#define ALIGN_TICKS     20000
#define ALIGN_AMPLITUDE (0.35f * MAX_DUTY)
#define RAMP_MIN_TICKS  4000
#define RAMP_SPEED_INIT 0.005f
#define RAMP_SPEED_INC  0.000003f
#define RAMP_AMP_INIT   (0.15f * MAX_DUTY)
#define RAMP_AMP_INC    (0.000004f * MAX_DUTY)

// Calibration: 4 angles x 1s each, average last 25%
#define CAL_STEPS       4
#define CAL_TICKS       40000
#define CAL_AVG_START   30000

// ── State ──────────────────────────────────────────────────────────────────

typedef enum {
    STATE_IDLE,         // 0
    STATE_CALIBRATE,    // 1  CP2: lock at 4 angles, log encoder
    STATE_ALIGN,        // 2  lock rotor, capture offset
    STATE_RAMP,         // 3  open-loop acceleration
    STATE_OPENLOOP,     // 4  steady open-loop (CP1/CP3/CP4)
    STATE_CLOSEDLOOP,   // 5  full FOC (CP5/CP6)
} state_e;

static state_e  g_state;
static float    g_throttle;
static uint8_t  g_log_class;

// Open-loop
static float    g_ol_angle;
static float    g_ol_speed;
static float    g_ol_speed_smooth;  // low-pass filtered speed
static float    g_amplitude;
static uint32_t g_align_cnt;
static uint32_t g_ramp_cnt;

// Calibration
static uint32_t g_cal_cnt;
static uint8_t  g_cal_step;
static float    g_cal_table_idx[CAL_STEPS];
static float    g_cal_enc[CAL_STEPS];
static float    g_cal_sin_sum[CAL_STEPS];
static float    g_cal_cos_sum[CAL_STEPS];
static uint32_t g_cal_avg_cnt;

// Encoder
static float    g_enc_elec;
static float    g_enc_offset;
static uint8_t  g_enc_valid;

// Measurements
static float    g_iu, g_iv, g_iw;
static float    g_vbus = 12.0f;

// Clarke outputs
static float    g_i_alpha, g_i_beta;

// Park outputs
static float    g_id, g_iq;
static float    g_theta;

// FOC outputs
static float    g_vd, g_vq;
static float    g_v_alpha, g_v_beta;

// Duty cycles
static float    g_duty_a, g_duty_b, g_duty_c;

// PI controllers
static pi_controller_t g_pi_id = { .kp = 0.5f, .ki = 100.0f, .integral = 0.0f, .limit = 6.0f };
static pi_controller_t g_pi_iq = { .kp = 0.5f, .ki = 100.0f, .integral = 0.0f, .limit = 6.0f };

// ── Apply PWM ──────────────────────────────────────────────────────────────

static void apply_openloop(float step, float amplitude) {
    int idx = (int)step;
    if (idx >= SINE_TABLE_SIZE) idx -= SINE_TABLE_SIZE;
    if (idx < 0) idx += SINE_TABLE_SIZE;

    g_duty_a = (1.0f + g_sine[idx][0]) * amplitude;
    g_duty_b = (1.0f + g_sine[idx][1]) * amplitude;
    g_duty_c = (1.0f + g_sine[idx][2]) * amplitude;

    platform_pwm_set_duty(PWM_PHASE_U, (uint32_t)g_duty_a);
    platform_pwm_set_duty(PWM_PHASE_V, (uint32_t)g_duty_b);
    platform_pwm_set_duty(PWM_PHASE_W, (uint32_t)g_duty_c);
}

static void apply_foc(float v_alpha, float v_beta) {
    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.86602540f * v_beta;
    float vc = -0.5f * v_alpha - 0.86602540f * v_beta;

    // SVPWM centering
    float vmin = va;
    float vmax = va;
    if (vb < vmin) vmin = vb;
    if (vb > vmax) vmax = vb;
    if (vc < vmin) vmin = vc;
    if (vc > vmax) vmax = vc;
    float voff = -(vmin + vmax) * 0.5f;
    va += voff;  vb += voff;  vc += voff;

    float scale = (float)PWM_PERIOD / g_vbus;
    float half = (float)PWM_PERIOD * 0.5f;
    g_duty_a = half + va * scale;
    g_duty_b = half + vb * scale;
    g_duty_c = half + vc * scale;

    float max_d = (float)PWM_PERIOD * 0.95f;
    float min_d = (float)PWM_PERIOD * 0.05f;
    g_duty_a = LIMIT(g_duty_a, min_d, max_d);
    g_duty_b = LIMIT(g_duty_b, min_d, max_d);
    g_duty_c = LIMIT(g_duty_c, min_d, max_d);

    platform_pwm_set_duty(PWM_PHASE_U, (uint32_t)g_duty_a);
    platform_pwm_set_duty(PWM_PHASE_V, (uint32_t)g_duty_b);
    platform_pwm_set_duty(PWM_PHASE_W, (uint32_t)g_duty_c);
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
    g_enc_valid = enc.valid;
}

// ── Throttle callback ──────────────────────────────────────────────────────

static void on_motor_throttle(uint8_t *data, size_t size) {
    if (size < sizeof(motor_throttle_t)) return;
    motor_throttle_t cmd;
    memcpy(&cmd, data, sizeof(cmd));
    g_throttle = LIMIT(cmd.throttle, 0.0f, 1.0f);

    if (g_throttle < 0.01f && g_state != STATE_IDLE && g_state != STATE_CALIBRATE) {
        platform_pwm_stop();
        g_state = STATE_IDLE;
        g_ol_angle = 0.0f;
        g_ol_speed = 0.0f;
        g_ol_speed_smooth = 0.0f;
        g_amplitude = 0.0f;
        pi_controller_reset(&g_pi_id);
        pi_controller_reset(&g_pi_iq);
    }
    else if (g_throttle > 0.01f && g_state == STATE_IDLE) {
        g_state = STATE_ALIGN;
        g_align_cnt = 0;
        g_ramp_cnt = 0;
        g_ol_angle = 0.0f;
        g_ol_speed = RAMP_SPEED_INIT;
        g_ol_speed_smooth = OL_SPEED;
        g_amplitude = RAMP_AMP_INIT;
        pi_controller_reset(&g_pi_id);
        pi_controller_reset(&g_pi_iq);
        platform_pwm_start();
    }
}

// ── Log class callback (also triggers calibration) ─────────────────────────

static void on_notify_log_class(uint8_t *data, size_t size) {
    if (size < 1) return;
    uint8_t requested = data[0];

    // LOG_CLASS_FOC (1) while IDLE = start calibration
    if (requested == LOG_CLASS_FOC && g_state == STATE_IDLE) {
        g_log_class = requested;
        g_state = STATE_CALIBRATE;
        g_cal_step = 0;
        g_cal_cnt = 0;
        g_cal_table_idx[0] = 0.0f;
        g_cal_table_idx[1] = (float)SINE_TABLE_SIZE * 0.25f;
        g_cal_table_idx[2] = (float)SINE_TABLE_SIZE * 0.50f;
        g_cal_table_idx[3] = (float)SINE_TABLE_SIZE * 0.75f;
        for (int i = 0; i < CAL_STEPS; i++) {
            g_cal_enc[i] = 0.0f;
            g_cal_sin_sum[i] = 0.0f;
            g_cal_cos_sum[i] = 0.0f;
        }
        g_cal_avg_cnt = 0;
        platform_pwm_start();
        return;
    }

    g_log_class = requested;
}

// ── 40 kHz commutation (triggered by SENSOR_PHASE_CURRENT) ─────────────────

static void on_commutate(uint8_t *data, size_t size) {
    if (size < sizeof(phase_current_t)) return;
    if (g_state == STATE_IDLE) return;

    // Read fresh phase currents from callback data
    phase_current_t c;
    memcpy(&c, data, sizeof(c));
    g_iu = c.phase_u;
    g_iv = c.phase_v;
    g_iw = c.phase_w;

    // Clarke transform
    clarke(g_iu, g_iv, g_iw, &g_i_alpha, &g_i_beta);

    // Calibrated rotor angle
    // Open-loop: use known voltage angle (encoder too noisy with 22 PP)
    // Closed-loop: use encoder
    float theta = 0.0f;
    if (g_state == STATE_OPENLOOP || g_state == STATE_RAMP) {
        theta = (float)(M_PI * 0.5) - g_ol_angle * (TWO_PI / (float)SINE_TABLE_SIZE);
        if (theta < 0.0f)    theta += TWO_PI;
        if (theta >= TWO_PI) theta -= TWO_PI;
    } else {
        theta = g_enc_elec - g_enc_offset;
        if (theta < 0.0f)    theta += TWO_PI;
        if (theta >= TWO_PI) theta -= TWO_PI;
    }

    g_theta = theta;

    // Always compute Park
    park(g_i_alpha, g_i_beta, theta, &g_id, &g_iq);

    switch (g_state) {

    case STATE_CALIBRATE:
        apply_openloop(g_cal_table_idx[g_cal_step], ALIGN_AMPLITUDE);
        g_cal_cnt++;
        // Average encoder angle over the last 25% using circular mean
        if (g_cal_cnt >= CAL_AVG_START) {
            g_cal_sin_sum[g_cal_step] += sinf(g_enc_elec);
            g_cal_cos_sum[g_cal_step] += cosf(g_enc_elec);
            g_cal_avg_cnt++;
        }
        if (g_cal_cnt >= CAL_TICKS) {
            g_cal_enc[g_cal_step] = atan2f(g_cal_sin_sum[g_cal_step],
                                           g_cal_cos_sum[g_cal_step]);
            if (g_cal_enc[g_cal_step] < 0.0f)
                g_cal_enc[g_cal_step] += TWO_PI;
            g_cal_step++;
            g_cal_cnt = 0;
            g_cal_avg_cnt = 0;
            if (g_cal_step >= CAL_STEPS) {
                // Same π/2 correction as ALIGN — sine table index 0
                // drives the voltage vector to 90° in α-β space.
                g_enc_offset = g_cal_enc[0] - (float)(M_PI * 0.5);
                if (g_enc_offset < 0.0f) g_enc_offset += TWO_PI;
                platform_pwm_stop();
                g_state = STATE_IDLE;

                // Save calibrated offset to flash
                param_storage_t p = { .id = PARAM_ID_ENC_OFFSET, .value = g_enc_offset };
                publish(LOCAL_STORAGE_SAVE, (uint8_t *)&p, sizeof(p));
            }
        }
        break;

    case STATE_ALIGN:
        apply_openloop(0.0f, ALIGN_AMPLITUDE);
        if (++g_align_cnt >= ALIGN_TICKS) {
            // Capture raw offset — correction angle determined empirically
            g_enc_offset = g_enc_elec;
            g_state = STATE_RAMP;
            g_ramp_cnt = 0;
        }
        break;

    case STATE_RAMP:
        g_ol_angle += g_ol_speed;
        if (g_ol_angle >= (float)SINE_TABLE_SIZE)
            g_ol_angle -= (float)SINE_TABLE_SIZE;

        g_ol_speed  += RAMP_SPEED_INC;
        g_amplitude += RAMP_AMP_INC;
        if (g_ol_speed > OL_SPEED)      g_ol_speed  = OL_SPEED;
        if (g_amplitude > OL_AMPLITUDE)  g_amplitude = OL_AMPLITUDE;

        apply_openloop(g_ol_angle, g_amplitude);

        if (++g_ramp_cnt > RAMP_MIN_TICKS && g_ol_speed >= OL_SPEED) {
            if (g_log_class == LOG_CLASS_ENCODER) {
                g_state = STATE_CLOSEDLOOP;
            } else {
                g_state = STATE_OPENLOOP;
            }
        }
        break;

    case STATE_OPENLOOP: {
        float speed_target = OL_SPEED + g_throttle * 0.15f;
        g_ol_speed_smooth += (speed_target - g_ol_speed_smooth) * 0.0002f;  // ~1s ramp at 40kHz
        g_amplitude = OL_AMPLITUDE;
        g_ol_angle += g_ol_speed_smooth;
        if (g_ol_angle >= (float)SINE_TABLE_SIZE)
            g_ol_angle -= (float)SINE_TABLE_SIZE;
        apply_openloop(g_ol_angle, g_amplitude);
        break;
    }

    case STATE_CLOSEDLOOP: {
        float iq_ref = g_throttle * 5.0f;
        float id_ref = 0.0f;

        g_vd = pi_controller_update(&g_pi_id, id_ref - g_id, DT);
        g_vq = pi_controller_update(&g_pi_iq, iq_ref - g_iq, DT);

        inv_park(g_vd, g_vq, theta, &g_v_alpha, &g_v_beta);
        apply_foc(g_v_alpha, g_v_beta);
        break;
    }

    default:
        break;
    }
}

// ── Logging (25 Hz) ────────────────────────────────────────────────────────

static void on_scheduler_25hz(uint8_t *data, size_t size) {
    if (g_log_class == 0) return;

    float log_data[8];

    switch (g_log_class) {
    case LOG_CLASS_FOC:
        // CP2: Calibration data
        // [0] state  [1] cal_step  [2] applied_angle(rad)  [3] enc_now(rad)
        // [4] enc@0  [5] enc@90    [6] enc@180             [7] enc@270
        log_data[0] = (float)g_state;
        log_data[1] = (float)g_cal_step;
        log_data[2] = (g_cal_step < CAL_STEPS)
                       ? g_cal_table_idx[g_cal_step] * (TWO_PI / (float)SINE_TABLE_SIZE)
                       : 0.0f;
        log_data[3] = g_enc_elec;
        log_data[4] = g_cal_enc[0];
        log_data[5] = g_cal_enc[1];
        log_data[6] = g_cal_enc[2];
        log_data[7] = g_cal_enc[3];
        break;

    case LOG_CLASS_CURRENT:
        // CP3: Clarke currents + phase currents
        // [0] state  [1] Ia  [2] Ib  [3] enc_elec
        // [4] Iu  [5] Iv  [6] Iw  [7] VBUS
        log_data[0] = (float)g_state;
        log_data[1] = g_i_alpha;
        log_data[2] = g_i_beta;
        log_data[3] = g_enc_elec;
        log_data[4] = g_iu;
        log_data[5] = g_iv;
        log_data[6] = g_iw;
        log_data[7] = g_vbus;
        break;

    case LOG_CLASS_OPENLOOP:
        // CP1: Open-loop + encoder verification
        // [0] state  [1] enc_elec  [2] ol_angle(rad)  [3] ol_speed_smooth
        // [4] amplitude  [5] enc_offset  [6] VBUS  [7] throttle
        log_data[0] = (float)g_state;
        log_data[1] = g_enc_elec;
        log_data[2] = g_ol_angle * (TWO_PI / (float)SINE_TABLE_SIZE);
        log_data[3] = g_ol_speed_smooth * (TWO_PI / (float)SINE_TABLE_SIZE) * 40000.0f;
        log_data[4] = g_amplitude;
        log_data[5] = g_enc_offset;
        log_data[6] = g_vbus;
        log_data[7] = g_throttle;
        break;

    case LOG_CLASS_VOLTAGE:
        // CP4: Park currents
        // [0] state  [1] Id  [2] Iq  [3] theta (angle used in Park)
        // [4] enc_elec  [5] enc_offset  [6] Ia  [7] Ib
        log_data[0] = (float)g_state;
        log_data[1] = g_id;
        log_data[2] = g_iq;
        log_data[3] = g_theta;  // actual angle used in Park (OL or encoder)
        log_data[4] = g_enc_elec;
        log_data[5] = g_enc_offset;
        log_data[6] = g_i_alpha;
        log_data[7] = g_i_beta;
        break;

    case LOG_CLASS_ENCODER:
        // CP5: Current loop
        // [0] state  [1] Id  [2] Iq  [3] Vd
        // [4] Vq  [5] Iq_ref  [6] Id_ref  [7] theta
        log_data[0] = (float)g_state;
        log_data[1] = g_id;
        log_data[2] = g_iq;
        log_data[3] = g_vd;
        log_data[4] = g_vq;
        log_data[5] = g_throttle * 5.0f;
        log_data[6] = 0.0f;
        {
            float th = g_enc_elec - g_enc_offset;
            if (th < 0.0f) th += TWO_PI;
            if (th >= TWO_PI) th -= TWO_PI;
            log_data[7] = th;
        }
        break;

    default:
        return;
    }

    publish(SEND_LOG, (uint8_t *)log_data, sizeof(log_data));
}

// ── Storage result callback ────────────────────────────────────────────────

static void on_storage_result(uint8_t *data, size_t size) {
    if (size < sizeof(param_storage_t)) return;
    param_storage_t p;
    memcpy(&p, data, sizeof(p));
    if (p.id == PARAM_ID_ENC_OFFSET && p.value != 0.0f) {
        g_enc_offset = p.value;
    }
}

// ── PWM enable/disable ─────────────────────────────────────────────────────

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
    g_ol_speed_smooth = 0.0f;
    g_amplitude = 0.0f;
    g_log_class = 0;
    pi_controller_reset(&g_pi_id);
    pi_controller_reset(&g_pi_iq);
}

// ── Setup ──────────────────────────────────────────────────────────────────

void foc_setup(void) {
    create_sine_table();

    subscribe(FOC_SETUP, on_foc_setup);
    subscribe(FOC_RELEASE, on_foc_release);
    subscribe(MOTOR_THROTTLE, on_motor_throttle);
    subscribe(SENSOR_PHASE_CURRENT, on_commutate);
    subscribe(SENSOR_BUS_VOLTAGE, on_bus_voltage);
    subscribe(SENSOR_ENCODER, on_encoder);
    subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
    subscribe(SCHEDULER_25HZ, on_scheduler_25hz);

    // Load saved encoder offset from flash
    subscribe(LOCAL_STORAGE_RESULT, on_storage_result);
    param_id_e id = PARAM_ID_ENC_OFFSET;
    publish(LOCAL_STORAGE_LOAD, (uint8_t *)&id, sizeof(id));
}
