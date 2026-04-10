// FOC Encoder — standalone closed-loop with AS5048A encoder
//
// Startup: IDLE → ALIGN → RAMP → auto-detect enc_dir/offset → CLOSEDLOOP
// Encoder provides rotor angle for Park/InvPark transform.
// Voltage-mode CL with Vq ramp.
//
// Build: bash build.sh encoder
//
// Log classes:
//   ENCODER  (5) — CL status: Id, Iq, Vq, enc_offset, enc_dir
//   FOC      (1) — Encoder diagnostics: enc_elec, enc_mech, theta, offset

#include "foc.h"
#ifdef FOC_MODE_ENCODER
#include "foc_math.h"
#include <pubsub.h>
#include <platform.h>
#include <messages.h>
#include <macro.h>
#include <pi_controller.h>
#include <string.h>
#include <math.h>

// ── Log classes ────────────────────────────────────────────────────────────

#define LOG_CLASS_FOC      1
#define LOG_CLASS_CURRENT  2
#define LOG_CLASS_ENCODER  5

// ── States ─────────────────────────────────────────────────────────────────

typedef enum {
    STATE_IDLE,         // 0
    STATE_ALIGN,        // 1
    STATE_RAMP,         // 2
    STATE_OPENLOOP,     // 3
    STATE_CLOSEDLOOP,   // 4
} state_e;

// ── Constants ──────────────────────────────────────────────────────────────

#define MAX_DUTY        ((float)PWM_PERIOD * 0.45f)

#define OL_SPEED        0.05f
#define OL_AMPLITUDE    (0.35f * MAX_DUTY)

#define ALIGN_TICKS     40000
#define ALIGN_AMPLITUDE (0.35f * MAX_DUTY)
#define RAMP_MIN_TICKS  20000
#define RAMP_SPEED_INIT 0.005f
#define RAMP_SPEED_INC  0.000002f
#define RAMP_AMP_INIT   (0.35f * MAX_DUTY)
#define OL_CRUISE_TICKS 40000   // 1s cruise at OL_SPEED before CL entry
#define VQ_MIN          0.8f    // minimum Vq to overcome 22PP cogging

// ── State ──────────────────────────────────────────────────────────────────

static state_e  g_state;
static float    g_throttle;
static uint8_t  g_log_class;

// Open-loop (used during ALIGN/RAMP)
static float    g_ol_angle;
static float    g_ol_speed;
static float    g_amplitude;
static uint32_t g_align_cnt;
static uint32_t g_ramp_cnt;
static uint32_t g_ol_cruise_cnt;

// Encoder
static float    g_enc_elec;
static float    g_enc_mech_rad;
static float    g_enc_offset;
static float    g_enc_dir = 1.0f;
static float    g_enc_elec_align;
static float    g_enc_mech_align;
static float    g_theta_align;

// Encoder interpolation (1kHz → 40kHz)
static float    g_enc_elec_vel;     // rad/tick at 40kHz
static uint32_t g_enc_interp_cnt;   // ticks since last encoder update

// CL voltage
static float    g_cl_vq;

// Measurements
static float    g_iu, g_iv, g_iw;
static float    g_vbus = 12.0f;

// Clarke / Park
static float    g_i_alpha, g_i_beta;
static float    g_id, g_iq;
static float    g_theta;

// FOC outputs
static float    g_vd, g_vq;
static float    g_v_alpha, g_v_beta;

// PI controllers — Id=0 (no flux weakening), Iq tracks current reference
static pi_controller_t g_pi_id = { .kp = 0.3f, .ki = 10.0f, .integral = 0.0f, .limit = 2.0f };
static pi_controller_t g_pi_iq = { .kp = 0.3f, .ki = 10.0f, .integral = 0.0f, .limit = 2.0f };

// Duty cycles
static float    g_duty_a, g_duty_b, g_duty_c;

// ── Sine table ─────────────────────────────────────────────────────────────

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

// ── PWM helpers ────────────────────────────────────────────────────────────

static void apply_svpwm(float va, float vb, float vc);

static inline float ol_theta_from_angle(float ol_angle) {
    float theta = ol_angle * (TWO_PI / (float)SINE_TABLE_SIZE);
    while (theta >= TWO_PI) theta -= TWO_PI;
    while (theta < 0.0f)    theta += TWO_PI;
    return theta;
}

static void apply_openloop(float step, float amplitude) {
    float theta = ol_theta_from_angle(step);
    float v_amp = amplitude * g_vbus / (float)PWM_PERIOD;
    
    // In OPENLOOP, apply Vd = amplitude, Vq = 0.0f to align rotor D-axis exactly to theta
    inv_park(v_amp, 0.0f, theta, &g_v_alpha, &g_v_beta);
    float va, vb, vc;
    inv_clarke(g_v_alpha, g_v_beta, &va, &vb, &vc);
    apply_svpwm(va, vb, vc);
}

static void apply_svpwm(float va, float vb, float vc) {
    float vmin = va, vmax = va;
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

    platform_pwm_send(PWM_PORT1, (uint32_t)g_duty_a);
    platform_pwm_send(PWM_PORT2, (uint32_t)g_duty_b);
    platform_pwm_send(PWM_PORT3, (uint32_t)g_duty_c);
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

    // Handle encoder SPI errors safely (0.00 fallback implies crash)
    if (!enc.valid && enc.electrical_angle == 0.0f) return;

    float new_mech = enc.mechanical_angle * (TWO_PI / 360.0f); // rad
    float new_elec = enc.electrical_angle;

    // Compute velocity on MECHANICAL angle to prevent pi-aliasing at high RPM
    if (g_enc_interp_cnt > 0) {
        float delta = new_mech - g_enc_mech_rad;
        if (delta > M_PI) delta -= TWO_PI;
        if (delta < -M_PI) delta += TWO_PI;
        
        // Mechanical velocity -> electrical velocity
        float elec_vel = (delta * (float)NUM_POLE_PAIRS) / (float)g_enc_interp_cnt;
        g_enc_elec_vel += 0.05f * (elec_vel - g_enc_elec_vel);
    }

    g_enc_mech_rad = new_mech;
    g_enc_elec = new_elec;
    g_enc_interp_cnt = 0;
}

// ── Throttle ───────────────────────────────────────────────────────────────

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
        g_cl_vq = 0.0f;
        pi_controller_reset(&g_pi_id);
        pi_controller_reset(&g_pi_iq);
    }
    else if (g_throttle > 0.01f && g_state == STATE_IDLE) {
        g_state = STATE_ALIGN;
        g_align_cnt = 0;
        g_ramp_cnt = 0;
        g_ol_angle = 0.0f;
        g_ol_speed = RAMP_SPEED_INIT;
        g_amplitude = RAMP_AMP_INIT;
        g_enc_dir = 1.0f;
        pi_controller_reset(&g_pi_id);
        pi_controller_reset(&g_pi_iq);
        platform_pwm_start();
    }
}

static void on_notify_log_class(uint8_t *data, size_t size) {
    if (size < 1) return;
    g_log_class = data[0];
}

// ── RAMP → CL transition ──────────────────────────────────────────────────

static void on_ramp_done(float theta) {
    // After OPENLOOP cruise, encoder velocity is well-established.
    // Primary: velocity sign (reliable after 1s cruise).
    // Fallback: mechanical displacement (immune to electrical aliasing).
    if (fabsf(g_enc_elec_vel) > 0.0001f) {
        g_enc_dir = (g_enc_elec_vel >= 0.0f) ? 1.0f : -1.0f;
    } else {
        float mech_delta = g_enc_mech_rad - g_enc_mech_align;
        if (mech_delta >  (float)M_PI) mech_delta -= TWO_PI;
        if (mech_delta < -(float)M_PI) mech_delta += TWO_PI;
        g_enc_dir = (mech_delta >= 0.0f) ? 1.0f : -1.0f;
    }

    // Offset: Use ALIGN values where load angle ≈ 0 (rotor fully settled).
    // This maps enc_elec to the true rotor D-axis, so Vq produces real torque.
    g_enc_offset = g_theta_align - g_enc_dir * g_enc_elec_align;
    g_enc_offset = fmodf(g_enc_offset, TWO_PI);
    if (g_enc_offset < 0.0f) g_enc_offset += TWO_PI;

    // One-shot diagnostic (state=99 marker)
    {
        float diag[8];
        diag[0] = 99.0f;
        diag[1] = g_enc_elec_vel * 40000.0f; // encoder velocity (rad/s)
        diag[2] = g_ol_speed * (TWO_PI / (float)SINE_TABLE_SIZE) * 40000.0f; // OL rad/s
        diag[3] = g_enc_dir;
        diag[4] = g_enc_offset;
        diag[5] = g_enc_elec;
        diag[6] = g_enc_mech_rad;
        diag[7] = theta;
        publish(SEND_LOG, (uint8_t *)diag, sizeof(diag));
    }

    pi_controller_reset(&g_pi_id);
    pi_controller_reset(&g_pi_iq);
    g_cl_vq = VQ_MIN;

    g_state = STATE_CLOSEDLOOP;
}

// ── CL commutation ────────────────────────────────────────────────────────

static void cl_commutate(float theta) {
    float v_max = g_vbus * 0.577f;
    float vq_target = 0.5f + g_throttle * (v_max - 0.5f);
    if (vq_target < VQ_MIN) vq_target = VQ_MIN;
    if (vq_target > v_max) vq_target = v_max;

    float ramp_rate = 0.0001f;
    if (g_cl_vq < vq_target) {
        g_cl_vq += ramp_rate;
        if (g_cl_vq > vq_target) g_cl_vq = vq_target;
    } else if (g_cl_vq > vq_target) {
        g_cl_vq -= ramp_rate;
        if (g_cl_vq < vq_target) g_cl_vq = vq_target;
    }

    g_vd = 0.0f;
    g_vq = g_cl_vq;

    if (g_vq < -v_max) g_vq = -v_max;
    if (g_vq >  v_max) g_vq =  v_max;

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

    // Compute theta
    g_enc_interp_cnt++;
    float theta;
    if (g_state <= STATE_OPENLOOP) {
        theta = ol_theta_from_angle(g_ol_angle);
    } else {
        // Interpolate encoder between 1kHz updates
        float enc_interp = g_enc_elec + g_enc_elec_vel * (float)g_enc_interp_cnt;
        theta = g_enc_dir * enc_interp + g_enc_offset;
        theta = fmodf(theta, TWO_PI);
        if (theta < 0.0f) theta += TWO_PI;
    }

    g_theta = theta;
    park(g_i_alpha, g_i_beta, theta, &g_id, &g_iq);

    switch (g_state) {

    case STATE_ALIGN:
        apply_openloop(0.0f, ALIGN_AMPLITUDE);
        if (++g_align_cnt >= ALIGN_TICKS) {
            g_enc_elec_align = g_enc_elec;
            g_enc_mech_align = g_enc_mech_rad;
            g_theta_align = theta;
            g_state = STATE_RAMP;
            g_ramp_cnt = 0;
        }
        break;

    case STATE_RAMP:
        g_ol_angle += g_ol_speed;
        if (g_ol_angle >= (float)SINE_TABLE_SIZE)
            g_ol_angle -= (float)SINE_TABLE_SIZE;

        g_ol_speed += RAMP_SPEED_INC;
        if (g_ol_speed > OL_SPEED) g_ol_speed = OL_SPEED;

        apply_openloop(g_ol_angle, g_amplitude);

        if (++g_ramp_cnt > RAMP_MIN_TICKS && g_ol_speed >= OL_SPEED) {
            g_state = STATE_OPENLOOP;
            g_ol_cruise_cnt = 0;
        }
        break;

    case STATE_OPENLOOP:
        g_ol_angle += OL_SPEED;
        if (g_ol_angle >= (float)SINE_TABLE_SIZE)
            g_ol_angle -= (float)SINE_TABLE_SIZE;
        apply_openloop(g_ol_angle, g_amplitude);
        if (++g_ol_cruise_cnt >= OL_CRUISE_TICKS) {
            on_ramp_done(theta);
        }
        break;

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
    case LOG_CLASS_ENCODER:
        // [0]state [1]Id [2]Iq [3]enc_elec [4]Vq [5]cl_vq [6]enc_dir [7]theta
        log_data[0] = (float)g_state;
        log_data[1] = g_id;
        log_data[2] = g_iq;
        log_data[3] = g_enc_elec;
        log_data[4] = g_vq;
        log_data[5] = g_cl_vq;
        log_data[6] = g_enc_dir;
        log_data[7] = g_theta;
        ok = 1;
        break;

    case LOG_CLASS_FOC:
        // [0]state [1]enc_elec [2]enc_mech [3]theta [4]enc_offset [5]Vbus [6]enc_dir [7]0
        log_data[0] = (float)g_state;
        log_data[1] = g_enc_elec;
        log_data[2] = g_enc_mech_rad;
        log_data[3] = g_theta;
        log_data[4] = g_enc_offset;
        log_data[5] = g_vbus;
        log_data[6] = g_enc_dir;
        log_data[7] = 0.0f;
        ok = 1;
        break;

    case LOG_CLASS_CURRENT:
        // [0]state [1]duty_a [2]duty_b [3]duty_c [4]v_alpha [5]v_beta [6]Vbus [7]cl_vq
        log_data[0] = (float)g_state;
        log_data[1] = g_duty_a;
        log_data[2] = g_duty_b;
        log_data[3] = g_duty_c;
        log_data[4] = g_v_alpha;
        log_data[5] = g_v_beta;
        log_data[6] = g_vbus;
        log_data[7] = g_cl_vq;
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
    g_state = STATE_IDLE;
    g_throttle = 0.0f;
    g_ol_angle = 0.0f;
    g_ol_speed = 0.0f;
    g_amplitude = 0.0f;
    g_cl_vq = 0.0f;
    g_log_class = 0;
    pi_controller_reset(&g_pi_id);
    pi_controller_reset(&g_pi_iq);
}

// ── Setup ──────────────────────────────────────────────────────────────────

void foc_encoder_setup(void) {
    create_sine_table();

    subscribe(FOC_SETUP, on_foc_setup);
    subscribe(FOC_RELEASE, on_foc_release);
    subscribe(MOTOR_THROTTLE, on_motor_throttle);
    subscribe(SENSOR_PHASE_CURRENT, on_commutate);
    subscribe(SENSOR_BUS_VOLTAGE, on_bus_voltage);
    subscribe(SENSOR_ENCODER, on_encoder);
    subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
    subscribe(SCHEDULER_25HZ, on_scheduler_25hz);
}
#endif // FOC_MODE_ENCODER
