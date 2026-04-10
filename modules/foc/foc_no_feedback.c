// FOC No-Feedback — standalone open-loop voltage drive
//
// Startup: IDLE → ALIGN → RAMP → OPENLOOP
// No encoder feedback for commutation — purely open-loop.
//
// Guarded by FOC_MODE_NO_FEEDBACK — only one foc_*.c compiles per build.
// Build: bash build.sh no_feedback
//
// Log classes:
//   VOLTAGE  (3) — Park currents Id/Iq + angles + Clarke
//   CURRENT  (2) — Clarke + phase currents + Vbus
//   OPENLOOP (6) — OL angle/speed + encoder readback

#include "foc.h"
#ifdef FOC_MODE_NO_FEEDBACK
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
#define LOG_CLASS_OPENLOOP 6

// ── States ─────────────────────────────────────────────────────────────────

typedef enum {
    STATE_IDLE,         // 0
    STATE_ALIGN,        // 1
    STATE_RAMP,         // 2
    STATE_OPENLOOP,     // 3
} state_e;

// ── Constants ──────────────────────────────────────────────────────────────

#define MAX_DUTY        ((float)PWM_PERIOD * 0.45f)

#define OL_SPEED        0.005f
#define OL_AMPLITUDE    (0.35f * MAX_DUTY)

#define ALIGN_TICKS     20000
#define ALIGN_AMPLITUDE (0.35f * MAX_DUTY)
#define RAMP_MIN_TICKS  4000
#define RAMP_SPEED_INIT 0.002f
#define RAMP_SPEED_INC  0.0000003f
#define RAMP_AMP_INIT   (0.35f * MAX_DUTY)

// ── State ──────────────────────────────────────────────────────────────────

static state_e  g_state;
static float    g_throttle;
static uint8_t  g_log_class;

// Open-loop
static float    g_ol_angle;
static float    g_ol_speed;
static float    g_ol_speed_smooth;
static float    g_amplitude;
static uint32_t g_align_cnt;
static uint32_t g_ramp_cnt;

// Encoder (read-only, for logging)
static float    g_enc_elec;
static float    g_enc_mech;
static float    g_enc_offset;

// Measurements
static float    g_iu, g_iv, g_iw;
static float    g_vbus = 12.0f;

// Clarke / Park
static float    g_i_alpha, g_i_beta;
static float    g_id, g_iq;
static float    g_theta;

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

static void apply_openloop(float step, float amplitude) {
    int idx = (int)step;
    if (idx >= SINE_TABLE_SIZE) idx -= SINE_TABLE_SIZE;
    if (idx < 0) idx += SINE_TABLE_SIZE;

    g_duty_a = (1.0f + g_sine[idx][0]) * amplitude;
    g_duty_b = (1.0f + g_sine[idx][1]) * amplitude;
    g_duty_c = (1.0f + g_sine[idx][2]) * amplitude;

    platform_pwm_send(PWM_PORT1, (uint32_t)g_duty_a);
    platform_pwm_send(PWM_PORT2, (uint32_t)g_duty_b);
    platform_pwm_send(PWM_PORT3, (uint32_t)g_duty_c);
}

static inline float ol_theta_from_angle(float ol_angle) {
    float theta = (float)(M_PI * 0.5) - ol_angle * (TWO_PI / (float)SINE_TABLE_SIZE);
    if (theta < 0.0f)    theta += TWO_PI;
    if (theta >= TWO_PI) theta -= TWO_PI;
    return theta;
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
    g_enc_mech = enc.mechanical_angle;
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
        g_ol_speed_smooth = 0.0f;
        g_amplitude = 0.0f;
    }
    else if (g_throttle > 0.01f && g_state == STATE_IDLE) {
        g_state = STATE_ALIGN;
        g_align_cnt = 0;
        g_ramp_cnt = 0;
        g_ol_angle = 0.0f;
        g_ol_speed = RAMP_SPEED_INIT;
        g_ol_speed_smooth = OL_SPEED;
        g_amplitude = RAMP_AMP_INIT;
        platform_pwm_start();
    }
}

static void on_notify_log_class(uint8_t *data, size_t size) {
    if (size < 1) return;
    g_log_class = data[0];
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

    float theta = ol_theta_from_angle(g_ol_angle);
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
        g_ol_angle += g_ol_speed;
        if (g_ol_angle >= (float)SINE_TABLE_SIZE)
            g_ol_angle -= (float)SINE_TABLE_SIZE;

        g_ol_speed += RAMP_SPEED_INC;
        if (g_ol_speed > OL_SPEED) g_ol_speed = OL_SPEED;

        apply_openloop(g_ol_angle, g_amplitude);

        if (++g_ramp_cnt > RAMP_MIN_TICKS && g_ol_speed >= OL_SPEED) {
            g_state = STATE_OPENLOOP;
        }
        break;

    case STATE_OPENLOOP: {
        float speed_target = OL_SPEED + g_throttle * 0.15f;
        g_ol_speed_smooth += (speed_target - g_ol_speed_smooth) * 0.0002f;
        g_amplitude = OL_AMPLITUDE;
        g_ol_angle += g_ol_speed_smooth;
        if (g_ol_angle >= (float)SINE_TABLE_SIZE)
            g_ol_angle -= (float)SINE_TABLE_SIZE;
        apply_openloop(g_ol_angle, g_amplitude);
        break;
    }

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
        // [0]state [1]Id [2]Iq [3]theta [4]enc_elec [5]enc_offset [6]Ia [7]Ib
        log_data[0] = (float)g_state;
        log_data[1] = g_id;
        log_data[2] = g_iq;
        log_data[3] = g_theta;
        log_data[4] = g_enc_elec;
        log_data[5] = g_enc_offset;
        log_data[6] = g_i_alpha;
        log_data[7] = g_i_beta;
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

    case LOG_CLASS_OPENLOOP:
        // [0]state [1]enc_elec [2]ol_angle(rad) [3]ol_speed(rad/s)
        // [4]amplitude [5]enc_offset [6]Vbus [7]throttle
        log_data[0] = (float)g_state;
        log_data[1] = g_enc_elec;
        log_data[2] = g_ol_angle * (TWO_PI / (float)SINE_TABLE_SIZE);
        log_data[3] = g_ol_speed_smooth * (TWO_PI / (float)SINE_TABLE_SIZE) * 40000.0f;
        log_data[4] = g_amplitude;
        log_data[5] = g_enc_offset;
        log_data[6] = g_vbus;
        log_data[7] = g_throttle;
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
    g_ol_speed_smooth = 0.0f;
    g_amplitude = 0.0f;
    g_log_class = 0;
}

// ── Setup ──────────────────────────────────────────────────────────────────

void foc_no_feedback_setup(void) {
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
#endif // FOC_MODE_NO_FEEDBACK
