#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

typedef struct {
    float kp;
    float ki;
    float integral;
    float limit;
} pi_controller_t;

static inline float pi_controller_update(pi_controller_t *pi, float error, float dt) {
    pi->integral += pi->ki * error * dt;
    if (pi->integral >  pi->limit) pi->integral =  pi->limit;
    if (pi->integral < -pi->limit) pi->integral = -pi->limit;
    float out = pi->kp * error + pi->integral;
    if (out >  pi->limit) out =  pi->limit;
    if (out < -pi->limit) out = -pi->limit;
    return out;
}

static inline void pi_controller_reset(pi_controller_t *pi) {
    pi->integral = 0.0f;
}

#endif // PI_CONTROLLER_H
