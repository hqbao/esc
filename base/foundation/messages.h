#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>

// Phase current measurement (from ADC injected callback)
typedef struct {
    float phase_u;   // Amps
    float phase_v;   // Amps
    float phase_w;   // Amps
} phase_current_t;

// Bus voltage and temperature (from ADC regular)
typedef struct {
    float bus_voltage;    // Volts
    float temperature;    // Celsius
} bus_measurement_t;

// Encoder data (from AS5048A)
typedef struct {
    float electrical_angle;  // radians, 0–2π
    float mechanical_angle;  // degrees, multi-turn
    uint16_t raw;            // 14-bit raw count
    uint8_t valid;           // 1 when readings are good
} encoder_data_t;

// Motor throttle command
typedef struct {
    float throttle;   // 0.0 – 1.0
} motor_throttle_t;

// FOC commutation state
typedef struct {
    uint16_t electrical_angle;  // 0 – 65535 maps to 0 – 2π
    float duty_a;
    float duty_b;
    float duty_c;
} foc_state_t;

// Log classes
#define LOG_CLASS_FOC          1
#define LOG_CLASS_CURRENT      2
#define LOG_CLASS_VOLTAGE      3
#define LOG_CLASS_RAW_CURRENT  4
#define LOG_CLASS_ENCODER      5
#define LOG_CLASS_OPENLOOP     6

#endif // MESSAGES_H
