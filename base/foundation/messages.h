#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>

// Bus voltage and temperature (from ADC regular)
typedef struct {
    float bus_voltage;    // Volts
    float temperature;    // Celsius
} bus_measurement_t;

// Motor throttle command
typedef struct {
    float throttle;   // 0.0 – 1.0
} motor_throttle_t;

// Log classes
#define LOG_CLASS_COMMUTATION  3
#define LOG_CLASS_VOLTAGE      4

#endif // MESSAGES_H
