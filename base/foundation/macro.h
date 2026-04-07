#ifndef MACRO_H
#define MACRO_H

#define LIMIT(val, lo, hi) ((val) < (lo) ? (lo) : ((val) > (hi) ? (hi) : (val)))

// PWM frequency: 170MHz / (2 × 2125) = 40 kHz center-aligned
#define PWM_FREQ           40000
#define PWM_PERIOD         2125

// Motor parameters (B-G431B-ESC1 reference design)
#define NUM_POLE_PAIRS     7       // Adjust per motor

// Current sensing
#define SHUNT_RESISTANCE   0.003f  // Ohms
#define OPAMP_GAIN         16.0f   // PGA gain
#define ADC_VREF           3.3f
#define ADC_RESOLUTION     4096
#define ADC_TO_CURRENT(raw) \
    (((float)(raw) / ADC_RESOLUTION * ADC_VREF) / (SHUNT_RESISTANCE * OPAMP_GAIN))

// Bus voltage divider (R1=169k, R2=18k → ratio = 18/187 = 0.09626)
#define VBUS_DIVIDER_RATIO  0.09626f
#define ADC_TO_VBUS(raw)    ((float)(raw) / ADC_RESOLUTION * ADC_VREF / VBUS_DIVIDER_RATIO)

// Motor electrical parameters (measure or from datasheet — TUNE PER MOTOR)
#define MOTOR_RESISTANCE   0.5f    // phase resistance (Ohms)
#define MOTOR_INDUCTANCE   0.001f  // phase inductance (Henries = 1mH)

// Sine table
#define SINE_TABLE_SIZE    256

// Math
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define TWO_PI             (2.0f * M_PI)

#endif // MACRO_H
