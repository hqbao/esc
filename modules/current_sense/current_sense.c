// Phase current sensing module for B-G431B-ESC1
//
// ADC1/ADC2 injected channels are triggered by TIM1_CC4 (OC4REF) at the PWM
// center point for clean measurement (no switching noise).
//   ADC1 inj: CH3 (OPAMP1 = Phase U), CH12 (OPAMP3 = Phase W)
//   ADC2 inj: CH3 (OPAMP2 = Phase V), VOPAMP3 (internal = Phase W redundant)
//
// The ADC injected complete ISR publishes raw ADC values; this module converts
// to Amps and publishes SENSOR_PHASE_CURRENT for the FOC module.

#include "current_sense.h"
#include <pubsub.h>
#include <platform.h>
#include <messages.h>
#include <macro.h>
#include <string.h>

// ── State ──────────────────────────────────────────────────────────────────
static phase_current_t g_current;
static uint16_t g_offset_u = 2048;  // ADC zero-current offset (calibrated)
static uint16_t g_offset_v = 2048;
static uint16_t g_offset_w = 2048;
static uint16_t g_cal_count = 0;
static uint32_t g_cal_sum_u = 0;
static uint32_t g_cal_sum_v = 0;
static uint32_t g_cal_sum_w = 0;
static uint8_t  g_calibrated = 0;
static uint8_t  g_active_log_class = 0;

#define CAL_SAMPLES 1000

// ── Callbacks ──────────────────────────────────────────────────────────────

// Called from ADC injected ISR — data contains 3 × uint16_t raw ADC readings
static void on_adc_injected(uint8_t *data, size_t size) {
    if (size < 3 * sizeof(uint16_t)) return;

    uint16_t raw_u, raw_v, raw_w;
    memcpy(&raw_u, &data[0], sizeof(uint16_t));
    memcpy(&raw_v, &data[2], sizeof(uint16_t));
    memcpy(&raw_w, &data[4], sizeof(uint16_t));

    // Zero-current offset calibration at startup (motor must be stopped)
    if (!g_calibrated) {
        g_cal_sum_u += raw_u;
        g_cal_sum_v += raw_v;
        g_cal_sum_w += raw_w;
        g_cal_count++;
        if (g_cal_count >= CAL_SAMPLES) {
            g_offset_u = (uint16_t)(g_cal_sum_u / CAL_SAMPLES);
            g_offset_v = (uint16_t)(g_cal_sum_v / CAL_SAMPLES);
            g_offset_w = (uint16_t)(g_cal_sum_w / CAL_SAMPLES);
            g_calibrated = 1;
        }
        return;
    }

    // Convert to Amps: (ADC - offset) / 4096 * 3.3V / (0.003 * 16)
    g_current.phase_u = ((float)raw_u - (float)g_offset_u) / (float)ADC_RESOLUTION
                        * ADC_VREF / (SHUNT_RESISTANCE * OPAMP_GAIN);
    g_current.phase_v = ((float)raw_v - (float)g_offset_v) / (float)ADC_RESOLUTION
                        * ADC_VREF / (SHUNT_RESISTANCE * OPAMP_GAIN);
    g_current.phase_w = ((float)raw_w - (float)g_offset_w) / (float)ADC_RESOLUTION
                        * ADC_VREF / (SHUNT_RESISTANCE * OPAMP_GAIN);

    publish(SENSOR_PHASE_CURRENT, (uint8_t *)&g_current, sizeof(g_current));
}

static void on_notify_log_class(uint8_t *data, size_t size) {
    if (size < 1) return;
    g_active_log_class = (data[0] == LOG_CLASS_CURRENT) ? LOG_CLASS_CURRENT : 0;
}

static void on_scheduler_25hz(uint8_t *data, size_t size) {
    if (g_active_log_class == 0) return;
    float log_data[3] = { g_current.phase_u, g_current.phase_v, g_current.phase_w };
    publish(SEND_LOG, (uint8_t *)log_data, sizeof(log_data));
}

// ── Setup ──────────────────────────────────────────────────────────────────
void current_sense_setup(void) {
    subscribe(ADC_INJECTED_COMPLETE, on_adc_injected);
    subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
    subscribe(SCHEDULER_25HZ, on_scheduler_25hz);
}
