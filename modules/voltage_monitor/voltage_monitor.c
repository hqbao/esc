// Bus voltage and temperature monitoring module
//
// ADC1 regular channels:
//   CH1 (PA0) = Bus voltage via resistor divider (169k/18k)
//   CH5 (PB14) = NTC temperature sensor
//
// Regular conversion is software-triggered separately from the injected
// (motor current) conversions.

#include "voltage_monitor.h"
#include <pubsub.h>
#include <platform.h>
#include <messages.h>
#include <macro.h>
#include <string.h>

// ── State ──────────────────────────────────────────────────────────────────
static bus_measurement_t g_bus;
static uint8_t g_active_log_class = 0;

// ── Callbacks ──────────────────────────────────────────────────────────────

// Called from ADC regular complete — data contains VBUS raw value (uint16_t, 12-bit)
static void on_adc_regular(uint8_t *data, size_t size) {
    if (size < sizeof(uint16_t)) return;

    uint16_t vbus_raw;
    memcpy(&vbus_raw, data, sizeof(vbus_raw));

    g_bus.bus_voltage = ADC_TO_VBUS(vbus_raw);
    g_bus.temperature = 0;

    publish(SENSOR_BUS_VOLTAGE, (uint8_t *)&g_bus, sizeof(g_bus));
}

static void on_notify_log_class(uint8_t *data, size_t size) {
    if (size < 1) return;
    g_active_log_class = (data[0] == LOG_CLASS_VOLTAGE) ? LOG_CLASS_VOLTAGE : 0;
}

static void on_scheduler_25hz(uint8_t *data, size_t size) {
    if (g_active_log_class == 0) return;
    float log_data[2] = { g_bus.bus_voltage, g_bus.temperature };
    publish(SEND_LOG, (uint8_t *)log_data, sizeof(log_data));
}

// ── Setup ──────────────────────────────────────────────────────────────────
void voltage_monitor_setup(void) {
    subscribe(ADC_REGULAR_COMPLETE, on_adc_regular);
    subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
    subscribe(SCHEDULER_25HZ, on_scheduler_25hz);
}
