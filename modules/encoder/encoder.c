// AS5048A magnetic encoder — bit-bang SPI on J8 connector
//
// Wiring:
//   PB6  = SCK   (J8 pin 1)
//   PB7  = MOSI  (J8 pin 2)
//   PB8  = MISO  (J8 pin 3)
//   PA15 = CS    (PWM input pad, active low)
//
// AS5048A SPI protocol:
//   - CPOL=0, CPHA=1 (SPI mode 1): data clocked on falling edge
//   - 16-bit frames: [parity][read/write][6-bit addr][8-bit data] (command)
//   - Response to a command arrives in the NEXT transaction
//   - Read angle: send 0xFFFF (read NOP), then 0xFFFF again → angle in response
//   - Angle field: bits [13:0] = 14-bit angle (16384 counts/rev)
//   - Error flag: bit 14
//
// Reads at 1 kHz via SCHEDULER_1KHZ. Publishes SENSOR_ENCODER with
// the electrical angle (radians, 0–2π) for FOC commutation.

#include "encoder.h"
#include <pubsub.h>
#include <platform.h>
#include <messages.h>
#include <macro.h>
#include <string.h>
#include <math.h>

// AS5048A constants
#define AS5048A_CMD_ANGLE  0x3FFF   // Read angle register (0x3FFF with R bit)
#define AS5048A_CMD_NOP    0xC000   // NOP command (to clock out previous response)
#define AS5048A_COUNTS     16384    // 14-bit resolution

// State
static encoder_data_t g_encoder;
static float g_prev_mech_angle = 0.0f;
static float g_turns = 0.0f;
static int   g_warmup = 5;         // skip first N reads for sensor settle
static int   g_reject_count = 0;   // consecutive glitch rejections
static uint8_t g_active_log_class = 0;

// Parity: even parity over 15 lower bits, placed in bit 15
static uint16_t add_parity(uint16_t cmd) {
    uint16_t v = cmd & 0x7FFF;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    if (v & 1) cmd |= 0x8000;
    return cmd;
}

// Bit-bang SPI transfer (mode 1: CPOL=0, CPHA=1)
// Sends 16 bits MSB first, simultaneously reads 16 bits
static uint16_t spi_transfer16(uint16_t tx) {
    uint16_t rx = 0;

    platform_encoder_cs_low();

    for (int i = 15; i >= 0; i--) {
        // Set MOSI before rising edge
        if (tx & (1u << i))
            platform_encoder_mosi_high();
        else
            platform_encoder_mosi_low();

        // Rising edge — AS5048A latches MOSI
        platform_encoder_sck_high();

        // Small delay for setup time (170 MHz MCU, ~6 ns/cycle)
        __asm volatile("nop\nnop\nnop\nnop\n");

        // Read MISO before falling edge
        if (platform_encoder_miso_read())
            rx |= (1u << i);

        // Falling edge — AS5048A shifts out next bit
        platform_encoder_sck_low();

        __asm volatile("nop\nnop\nnop\nnop\n");
    }

    platform_encoder_cs_high();

    // Short delay between frames (AS5048A needs ~350 ns min CS high)
    __asm volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");

    return rx;
}

// Read 14-bit angle from AS5048A
// Two SPI transactions: first sends command, second clocks out response
static uint16_t read_angle_raw(void) {
    uint16_t cmd = add_parity(0x4000 | AS5048A_CMD_ANGLE); // R=1, addr=0x3FFF
    spi_transfer16(cmd);                                     // send command
    uint16_t resp = spi_transfer16(add_parity(AS5048A_CMD_NOP)); // read response

    // Bit 14 = error flag — reject corrupted reads
    if (resp & 0x4000) return g_encoder.raw;  // keep previous

    return resp & 0x3FFF;  // 14-bit angle
}

// Max mechanical angle change per 1kHz tick at reasonable speed.
// 5° mech/ms = 5000°/s = 13.9 rev/s = 833 RPM — well above open-loop range.
// After 10 consecutive rejections (10ms), accept the reading — the motor
// legitimately moved (e.g. ALIGN jerk) and prev must resync.
#define MECH_GLITCH_THRESHOLD 5.0f
#define MAX_REJECT 10

// 1 kHz encoder read
static void on_scheduler_1khz(uint8_t *data, size_t size) {
    uint16_t raw = read_angle_raw();

    // Mechanical angle 0–360° (NOT negated — required for correct Park/FOC convention)
    float mech_angle = 360.0f * (float)raw / (float)AS5048A_COUNTS;
    if (mech_angle >= 360.0f) mech_angle -= 360.0f;

    if (g_warmup > 0) {
        g_warmup--;
        g_prev_mech_angle = mech_angle;
        return;
    }

    // Glitch filter: reject if mechanical angle jumps too far in one tick
    float delta = mech_angle - g_prev_mech_angle;
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    if ((delta > MECH_GLITCH_THRESHOLD || delta < -MECH_GLITCH_THRESHOLD)
        && g_reject_count < MAX_REJECT) {
        g_reject_count++;
        return;  // reject — keep previous encoder state
    }

    // Multi-turn tracking (use wrapped delta)
    if (delta < -170.0f) g_turns += 1.0f;
    if (delta >  170.0f) g_turns -= 1.0f;
    g_reject_count = 0;
    g_prev_mech_angle = mech_angle;

    // Electrical angle = (mechanical angle × pole pairs) mod 360°
    float elec_deg = fmodf(mech_angle * (float)NUM_POLE_PAIRS, 360.0f);
    if (elec_deg < 0.0f) elec_deg += 360.0f;

    // Convert to radians for FOC
    float elec_rad = elec_deg * (M_PI / 180.0f);

    g_encoder.electrical_angle = elec_rad;
    g_encoder.mechanical_angle = g_turns * 360.0f + mech_angle;
    g_encoder.raw = raw;
    g_encoder.valid = 1;

    publish(SENSOR_ENCODER, (uint8_t *)&g_encoder, sizeof(g_encoder));
}

// Logging
static void on_notify_log_class(uint8_t *data, size_t size) {
    if (size < 1) return;
    g_active_log_class = (data[0] == LOG_CLASS_ENCODER) ? LOG_CLASS_ENCODER : 0;
}

static void on_scheduler_25hz(uint8_t *data, size_t size) {
    if (g_active_log_class == 0) return;

    float log_data[4] = {
        g_encoder.electrical_angle,   // rad
        g_encoder.mechanical_angle,   // degrees (multi-turn)
        (float)g_encoder.raw,         // raw 14-bit count
        (float)g_encoder.valid
    };
    publish(SEND_LOG, (uint8_t *)log_data, sizeof(log_data));
}

// Setup
void encoder_setup(void) {
    platform_encoder_init();

    subscribe(SCHEDULER_1KHZ, on_scheduler_1khz);
    subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
    subscribe(SCHEDULER_25HZ, on_scheduler_25hz);
}
