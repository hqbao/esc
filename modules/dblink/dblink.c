// DB-link: UART frame parser + log sender for ESC
//
// DB frame: ['d']['b'][CMD][CLASS][SIZE_LO][SIZE_HI][PAYLOAD...][CK_LO][CK_HI]
// Checksum: sum of bytes [2..end_of_payload] as uint16_t
//
// Inbound commands:
//   CMD=0x01 THROTTLE  — payload: 1 float (0.0–1.0) → publishes MOTOR_THROTTLE
//   CMD=0x03 LOG_CLASS — payload: 1 byte (class ID) → publishes NOTIFY_LOG_CLASS
//   CMD=0x07 RESET     — no payload → calls platform_reset()
//
// Outbound: subscribes to SEND_LOG, wraps in DB frame, sends over UART

#include "dblink.h"
#include <pubsub.h>
#include <platform.h>
#include <messages.h>
#include <string.h>

#define DB_SYNC1          'd'
#define DB_SYNC2          'b'
#define DB_HEADER_SIZE    6
#define DB_MAX_PAYLOAD    120
#define DB_MAX_FRAME      128

#define DB_CMD_THROTTLE       0x01
#define DB_CMD_LOG_CLASS      0x03
#define DB_CMD_RESET          0x07

// ── Frame parser state machine ─────────────────────────────────────────────
typedef enum {
    PARSE_SYNC1,
    PARSE_SYNC2,
    PARSE_CMD,
    PARSE_CLASS,
    PARSE_SIZE_LO,
    PARSE_SIZE_HI,
    PARSE_PAYLOAD,
    PARSE_CK_LO,
    PARSE_CK_HI,
} parse_state_e;

static parse_state_e g_parse_state;
static uint8_t  g_cmd;
static uint8_t  g_class;
static uint16_t g_payload_size;
static uint16_t g_payload_idx;
static uint8_t  g_payload[DB_MAX_PAYLOAD];
static uint16_t g_checksum_acc;
static uint8_t  g_ck_lo;

static void parse_reset(void) {
    g_parse_state = PARSE_SYNC1;
    g_payload_idx = 0;
    g_checksum_acc = 0;
}

static void process_frame(void) {
    switch (g_cmd) {
    case DB_CMD_THROTTLE:
        if (g_payload_size >= sizeof(float)) {
            motor_throttle_t cmd;
            memcpy(&cmd.throttle, g_payload, sizeof(float));
            publish(MOTOR_THROTTLE, (uint8_t *)&cmd, sizeof(cmd));
        }
        break;

    case DB_CMD_LOG_CLASS:
        if (g_payload_size >= 1) {
            publish(NOTIFY_LOG_CLASS, g_payload, 1);
        }
        break;

    case DB_CMD_RESET:
        platform_reset();
        break;

    default:
        break;
    }
}

static void parse_byte(uint8_t byte) {
    switch (g_parse_state) {
    case PARSE_SYNC1:
        if (byte == DB_SYNC1) g_parse_state = PARSE_SYNC2;
        break;

    case PARSE_SYNC2:
        if (byte == DB_SYNC2) g_parse_state = PARSE_CMD;
        else parse_reset();
        break;

    case PARSE_CMD:
        g_cmd = byte;
        g_checksum_acc = byte;
        g_parse_state = PARSE_CLASS;
        break;

    case PARSE_CLASS:
        g_class = byte;
        g_checksum_acc += byte;
        g_parse_state = PARSE_SIZE_LO;
        break;

    case PARSE_SIZE_LO:
        g_payload_size = byte;
        g_checksum_acc += byte;
        g_parse_state = PARSE_SIZE_HI;
        break;

    case PARSE_SIZE_HI:
        g_payload_size |= (uint16_t)byte << 8;
        g_checksum_acc += byte;
        if (g_payload_size > DB_MAX_PAYLOAD) {
            parse_reset();
        } else if (g_payload_size == 0) {
            g_parse_state = PARSE_CK_LO;
        } else {
            g_payload_idx = 0;
            g_parse_state = PARSE_PAYLOAD;
        }
        break;

    case PARSE_PAYLOAD:
        g_payload[g_payload_idx++] = byte;
        g_checksum_acc += byte;
        if (g_payload_idx >= g_payload_size) {
            g_parse_state = PARSE_CK_LO;
        }
        break;

    case PARSE_CK_LO:
        g_ck_lo = byte;
        g_parse_state = PARSE_CK_HI;
        break;

    case PARSE_CK_HI: {
        uint16_t received_ck = g_ck_lo | ((uint16_t)byte << 8);
        if (received_ck == g_checksum_acc) {
            process_frame();
        }
        parse_reset();
        break;
    }

    default:
        parse_reset();
        break;
    }
}

// ── UART RX callback ───────────────────────────────────────────────────────
static void on_uart_raw_received(uint8_t *data, size_t size) {
    for (size_t i = 0; i < size; i++) {
        parse_byte(data[i]);
    }
}

// ── Log sender ─────────────────────────────────────────────────────────────
static uint8_t g_tx_buf[DB_MAX_FRAME];

static void on_send_log(uint8_t *data, size_t size) {
    if (size == 0 || size > DB_MAX_PAYLOAD) return;

    uint16_t payload_size = (uint16_t)size;

    g_tx_buf[0] = DB_SYNC1;
    g_tx_buf[1] = DB_SYNC2;
    g_tx_buf[2] = 0x00;  // CMD = log data
    g_tx_buf[3] = 0x00;  // CLASS = 0
    g_tx_buf[4] = (uint8_t)(payload_size & 0xFF);
    g_tx_buf[5] = (uint8_t)(payload_size >> 8);
    memcpy(&g_tx_buf[6], data, size);

    // Checksum: sum of bytes [2..end_of_payload]
    uint16_t ck = 0;
    for (uint16_t i = 2; i < 6 + payload_size; i++) {
        ck += g_tx_buf[i];
    }
    g_tx_buf[6 + payload_size] = (uint8_t)(ck & 0xFF);
    g_tx_buf[7 + payload_size] = (uint8_t)(ck >> 8);

    platform_uart_send(UART_PORT1, g_tx_buf, 8 + payload_size);
}

// ── Heartbeat (1 Hz) ───────────────────────────────────────────────────────
static uint32_t g_heartbeat_count;
static uint8_t g_hb_buf[12];  // 'd','b', CMD, CLASS, SIZE(2), COUNT(4)=payload, CK(2)

static void on_scheduler_1hz(uint8_t *data, size_t size) {
    g_heartbeat_count++;

    g_hb_buf[0] = DB_SYNC1;
    g_hb_buf[1] = DB_SYNC2;
    g_hb_buf[2] = 0xFF;  // CMD = heartbeat
    g_hb_buf[3] = 0x00;
    g_hb_buf[4] = 4;     // payload size = 4 bytes
    g_hb_buf[5] = 0;
    memcpy(&g_hb_buf[6], &g_heartbeat_count, 4);

    uint16_t ck = 0;
    for (uint16_t i = 2; i < 10; i++) ck += g_hb_buf[i];
    g_hb_buf[10] = (uint8_t)(ck & 0xFF);
    g_hb_buf[11] = (uint8_t)(ck >> 8);

    platform_uart_send(UART_PORT1, g_hb_buf, 12);
}

// ── Setup ──────────────────────────────────────────────────────────────────
void dblink_setup(void) {
    parse_reset();
    g_heartbeat_count = 0;
    subscribe(UART_RAW_RECEIVED, on_uart_raw_received);
    subscribe(SEND_LOG, on_send_log);
    subscribe(SCHEDULER_1HZ, on_scheduler_1hz);
}
