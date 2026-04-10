// DB frame reader: UART RX state machine → dispatches inbound commands
//
// DB frame: ['d']['b'][CMD][CLASS][SIZE_LO][SIZE_HI][PAYLOAD...][CK_LO][CK_HI]
// Checksum: sum of bytes [2..end_of_payload] as uint16_t

#include "db_reader.h"
#include <pubsub.h>
#include <platform.h>
#include <messages.h>
#include <string.h>

#define DB_SYNC1        'd'
#define DB_SYNC2        'b'
#define DB_MAX_PAYLOAD  120

#define DB_CMD_THROTTLE   0x01
#define DB_CMD_LOG_CLASS  0x03
#define DB_CMD_RESET      0x07


typedef enum {
    SYNC1, SYNC2, CMD, CLASS, SIZE_LO, SIZE_HI, PAYLOAD, CK_LO, CK_HI
} state_e;

static state_e  g_state;
static uint8_t  g_cmd;
static uint16_t g_size;
static uint16_t g_idx;
static uint8_t  g_payload[DB_MAX_PAYLOAD];
static uint16_t g_ck_acc;
static uint8_t  g_ck_lo;

static void reset(void) {
    g_state = SYNC1;
    g_idx = 0;
    g_ck_acc = 0;
}

static void dispatch(void) {
    switch (g_cmd) {
    case DB_CMD_THROTTLE:
        if (g_size >= sizeof(float)) {
            motor_throttle_t m;
            memcpy(&m.throttle, g_payload, sizeof(float));
            publish(MOTOR_THROTTLE, (uint8_t *)&m, sizeof(m));
        }
        break;
    case DB_CMD_LOG_CLASS:
        if (g_size >= 1) publish(NOTIFY_LOG_CLASS, g_payload, 1);
        break;
    case DB_CMD_RESET:
        platform_reset();
        break;
    }
}

static void feed(uint8_t b) {
    switch (g_state) {
    case SYNC1:   if (b == DB_SYNC1) g_state = SYNC2; break;
    case SYNC2:   if (b == DB_SYNC2) g_state = CMD; else reset(); break;
    case CMD:     g_cmd = b; g_ck_acc = b; g_state = CLASS; break;
    case CLASS:   g_ck_acc += b; g_state = SIZE_LO; break;
    case SIZE_LO: g_size = b; g_ck_acc += b; g_state = SIZE_HI; break;
    case SIZE_HI:
        g_size |= (uint16_t)b << 8;
        g_ck_acc += b;
        if (g_size > DB_MAX_PAYLOAD) reset();
        else if (g_size == 0) g_state = CK_LO;
        else { g_idx = 0; g_state = PAYLOAD; }
        break;
    case PAYLOAD:
        g_payload[g_idx++] = b;
        g_ck_acc += b;
        if (g_idx >= g_size) g_state = CK_LO;
        break;
    case CK_LO: g_ck_lo = b; g_state = CK_HI; break;
    case CK_HI:
        if ((g_ck_lo | ((uint16_t)b << 8)) == g_ck_acc) dispatch();
        reset();
        break;
    }
}

static void on_uart_rx(uint8_t *data, size_t size) {
    for (size_t i = 0; i < size; i++) feed(data[i]);
}

void db_reader_setup(void) {
    reset();
    subscribe(UART_RAW_RECEIVED, on_uart_rx);
}
