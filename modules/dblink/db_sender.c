// DB frame sender: TX queue + log/heartbeat framing
//
// 8-slot FIFO — each frame gets its own buffer slot so DMA never
// reads from a buffer that's being overwritten.

#include "db_sender.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>

#define DB_SYNC1       'd'
#define DB_SYNC2       'b'
#define DB_HEADER      6
#define DB_MAX_FRAME   128
#define TX_SLOTS       8

// ── TX queue ───────────────────────────────────────────────────────────────

static struct {
    uint8_t  buf[TX_SLOTS][DB_MAX_FRAME];
    uint16_t len[TX_SLOTS];
    volatile uint8_t head, tail, busy;
} g_q;

static void start_next(void) {
    if (g_q.head == g_q.tail) { g_q.busy = 0; return; }
    g_q.busy = 1;
    platform_uart_send(UART_PORT1, g_q.buf[g_q.tail], g_q.len[g_q.tail]);
}

static void enqueue(uint8_t *data, uint16_t size) {
    uint8_t next = (g_q.head + 1) % TX_SLOTS;
    if (next == g_q.tail || size > DB_MAX_FRAME) return;
    memcpy(g_q.buf[g_q.head], data, size);
    g_q.len[g_q.head] = size;
    g_q.head = next;
    if (!g_q.busy) start_next();
}

static void on_tx_complete(uint8_t *data, size_t size) {
    (void)data; (void)size;
    g_q.tail = (g_q.tail + 1) % TX_SLOTS;
    start_next();
}

// ── Frame builder helper ───────────────────────────────────────────────────

static uint16_t build_frame(uint8_t *out, uint8_t cmd,
                             const uint8_t *payload, uint16_t plen) {
    out[0] = DB_SYNC1;
    out[1] = DB_SYNC2;
    out[2] = cmd;
    out[3] = 0x00;
    out[4] = (uint8_t)(plen & 0xFF);
    out[5] = (uint8_t)(plen >> 8);
    if (plen) memcpy(&out[DB_HEADER], payload, plen);

    uint16_t ck = 0;
    for (uint16_t i = 2; i < DB_HEADER + plen; i++) ck += out[i];
    out[DB_HEADER + plen]     = (uint8_t)(ck & 0xFF);
    out[DB_HEADER + plen + 1] = (uint8_t)(ck >> 8);
    return DB_HEADER + plen + 2;
}

// ── Log (25 Hz from FOC/encoder modules) ───────────────────────────────────

static void on_send_log(uint8_t *data, size_t size) {
    if (size == 0 || size > DB_MAX_FRAME - 8) return;
    uint8_t frame[DB_MAX_FRAME];
    uint16_t len = build_frame(frame, 0x00, data, (uint16_t)size);
    enqueue(frame, len);
}

// ── Heartbeat (1 Hz) ──────────────────────────────────────────────────────

static uint32_t g_hb_count;

static void on_1hz(uint8_t *data, size_t size) {
    (void)data; (void)size;
    g_hb_count++;
    uint8_t frame[14];
    build_frame(frame, 0xFF, (uint8_t *)&g_hb_count, 4);
    enqueue(frame, 14);
}

// ── Setup ──────────────────────────────────────────────────────────────────

void db_sender_setup(void) {
    g_hb_count = 0;
    subscribe(UART_TX_COMPLETE, on_tx_complete);
    subscribe(SEND_LOG, on_send_log);
    subscribe(SCHEDULER_1HZ, on_1hz);
}
