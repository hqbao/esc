// Local storage module — persists parameters to flash with CRC32 validation
//
// RAM array holds PARAM_ID_COUNT floats (32 bytes). On save, sets dirty flag.
// 1Hz scheduler flushes dirty data to flash (page erase + write).
// Motor is always idle when save fires, so the ~1ms flash stall is harmless.
//
// Flash layout: [PARAM_ID_COUNT × float (32B)] [CRC32 (4B)] = 36 bytes

#include "local_storage.h"
#include <pubsub.h>
#include <platform.h>
#include <messages.h>
#include <string.h>

#define DATA_SIZE       (PARAM_ID_COUNT * sizeof(float))
#define CHECKSUM_SIZE   4
#define STORAGE_SIZE    (DATA_SIZE + CHECKSUM_SIZE)

static uint8_t g_data[STORAGE_SIZE];
static uint8_t g_dirty;
static uint8_t g_log_class;

// ── CRC32 ──────────────────────────────────────────────────────────────────

static uint32_t crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return ~crc;
}

// ── Save param to RAM (deferred flush) ─────────────────────────────────────

static void local_storage_save(uint8_t *data, size_t size) {
    if (size < sizeof(param_storage_t)) return;
    param_storage_t p;
    memcpy(&p, data, sizeof(p));
    if (p.id < 0 || p.id >= PARAM_ID_COUNT) return;

    memcpy(&g_data[p.id * sizeof(float)], &p.value, sizeof(float));
    g_dirty = 1;
}

// ── Load param from RAM, publish result ────────────────────────────────────

static void local_storage_load(uint8_t *data, size_t size) {
    if (size < sizeof(param_id_e)) return;
    param_id_e id;
    memcpy(&id, data, sizeof(id));
    if (id < 0 || id >= PARAM_ID_COUNT) return;

    param_storage_t result;
    result.id = id;
    memcpy(&result.value, &g_data[id * sizeof(float)], sizeof(float));
    publish(LOCAL_STORAGE_RESULT, (uint8_t *)&result, sizeof(result));
}

// ── Flush to flash at 1Hz ──────────────────────────────────────────────────

static void on_scheduler_1hz(uint8_t *data, size_t size) {
    // Flush dirty data
    if (g_dirty) {
        g_dirty = 0;
        uint32_t checksum = crc32(g_data, DATA_SIZE);
        memcpy(&g_data[DATA_SIZE], &checksum, CHECKSUM_SIZE);
        platform_storage_write(0, STORAGE_SIZE, g_data);
    }

    // Log readback if active
    if (g_log_class != LOG_CLASS_STORAGE) return;
    float log_data[8];
    for (int i = 0; i < PARAM_ID_COUNT && i < 8; i++) {
        memcpy(&log_data[i], &g_data[i * sizeof(float)], sizeof(float));
    }
    publish(SEND_LOG, (uint8_t *)log_data, sizeof(log_data));
}

// ── Log class ──────────────────────────────────────────────────────────────

static void on_notify_log_class(uint8_t *data, size_t size) {
    if (size < 1) return;
    g_log_class = data[0];
}

// ── Load from flash at startup ─────────────────────────────────────────────

static void load_from_flash(void) {
    uint8_t temp[STORAGE_SIZE];
    platform_storage_read(0, STORAGE_SIZE, temp);

    uint32_t stored_crc;
    memcpy(&stored_crc, &temp[DATA_SIZE], CHECKSUM_SIZE);
    uint32_t calc_crc = crc32(temp, DATA_SIZE);

    if (stored_crc == calc_crc) {
        memcpy(g_data, temp, DATA_SIZE);
    }
    // else: g_data stays zeroed (defaults)
}

// ── Setup ──────────────────────────────────────────────────────────────────

void local_storage_setup(void) {
    memset(g_data, 0, sizeof(g_data));
    load_from_flash();

    subscribe(LOCAL_STORAGE_SAVE, local_storage_save);
    subscribe(LOCAL_STORAGE_LOAD, local_storage_load);
    subscribe(SCHEDULER_1HZ, on_scheduler_1hz);
    subscribe(NOTIFY_LOG_CLASS, on_notify_log_class);
}
