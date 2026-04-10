# Local Storage Module

## Overview

Persists parameters to STM32 flash with CRC32 validation. Provides a RAM-cached key-value store (indexed by `param_id_e`) that flushes to flash at 1 Hz when dirty. Motor is always idle when the flush fires, so the ~1 ms flash erase/write stall is harmless.

## Data Flow

```
LOCAL_STORAGE_SAVE (from other modules)
    │
    ▼  local_storage_save()
  Write float to RAM array, set dirty flag
    │
    ▼  on_scheduler_1hz()  [deferred flush]
  CRC32 over data → page erase → write 36 bytes to flash

LOCAL_STORAGE_LOAD (from other modules)
    │
    ▼  local_storage_load()
  Read float from RAM array
    │
    └─► LOCAL_STORAGE_RESULT (param_storage_t)
```

## Flash Layout

```
Offset  Size   Content
0x00    32 B   PARAM_ID_COUNT × float (parameter values)
0x20     4 B   CRC32 checksum
─────   ────
Total   36 B
```

On startup, the module reads 36 bytes from flash, validates CRC32, and copies to the RAM cache. If CRC fails (erased flash or corruption), all parameters default to 0.0.

## CRC32

Standard Ethernet CRC32 (polynomial 0xEDB88320, reflected). Computed over the 32-byte data region only.

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `PARAM_ID_COUNT` | 8 | Max stored parameters |
| `DATA_SIZE` | 32 B | Parameter storage region |
| `STORAGE_SIZE` | 36 B | Data + CRC32 |
| `LOG_CLASS_STORAGE` | 6 | Log class for readback |

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `LOCAL_STORAGE_SAVE` | Event | Store parameter to RAM (deferred flash write) |
| `LOCAL_STORAGE_LOAD` | Event | Read parameter from RAM cache |
| `SCHEDULER_1HZ` | 1 Hz | Flush dirty data to flash, optional log output |
| `NOTIFY_LOG_CLASS` | Event | Activate storage readback logging |

### Publications
| Topic | Rate | Data |
|-------|------|------|
| `LOCAL_STORAGE_RESULT` | Event | `param_storage_t` (id + value) in response to load request |
| `SEND_LOG` | 1 Hz | 8 floats (all stored params) when `LOG_CLASS_STORAGE` active |
