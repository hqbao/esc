# DBLink Module

## Overview

UART communication bridge using the DB framing protocol. Parses inbound commands from Python tools (throttle, log class selection, reset) and wraps outbound log data from other modules into checksummed frames for transmission.

## Data Flow

```
UART_RAW_RECEIVED (DMA + IDLE detection)
    │
    ▼  on_uart_raw_received()
  State-machine parser: sync → header → payload → checksum
    │
    ├─ CMD 0x01 (THROTTLE)  → MOTOR_THROTTLE
    ├─ CMD 0x03 (LOG_CLASS) → NOTIFY_LOG_CLASS
    └─ CMD 0x07 (RESET)     → platform_reset()

SEND_LOG (from other modules)
    │
    ▼  on_send_log()
  Wrap in DB frame → platform_uart_send() (DMA TX)
```

## Frame Format

```
['d']['b'][CMD][CLASS][SIZE_LO][SIZE_HI][PAYLOAD...][CK_LO][CK_HI]
  sync       header (4 bytes)             0–120 bytes    checksum
```

- **Checksum**: `sum(bytes[2 .. 5+size]) & 0xFFFF` (CMD through end of payload)
- **Max payload**: 120 bytes
- **Max frame**: 128 bytes

## Inbound Commands

| CMD | ID | Payload | Action |
|-----|----|---------|--------|
| THROTTLE | 0x01 | 1 float (0.0–1.0) | Publish `MOTOR_THROTTLE` |
| LOG_CLASS | 0x03 | 1 byte (class ID) | Publish `NOTIFY_LOG_CLASS` |
| RESET | 0x07 | — | `platform_reset()` (NVIC_SystemReset) |

## Outbound

- **Log frames** (25 Hz): CMD=0x00, payload from `SEND_LOG` subscribers
- **Heartbeat** (1 Hz): CMD=0xFF, payload = 4-byte counter

## DMA TX Contention

Only one `SEND_LOG` publisher should be active per log class at a time. `HAL_UART_Transmit_DMA` returns `HAL_BUSY` if a transfer is already in progress, silently dropping the frame. See [docs/DMA_TX_CONTENTION_BUG.md](../../docs/DMA_TX_CONTENTION_BUG.md).

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `UART_RAW_RECEIVED` | Event | Parse inbound DB frames |
| `SEND_LOG` | 25 Hz | Wrap and transmit log data |
| `SCHEDULER_1HZ` | 1 Hz | Send heartbeat frame |

### Publications
| Topic | Rate | Data |
|-------|------|------|
| `MOTOR_THROTTLE` | Event | `motor_throttle_t` (from CMD 0x01) |
| `NOTIFY_LOG_CLASS` | Event | 1 byte log class ID (from CMD 0x03) |
