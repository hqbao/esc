# Encoder Module

## Overview

Reads the **AS5048A** 14-bit magnetic encoder via bit-bang SPI at 1 kHz. Converts raw counts to electrical angle (radians) for FOC commutation, with multi-turn mechanical angle tracking.

## Data Flow

```
SCHEDULER_1KHZ
    │
    ▼  on_scheduler_1khz()
  Bit-bang SPI → AS5048A (2 × 16-bit transfers)
    │
    ▼  raw 14-bit count (0–16383)
  Negate direction (360° - raw°)
    │
    ▼  Electrical angle = mechanical × 11 pole pairs (mod 2π)
    │
    └─► SENSOR_ENCODER (encoder_data_t, 1 kHz)
```

## Hardware

| Parameter | Value |
|-----------|-------|
| Sensor | AS5048A |
| Interface | Bit-bang SPI (mode 1: CPOL=0, CPHA=1) |
| Resolution | 14-bit (16384 counts/rev) |
| SCK | PB6 (J8 pin 1) |
| MOSI | PB7 (J8 pin 2) |
| MISO | PB8 (J8 pin 3) |
| CS | PA15 (PWM input pad, active low) |

### SPI Protocol

The AS5048A uses a two-transaction read protocol:
1. Send read command (with even parity) → response is from previous command
2. Send NOP (0xC000) → response contains the angle data

Each 16-bit frame: `[parity(1)][R/W(1)][address(6)][data(8)]`. The angle is in bits [13:0] of the response.

## Direction Convention

The raw encoder counts are **negated** (`360° - raw°`) because the encoder's rotation direction is opposite to the electrical field direction on this motor.

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `NUM_POLE_PAIRS` | 11 | Motor pole pairs |
| `AS5048A_COUNTS` | 16384 | 14-bit resolution |
| Warmup samples | 5 | Discarded at startup for sensor settling |

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `SCHEDULER_1KHZ` | 1 kHz | Read encoder angle |
| `NOTIFY_LOG_CLASS` | Event | Activate `LOG_CLASS_ENCODER` (5) logging |
| `SCHEDULER_25HZ` | 25 Hz | Publish encoder log |

### Publications
| Topic | Rate | Data |
|-------|------|------|
| `SENSOR_ENCODER` | 1 kHz | `encoder_data_t` (electrical angle rad, mechanical angle deg, raw count) |
| `SEND_LOG` | 25 Hz | 4 floats (elec_rad, mech_deg, raw, valid) when `LOG_CLASS_ENCODER` active |
