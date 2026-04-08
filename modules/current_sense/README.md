# Current Sense Module

## Overview

Converts raw ADC readings from the 3-shunt current sensing circuit into calibrated phase currents (Amps). Performs automatic zero-current offset calibration at startup (motor must be stopped).

## Data Flow

```
ADC_INJECTED_COMPLETE (40 kHz, TIM1_CC4 triggered)
    │
    ▼  on_adc_injected()
  Startup calibration (first 1000 samples)
    │   → average offsets for U, V, W
    │
    ▼  After calibration:
  (raw - offset) / 4096 × 3.3V / (0.003Ω × 16)
    │
    └─► SENSOR_PHASE_CURRENT (phase_current_t, 40 kHz)
```

## Hardware

| Parameter | Value |
|-----------|-------|
| Shunt resistors | 3 × 0.003 Ω |
| OPAMP gain | ×16 (internal PGA) |
| ADC channels | ADC1 CH3 (U), ADC2 CH3 (V), ADC1 CH12 (W) |
| Trigger | TIM1 OC4REF (center of PWM period) |
| Alignment | Left-aligned → right-shifted by 4 in ISR |

## Offset Calibration

At startup, collects 1000 ADC samples per phase with the motor stopped. The average becomes the zero-current offset (~2048 for a 12-bit ADC at mid-rail). No motor commands are accepted by FOC until calibration completes (25 ms).

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `CAL_SAMPLES` | 1000 | Offset calibration sample count |
| `SHUNT_RESISTANCE` | 0.003 Ω | Current sense resistor |
| `OPAMP_GAIN` | 16.0 | Internal OPAMP PGA gain |
| `ADC_VREF` | 3.3 V | ADC reference voltage |
| `ADC_RESOLUTION` | 4096 | 12-bit (after >>4 correction) |

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `ADC_INJECTED_COMPLETE` | 40 kHz | Raw ADC values from ISR |
| `NOTIFY_LOG_CLASS` | Event | Activate `LOG_CLASS_RAW_CURRENT` (4) logging |
| `SCHEDULER_25HZ` | 25 Hz | Publish raw current log |

### Publications
| Topic | Rate | Data |
|-------|------|------|
| `SENSOR_PHASE_CURRENT` | 40 kHz | `phase_current_t` (Iu, Iv, Iw in Amps) |
| `SEND_LOG` | 25 Hz | 3 floats (Iu, Iv, Iw) when `LOG_CLASS_RAW_CURRENT` active |

**Note**: Uses `LOG_CLASS_RAW_CURRENT` (4) — NOT `LOG_CLASS_CURRENT` (2) — to avoid DMA TX contention with the FOC module. See [docs/DMA_TX_CONTENTION_BUG.md](../../docs/DMA_TX_CONTENTION_BUG.md).
