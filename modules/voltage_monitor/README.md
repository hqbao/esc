# Voltage Monitor Module

## Overview

Reads bus voltage (VBUS) from the ADC regular group, converts to volts using the resistor divider ratio, and publishes for use by the FOC module's SVPWM scaling.

## Data Flow

```
ADC_REGULAR_COMPLETE (100 Hz, software-polled in ISR)
    │
    ▼  on_adc_regular()
  raw / 4096 × 3.3V / 0.09626
    │
    └─► SENSOR_BUS_VOLTAGE (bus_measurement_t, 100 Hz)
```

## Hardware

| Parameter | Value |
|-----------|-------|
| ADC pin | PA0 (ADC1 CH1) |
| Divider | R1=169 kΩ, R2=18 kΩ |
| Ratio | 0.09626 |
| Trigger | Software-polled at 100 Hz |
| Alignment | Left-aligned → right-shifted by 4 in ISR |

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `VBUS_DIVIDER_RATIO` | 0.09626 | R2 / (R1 + R2) |
| `ADC_VREF` | 3.3 V | ADC reference voltage |
| `ADC_RESOLUTION` | 4096 | 12-bit (after >>4 correction) |

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `ADC_REGULAR_COMPLETE` | 100 Hz | Raw VBUS ADC value from ISR |
| `NOTIFY_LOG_CLASS` | Event | Activate `LOG_CLASS_VOLTAGE` (4) logging |
| `SCHEDULER_25HZ` | 25 Hz | Publish voltage log |

### Publications
| Topic | Rate | Data |
|-------|------|------|
| `SENSOR_BUS_VOLTAGE` | 100 Hz | `bus_measurement_t` (voltage + temperature) |
| `SEND_LOG` | 25 Hz | 2 floats (Vbus, temperature) when `LOG_CLASS_VOLTAGE` active |
