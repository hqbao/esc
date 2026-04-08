# FOC Module

## Overview

Field-Oriented Control state machine. Receives phase currents and encoder angle at 40 kHz, performs Clarke and Park transforms, runs PI current loops, and outputs PWM via inverse Park + SVPWM. Also handles encoder offset calibration (CP2) and open-loop commutation for verification checkpoints.

## Data Flow

```
SENSOR_PHASE_CURRENT (40 kHz)
    │
    ▼  on_commutate()
  Clarke: Iu, Iv, Iw → Iα, Iβ
    │
    ▼
  Park: Iα, Iβ, θ → Id, Iq
    │
    ├─ Open-loop: sine table → PWM duty (CP1/CP3/CP4)
    └─ Closed-loop: PI(Id,Iq) → Vd,Vq → inv Park → SVPWM → PWM (CP5/CP6)
```

## State Machine

| State | ID | Description |
|-------|----|-------------|
| IDLE | 0 | Motors off, waiting for throttle |
| CALIBRATE | 1 | Lock rotor at 4 angles, measure encoder offset (CP2) |
| ALIGN | 2 | Lock rotor at 0°, capture encoder offset |
| RAMP | 3 | Open-loop acceleration to target speed |
| OPENLOOP | 4 | Steady open-loop commutation (CP1/CP3/CP4) |
| CLOSEDLOOP | 5 | Full FOC with PI current control (CP5/CP6) |

### Transitions

- **IDLE → ALIGN**: Throttle > 0.01
- **IDLE → CALIBRATE**: `LOG_CLASS_FOC` received while idle (CP2 procedure)
- **ALIGN → RAMP**: After 0.5s lock (20000 ticks)
- **RAMP → OPENLOOP**: Speed and amplitude reach targets (default path)
- **RAMP → CLOSEDLOOP**: Speed target reached while `LOG_CLASS_ENCODER` active
- **Any → IDLE**: Throttle < 0.01 or `FOC_RELEASE` event

## Calibration (CP2)

Triggered by sending `LOG_CLASS_FOC` while in IDLE state. Locks the rotor at 4 electrical angles (0°, 90°, 180°, 270°) for 1 second each. At each angle, averages the encoder reading over the last 25% of the dwell time using circular mean (`atan2(sin_sum, cos_sum)`). The offset at angle 0° becomes `g_enc_offset`.

## Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `DT` | 1/40000 | Control loop period (25 µs) |
| `MAX_DUTY` | 0.45 × PWM_PERIOD | Maximum PWM duty (956 ticks) |
| `OL_SPEED` | 0.02 | Open-loop speed (3.125 Hz electrical) |
| `OL_AMPLITUDE` | 0.20 × MAX_DUTY | Open-loop PWM amplitude |
| `ALIGN_TICKS` | 20000 | Align dwell time (0.5s at 40 kHz) |
| `ALIGN_AMPLITUDE` | 0.35 × MAX_DUTY | Align lock strength |

### PI Controllers (Closed-Loop)

| Parameter | Id | Iq |
|-----------|----|----|
| Kp | 0.5 | 0.5 |
| Ki | 100.0 | 100.0 |
| Limit | ±6.0 V | ±6.0 V |

## Log Classes

| Class | Checkpoint | Payload (8 floats) |
|-------|-----------|-------------------|
| `LOG_CLASS_FOC` (1) | CP2 | state, cal_step, applied_angle, enc_elec, enc@0°/90°/180°/270° |
| `LOG_CLASS_CURRENT` (2) | CP3 | state, Iα, Iβ, enc_elec, Vbus, 0, 0, 0 |
| `LOG_CLASS_VOLTAGE` (3) | CP4 | state, Id, Iq, θ_rotor, enc_elec, enc_offset, Iα, Iβ |
| `LOG_CLASS_ENCODER` (5) | CP5 | state, Id, Iq, Vd, Vq, Iq_ref, Id_ref, θ |

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `SENSOR_PHASE_CURRENT` | 40 kHz | Trigger commutation loop |
| `SENSOR_BUS_VOLTAGE` | 100 Hz | Update Vbus for SVPWM scaling |
| `SENSOR_ENCODER` | 1 kHz | Update electrical angle |
| `MOTOR_THROTTLE` | Event | Start/stop motor, set speed reference |
| `NOTIFY_LOG_CLASS` | Event | Select log output, trigger calibration |
| `SCHEDULER_25HZ` | 25 Hz | Publish telemetry log |
| `FOC_RELEASE` | Event | Emergency stop (overcurrent) |

### Publications
| Topic | Rate | Data |
|-------|------|------|
| `SEND_LOG` | 25 Hz | 8 floats (format depends on active log class) |
