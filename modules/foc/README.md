# FOC Module

## Overview

Field-Oriented Control for BLDC motors. Three compile-time modes share the same PubSub interface and math transforms but differ in rotor angle source:

| Mode | File | Angle Source | Build |
|------|------|-------------|-------|
| **Encoder** | `foc_encoder.c` | AS5048A encoder | `bash build.sh encoder` |
| **BEMF** | `foc_bemf.c` | Sensorless flux observer | `bash build.sh bemf` |
| **No-feedback** | `foc_no_feedback.c` | Open-loop only | `bash build.sh no_feedback` |

Only one mode compiles per build. `foc.h` defines `FOC_MODE_ENCODER` / `FOC_MODE_BEMF` / `FOC_MODE_NO_FEEDBACK` — each source file is guarded by `#ifdef`.

All modes use **voltage-mode** control (Vd/Vq applied directly, no PI current loop).

## Shared Files

| File | Purpose |
|------|---------|
| `foc.h` | Mode guard macros, `foc_setup()` prototype |
| `foc_math.h` | Inline Clarke, Park, Inverse Park, Inverse Clarke transforms |

## Data Flow (All Modes)

```
SENSOR_PHASE_CURRENT (40 kHz)
    │
    ▼  on_commutate()
  Clarke: Iu, Iv, Iw → Iα, Iβ
    │
    ▼
  Park: Iα, Iβ, θ → Id, Iq        (θ from encoder / observer / open-loop)
    │
    ▼
  Voltage-mode: set Vd, Vq
    │
    ▼
  Inv Park → Inv Clarke → PWM duty
```

---

## Encoder Mode (`foc_encoder.c`)

### State Machine

| State | ID | Description |
|-------|----|-------------|
| IDLE | 0 | Motors off, waiting for throttle |
| ALIGN | 1 | Lock rotor at 0°, measure encoder offset |
| RAMP | 2 | Open-loop acceleration |
| OPENLOOP | 3 | Steady OL with CRUISE dwell for direction detection |
| CLOSEDLOOP | 4 | Encoder-based commutation, Vq ramp |

### Transitions

- **IDLE → ALIGN**: Throttle > 0.01
- **ALIGN → RAMP**: After 0.5s lock
- **RAMP → OPENLOOP**: Speed reaches target
- **OPENLOOP → CLOSEDLOOP**: Auto-detect encoder direction + offset
- **Any → IDLE**: Throttle < 0.01 or `FOC_RELEASE`

### Log Classes

| ID | Constant | Payload (8 floats) |
|----|----------|-------------------|
| 1 | `LOG_CLASS_FOC` | enc_elec, enc_mech, theta, offset, ... |
| 2 | `LOG_CLASS_CURRENT` | state, Iα, Iβ, enc_elec, Vbus, ... |
| 5 | `LOG_CLASS_ENCODER` | state, Id, Iq, Vq, enc_offset, enc_dir, ... |

---

## BEMF Mode (`foc_bemf.c`)

Sensorless using a leaky flux observer (voltage model):

```
dpsi/dt = (v_applied - R * i) - wc * psi
theta_e = atan2(psi_beta, psi_alpha)
```

The leaky term `wc` prevents DC drift. Phase error `atan(wc/ω_e)` is compensated at runtime.

### State Machine

| State | ID | Description |
|-------|----|-------------|
| IDLE | 0 | Motors off |
| ALIGN | 1 | Lock rotor at 0° |
| RAMP | 2 | Open-loop acceleration |
| OPENLOOP | 3 | Steady OL, observer runs in background |
| BLENDING | 4 | Gradual OL→CL angle/voltage blend (0.5s) |
| CLOSEDLOOP | 5 | Full sensorless commutation |

### Transitions

- **IDLE → ALIGN**: Throttle > 0.01
- **ALIGN → RAMP**: After 0.5s
- **RAMP → OPENLOOP**: Speed reaches `OL_SPEED`
- **OPENLOOP → BLENDING**: After 1s holdoff + flux magnitude above threshold
- **BLENDING → CLOSEDLOOP**: Blend counter expires (0.5s)
- **Any → IDLE**: Throttle < 0.01 or `FOC_RELEASE`

### Observer Tuning

| Constant | Value | Description |
|----------|-------|-------------|
| `OBSERVER_R` | 0.05 | Observer resistance (low for high-KV motors) |
| `OBSERVER_WC` | 20.0 | Leaky integrator cutoff (rad/s) |
| `BEMF_MAG_THRESH` | 0.005 | Min flux magnitude to start blend |
| `OL_HOLDOFF_TICKS` | 40000 | 1s in OL before blend allowed |
| `BLEND_TICKS` | 20000 | 0.5s blend duration |

### Phase Compensation

- **Observer lead**: Subtract `atan(wc / |speed|)` from observer angle
- **Timing advance**: Add `speed × 30µs`, capped at 15° (0.26 rad)

### Log Classes

| ID | Constant | Payload (8 floats) |
|----|----------|-------------------|
| 2 | `LOG_CLASS_CURRENT` | state, Iα, Iβ, enc_elec, Vbus, ... |
| 3 | `LOG_CLASS_VOLTAGE` | state, Id, Iq, theta, bemf_theta, enc_elec, Iα, Iβ |
| 8 | `LOG_CLASS_BEMF` | state, bemf_theta, ol_theta, enc_elec, flux_mag, speed, ... |

---

## No-Feedback Mode (`foc_no_feedback.c`)

Open-loop only — no encoder or observer for commutation.

### State Machine

| State | ID | Description |
|-------|----|-------------|
| IDLE | 0 | Motors off |
| ALIGN | 1 | Lock rotor |
| RAMP | 2 | Acceleration |
| OPENLOOP | 3 | Steady OL drive |

### Log Classes

| ID | Constant | Payload (8 floats) |
|----|----------|-------------------|
| 2 | `LOG_CLASS_CURRENT` | state, Iα, Iβ, enc_elec, Vbus, ... |
| 3 | `LOG_CLASS_VOLTAGE` | state, Id, Iq, theta, enc_elec, enc_offset, Iα, Iβ |
| 6 | `LOG_CLASS_OPENLOOP` | OL angle, speed, encoder readback, ... |

---

## Common Configuration

| Constant | Value | Description |
|----------|-------|-------------|
| `PWM_PERIOD` | 2125 | Timer period (170 MHz / 2 / 40 kHz) |
| `MAX_DUTY` | 0.45 × PWM_PERIOD | Maximum PWM duty (956 ticks) |
| `OL_SPEED` | 0.03 (BEMF), 0.02 (encoder) | OL electrical speed increment per tick |
| `OL_AMPLITUDE` | 0.10 × MAX_DUTY | Open-loop PWM amplitude |
| `ALIGN_TICKS` | 20000 | Align dwell time (0.5s at 40 kHz) |
| `ALIGN_AMPLITUDE` | 0.10 × MAX_DUTY | Align lock strength |

## PubSub Interface

### Subscriptions
| Topic | Rate | Purpose |
|-------|------|---------|
| `SENSOR_PHASE_CURRENT` | 40 kHz | Trigger commutation loop |
| `SENSOR_BUS_VOLTAGE` | 100 Hz | Update Vbus |
| `SENSOR_ENCODER` | 1 kHz | Encoder angle (encoder mode, or read-only logging in BEMF) |
| `MOTOR_THROTTLE` | Event | Start/stop motor, set speed reference |
| `NOTIFY_LOG_CLASS` | Event | Select log output |
| `SCHEDULER_25HZ` | 25 Hz | Publish telemetry log |
| `FOC_RELEASE` | Event | Emergency stop (overcurrent) |

### Publications
| Topic | Rate | Data |
|-------|------|------|
| `SEND_LOG` | 25 Hz | 8 floats (format depends on active log class) |
