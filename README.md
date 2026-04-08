# ESC — Brushless Motor Controller

Field-Oriented Control (FOC) firmware for the **B-G431B-ESC1** discovery kit (STM32G431CBU6).
Built from scratch with the same PubSub architecture as the [flight-controller](../flight-controller/) project.

## Hardware

| Item | Detail |
|------|--------|
| Board | B-G431B-ESC1 (MB1419) |
| MCU | STM32G431CBU6, Cortex-M4F, 170 MHz |
| Flash / RAM | 128 KB / 32 KB |
| Current sense | 3-shunt (3 mΩ), OPAMP PGA ×16 |
| Encoder | AS5048A (14-bit, bit-bang SPI) |
| Comms | USART2 @ 19200 baud (DMA TX+RX, IDLE detection) |
| PWM | TIM1 center-aligned, 40 kHz |
| Protection | Comparator-based overcurrent → TIM1 BKIN |

## Project Structure

```
esc/
├── base/
│   ├── boards/
│   │   └── g4esc1/                 # B-G431B-ESC1 board package
│   │       ├── Core/               # CubeMX-generated HAL init
│   │       ├── Drivers/            # HAL drivers
│   │       ├── platform/           # Board-specific drivers
│   │       │   ├── module.c        # Module init + peripheral start
│   │       │   ├── platform_isr.c  # ISR → PubSub bridge
│   │       │   ├── platform_pwm.c  # PWM + DShot output
│   │       │   ├── platform_encoder.c  # AS5048A SPI bit-bang
│   │       │   ├── platform_uart.c # UART DMA + IDLE detection
│   │       │   ├── platform_common.c   # LED, delay, time, reset
│   │       │   └── platform_hw.h   # Extern HAL handle declarations
│   │       ├── Makefile            # Standalone ARM GCC build
│   │       ├── build.sh            # Build script
│   │       └── build-flash.sh      # Build + ST-Link flash
│   └── foundation/                 # Shared core
│       ├── pubsub.h / pubsub.c    # Topic-based pub/sub
│       ├── messages.h              # Message structs + log classes
│       ├── macro.h                 # Constants + conversion macros
│       ├── platform.h              # Platform abstraction API
│       └── pi_controller.h        # Generic PI controller
├── modules/
│   ├── current_sense/   # 3-phase shunt current → Amps
│   ├── voltage_monitor/ # Bus voltage + temperature
│   ├── encoder/         # AS5048A → electrical/mechanical angle
│   ├── foc/             # FOC state machine + transforms + PI loops
│   └── dblink/          # UART protocol (DB framing) + commands
│   Each module has its own README.md with data flow, PubSub interface, and configuration.
├── tools/                          # Python visualization & test tools
│   ├── cp2_encoder_offset.py       # CP2: Encoder offset calibration
│   ├── cp3_clarke_verify.py        # CP3: Clarke transform verification
│   └── esc_foc_test.py             # General FOC diagnostic
└── docs/
    ├── ADC_ALIGNMENT_BUG.md        # Left-aligned ADC data issue
    └── DMA_TX_CONTENTION_BUG.md    # DMA UART TX silent frame drops
```

## Architecture

### PubSub Decoupling

Same pattern as flight-controller — modules never include each other's headers. All communication via PubSub topics.

```c
// ❌ FORBIDDEN
#include "../encoder/encoder.h"

// ✅ REQUIRED — PubSub only
subscribe(SENSOR_ENCODER, on_encoder_data);
```

### Topics

| Topic | Data | Rate | Description |
|-------|------|------|-------------|
| `SCHEDULER_1KHZ` | — | 1 kHz | Scheduler tick (TIM6) |
| `SCHEDULER_100HZ` | — | 100 Hz | |
| `SCHEDULER_25HZ` | — | 25 Hz | Log output |
| `SCHEDULER_10HZ` | — | 10 Hz | |
| `SCHEDULER_1HZ` | — | 1 Hz | Heartbeat |
| `ADC_INJECTED_COMPLETE` | `uint16_t[3]` | 40 kHz | Raw phase currents (TIM1-triggered) |
| `ADC_REGULAR_COMPLETE` | `uint16_t` | 100 Hz | Raw VBUS (12-bit, >>4 shifted) |
| `SENSOR_PHASE_CURRENT` | `phase_current_t` | 40 kHz | Calibrated 3-phase currents (Amps) |
| `SENSOR_BUS_VOLTAGE` | `bus_measurement_t` | 100 Hz | Bus voltage (Volts) + temperature |
| `SENSOR_ENCODER` | `encoder_data_t` | 1 kHz | Electrical + mechanical angle |
| `MOTOR_THROTTLE` | `motor_throttle_t` | On demand | Throttle command (0–1) |
| `FOC_RELEASE` | — | On event | Overcurrent emergency stop |
| `NOTIFY_LOG_CLASS` | `uint8_t` | On demand | Select active log stream |
| `SEND_LOG` | `float[8]` | 25 Hz | Telemetry to Python tools |

### FOC Control Loop (40 kHz)

```
ADC injected ISR (TIM1_CC4)
  → current_sense: raw ADC → calibrated Amps → SENSOR_PHASE_CURRENT
    → foc: Clarke → Park → PI (Id, Iq) → Inv Park → SVPWM → PWM duty
```

### State Machine

| State | ID | Description |
|-------|----|-------------|
| IDLE | 0 | Waiting, PWM off |
| CALIBRATE | 1 | Lock at 4 angles, measure encoder offset (CP2) |
| ALIGN | 2 | Lock rotor, capture offset |
| RAMP | 3 | Open-loop acceleration |
| OPENLOOP | 4 | Steady open-loop (CP1/CP3/CP4) |
| CLOSEDLOOP | 5 | Full FOC with PI current control (CP5/CP6) |

### Log Classes

| ID | Constant | Checkpoint | Payload |
|----|----------|------------|---------|
| 1 | `LOG_CLASS_FOC` | CP2 | state, cal_step, applied_angle, enc_elec, enc@0/90/180/270 |
| 2 | `LOG_CLASS_CURRENT` | CP3 | state, Iα, Iβ, enc_elec, Vbus, 0, 0, 0 |
| 3 | `LOG_CLASS_VOLTAGE` | CP4 | state, Id, Iq, theta_rotor, enc_elec, enc_offset, Iα, Iβ |
| 4 | `LOG_CLASS_RAW_CURRENT` | — | Ia, Ib, Ic (raw phase currents, current_sense module) |
| 5 | `LOG_CLASS_ENCODER` | CP5 | state, Id, Iq, Vd, Vq, Iq_ref, Id_ref, theta |

## Checkpoint Verification

Each subsystem is verified before moving to the next. Python tools visualize each checkpoint.

| CP | Name | Tool | Status |
|----|------|------|--------|
| CP1 | Open-loop + encoder | `esc_foc_test.py` | ✅ Pass (11 pole pairs confirmed) |
| CP2 | Encoder offset calibration | `cp2_encoder_offset.py` | ✅ Pass (84.0°, errors <5°) |
| CP3 | Clarke transform | `cp3_clarke_verify.py` | 🔨 In progress |
| CP4 | Park transform (Id/Iq DC) | — | Blocked by CP3 |
| CP5 | Closed-loop current control | — | — |
| CP6 | Speed control | — | — |

## Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `PWM_FREQ` | 40 kHz | Center-aligned PWM frequency |
| `PWM_PERIOD` | 2125 | Timer period (170 MHz / 2 / 40 kHz) |
| `NUM_POLE_PAIRS` | 11 | Motor pole pairs |
| `SHUNT_RESISTANCE` | 0.003 Ω | Current shunt value |
| `OPAMP_GAIN` | 16.0 | Internal OPAMP PGA gain |
| `ADC_VREF` | 3.3 V | ADC reference voltage |
| `ADC_RESOLUTION` | 4096 | 12-bit ADC |
| `VBUS_DIVIDER_RATIO` | 0.09626 | R1=169k, R2=18k |
| `MOTOR_RESISTANCE` | 0.5 Ω | Phase resistance (tune per motor) |
| `MOTOR_INDUCTANCE` | 1 mH | Phase inductance (tune per motor) |
| `SINE_TABLE_SIZE` | 256 | Open-loop sine LUT |

## ADC Architecture

**CRITICAL**: CubeMX configures `DataAlign = ADC_DATAALIGN_LEFT` for both ADC1 and ADC2. All raw values from DR/JDR registers are **left-shifted by 4** and must be right-shifted (`>> 4`) before use. See [docs/ADC_ALIGNMENT_BUG.md](docs/ADC_ALIGNMENT_BUG.md).

- **Injected group** (40 kHz, TIM1_CC4 triggered): 3 phase currents via OPAMPs
- **Regular group** (100 Hz, software polled): Bus voltage + temperature

## Building & Flashing

```bash
# Build
cd base/boards/g4esc1 && ./build.sh

# Build + flash via ST-Link
cd base/boards/g4esc1 && ./build-flash.sh
```

## Python Tools

Deps: `pyserial matplotlib numpy`. Validate: `python3 -m py_compile <script>.py`.

| Tool | Purpose |
|------|---------|
| `cp2_encoder_offset.py` | CP2: Lock rotor at 4 angles, measure encoder offset |
| `cp3_clarke_verify.py` | CP3: Verify Clarke Iα/Iβ sinusoids + Lissajous |
| `esc_foc_test.py` | General FOC diagnostic (state, currents, encoder) |

Each tool has **Start Log** / **Stop Motor** / **Reset FC** buttons and a throttle slider.

## Module Convention

Same as flight-controller:
1. Header exposes only `void <name>_setup(void);`
2. Source includes own header + `<pubsub.h>`, `<platform.h>`, `<macro.h>`, `<messages.h>`
3. State in file-scope `static` globals
4. `_setup()` subscribes to topics; called from `platform/module.c`
5. Runtime log activation via `NOTIFY_LOG_CLASS`

## DB Protocol (UART)

Sync: `'d','b'` | CMD_ID (1B) | LOG_CLASS (1B) | SIZE (2B LE) | PAYLOAD (0–120B) | CHECKSUM (2B LE)

| CMD ID | Constant | Direction | Purpose |
|--------|----------|-----------|---------|
| 0x00 | — | FC → PC | Log data (8 floats = 32 bytes) |
| 0x01 | `DB_CMD_THROTTLE` | PC → FC | Set throttle (1 float) |
| 0x03 | `DB_CMD_LOG_CLASS` | PC → FC | Select log class |
| 0x07 | `DB_CMD_RESET` | PC → FC | Hardware reset |
| 0xFF | — | Both | Heartbeat |

## Byte Buffer Alignment Rule

Same as flight-controller — **never cast `uint8_t*` to `float*`**. Use `memcpy()`.

```c
// ❌ HardFault on Cortex-M4
float *values = (float *)&data[4];

// ✅ Safe
float values[N];
memcpy(values, &data[4], N * sizeof(float));
```
