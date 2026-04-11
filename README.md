# ESC — Brushless Motor Controller

6-step trapezoidal (120° block) BLDC motor controller for the **B-G431B-ESC1** discovery kit (STM32G431CBU6).
Built from scratch with the same PubSub architecture as the [flight-controller](../flight-controller/) project.

Sensorless BEMF zero-crossing detection via **ADC2** on the hardware BEMF divider pins (PA4/PC4/PB11).
Startup: forced ALIGN → RAMP → CLOSEDLOOP with ZC-health gated speed control.

## Hardware

| Item | Detail |
|------|--------|
| Board | B-G431B-ESC1 (MB1419) |
| MCU | STM32G431CBU6, Cortex-M4F, 170 MHz |
| Flash / RAM | 128 KB / 32 KB |
| Current sense | 3-shunt (3 mΩ), OPAMP PGA ×16 |
| BEMF sensing | ADC2: PA4 (Phase U), PC4 (Phase V), PB11 (Phase W) |
| BEMF divider ratio | 0.50 (empirical, threshold = Vbus/2 × 0.50) |
| Encoder | AS5048A (14-bit, bit-bang SPI) — unused in sensorless mode |
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
│   │       │   ├── platform_pwm.c  # PWM output
│   │       │   ├── platform_encoder.c  # AS5048A SPI bit-bang
│   │       │   ├── platform_uart.c # UART DMA + IDLE detection
│   │       │   ├── platform_common.c   # LED, delay, time, reset
│   │       │   └── platform_hw.h   # Extern HAL handle declarations
│   │       ├── Makefile            # Standalone ARM GCC build
│   │       ├── build.sh            # Build script (accepts mode arg)
│   │       └── build-flash.sh      # Build + ST-Link flash
│   └── foundation/                 # Shared core
│       ├── pubsub.h / pubsub.c    # Topic-based pub/sub
│       ├── messages.h              # Message structs + log classes
│       ├── macro.h                 # Constants + conversion macros
│       ├── platform.h              # Platform abstraction API
│       └── pi_controller.h        # Generic PI controller
├── modules/
│   ├── config/          # Runtime configuration (motor params, PID gains)
│   ├── current_sense/   # 3-phase shunt current → Amps
│   ├── dblink/          # UART protocol (DB framing) + commands
│   ├── encoder/         # AS5048A → electrical/mechanical angle
│   ├── foc/             # FOC: encoder / BEMF / no-feedback modes
│   ├── local_storage/   # Flash storage with CRC32 integrity
│   └── voltage_monitor/ # Bus voltage + temperature
│   Each module has its own README.md with data flow, PubSub interface, and configuration.
├── tools/                          # Python visualization & test tools
│   └── foc_6step_view.py           # 6-step commutation: state, ZC, speed, duty
└── docs/
    ├── ADC_ALIGNMENT_BUG.md        # Left-aligned ADC data issue
    ├── BEMF_HARDWARE_DISCOVERY.md  # BEMF pins: COMPs vs ADC2
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

### Commutation Loop (40 kHz)

```
ADC injected ISR (TIM1_CC4)
  → current_sense: raw ADC → calibrated Amps → SENSOR_PHASE_CURRENT
  → BEMF ADC2: read floating phase → compare vs threshold → cache comp[3]
    → foc (on_tick): apply_step → ZC detection → forced commutation → speed control
```

6-step trapezoidal: only 2 of 3 phases driven at any time, 3rd floats. BEMF zero-crossing on the floating phase triggers timing validation.

### State Machine

| State | ID | Description |
|-------|----|-------------|
| IDLE | 0 | Motor stopped, PWM off |
| ALIGN | 1 | Lock rotor at 0° (200ms, 12% duty) |
| RAMP | 2 | Forced commutation ramp-up (~4s, period 1600→320) |
| CLOSEDLOOP | 3 | Forced timing + ZC-health gated speed control |

### Log Classes

| ID | Constant | Payload |
|----|----------|---------|
| 1 | `LOG_CLASS_FOC` | enc_elec, enc_mech, theta, offset, ... |
| 2 | `LOG_CLASS_CURRENT` | state, Iα, Iβ, enc_elec, Vbus, ... |
| 3 | `LOG_CLASS_COMMUTATION` | state, step, speed, duty, Vbus, step_period, zc_count, ramp_period, bemf_raw[3], zc_map, zc_win |
| 4 | `LOG_CLASS_RAW_CURRENT` | Ia, Ib, Ic (raw phase currents) |

## Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `PWM_FREQ` | 40 kHz | Center-aligned PWM frequency |
| `PWM_PERIOD` | 2125 | Timer period (170 MHz / 2 / 40 kHz) |
| `SHUNT_RESISTANCE` | 0.003 Ω | Current shunt value |
| `OPAMP_GAIN` | 16.0 | Internal OPAMP PGA gain |
| `ADC_VREF` | 3.3 V | ADC reference voltage |
| `ADC_RESOLUTION` | 4096 | 12-bit ADC |
| `VBUS_DIVIDER_RATIO` | 0.09626 | R1=169k, R2=18k |
| `BEMF_DIVIDER_RATIO` | 0.50 | Empirical — threshold = Vbus × 0.5 × 0.50 |
| `ALIGN_TICKS` | 8000 | 200ms at 40kHz |
| `ALIGN_DUTY` | 0.12 | 12% alignment pulse |
| `RAMP_INITIAL_PERIOD` | 1600 | 40ms initial step |
| `RAMP_FINAL_PERIOD` | 320 | 8ms target (handoff speed) |
| `RAMP_ACCEL` | 10 | Period decrease per step → ~128 steps, ~4s ramp |
| `RAMP_DUTY` | 0.20 | 20% duty during ramp |
| `ZC_BLANKING_PCT` | 0.10 | Ignore first 10% of step (PWM noise) |
| `ZC_MISS_MAX` | 6 | Consecutive misses → stall protection |

## ADC Architecture

**CRITICAL**: CubeMX configures `DataAlign = ADC_DATAALIGN_LEFT` for both ADC1 and ADC2. All raw values from DR/JDR registers are **left-shifted by 4** and must be right-shifted (`>> 4`) before use. See [docs/ADC_ALIGNMENT_BUG.md](docs/ADC_ALIGNMENT_BUG.md).

- **Injected group** (40 kHz, TIM1_CC4 triggered): 3 phase currents via OPAMPs
- **Regular group** (100 Hz, software polled): Bus voltage + temperature
- **ADC2** (40 kHz, read in TIM1_CC4 ISR): BEMF on floating phase (PA4/PC4/PB11), software comparison against threshold

**CRITICAL**: Hardware comparators (COMP1/2/4) on this board read **current-sense OPAmp outputs**, NOT BEMF dividers. See [docs/BEMF_HARDWARE_DISCOVERY.md](docs/BEMF_HARDWARE_DISCOVERY.md).

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
| `foc_6step_view.py` | 6-step commutation: state, ZC, speed, duty |

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
