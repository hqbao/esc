# BEMF Sensing: Hardware Comparators vs ADC2

## Discovery

The B-G431B-ESC1 board (UM2516) has hardware comparators COMP1, COMP2, COMP4 — but they are **NOT connected to BEMF dividers**. They read the current-sense OPAmp outputs:

| Comparator | Pin | Actual Signal |
|------------|-----|---------------|
| COMP1 | PA1 | OPAMP1 output (Phase U current) |
| COMP2 | PA7 | OPAMP2 output (Phase V current) |
| COMP4 | PB0 | OPAMP3 output (Phase W current) |

The actual BEMF divider outputs are on **different pins**:

| Phase | BEMF Pin | ADC2 Channel |
|-------|----------|-------------|
| U | PA4 | ADC2_IN17 |
| V | PC4 | ADC2_IN5 |
| W | PB11 | ADC2_IN14 |

PB5 (`GPIO_BEMF`) is an enable/disable for the BEMF divider network — must be left as **input (high-Z)** to avoid biasing the dividers.

## Original Approach (Failed)

Hardware comparators (COMP1/2/4) + DAC3 threshold → reading current sense instead of BEMF → random ZC detections unrelated to motor position.

## Working Approach

ADC2 register-level initialization in `platform_isr.c`:
1. Configure PA4/PC4/PB11 as analog inputs
2. Configure PB5 as input (high-Z)
3. Initialize ADC2: deep power-down exit → regulator enable → calibration
4. Single-channel blocking reads (~0.4µs each) at PWM valley (TIM1_CC4 ISR)
5. Software comparison against threshold = `Vbus × 0.5 × BEMF_DIVIDER_RATIO`

## BEMF Divider Ratio

Empirical measurement during ALIGN phase: with motor locked and one phase driven at 12% duty, the floating phase ADC reading settles at ~3600/4095 when Vbus≈12V. This gives an effective ratio of **0.50** (Vbus/2 midpoint maps to ~half ADC range after the resistor divider).

Both RAMP and CLOSEDLOOP use the same threshold — level detection (not edge) with a 12-step sliding window to track ZC reliability.
