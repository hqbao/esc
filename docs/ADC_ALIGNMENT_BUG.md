# ADC Left-Alignment Bug

## Date
2026-04-08

## Status
**Fixed.** All channels (injected + regular) corrected with `>> 4`.

## Problem

The B-G431B-ESC1 CubeMX project configures both ADC1 and ADC2 with:
```c
hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
hadc2.Init.DataAlign = ADC_DATAALIGN_LEFT;
```

With 12-bit resolution and left alignment, the data registers (DR, JDR1–4) contain:
```
Bits [15:4] = 12-bit ADC result
Bits [3:0]  = 0 (zero-padded)
```

This means raw values range from **0 to 65520** (not 0 to 4095).

## Symptom

All conversion macros in `macro.h` assumed right-aligned data (`ADC_RESOLUTION = 4096`):
```c
#define ADC_TO_CURRENT(raw)  (((float)(raw) / 4096 * 3.3) / (0.003 * 16))
#define ADC_TO_VBUS(raw)     ((float)(raw) / 4096 * 3.3 / 0.09626)
```

With left-aligned raw values:
- **Currents**: Offsets calibrated to ~32768 instead of ~2048. After subtraction, current values were 16× too large (up to 35A from noise alone).
- **Bus voltage**: Raw ~22928 instead of ~1433, producing Vbus ≈ 192V instead of ~12V.

Observed in CP3 (Clarke verification): DC-offset currents (~0.5–1.0A instead of zero-centered), Vbus=0.2V (before any >>4 fix, the guard `> 1.0` blocked updates).

## Root Cause

CubeMX default for B-G431B-ESC1 is `ADC_DATAALIGN_LEFT`. The ST Motor Control Workbench firmware accounts for this, but our from-scratch code did not.

The `DataAlign` setting in `CFGR` register affects **both** the regular data register (DR) and the injected data registers (JDR1–4) on STM32G4.

## Fix

Right-shift all raw ADC reads by 4 in `platform_isr.c`:

```c
// Injected channels (phase currents) — 40 kHz
raw[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) >> 4;
raw[1] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1) >> 4;
raw[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2) >> 4;

// Regular channels (VBUS, temp) — 100 Hz sync poll
uint32_t dr = ADC1->DR;
uint16_t vbus_12bit = dr >> 4;
```

All downstream code (current_sense.c, voltage_monitor.c, macro.h) continues to use `ADC_RESOLUTION = 4096` unchanged.

## Alternative Fix (Not Chosen)

Change `ADC_RESOLUTION` to 65536 in macro.h. Rejected because it would require updating all conversion formulas and would be confusing (a 12-bit ADC claiming 16-bit resolution).

## Affected Files

- `platform/platform_isr.c` — Added `>> 4` to all ADC reads
- `platform/module.c` — Removed `HAL_ADC_Start_IT` (regular ADC now sync-polled)
- `modules/current_sense/current_sense.c` — No changes needed (receives corrected values)
- `modules/voltage_monitor/voltage_monitor.c` — No changes needed (receives corrected values)

## Lesson

Always check `DataAlign` in CubeMX when writing ADC code from scratch on STM32. The B-G431B-ESC1 kit defaults to left-aligned, which is unusual.
