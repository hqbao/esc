# DMA UART TX Contention Bug

## Date
2026-04-08

## Status
**Fixed.**

## Summary

VBUS telemetry always read 0 despite the ADC returning correct values. Debugging spanned 15+ build/flash/test cycles across multiple dead-end theories (injected ADC channel config, register-level JSQR manipulation, extern globals, hardcoded constants). The root cause turned out to be **two interacting bugs** — neither in the ADC at all.

## Root Cause 1 — Wrong Log Class

The Python test tool (`quick_test.py`) was sending `LOG_CLASS_FOC` (1) to the flight controller, but the VBUS debug data was being logged under `LOG_CLASS_CURRENT` (2). The firmware's `on_send_log` callback uses a `switch` on the active log class, and different cases fill the 8-float payload with completely different data.

With `LOG_CLASS_FOC` active, the payload slots that the Python tool interpreted as "VBUS debug values" were actually `g_cal_enc[0..3]` — the encoder calibration array, which is all zeros until calibration runs. Every "register readback = 0" result was actually reading zeros from the wrong data source.

**Why it was hard to catch**: The symptom (always 0) was identical to "ADC not working." The log class mismatch was invisible because the DB protocol frame format is the same for all classes — 8 floats, same size, same structure. Nothing about the raw bytes says "you're reading the wrong class."

## Root Cause 2 — DMA TX Silently Drops Frames

After fixing the log class, VBUS was still 0 in some test configurations. The real issue:

Two modules both subscribed to `NOTIFY_LOG_CLASS` and both published to `SEND_LOG` when the active class was `LOG_CLASS_CURRENT`:

1. **`current_sense`** — published 12 bytes (3 raw phase currents)
2. **`foc`** — published 32 bytes (state, Iα, Iβ, enc_elec, VBUS, ...)

PubSub callbacks fire in subscription order. `current_sense` subscribed first, so its `SEND_LOG` publish fired first. The `dblink` module received the 12-byte payload, built a 20-byte frame, and started `HAL_UART_Transmit_DMA`. Microseconds later, `foc`'s 32-byte `SEND_LOG` arrived. `dblink` called `HAL_UART_Transmit_DMA` again — but the first transfer was still in progress. **HAL returned `HAL_BUSY` and silently discarded the second frame.**

Result: only the 12-byte `current_sense` frames reached the serial port. The 32-byte FOC frames (with VBUS data) were dropped every single time.

**Why it was hard to catch**: `HAL_UART_Transmit_DMA` returns `HAL_BUSY` but the return value was ignored (common pattern in embedded code). The serial port was still producing frames at the expected rate — they just contained the wrong module's data. A hex dump showed 20-byte frames instead of the expected 40-byte frames, but this was initially attributed to parser false syncs rather than DMA contention.

## The Debugging Red Herrings

Each of these led to dead-end investigations:

| Theory | Investigation | Actual Explanation |
|--------|--------------|-------------------|
| ADC injected channels misconfigured | Added VBUS as injected RANK_3, rewrote JSQR register | ADC was fine — wrong log class |
| HAL calibration clobbering ADC config | Re-called `HAL_ADCEx_Calibration_Start` in various orders | Calibration was fine |
| `JSQR` register not updating | Force-wrote JSQR via direct register access | JSQR was correct all along |
| Regular ADC channel broken | Switched from injected to regular polling | Both worked — data just wasn't reaching serial |
| PubSub broken | Tried extern global to bypass PubSub | PubSub was fine — DMA dropped the frame |
| Frame format wrong | Hardcoded `42.0f` in payload | Value was correct (verified with blocking TX) |
| Parser false syncs | `0x64 0x62` ('db') in payload data | Real problem — but secondary to DMA contention |

The breakthrough came when switching from DMA to **blocking** `HAL_UART_Transmit()` — suddenly TWO frame types appeared (12-byte and 32-byte), both with valid checksums. This proved:
1. Both modules were publishing correctly
2. The FOC data (including VBUS=7.87V) was present
3. DMA was the bottleneck

## Fix

Added a new log class to separate current_sense from FOC:

```c
// foundation/messages.h
#define LOG_CLASS_RAW_CURRENT  4   // NEW — raw phase currents (current_sense module)
```

Changed `current_sense.c` to use `LOG_CLASS_RAW_CURRENT` instead of `LOG_CLASS_CURRENT`. Now only one module publishes `SEND_LOG` per log class — no DMA contention.

## Affected Files

- `foundation/messages.h` — Added `LOG_CLASS_RAW_CURRENT = 4`
- `modules/current_sense/current_sense.c` — Changed log class from 2 to 4
- `platform/platform_uart.c` — Restored to DMA TX (was temporarily switched to blocking)
- `modules/dblink/dblink.c` — Removed debug byte in class field
- `tools/quick_test.py` — Added checksum validation
- `tools/cp3_clarke_verify.py` — Added checksum validation, fixed display code

## Lessons

1. **Always validate the log class selector end-to-end.** When the test tool sends a log class ID and receives 8 floats back, there's no in-band indication of *which* switch-case produced them. A class mismatch is invisible at the byte level.

2. **Never ignore `HAL_UART_Transmit_DMA` return value in systems where multiple publishers share a UART.** If the TX DMA is busy, the frame is silently lost. Options:
   - Check return value and queue/retry
   - Use a TX queue with a single DMA consumer
   - Ensure only one publisher can fire per TX cycle

3. **Add checksum validation to all serial parsers.** Without it, false sync bytes in payload data cause the parser to construct garbage frames that look structurally valid but contain wrong data. This was a secondary source of confusion during debugging.

4. **When "everything reads 0," suspect the data routing before the hardware.** In a PubSub system, the sensor might be working perfectly while the wrong callback, wrong topic, or a transport-layer drop prevents the data from reaching the output.

5. **Switching DMA to blocking TX is a powerful diagnostic.** It eliminates the "HAL_BUSY silent drop" variable and reveals whether the data is actually being generated correctly.
