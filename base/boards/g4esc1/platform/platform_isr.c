// ISR-to-PubSub bridge for B-G431B-ESC1
//
// HAL callbacks fire from IRQ handlers in stm32g4xx_it.c and are routed here.
// Each callback publishes the appropriate PubSub topic so modules stay decoupled.
//
// BEMF sensing: ADC2 DMA circular continuously fills g_bemf_dma[3] with 10-bit
// BEMF readings. TIM1 CC4 interrupt fires at 40kHz (PWM valley), snapshots the
// DMA buffer, and publishes virtual comparator results for the FOC module.

#include "platform_hw.h"
#include <pubsub.h>
#include <messages.h>
#include <string.h>

// ── BEMF ADC DMA buffer ───────────────────────────────────────────────────
// ADC2 DMA circular writes here continuously (10-bit, 3 channels).
// Channel order matches ADC2 regular sequence: CH17(U), CH5(V), CH14(W).
volatile uint16_t g_bemf_dma[3];

// Threshold and snapshot — read by foc.c for zero-crossing and logging
volatile uint16_t g_bemf_threshold = 512;   // Default ~Vbus/2 for 10-bit
volatile uint16_t g_bemf_raw[3];            // Snapshot at PWM valley [U,V,W]

// ── Scheduler tick ─────────────────────────────────────────────────────────
// TIM6 at 1kHz provides the scheduler timebase
static volatile uint32_t g_tick = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM6) return;

    g_tick++;
    publish(SCHEDULER_1KHZ, NULL, 0);

    if (g_tick % 10 == 0)   publish(SCHEDULER_100HZ, NULL, 0);
    if (g_tick % 40 == 0)   publish(SCHEDULER_25HZ,  NULL, 0);
    if (g_tick % 100 == 0)  publish(SCHEDULER_10HZ,  NULL, 0);
    if (g_tick % 1000 == 0) {
        publish(SCHEDULER_1HZ, NULL, 0);
        g_tick = 0;
    }
}

// ── TIM1 CC4 — 40kHz commutation tick at PWM valley ───────────────────────
// In center-aligned mode 1 with CCR4=1, CC4 fires near CNT=0 (valley),
// when the active high-side FET places the neutral point at ~Vbus/2 —
// correct timing for BEMF threshold comparison.

void TIM1_CC_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC4) &&
        __HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC4)) {
        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);

        // Snapshot BEMF values from DMA buffer
        uint16_t bemf_u = g_bemf_dma[0];  // CH17 = PA4  (Phase U)
        uint16_t bemf_v = g_bemf_dma[1];  // CH5  = PC4  (Phase V)
        uint16_t bemf_w = g_bemf_dma[2];  // CH14 = PB11 (Phase W)

        // Cache raw values for logging
        g_bemf_raw[0] = bemf_u;
        g_bemf_raw[1] = bemf_v;
        g_bemf_raw[2] = bemf_w;

        // Software zero-crossing comparison (same interface as before)
        uint16_t thr = g_bemf_threshold;
        uint8_t comp[3];
        comp[0] = (bemf_u > thr) ? 1 : 0;  // Phase U
        comp[1] = (bemf_v > thr) ? 1 : 0;  // Phase V
        comp[2] = (bemf_w > thr) ? 1 : 0;  // Phase W
        publish(ADC_INJECTED_COMPLETE, comp, 3);
    }
}

// ── ADC DMA complete — not used ────────────────────────────────────────────
// ADC2 runs continuously; DMA TC/HT interrupts are disabled after start.
// This callback is kept as a no-op in case HAL calls it unexpectedly.
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    (void)hadc;
}

// ── TIM1 break (overcurrent) ──────────────────────────────────────────────
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM1) return;
    // Overcurrent detected — FOC module handles emergency stop
    publish(FOC_RELEASE, NULL, 0);
}

// ── UART callbacks ────────────────────────────────────────────────────────
// Declared in platform_uart.c
extern void platform_uart_rx_event(uint16_t size);
extern void platform_uart_rx_error(void);
extern void platform_uart_tx_complete(void);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART2) {
        platform_uart_rx_event(Size);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        platform_uart_tx_complete();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        platform_uart_rx_error();
    }
}
