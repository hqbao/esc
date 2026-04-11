// ISR-to-PubSub bridge for B-G431B-ESC1
//
// HAL callbacks fire from IRQ handlers in stm32g4xx_it.c and are routed here.
// Each callback publishes the appropriate PubSub topic so modules stay decoupled.

#include "platform_hw.h"
#include <pubsub.h>
#include <messages.h>
#include <string.h>

// ── Scheduler tick ─────────────────────────────────────────────────────────
// TIM6 at 1kHz provides the scheduler timebase
static volatile uint32_t g_tick = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM6) return;

    g_tick++;
    publish(SCHEDULER_1KHZ, NULL, 0);

    if (g_tick % 10 == 0) {
        publish(SCHEDULER_100HZ, NULL, 0);

        // Poll VBUS via regular group — direct register access to avoid
        // HAL state machine conflict with active injected conversions.
        ADC1->CR |= ADC_CR_ADSTART;
        for (volatile int i = 0; i < 500; i++) {}       // ~5µs at 170MHz
        if (ADC1->ISR & ADC_ISR_EOC) {
            uint16_t vbus_raw = (uint16_t)(ADC1->DR & 0x0FFF); // right-aligned 12-bit
            ADC1->ISR = ADC_ISR_EOC;
            publish(ADC_REGULAR_COMPLETE, (uint8_t *)&vbus_raw, sizeof(vbus_raw));
        }
    }
    if (g_tick % 40 == 0)  publish(SCHEDULER_25HZ,  NULL, 0);
    if (g_tick % 100 == 0) publish(SCHEDULER_10HZ,  NULL, 0);
    if (g_tick % 1000 == 0) {
        publish(SCHEDULER_1HZ, NULL, 0);
        g_tick = 0;
    }
}

// ── ADC injected complete ──────────────────────────────────────────────────
// Triggered by TIM1_CC4 at the PWM valley (40kHz) — provides commutation tick.
// OPAMPs are disabled (6-step mode), so no current values to read.

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance != ADC1) return;
    // Capture comparator outputs NOW, at the PWM valley (center of ON window).
    // Reading them later in on_tick adds ~4μs latency, pushing past the ON edge
    // at low duty cycles and giving wrong results.
    uint8_t comp[3];
    comp[0] = (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUT_LEVEL_HIGH) ? 1 : 0;
    comp[1] = (HAL_COMP_GetOutputLevel(&hcomp2) == COMP_OUTPUT_LEVEL_HIGH) ? 1 : 0;
    comp[2] = (HAL_COMP_GetOutputLevel(&hcomp4) == COMP_OUTPUT_LEVEL_HIGH) ? 1 : 0;
    publish(ADC_INJECTED_COMPLETE, comp, 3);
}

// ── ADC regular complete ───────────────────────────────────────────────────
// Handled synchronously in the 100Hz scheduler tick (see above).
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
