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
            uint16_t vbus_raw = (uint16_t)(ADC1->DR >> 4); // left-aligned → 12-bit
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
// Triggered by TIM1_CC4 at the PWM center point (40kHz)
// Reads 3-phase current + VBUS from ADC1/ADC2 injected channels
// NOTE: ADC DataAlign=LEFT → raw values are left-shifted by 4, so >>4 to get 12-bit

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance != ADC1) return;  // ADC1 triggers first, read both

    uint16_t raw[3];
    raw[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) >> 4; // Phase U (OPAMP1→ADC1 CH3)
    raw[1] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1) >> 4; // Phase V (OPAMP2→ADC2 CH3)
    raw[2] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2) >> 4; // Phase W (OPAMP3→ADC2 internal)

    publish(ADC_INJECTED_COMPLETE, (uint8_t *)raw, sizeof(raw));
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

// ── UART RX (IDLE line detection) ─────────────────────────────────────────
// Declared in platform_uart.c
extern void platform_uart_rx_event(uint16_t size);
extern void platform_uart_rx_error(void);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART2) {
        platform_uart_rx_event(Size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        platform_uart_rx_error();
    }
}
