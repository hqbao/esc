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

    if (g_tick % 10 == 0)  publish(SCHEDULER_100HZ, NULL, 0);
    if (g_tick % 40 == 0)  publish(SCHEDULER_25HZ,  NULL, 0);
    if (g_tick % 100 == 0) publish(SCHEDULER_10HZ,  NULL, 0);
    if (g_tick % 1000 == 0) {
        publish(SCHEDULER_1HZ, NULL, 0);
        g_tick = 0;
    }
}

// ── ADC injected complete ──────────────────────────────────────────────────
// Triggered by TIM1_CC4 at the PWM center point (40kHz)
// Reads 3-phase current from ADC1/ADC2 injected channels
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance != ADC1) return;  // ADC1 triggers first, read both

    uint16_t raw[3];
    raw[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1); // Phase U
    raw[1] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1); // Phase V
    raw[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2); // Phase W

    publish(ADC_INJECTED_COMPLETE, (uint8_t *)raw, sizeof(raw));
}

// ── ADC regular complete ───────────────────────────────────────────────────
// Bus voltage (CH1) and temperature (CH5) from ADC1 regular group
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance != ADC1) return;

    uint16_t raw[2];
    raw[0] = HAL_ADC_GetValue(&hadc1); // VBUS
    // Temperature requires a second read — start regular conversion in scheduler
    raw[1] = 0; // placeholder — filled when second channel is read

    publish(ADC_REGULAR_COMPLETE, (uint8_t *)raw, sizeof(raw));
}

// ── TIM1 break (overcurrent) ──────────────────────────────────────────────
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM1) return;
    // Overcurrent detected — FOC module handles emergency stop
    publish(FOC_RELEASE, NULL, 0);
}
