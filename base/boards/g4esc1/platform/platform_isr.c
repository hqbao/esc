// ISR-to-PubSub bridge for B-G431B-ESC1
//
// HAL callbacks fire from IRQ handlers in stm32g4xx_it.c and are routed here.
// Each callback publishes the appropriate PubSub topic so modules stay decoupled.

#include "platform_hw.h"
#include <pubsub.h>
#include <messages.h>
#include <string.h>

// ── BEMF ADC sensing ───────────────────────────────────────────────────────
// B-G431B-ESC1 BEMF voltage dividers output to:
//   BEMF1 (Phase U) = PA4  = ADC2_IN17
//   BEMF2 (Phase V) = PC4  = ADC2_IN5
//   BEMF3 (Phase W) = PB11 = ADC2_IN14
// GPIO_BEMF (PB5) must be HIGH to enable the BEMF divider circuit.

volatile uint16_t g_bemf_threshold = 1241;  // Default for ~11V bus
volatile uint16_t g_bemf_raw[3];            // Last ADC readings [U,V,W]

// Quick single-channel ADC2 read (blocking, ~0.4µs per read)
static inline uint16_t adc2_read_channel(uint32_t channel) {
    ADC2->SQR1 = (channel << ADC_SQR1_SQ1_Pos);  // 1 conversion
    ADC2->CR |= ADC_CR_ADSTART;
    while (!(ADC2->ISR & ADC_ISR_EOC)) {}
    uint16_t val = (uint16_t)(ADC2->DR & 0x0FFF);
    return val;
}

void bemf_adc_init(void) {
    // ── GPIO: set BEMF pins to analog mode ──
    // PA4 (BEMF1): GPIOA MODER[9:8] = 0b11
    GPIOA->MODER |= (3u << (4 * 2));
    // PC4 (BEMF2): GPIOC MODER[9:8] = 0b11
    GPIOC->MODER |= (3u << (4 * 2));
    // PB11 (BEMF3): GPIOB MODER[23:22] = 0b11
    GPIOB->MODER |= (3u << (11 * 2));

    // ── PB5 (GPIO_BEMF): leave as input (high-Z) ──
    // PB5=HIGH biases BEMF dividers through Schottky diodes (inflated readings).
    // PB5=LOW pulls them down via forward-biased diode.
    // Input mode (default after reset) = no interference with divider.
    // Explicitly set to input just in case something else configured it.
    GPIOB->MODER &= ~(3u << (5 * 2));  // Input mode (00)

    // ── ADC2 initialization (register-level, clock already enabled by ADC1 MSP) ──
    ADC2->CR &= ~ADC_CR_DEEPPWD;       // Exit deep power-down
    ADC2->CR |= ADC_CR_ADVREGEN;       // Enable voltage regulator
    for (volatile int i = 0; i < 4000; i++) {}  // Wait ~20µs for regulator

    // Calibrate single-ended
    ADC2->CR &= ~ADC_CR_ADCALDIF;
    ADC2->CR |= ADC_CR_ADCAL;
    while (ADC2->CR & ADC_CR_ADCAL) {}

    // Configure: 12-bit, right-aligned, single conversion, software trigger
    ADC2->CFGR = 0;
    ADC2->SMPR1 = 0;  // 2.5 cycles for channels 0-9
    ADC2->SMPR2 = 0;  // 2.5 cycles for channels 10-18

    // Enable ADC2
    ADC2->ISR = ADC_ISR_ADRDY;
    ADC2->CR |= ADC_CR_ADEN;
    while (!(ADC2->ISR & ADC_ISR_ADRDY)) {}
}

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
// Reads BEMF voltages via ADC2 and compares against threshold in software.

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance != ADC1) return;

    // Read BEMF voltages via ADC2 at PWM valley (center of ON window)
    uint16_t bemf_u = adc2_read_channel(17);  // PA4  = BEMF1 (Phase U)
    uint16_t bemf_v = adc2_read_channel(5);   // PC4  = BEMF2 (Phase V)
    uint16_t bemf_w = adc2_read_channel(14);  // PB11 = BEMF3 (Phase W)

    // Cache raw values for logging
    g_bemf_raw[0] = bemf_u;
    g_bemf_raw[1] = bemf_v;
    g_bemf_raw[2] = bemf_w;

    // Software zero-crossing comparison (same interface as hardware comparators)
    uint16_t thr = g_bemf_threshold;
    uint8_t comp[3];
    comp[0] = (bemf_u > thr) ? 1 : 0;  // Phase U
    comp[1] = (bemf_v > thr) ? 1 : 0;  // Phase V
    comp[2] = (bemf_w > thr) ? 1 : 0;  // Phase W
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
