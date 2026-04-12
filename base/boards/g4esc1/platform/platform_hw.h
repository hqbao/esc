#ifndef PLATFORM_HW_H
#define PLATFORM_HW_H

#include "main.h"

// HAL handles declared in main.c — extern here for platform drivers
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

// ── BEMF ADC sensing (6-step mode) ────────────────────────────────────────
// ADC2 DMA circular continuously samples BEMF voltage dividers:
//   CH17 (PA4)  = Phase U, CH5 (PC4) = Phase V, CH14 (PB11) = Phase W
// 10-bit resolution. TIM1 CC4 ISR snapshots the DMA buffer at each PWM valley.
extern volatile uint16_t g_bemf_dma[3];     // DMA target [U,V,W], 10-bit
extern volatile uint16_t g_bemf_threshold;  // ADC counts, set by foc.c
extern volatile uint16_t g_bemf_raw[3];     // Snapshot at PWM valley [U,V,W]

#endif // PLATFORM_HW_H
