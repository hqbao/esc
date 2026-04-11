#ifndef PLATFORM_HW_H
#define PLATFORM_HW_H

#include "main.h"

// HAL handles declared in main.c — extern here for platform drivers
extern ADC_HandleTypeDef hadc1;
extern COMP_HandleTypeDef hcomp1;
extern COMP_HandleTypeDef hcomp2;
extern COMP_HandleTypeDef hcomp4;
extern DAC_HandleTypeDef hdac3;
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

// ── BEMF ADC sensing (6-step mode) ────────────────────────────────────────
// B-G431B-ESC1 BEMF voltage dividers are on PA4/PC4/PB11 (ADC2 channels),
// NOT on PA1/PA7/PB0 (which are current sense OPAmp inputs).
// Software comparison replaces hardware comparators for zero-crossing.
void bemf_adc_init(void);
extern volatile uint16_t g_bemf_threshold;  // ADC counts, set by foc.c
extern volatile uint16_t g_bemf_raw[3];     // Last ADC readings [U,V,W]

#endif // PLATFORM_HW_H
