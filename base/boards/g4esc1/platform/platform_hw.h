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

#endif // PLATFORM_HW_H
