// Board-specific module initialization for B-G431B-ESC1 (6-Step sensorless)
// Called from main.c — starts hardware peripherals, then calls all module _setup()

#include "platform_hw.h"
#include <platform.h>
#include <foc/foc.h>
#include <dblink/dblink.h>


// Start only the hardware needed for 6-step sensorless BEMF commutation
static void hw_init(void) {
    // ADC2 calibration (must happen before any conversion, ADC disabled)
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    // PB5 (GPIO_BEMF): drive LOW to ground the BEMF divider reference.
    // CubeMX already configures PB5 as output-low in MX_GPIO_Init(), so this
    // is just explicit confirmation. Do NOT set to input (high-Z) — the divider
    // needs a defined reference.
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}

void platform_setup(void) {
    hw_init();

    // Module setup — each subscribes to PubSub topics
    foc_setup();
    dblink_setup();

    // Start ADC2 DMA circular — continuously samples BEMF channels
    extern volatile uint16_t g_bemf_dma[3];
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)g_bemf_dma, 3);
    // Disable DMA transfer interrupts — buffer is read on demand at PWM valley
    __HAL_DMA_DISABLE_IT(hadc2.DMA_Handle, DMA_IT_TC | DMA_IT_HT);

    // Enable TIM1 CC4 interrupt for 40kHz commutation tick at PWM valley
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);

    // Start TIM6 scheduler (1kHz)
    HAL_TIM_Base_Start_IT(&htim6);

    // Start UART RX (IDLE-line DMA)
    platform_uart_start_rx();

    platform_led_on();
}
