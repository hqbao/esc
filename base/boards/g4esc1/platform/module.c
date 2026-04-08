// Board-specific module initialization for B-G431B-ESC1
// Called from main.c — starts hardware peripherals, then calls all module _setup()

#include "platform_hw.h"
#include <platform.h>
#include <current_sense/current_sense.h>
#include <voltage_monitor/voltage_monitor.h>
#include <encoder/encoder.h>
#include <foc/foc.h>
#include <dblink/dblink.h>

// Start all analog hardware required before modules can run
static void hw_init(void) {
    // OPAMPs — current sense amplifiers (PGA x16)
    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);

    // Comparators — overcurrent protection → TIM1 BKIN
    HAL_COMP_Start(&hcomp1);
    HAL_COMP_Start(&hcomp2);
    HAL_COMP_Start(&hcomp4);

    // DAC3 — overcurrent thresholds
    // 0.003Ω shunt × gain 16 × 30A trip = 1.44V → 1788/4096 × 3.3V
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1788);
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 1788);
    HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac3, DAC_CHANNEL_2);

    // ADC calibration (must happen before any conversion, ADC disabled)
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
}

void platform_setup(void) {
    hw_init();

    // Module setup — each subscribes to PubSub topics
    current_sense_setup();
    voltage_monitor_setup();
    encoder_setup();
    foc_setup();
    dblink_setup();

    // Start ADC injected conversions (triggered by TIM1_CC4)
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);

    // Regular ADC (VBUS + temp) is polled synchronously at 100Hz in platform_isr.c

    // Start TIM6 scheduler (1kHz)
    HAL_TIM_Base_Start_IT(&htim6);

    // Start UART RX (IDLE-line DMA)
    platform_uart_start_rx();

    platform_led_on();
}
