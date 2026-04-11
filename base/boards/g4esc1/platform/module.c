// Board-specific module initialization for B-G431B-ESC1 (6-Step sensorless)
// Called from main.c — starts hardware peripherals, then calls all module _setup()

#include "platform_hw.h"
#include <platform.h>
#include <voltage_monitor/voltage_monitor.h>
#include <foc/foc.h>
#include <dblink/dblink.h>


// Start only the hardware needed for 6-step sensorless BEMF commutation
static void hw_init(void) {
    // NO OPAMPs — they share PA1/PA7/PB0 with BEMF voltage dividers.
    // Starting OPAMPs overrides the BEMF signal on comparator inputs.

    // Comparators and DAC3 are managed by foc.c at motor start/stop.
    // Do NOT start them here — foc_6step sets DAC3 to Vbus/2 (BEMF threshold),
    // NOT the overcurrent threshold (1788) that was previously here.

    // ADC calibration (must happen before any conversion, ADC disabled)
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
}

void platform_setup(void) {
    hw_init();

    // Module setup — each subscribes to PubSub topics
    voltage_monitor_setup();
    foc_setup();
    dblink_setup();

    // Start ADC injected conversions (triggered by TIM1_CC4)
    // Provides the 40kHz tick for commutation even without current sensing
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    // Start TIM6 scheduler (1kHz)
    HAL_TIM_Base_Start_IT(&htim6);

    // Start UART RX (IDLE-line DMA)
    platform_uart_start_rx();

    platform_led_on();
}
