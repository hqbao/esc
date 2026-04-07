#include "platform_hw.h"

// LED
void platform_led_on(void) {
    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
}

void platform_led_off(void) {
    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
}

void platform_led_toggle(void) {
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
}

// Delay
void platform_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

// Time
uint32_t platform_get_tick_ms(void) {
    return HAL_GetTick();
}

// Reset
void platform_reset(void) {
    NVIC_SystemReset();
}
