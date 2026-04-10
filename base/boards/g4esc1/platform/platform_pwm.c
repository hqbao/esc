// TIM1 3-phase complementary PWM for B-G431B-ESC1
// TIM1_CH1/CH1N = Phase U (PA8/PC13)
// TIM1_CH2/CH2N = Phase V (PA9/PA12)
// TIM1_CH3/CH3N = Phase W (PA10/PB15)

#include "platform_hw.h"
#include <platform.h>

static TIM_HandleTypeDef *g_pwm_timers[3] = {&htim1, &htim1, &htim1};
static uint32_t g_pwm_channels[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};

char platform_pwm_init(pwm_port_t port) {
    HAL_TIM_PWM_Start(g_pwm_timers[port], g_pwm_channels[port]);
    HAL_TIMEx_PWMN_Start(g_pwm_timers[port], g_pwm_channels[port]);
    return PLATFORM_OK;
}

char platform_pwm_send(pwm_port_t port, uint32_t data) {
    TIM_TypeDef *base = g_pwm_timers[port]->Instance;
    switch (g_pwm_channels[port]) {
    case TIM_CHANNEL_1: base->CCR1 = data; break;
    case TIM_CHANNEL_2: base->CCR2 = data; break;
    case TIM_CHANNEL_3: base->CCR3 = data; break;
    default: break;
    }
    return PLATFORM_OK;
}

void platform_pwm_start(void) {
    for (int i = 0; i < 3; i++) {
        HAL_TIM_PWM_Start(g_pwm_timers[i], g_pwm_channels[i]);
        HAL_TIMEx_PWMN_Start(g_pwm_timers[i], g_pwm_channels[i]);
    }
    // OC4 for ADC trigger (TRGO = OC4REF)
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
}

void platform_pwm_stop(void) {
    HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);
    for (int i = 2; i >= 0; i--) {
        HAL_TIMEx_PWMN_Stop(g_pwm_timers[i], g_pwm_channels[i]);
        HAL_TIM_PWM_Stop(g_pwm_timers[i], g_pwm_channels[i]);
    }
}
