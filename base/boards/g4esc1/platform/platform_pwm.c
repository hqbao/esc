// TIM1 3-phase complementary PWM for B-G431B-ESC1
// TIM1_CH1/CH1N = Phase U (PA8/PC13)
// TIM1_CH2/CH2N = Phase V (PA9/PA12)
// TIM1_CH3/CH3N = Phase W (PA10/PB15)

#include "platform_hw.h"
#include <platform.h>

void platform_pwm_set_duty(pwm_phase_t phase, uint32_t duty) {
    switch (phase) {
        case PWM_PHASE_U: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty); break;
        case PWM_PHASE_V: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty); break;
        case PWM_PHASE_W: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty); break;
    }
}

void platform_pwm_start(void) {
    // Start all 3 channels + complementary outputs
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    // Start OC4 for ADC trigger (TRGO = OC4REF)
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
}

void platform_pwm_stop(void) {
    HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

void platform_pwm_enable_phase(pwm_phase_t phase) {
    uint32_t channel;
    switch (phase) {
        case PWM_PHASE_U: channel = TIM_CHANNEL_1; break;
        case PWM_PHASE_V: channel = TIM_CHANNEL_2; break;
        case PWM_PHASE_W: channel = TIM_CHANNEL_3; break;
        default: return;
    }
    HAL_TIM_PWM_Start(&htim1, channel);
    HAL_TIMEx_PWMN_Start(&htim1, channel);
}

void platform_pwm_disable_phase(pwm_phase_t phase) {
    uint32_t channel;
    switch (phase) {
        case PWM_PHASE_U: channel = TIM_CHANNEL_1; break;
        case PWM_PHASE_V: channel = TIM_CHANNEL_2; break;
        case PWM_PHASE_W: channel = TIM_CHANNEL_3; break;
        default: return;
    }
    HAL_TIMEx_PWMN_Stop(&htim1, channel);
    HAL_TIM_PWM_Stop(&htim1, channel);
}
