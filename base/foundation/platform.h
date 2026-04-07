#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stddef.h>

#define PLATFORM_OK    0
#define PLATFORM_ERROR -1

typedef enum {
    PWM_PHASE_U = 0,
    PWM_PHASE_V,
    PWM_PHASE_W,
} pwm_phase_t;

typedef enum {
    UART_PORT1 = 0,
} uart_port_t;

// Motor PWM
void platform_pwm_set_duty(pwm_phase_t phase, uint32_t duty);
void platform_pwm_start(void);
void platform_pwm_stop(void);
void platform_pwm_enable_phase(pwm_phase_t phase);
void platform_pwm_disable_phase(pwm_phase_t phase);

// Encoder (bit-bang SPI)
void platform_encoder_init(void);
void platform_encoder_cs_low(void);
void platform_encoder_cs_high(void);
void platform_encoder_sck_high(void);
void platform_encoder_sck_low(void);
void platform_encoder_mosi_high(void);
void platform_encoder_mosi_low(void);
uint8_t platform_encoder_miso_read(void);

// UART
char platform_uart_send(uart_port_t port, uint8_t *data, uint16_t size);

// System
void platform_led_on(void);
void platform_led_off(void);
void platform_led_toggle(void);
void platform_delay_ms(uint32_t ms);
uint32_t platform_get_tick_ms(void);
void platform_reset(void);

// Board init (called from main, calls all module _setup() functions)
void platform_setup(void);

#endif // PLATFORM_H
