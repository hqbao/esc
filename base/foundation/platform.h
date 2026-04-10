#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stddef.h>

#define PLATFORM_OK    0
#define PLATFORM_ERROR -1

typedef enum {
    PWM_PORT1 = 0,
    PWM_PORT2,
    PWM_PORT3,
} pwm_port_t;

typedef enum {
    UART_PORT1 = 0,
} uart_port_t;

// Motor PWM
char platform_pwm_init(pwm_port_t port);
char platform_pwm_send(pwm_port_t port, uint32_t data);
void platform_pwm_start(void);
void platform_pwm_stop(void);

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
void platform_uart_start_rx(void);

// System
void platform_led_on(void);
void platform_led_off(void);
void platform_led_toggle(void);
void platform_delay_ms(uint32_t ms);
uint32_t platform_get_tick_ms(void);
void platform_reset(void);

// Flash storage
char platform_storage_read(uint16_t start, uint16_t size, uint8_t *data);
char platform_storage_write(uint16_t start, uint16_t size, uint8_t *data);

// Board init (called from main, calls all module _setup() functions)
void platform_setup(void);

#endif // PLATFORM_H
