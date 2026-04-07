#ifndef PLATFORM_COMMON_H
#define PLATFORM_COMMON_H

#include <stdint.h>

void platform_led_on(void);
void platform_led_off(void);
void platform_led_toggle(void);
void platform_delay_ms(uint32_t ms);
uint32_t platform_get_tick_ms(void);
void platform_reset(void);

#endif // PLATFORM_COMMON_H
