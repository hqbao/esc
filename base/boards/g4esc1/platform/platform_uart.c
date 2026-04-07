// USART2 driver for B-G431B-ESC1 (PB3 TX, PB4 RX)

#include "platform_hw.h"
#include <platform.h>

char platform_uart_send(uart_port_t port, uint8_t *data, uint16_t size) {
    (void)port; // only one UART
    if (HAL_UART_Transmit_DMA(&huart2, data, size) == HAL_OK) {
        return PLATFORM_OK;
    }
    return PLATFORM_ERROR;
}
