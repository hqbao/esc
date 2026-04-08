// USART2 driver for B-G431B-ESC1 (PB3 TX, PB4 RX)
// RX: IDLE-line DMA with circular buffer → publishes UART_RAW_RECEIVED

#include "platform_hw.h"
#include <platform.h>
#include <pubsub.h>
#include <string.h>

#define UART_RX_BUF_SIZE 32

static uint8_t g_rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t g_last_pos = 0;

char platform_uart_send(uart_port_t port, uint8_t *data, uint16_t size) {
    (void)port; // only one UART
    if (HAL_UART_Transmit_DMA(&huart2, data, size) == HAL_OK) {
        return PLATFORM_OK;
    }
    return PLATFORM_ERROR;
}

void platform_uart_start_rx(void) {
    g_last_pos = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_rx_buf, UART_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

// Called from HAL_UARTEx_RxEventCallback in platform_isr.c
void platform_uart_rx_event(uint16_t size) {
    uint16_t start = g_last_pos;
    if (size > start) {
        publish(UART_RAW_RECEIVED, &g_rx_buf[start], size - start);
    } else if (size < start) {
        // Wrap-around
        publish(UART_RAW_RECEIVED, &g_rx_buf[start], UART_RX_BUF_SIZE - start);
        if (size > 0) {
            publish(UART_RAW_RECEIVED, &g_rx_buf[0], size);
        }
    }
    g_last_pos = size % UART_RX_BUF_SIZE;
}

void platform_uart_rx_error(void) {
    g_last_pos = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_rx_buf, UART_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}
