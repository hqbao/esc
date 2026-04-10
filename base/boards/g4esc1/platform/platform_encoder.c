// Bit-bang SPI GPIO driver for AS5048A encoder on B-G431B-ESC1
//
// Pin mapping:
//   PB6  = SCK   (J8 pin 1)
//   PB7  = MOSI  (J8 pin 2)
//   PB8  = MISO  (J8 pin 3)
//   PA15 = CS    (PWM input pad)

#include "platform_hw.h"
#include <platform.h>

// GPIO port/pin defines
#define ENC_SCK_PORT   GPIOB
#define ENC_SCK_PIN    GPIO_PIN_6
#define ENC_MOSI_PORT  GPIOB
#define ENC_MOSI_PIN   GPIO_PIN_7
#define ENC_MISO_PORT  GPIOB
#define ENC_MISO_PIN   GPIO_PIN_8
#define ENC_CS_PORT    GPIOA
#define ENC_CS_PIN     GPIO_PIN_15

void platform_encoder_init(void) {
    // Enable clocks (should already be on from CubeMX, but ensure)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Disable TIM2 — CubeMX configures PA15 as TIM2_CH1 (PWM input capture).
    // We reuse PA15 as encoder CS. Fully disable TIM2 to prevent any
    // interference with GPIO output on PA15.
    TIM2->CR1 = 0;              // stop counter
    TIM2->DIER = 0;             // disable all TIM2 interrupts
    TIM2->SR = 0;               // clear all pending flags
    NVIC_DisableIRQ(TIM2_IRQn); // disable NVIC vector

    GPIO_InitTypeDef gpio = {0};

    // SCK — push-pull output, fast
    gpio.Pin   = ENC_SCK_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(ENC_SCK_PORT, &gpio);
    HAL_GPIO_WritePin(ENC_SCK_PORT, ENC_SCK_PIN, GPIO_PIN_RESET); // CPOL=0

    // MOSI — push-pull output, fast
    gpio.Pin = ENC_MOSI_PIN;
    HAL_GPIO_Init(ENC_MOSI_PORT, &gpio);
    HAL_GPIO_WritePin(ENC_MOSI_PORT, ENC_MOSI_PIN, GPIO_PIN_RESET);

    // CS — push-pull output, fast, idle high
    gpio.Pin = ENC_CS_PIN;
    HAL_GPIO_Init(ENC_CS_PORT, &gpio);
    HAL_GPIO_WritePin(ENC_CS_PORT, ENC_CS_PIN, GPIO_PIN_SET); // CS idle high

    // MISO — input with pull-up
    gpio.Pin  = ENC_MISO_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ENC_MISO_PORT, &gpio);
}

void platform_encoder_cs_low(void) {
    ENC_CS_PORT->BRR = ENC_CS_PIN;  // atomic reset
}

void platform_encoder_cs_high(void) {
    ENC_CS_PORT->BSRR = ENC_CS_PIN;  // atomic set
}

void platform_encoder_sck_high(void) {
    ENC_SCK_PORT->BSRR = ENC_SCK_PIN;
}

void platform_encoder_sck_low(void) {
    ENC_SCK_PORT->BRR = ENC_SCK_PIN;
}

void platform_encoder_mosi_high(void) {
    ENC_MOSI_PORT->BSRR = ENC_MOSI_PIN;
}

void platform_encoder_mosi_low(void) {
    ENC_MOSI_PORT->BRR = ENC_MOSI_PIN;
}

uint8_t platform_encoder_miso_read(void) {
    return (ENC_MISO_PORT->IDR & ENC_MISO_PIN) ? 1 : 0;
}
