// Flash storage driver for STM32G431CBU6 (B-G431B-ESC1)
//
// Uses last page of flash (2KB) for parameter storage.
// STM32G431CB: 128KB flash, single bank, 64 pages × 2KB. Page 63 = 0x0801F800.
// G4 programs in doublewords (8 bytes), page erase ~1ms.

#include "platform_hw.h"
#include <platform.h>
#include <string.h>

#define STORAGE_FLASH_ADDRESS  0x0801F800
#define STORAGE_FLASH_PAGE     63
#define STORAGE_FLASH_BANK     FLASH_BANK_1

char platform_storage_read(uint16_t start, uint16_t size, uint8_t *data) {
    uint32_t addr = STORAGE_FLASH_ADDRESS + start;
    for (uint16_t i = 0; i < size; i++) {
        data[i] = *(volatile uint8_t *)(addr + i);
    }
    return PLATFORM_OK;
}

char platform_storage_write(uint16_t start, uint16_t size, uint8_t *data) {
    uint32_t addr = STORAGE_FLASH_ADDRESS + start;

    HAL_FLASH_Unlock();

    // Erase the page
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = STORAGE_FLASH_BANK;
    erase.Page      = STORAGE_FLASH_PAGE;
    erase.NbPages   = 1;

    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return PLATFORM_ERROR;
    }

    // Program doublewords (8 bytes at a time)
    for (uint16_t i = 0; i < size; i += 8) {
        uint64_t dword = 0xFFFFFFFFFFFFFFFFULL;  // erased default
        uint16_t bytes_to_copy = (size - i) < 8 ? (size - i) : 8;
        memcpy(&dword, &data[i], bytes_to_copy);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              addr + i, dword) != HAL_OK) {
            HAL_FLASH_Lock();
            return PLATFORM_ERROR;
        }
    }

    HAL_FLASH_Lock();
    return PLATFORM_OK;
}
