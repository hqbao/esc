######################################################################
# Makefile for B-G431B-ESC1 (STM32G431CBU6)
# Usage: make -j8 [clean]
######################################################################

# Target
TARGET = g4esc1

# Build directory
BUILD_DIR = build

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

# MCU flags
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

######################################################################
# Sources
######################################################################

# Paths relative to board directory
FOUNDATION_DIR = ../../foundation
MODULES_DIR = ../../../modules

# VPATH for out-of-tree sources
VPATH += $(FOUNDATION_DIR)
VPATH += $(MODULES_DIR)/foc
VPATH += $(MODULES_DIR)/voltage_monitor
VPATH += $(MODULES_DIR)/dblink

# C sources — local (relative to board dir)
C_SOURCES = \
Core/Src/main.c \
Core/Src/stm32g4xx_it.c \
Core/Src/stm32g4xx_hal_msp.c \
Core/Src/system_stm32g4xx.c \
Core/Src/sysmem.c \
Core/Src/syscalls.c \
platform/module.c \
platform/platform_common.c \
platform/platform_pwm.c \
platform/platform_isr.c \
platform/platform_uart.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c

# Out-of-tree sources (found via VPATH)
EXT_SOURCES = \
pubsub.c \
foc.c \
dblink.c \
db_reader.c \
db_sender.c

# ASM sources
ASM_SOURCES = \
Core/Startup/startup_stm32g431cbux.s

######################################################################
# Includes
######################################################################

C_INCLUDES = \
-ICore/Inc \
-IDrivers/STM32G4xx_HAL_Driver/Inc \
-IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
-IDrivers/CMSIS/Include \
-Iplatform \
-I$(FOUNDATION_DIR) \
-I$(MODULES_DIR)

######################################################################
# Compile flags
######################################################################

C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32G431xx

ASFLAGS = $(MCU) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) -std=gnu11 -Os \
-ffunction-sections -fdata-sections -Wall \
-fstack-usage --specs=nano.specs

# Debug flags (default on)
CFLAGS += -g3 -DDEBUG

######################################################################
# Linker
######################################################################

LDSCRIPT = STM32G431CBUX_FLASH.ld

LIBS = -lc -lm -lnosys
LDFLAGS = $(MCU) --specs=nano.specs -T$(LDSCRIPT) $(LIBS) \
-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
-Wl,--gc-sections \
-Wl,--print-memory-usage

######################################################################
# Build rules
######################################################################

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(EXT_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))

# Dependency files
DEPS = $(OBJECTS:.o=.d)

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin

# C source compile
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

# ASM source compile
$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	$(AS) -c $(ASFLAGS) -MMD -MP -MF"$(@:%.o=%.d)" $< -o $@

# Link
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

# Generate binary
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O binary $< $@

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)

# Include dependency files
-include $(DEPS)

.PHONY: all clean
