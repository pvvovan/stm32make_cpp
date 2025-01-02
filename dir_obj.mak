# Rework of existing Makefile to keep objects in directory tree

TARGET = gpsRadar
BUILD_DIR = build

C_SOURCES = \
Core/Src/main.c \
Core/Src/freertos.c \
Core/Src/stm32f4xx_it.c \
Core/Src/stm32f4xx_hal_msp.c \
Core/Src/stm32f4xx_hal_timebase_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
Core/Src/system_stm32f4xx.c \
Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
Middlewares/Third_Party/FreeRTOS/Source/list.c \
Middlewares/Third_Party/FreeRTOS/Source/queue.c \
Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
Middlewares/Third_Party/FreeRTOS/Source/timers.c \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c

ASM_SOURCES = \
startup_stm32f407xx.s

CPP_SOURCES = \
Core/Src/cpp_task1.cpp \
Core/Src/nmea2pwm.cpp \
Core/Src/uart.cpp

PREFIX = arm-none-eabi-

ifdef GCC_PATH
	CC = $(GCC_PATH)/$(PREFIX)gcc
	CXX = $(GCC_PATH)/$(PREFIX)g++
	AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
	CP = $(GCC_PATH)/$(PREFIX)objcopy
	SZ = $(GCC_PATH)/$(PREFIX)size
else
	CC = $(PREFIX)gcc
	CXX = $(PREFIX)g++
	AS = $(PREFIX)gcc -x assembler-with-cpp
	CP = $(PREFIX)objcopy
	SZ = $(PREFIX)size
endif

HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

C_DEFS = \
-D USE_HAL_DRIVER \
-D STM32F407xx \
-D USE_HAL_DRIVER \
-D STM32F407xx

C_INCLUDES = \
-I Core/Inc \
-I Drivers/STM32F4xx_HAL_Driver/Inc \
-I Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-I Middlewares/Third_Party/FreeRTOS/Source/include \
-I Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
-I Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-I Drivers/CMSIS/Device/ST/STM32F4xx/Include \
-I Drivers/CMSIS/Include

ifeq ($(DEBUG), 1)
	OPT = -O0 -g3 -gdwarf-5
else
	OPT = -O3 -g0
endif

COMN_FLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections \
	-MMD -MP -MF"$(@:%.o=%.d)"

CFLAGS = $(COMN_FLAGS) -std=c11
CPPFLAGS = $(COMN_FLAGS) -std=c++17

LDSCRIPT = STM32F407VGTx_FLASH.ld

LDFLAGS = $(MCU) -specs=nano.specs -specs=nosys.specs -T$(LDSCRIPT) -lc -lm -lnosys \
	-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -lstdc++

.PHONY: all clean
clean:
	-rm -fR $(BUILD_DIR)

.DEFAULT_GOAL := all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

C_OBJECTS = $(addprefix $(BUILD_DIR)/, $(C_SOURCES:.c=.o))
CXX_OBJECTS = $(addprefix $(BUILD_DIR)/, $(CPP_SOURCES:.cpp=.o))
ASM_OBJECTS = $(addprefix $(BUILD_DIR)/, $(ASM_SOURCES:.s=.o))

$(C_OBJECTS): $(BUILD_DIR)/%.o: %.c
	@mkdir -p $(@D)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(<:.c=.lst) $< -o $@

$(CXX_OBJECTS): $(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) -c $(CPPFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(<:.cpp=.lst) $< -o $@

$(ASM_OBJECTS): $(BUILD_DIR)/%.o: %.s
	@mkdir -p $(@D)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(C_OBJECTS) $(CXX_OBJECTS) $(ASM_OBJECTS)
	$(CC) $^ $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(BIN) $< $@

DEPS = $(C_OBJECTS:.o=.d) $(CXX_OBJECTS:.o=.d) $(ASP_OBJECTS:.o=.d)
-include $(DEPS)
