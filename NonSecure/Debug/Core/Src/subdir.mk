################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/crc.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/rng.c \
../Core/Src/rtc.c \
../Core/Src/spi.c \
../Core/Src/stm32l5xx_hal_msp.c \
../Core/Src/stm32l5xx_hal_timebase_tim.c \
../Core/Src/stm32l5xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l5xx_ns.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/crc.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/rng.o \
./Core/Src/rtc.o \
./Core/Src/spi.o \
./Core/Src/stm32l5xx_hal_msp.o \
./Core/Src/stm32l5xx_hal_timebase_tim.o \
./Core/Src/stm32l5xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l5xx_ns.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/crc.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/rng.d \
./Core/Src/rtc.d \
./Core/Src/spi.d \
./Core/Src/stm32l5xx_hal_msp.d \
./Core/Src/stm32l5xx_hal_timebase_tim.d \
./Core/Src/stm32l5xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l5xx_ns.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/adc.o: ../Core/Src/adc.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/adc.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/crc.o: ../Core/Src/crc.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/crc.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/dma.o: ../Core/Src/dma.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/dma.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/gpio.o: ../Core/Src/gpio.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/gpio.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/i2c.o: ../Core/Src/i2c.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/i2c.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/main.o: ../Core/Src/main.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/rng.o: ../Core/Src/rng.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/rng.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/rtc.o: ../Core/Src/rtc.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/rtc.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/spi.o: ../Core/Src/spi.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/spi.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/stm32l5xx_hal_msp.o: ../Core/Src/stm32l5xx_hal_msp.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32l5xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/stm32l5xx_hal_timebase_tim.o: ../Core/Src/stm32l5xx_hal_timebase_tim.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32l5xx_hal_timebase_tim.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/stm32l5xx_it.o: ../Core/Src/stm32l5xx_it.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32l5xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/syscalls.o: ../Core/Src/syscalls.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/sysmem.o: ../Core/Src/sysmem.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/system_stm32l5xx_ns.o: ../Core/Src/system_stm32l5xx_ns.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/system_stm32l5xx_ns.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/tim.o: ../Core/Src/tim.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/tim.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/usart.o: ../Core/Src/usart.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/usart.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
