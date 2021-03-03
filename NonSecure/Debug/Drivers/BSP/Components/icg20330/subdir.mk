################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330/icg20330.c 

OBJS += \
./Drivers/BSP/Components/icg20330/icg20330.o 

C_DEPS += \
./Drivers/BSP/Components/icg20330/icg20330.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/icg20330/icg20330.o: C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330/icg20330.c Drivers/BSP/Components/icg20330/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/icg20330/icg20330.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

