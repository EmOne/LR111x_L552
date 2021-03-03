################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV/stm32l552e_eval.c \
C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV/stm32l552e_eval_bus.c \
C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV/stm32l552e_eval_idd.c \
C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV/stm32l552e_eval_io.c \
C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV/stm32l5xx_gyroscope.c 

OBJS += \
./Drivers/BSP/STM32L552E-EV/stm32l552e_eval.o \
./Drivers/BSP/STM32L552E-EV/stm32l552e_eval_bus.o \
./Drivers/BSP/STM32L552E-EV/stm32l552e_eval_idd.o \
./Drivers/BSP/STM32L552E-EV/stm32l552e_eval_io.o \
./Drivers/BSP/STM32L552E-EV/stm32l5xx_gyroscope.o 

C_DEPS += \
./Drivers/BSP/STM32L552E-EV/stm32l552e_eval.d \
./Drivers/BSP/STM32L552E-EV/stm32l552e_eval_bus.d \
./Drivers/BSP/STM32L552E-EV/stm32l552e_eval_idd.d \
./Drivers/BSP/STM32L552E-EV/stm32l552e_eval_io.d \
./Drivers/BSP/STM32L552E-EV/stm32l5xx_gyroscope.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32L552E-EV/stm32l552e_eval.o: C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV/stm32l552e_eval.c Drivers/BSP/STM32L552E-EV/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32L552E-EV/stm32l552e_eval.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/STM32L552E-EV/stm32l552e_eval_bus.o: C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV/stm32l552e_eval_bus.c Drivers/BSP/STM32L552E-EV/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32L552E-EV/stm32l552e_eval_bus.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/STM32L552E-EV/stm32l552e_eval_idd.o: C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV/stm32l552e_eval_idd.c Drivers/BSP/STM32L552E-EV/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32L552E-EV/stm32l552e_eval_idd.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/STM32L552E-EV/stm32l552e_eval_io.o: C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV/stm32l552e_eval_io.c Drivers/BSP/STM32L552E-EV/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32L552E-EV/stm32l552e_eval_io.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/STM32L552E-EV/stm32l5xx_gyroscope.o: C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV/stm32l5xx_gyroscope.c Drivers/BSP/STM32L552E-EV/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../../Secure_nsclib -I../../Drivers/STM32L5xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Include -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/STM32L552E-EV" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/Common" -I"C:/Users/anol_/STM32CubeIDE/workspace_1.6.0/LR111x_L552VE/Drivers/BSP/Components/icg20330" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32L552E-EV/stm32l5xx_gyroscope.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

