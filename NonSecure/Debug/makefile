################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/LoRaWAN/Utilities/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/Patterns/Basic/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/Patterns/Advanced/LmHandler/packages/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/Patterns/Advanced/LmHandler/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/Patterns/Advanced/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/Mac/region/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/Mac/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/Crypto/subdir.mk
-include Drivers/STM32L5xx_HAL_Driver/subdir.mk
-include Drivers/BSP/lr1110_driver/src/subdir.mk
-include Drivers/BSP/STM32L552E-EV/subdir.mk
-include Drivers/BSP/Components/lr1110/subdir.mk
-include Drivers/BSP/Components/icg20330/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := LR111x_L552VE_NonSecure
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
LR111x_L552VE_NonSecure.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
LR111x_L552VE_NonSecure.list \

OBJCOPY_HEX += \
LR111x_L552VE_NonSecure.hex \

OBJCOPY_BIN += \
LR111x_L552VE_NonSecure.bin \


# All Target
all: main-build

# Main-build Target
main-build: LR111x_L552VE_NonSecure.elf secondary-outputs

# Tool invocations
LR111x_L552VE_NonSecure.elf: $(OBJS) $(USER_OBJS) C:\Users\anol_\STM32CubeIDE\workspace_1.6.0\LR111x_L552VE\NonSecure\STM32L552VETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "LR111x_L552VE_NonSecure.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m33 -T"C:\Users\anol_\STM32CubeIDE\workspace_1.6.0\LR111x_L552VE\NonSecure\STM32L552VETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="LR111x_L552VE_NonSecure.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

LR111x_L552VE_NonSecure.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "LR111x_L552VE_NonSecure.list"
	@echo 'Finished building: $@'
	@echo ' '

LR111x_L552VE_NonSecure.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "LR111x_L552VE_NonSecure.hex"
	@echo 'Finished building: $@'
	@echo ' '

LR111x_L552VE_NonSecure.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "LR111x_L552VE_NonSecure.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
.SECONDARY:

-include ../makefile.targets
