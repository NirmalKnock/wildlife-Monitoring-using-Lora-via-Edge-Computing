################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include drivers/src/subdir.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include BSP/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM32F446RE\ Driver.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32F446RE\ Driver.list \

OBJCOPY_BIN += \
STM32F446RE\ Driver.bin \


# All Target
all: STM32F446RE\ Driver.elf secondary-outputs

# Tool invocations
STM32F446RE\ Driver.elf: $(OBJS) $(USER_OBJS) D:\Embedded\ C\MCU\ master\ 1\STM32F446RE\ Driver\STM32F446RETX_FLASH.ld
	arm-none-eabi-gcc -o "STM32F446RE Driver.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\Embedded C\MCU master 1\STM32F446RE Driver\STM32F446RETX_FLASH.ld" -Wl,-Map="STM32F446RE Driver.map" -Wl,--gc-sections -static -specs=rdimon.specs --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32F446RE\ Driver.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32F446RE Driver.list"
	@echo 'Finished building: $@'
	@echo ' '

STM32F446RE\ Driver.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "STM32F446RE Driver.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
