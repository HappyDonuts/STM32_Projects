################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
F401_Capacitance_meter.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
F401_Capacitance_meter.list \

OBJCOPY_BIN += \
F401_Capacitance_meter.bin \


# All Target
all: F401_Capacitance_meter.elf secondary-outputs

# Tool invocations
F401_Capacitance_meter.elf: $(OBJS) $(USER_OBJS) D:\Users\Javi\GitHub\STM32_Projects\F401_Capacitance_meter\STM32F401CCUX_FLASH.ld
	arm-none-eabi-gcc -o "F401_Capacitance_meter.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\Users\Javi\GitHub\STM32_Projects\F401_Capacitance_meter\STM32F401CCUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="F401_Capacitance_meter.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

F401_Capacitance_meter.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "F401_Capacitance_meter.list"
	@echo 'Finished building: $@'
	@echo ' '

F401_Capacitance_meter.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "F401_Capacitance_meter.bin"
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
