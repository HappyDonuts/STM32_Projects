################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
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
F303_Electronic_Load.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
F303_Electronic_Load.list \


# All Target
all: F303_Electronic_Load.elf secondary-outputs

# Tool invocations
F303_Electronic_Load.elf: $(OBJS) $(USER_OBJS) D:\Users\Javi\GitHub\STM32_Projects\F303_Electronic_Load\STM32F303K8TX_FLASH.ld
	arm-none-eabi-gcc -o "F303_Electronic_Load.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\Users\Javi\GitHub\STM32_Projects\F303_Electronic_Load\STM32F303K8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="F303_Electronic_Load.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

F303_Electronic_Load.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "F303_Electronic_Load.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
