################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
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
bp_DC_Load.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
bp_DC_Load.list \

OBJCOPY_BIN += \
bp_DC_Load.bin \


# All Target
all: bp_DC_Load.elf secondary-outputs

# Tool invocations
bp_DC_Load.elf: $(OBJS) $(USER_OBJS) D:\Users\Javi\GitHub\STM32_Projects\bp_DC_Load\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "bp_DC_Load.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\Users\Javi\GitHub\STM32_Projects\bp_DC_Load\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="bp_DC_Load.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

bp_DC_Load.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "bp_DC_Load.list"
	@echo 'Finished building: $@'
	@echo ' '

bp_DC_Load.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "bp_DC_Load.bin"
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
