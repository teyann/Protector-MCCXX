################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include lib/printf/subdir.mk
-include lib/USB_Port/subdir.mk
-include lib/STM32_USB-FS-Device_Driver/src/subdir.mk
-include lib/STM32F10x_StdPeriph_Driver/src/subdir.mk
-include lib/Redifei\ Library/subdir.mk
-include lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/TrueSTUDIO/subdir.mk
-include lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/subdir.mk
-include lib/CMSIS/CM3/CoreSupport/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
protector_MCCXX.hex \

SECONDARY_SIZE += \
protector_MCCXX.siz \


# All Target
all: protector_MCCXX.elf secondary-outputs

# Tool invocations
protector_MCCXX.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM GNU C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -T "/home/luc/Documents/workspace/eclipse/protector_MCCXX/stm32_flash.ld" -Xlinker --gc-sections -Wl,-Map,"protector_MCCXX.map" -o "protector_MCCXX.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

protector_MCCXX.hex: protector_MCCXX.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex "protector_MCCXX.elf"  "protector_MCCXX.hex"
	@echo 'Finished building: $@'
	@echo ' '

protector_MCCXX.siz: protector_MCCXX.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=sysv --totals "protector_MCCXX.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) protector_MCCXX.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
