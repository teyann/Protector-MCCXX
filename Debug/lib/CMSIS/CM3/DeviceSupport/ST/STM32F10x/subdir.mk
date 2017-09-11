################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c 

OBJS += \
./lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.o 

C_DEPS += \
./lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.d 


# Each subdirectory must supply rules for building sources it contributes
lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/%.o: ../lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/CoreSupport" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/printf" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/Redifei Library" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32_USB-FS-Device_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32F10x_StdPeriph_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/USB_Port" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/src" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


