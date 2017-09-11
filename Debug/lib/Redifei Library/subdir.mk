################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/Redifei\ Library/i2c.c \
../lib/Redifei\ Library/mpu6050.c \
../lib/Redifei\ Library/serialUart.c \
../lib/Redifei\ Library/systickTimer.c \
../lib/Redifei\ Library/timerPwm.c \
../lib/Redifei\ Library/vcom.c 

OBJS += \
./lib/Redifei\ Library/i2c.o \
./lib/Redifei\ Library/mpu6050.o \
./lib/Redifei\ Library/serialUart.o \
./lib/Redifei\ Library/systickTimer.o \
./lib/Redifei\ Library/timerPwm.o \
./lib/Redifei\ Library/vcom.o 

C_DEPS += \
./lib/Redifei\ Library/i2c.d \
./lib/Redifei\ Library/mpu6050.d \
./lib/Redifei\ Library/serialUart.d \
./lib/Redifei\ Library/systickTimer.d \
./lib/Redifei\ Library/timerPwm.d \
./lib/Redifei\ Library/vcom.d 


# Each subdirectory must supply rules for building sources it contributes
lib/Redifei\ Library/i2c.o: ../lib/Redifei\ Library/i2c.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/CoreSupport" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/printf" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/Redifei Library" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32_USB-FS-Device_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32F10x_StdPeriph_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/USB_Port" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/src" -std=gnu11 -MMD -MP -MF"lib/Redifei Library/i2c.d" -MT"lib/Redifei\ Library/i2c.d" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

lib/Redifei\ Library/mpu6050.o: ../lib/Redifei\ Library/mpu6050.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/CoreSupport" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/printf" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/Redifei Library" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32_USB-FS-Device_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32F10x_StdPeriph_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/USB_Port" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/src" -std=gnu11 -MMD -MP -MF"lib/Redifei Library/mpu6050.d" -MT"lib/Redifei\ Library/mpu6050.d" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

lib/Redifei\ Library/serialUart.o: ../lib/Redifei\ Library/serialUart.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/CoreSupport" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/printf" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/Redifei Library" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32_USB-FS-Device_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32F10x_StdPeriph_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/USB_Port" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/src" -std=gnu11 -MMD -MP -MF"lib/Redifei Library/serialUart.d" -MT"lib/Redifei\ Library/serialUart.d" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

lib/Redifei\ Library/systickTimer.o: ../lib/Redifei\ Library/systickTimer.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/CoreSupport" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/printf" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/Redifei Library" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32_USB-FS-Device_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32F10x_StdPeriph_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/USB_Port" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/src" -std=gnu11 -MMD -MP -MF"lib/Redifei Library/systickTimer.d" -MT"lib/Redifei\ Library/systickTimer.d" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

lib/Redifei\ Library/timerPwm.o: ../lib/Redifei\ Library/timerPwm.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/CoreSupport" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/printf" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/Redifei Library" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32_USB-FS-Device_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32F10x_StdPeriph_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/USB_Port" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/src" -std=gnu11 -MMD -MP -MF"lib/Redifei Library/timerPwm.d" -MT"lib/Redifei\ Library/timerPwm.d" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

lib/Redifei\ Library/vcom.o: ../lib/Redifei\ Library/vcom.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/CoreSupport" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/printf" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/Redifei Library" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32_USB-FS-Device_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32F10x_StdPeriph_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/USB_Port" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/src" -std=gnu11 -MMD -MP -MF"lib/Redifei Library/vcom.d" -MT"lib/Redifei\ Library/vcom.d" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


