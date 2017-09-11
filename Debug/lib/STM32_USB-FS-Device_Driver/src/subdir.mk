################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/STM32_USB-FS-Device_Driver/src/usb_core.c \
../lib/STM32_USB-FS-Device_Driver/src/usb_init.c \
../lib/STM32_USB-FS-Device_Driver/src/usb_int.c \
../lib/STM32_USB-FS-Device_Driver/src/usb_mem.c \
../lib/STM32_USB-FS-Device_Driver/src/usb_regs.c \
../lib/STM32_USB-FS-Device_Driver/src/usb_sil.c 

OBJS += \
./lib/STM32_USB-FS-Device_Driver/src/usb_core.o \
./lib/STM32_USB-FS-Device_Driver/src/usb_init.o \
./lib/STM32_USB-FS-Device_Driver/src/usb_int.o \
./lib/STM32_USB-FS-Device_Driver/src/usb_mem.o \
./lib/STM32_USB-FS-Device_Driver/src/usb_regs.o \
./lib/STM32_USB-FS-Device_Driver/src/usb_sil.o 

C_DEPS += \
./lib/STM32_USB-FS-Device_Driver/src/usb_core.d \
./lib/STM32_USB-FS-Device_Driver/src/usb_init.d \
./lib/STM32_USB-FS-Device_Driver/src/usb_int.d \
./lib/STM32_USB-FS-Device_Driver/src/usb_mem.d \
./lib/STM32_USB-FS-Device_Driver/src/usb_regs.d \
./lib/STM32_USB-FS-Device_Driver/src/usb_sil.d 


# Each subdirectory must supply rules for building sources it contributes
lib/STM32_USB-FS-Device_Driver/src/%.o: ../lib/STM32_USB-FS-Device_Driver/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/CoreSupport" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/printf" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/Redifei Library" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32_USB-FS-Device_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32F10x_StdPeriph_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/USB_Port" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/src" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


