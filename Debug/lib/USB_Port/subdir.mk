################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/USB_Port/hw_config.c \
../lib/USB_Port/usb_desc.c \
../lib/USB_Port/usb_endp.c \
../lib/USB_Port/usb_istr.c \
../lib/USB_Port/usb_prop.c \
../lib/USB_Port/usb_pwr.c 

OBJS += \
./lib/USB_Port/hw_config.o \
./lib/USB_Port/usb_desc.o \
./lib/USB_Port/usb_endp.o \
./lib/USB_Port/usb_istr.o \
./lib/USB_Port/usb_prop.o \
./lib/USB_Port/usb_pwr.o 

C_DEPS += \
./lib/USB_Port/hw_config.d \
./lib/USB_Port/usb_desc.d \
./lib/USB_Port/usb_endp.d \
./lib/USB_Port/usb_istr.d \
./lib/USB_Port/usb_prop.d \
./lib/USB_Port/usb_pwr.d 


# Each subdirectory must supply rules for building sources it contributes
lib/USB_Port/%.o: ../lib/USB_Port/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/CoreSupport" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/printf" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/Redifei Library" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32_USB-FS-Device_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/STM32F10x_StdPeriph_Driver/inc" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/lib/USB_Port" -I"/home/luc/Documents/workspace/eclipse/protector_MCCXX/src" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


