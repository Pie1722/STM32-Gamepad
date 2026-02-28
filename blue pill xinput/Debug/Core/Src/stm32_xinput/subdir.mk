################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/stm32_xinput/stm32_xinput.c \
../Core/Src/stm32_xinput/usb_xinput.c \
../Core/Src/stm32_xinput/xinput.c 

OBJS += \
./Core/Src/stm32_xinput/stm32_xinput.o \
./Core/Src/stm32_xinput/usb_xinput.o \
./Core/Src/stm32_xinput/xinput.o 

C_DEPS += \
./Core/Src/stm32_xinput/stm32_xinput.d \
./Core/Src/stm32_xinput/usb_xinput.d \
./Core/Src/stm32_xinput/xinput.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/stm32_xinput/%.o Core/Src/stm32_xinput/%.su Core/Src/stm32_xinput/%.cyclo: ../Core/Src/stm32_xinput/%.c Core/Src/stm32_xinput/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc -I../Core/Src/stm32_xinput -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-stm32_xinput

clean-Core-2f-Src-2f-stm32_xinput:
	-$(RM) ./Core/Src/stm32_xinput/stm32_xinput.cyclo ./Core/Src/stm32_xinput/stm32_xinput.d ./Core/Src/stm32_xinput/stm32_xinput.o ./Core/Src/stm32_xinput/stm32_xinput.su ./Core/Src/stm32_xinput/usb_xinput.cyclo ./Core/Src/stm32_xinput/usb_xinput.d ./Core/Src/stm32_xinput/usb_xinput.o ./Core/Src/stm32_xinput/usb_xinput.su ./Core/Src/stm32_xinput/xinput.cyclo ./Core/Src/stm32_xinput/xinput.d ./Core/Src/stm32_xinput/xinput.o ./Core/Src/stm32_xinput/xinput.su

.PHONY: clean-Core-2f-Src-2f-stm32_xinput

