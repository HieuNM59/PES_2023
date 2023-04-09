################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../user/dwt_stm32_delay.c \
../user/fifo.c \
../user/fonts.c \
../user/main-app.c \
../user/ps2.c \
../user/ssd1306.c 

OBJS += \
./user/dwt_stm32_delay.o \
./user/fifo.o \
./user/fonts.o \
./user/main-app.o \
./user/ps2.o \
./user/ssd1306.o 

C_DEPS += \
./user/dwt_stm32_delay.d \
./user/fifo.d \
./user/fonts.d \
./user/main-app.d \
./user/ps2.d \
./user/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
user/%.o user/%.su: ../user/%.c user/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/LumiQA/Desktop/PES_2023/user" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-user

clean-user:
	-$(RM) ./user/dwt_stm32_delay.d ./user/dwt_stm32_delay.o ./user/dwt_stm32_delay.su ./user/fifo.d ./user/fifo.o ./user/fifo.su ./user/fonts.d ./user/fonts.o ./user/fonts.su ./user/main-app.d ./user/main-app.o ./user/main-app.su ./user/ps2.d ./user/ps2.o ./user/ps2.su ./user/ssd1306.d ./user/ssd1306.o ./user/ssd1306.su

.PHONY: clean-user

