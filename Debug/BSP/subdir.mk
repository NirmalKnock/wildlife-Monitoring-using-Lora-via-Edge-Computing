################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/ds1307.c \
../BSP/lcd.c 

OBJS += \
./BSP/ds1307.o \
./BSP/lcd.o 

C_DEPS += \
./BSP/ds1307.d \
./BSP/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/ds1307.o: ../BSP/ds1307.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -c -I"D:/Embedded C/MCU master 1/STM32F446RE Driver/drivers/inc" -I"D:/Embedded C/MCU master 1/STM32F446RE Driver/BSP" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"BSP/ds1307.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BSP/lcd.o: ../BSP/lcd.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -c -I"D:/Embedded C/MCU master 1/STM32F446RE Driver/drivers/inc" -I"D:/Embedded C/MCU master 1/STM32F446RE Driver/BSP" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"BSP/lcd.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

