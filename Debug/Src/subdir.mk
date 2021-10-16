################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/017rtc_lcd.c \
../Src/sysmem.c 

OBJS += \
./Src/017rtc_lcd.o \
./Src/sysmem.o 

C_DEPS += \
./Src/017rtc_lcd.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/017rtc_lcd.o: ../Src/017rtc_lcd.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -c -I"D:/Embedded C/MCU master 1/STM32F446RE Driver/drivers/inc" -I"D:/Embedded C/MCU master 1/STM32F446RE Driver/BSP" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/017rtc_lcd.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -c -I"D:/Embedded C/MCU master 1/STM32F446RE Driver/drivers/inc" -I"D:/Embedded C/MCU master 1/STM32F446RE Driver/BSP" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

