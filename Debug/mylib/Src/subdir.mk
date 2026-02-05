################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../mylib/Src/i2c_lcd.c \
../mylib/Src/mylib.c \
../mylib/Src/sht3x.c 

OBJS += \
./mylib/Src/i2c_lcd.o \
./mylib/Src/mylib.o \
./mylib/Src/sht3x.o 

C_DEPS += \
./mylib/Src/i2c_lcd.d \
./mylib/Src/mylib.d \
./mylib/Src/sht3x.d 


# Each subdirectory must supply rules for building sources it contributes
mylib/Src/%.o mylib/Src/%.su mylib/Src/%.cyclo: ../mylib/Src/%.c mylib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Admin/STM32CubeIDE/workspace_1.19.0/TT_Scheduler/mylib/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-mylib-2f-Src

clean-mylib-2f-Src:
	-$(RM) ./mylib/Src/i2c_lcd.cyclo ./mylib/Src/i2c_lcd.d ./mylib/Src/i2c_lcd.o ./mylib/Src/i2c_lcd.su ./mylib/Src/mylib.cyclo ./mylib/Src/mylib.d ./mylib/Src/mylib.o ./mylib/Src/mylib.su ./mylib/Src/sht3x.cyclo ./mylib/Src/sht3x.d ./mylib/Src/sht3x.o ./mylib/Src/sht3x.su

.PHONY: clean-mylib-2f-Src

