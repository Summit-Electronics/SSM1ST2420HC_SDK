################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/tmc/ramp/LinearRamp.c \
../Core/Inc/tmc/ramp/LinearRamp1.c \
../Core/Inc/tmc/ramp/Ramp.c 

OBJS += \
./Core/Inc/tmc/ramp/LinearRamp.o \
./Core/Inc/tmc/ramp/LinearRamp1.o \
./Core/Inc/tmc/ramp/Ramp.o 

C_DEPS += \
./Core/Inc/tmc/ramp/LinearRamp.d \
./Core/Inc/tmc/ramp/LinearRamp1.d \
./Core/Inc/tmc/ramp/Ramp.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/tmc/ramp/%.o Core/Inc/tmc/ramp/%.su Core/Inc/tmc/ramp/%.cyclo: ../Core/Inc/tmc/ramp/%.c Core/Inc/tmc/ramp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Github_Local/SSM1ST2420HC/1_AXIS_F091_Knikker_Demo_V1.0/Core/Inc/Summit" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-tmc-2f-ramp

clean-Core-2f-Inc-2f-tmc-2f-ramp:
	-$(RM) ./Core/Inc/tmc/ramp/LinearRamp.cyclo ./Core/Inc/tmc/ramp/LinearRamp.d ./Core/Inc/tmc/ramp/LinearRamp.o ./Core/Inc/tmc/ramp/LinearRamp.su ./Core/Inc/tmc/ramp/LinearRamp1.cyclo ./Core/Inc/tmc/ramp/LinearRamp1.d ./Core/Inc/tmc/ramp/LinearRamp1.o ./Core/Inc/tmc/ramp/LinearRamp1.su ./Core/Inc/tmc/ramp/Ramp.cyclo ./Core/Inc/tmc/ramp/Ramp.d ./Core/Inc/tmc/ramp/Ramp.o ./Core/Inc/tmc/ramp/Ramp.su

.PHONY: clean-Core-2f-Inc-2f-tmc-2f-ramp

