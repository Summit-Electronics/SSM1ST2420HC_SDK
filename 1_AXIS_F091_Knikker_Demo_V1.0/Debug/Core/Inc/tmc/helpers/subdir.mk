################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/tmc/helpers/CRC.c \
../Core/Inc/tmc/helpers/Functions.c 

OBJS += \
./Core/Inc/tmc/helpers/CRC.o \
./Core/Inc/tmc/helpers/Functions.o 

C_DEPS += \
./Core/Inc/tmc/helpers/CRC.d \
./Core/Inc/tmc/helpers/Functions.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/tmc/helpers/%.o Core/Inc/tmc/helpers/%.su Core/Inc/tmc/helpers/%.cyclo: ../Core/Inc/tmc/helpers/%.c Core/Inc/tmc/helpers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Github_Local/SSM1ST2420HC/1_AXIS_F091_Knikker_Demo_V1.0/Core/Inc/Summit" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-tmc-2f-helpers

clean-Core-2f-Inc-2f-tmc-2f-helpers:
	-$(RM) ./Core/Inc/tmc/helpers/CRC.cyclo ./Core/Inc/tmc/helpers/CRC.d ./Core/Inc/tmc/helpers/CRC.o ./Core/Inc/tmc/helpers/CRC.su ./Core/Inc/tmc/helpers/Functions.cyclo ./Core/Inc/tmc/helpers/Functions.d ./Core/Inc/tmc/helpers/Functions.o ./Core/Inc/tmc/helpers/Functions.su

.PHONY: clean-Core-2f-Inc-2f-tmc-2f-helpers

