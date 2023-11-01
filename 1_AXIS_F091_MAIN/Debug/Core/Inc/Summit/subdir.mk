################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/Summit/SSM1ST2420HC.c 

OBJS += \
./Core/Inc/Summit/SSM1ST2420HC.o 

C_DEPS += \
./Core/Inc/Summit/SSM1ST2420HC.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Summit/%.o Core/Inc/Summit/%.su Core/Inc/Summit/%.cyclo: ../Core/Inc/Summit/%.c Core/Inc/Summit/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Github_Local/SSM1ST2420HC/1_AXIS_F091_MAIN/Core/Inc/Summit" -I"C:/Github_Local/SSM1ST2420HC/1_AXIS_F091_MAIN/Core/Inc/tmc/helpers" -I"C:/Github_Local/SSM1ST2420HC/1_AXIS_F091_MAIN/Core/Inc/tmc/ramp" -I"C:/Github_Local/SSM1ST2420HC/1_AXIS_F091_MAIN/Core/Inc/tmc/ic" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Summit

clean-Core-2f-Inc-2f-Summit:
	-$(RM) ./Core/Inc/Summit/SSM1ST2420HC.cyclo ./Core/Inc/Summit/SSM1ST2420HC.d ./Core/Inc/Summit/SSM1ST2420HC.o ./Core/Inc/Summit/SSM1ST2420HC.su

.PHONY: clean-Core-2f-Inc-2f-Summit

