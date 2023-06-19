################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/tmc/ic/TMC5160/TMC5160.c 

OBJS += \
./Core/Inc/tmc/ic/TMC5160/TMC5160.o 

C_DEPS += \
./Core/Inc/tmc/ic/TMC5160/TMC5160.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/tmc/ic/TMC5160/%.o Core/Inc/tmc/ic/TMC5160/%.su: ../Core/Inc/tmc/ic/TMC5160/%.c Core/Inc/tmc/ic/TMC5160/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/wesley/OneDrive - TOP-electronics/STM32CubeIDE/workspace_1.9.0/1_AXIS_F091_Send/Core/Inc/tmc/helpers" -I"C:/Users/wesley/OneDrive - TOP-electronics/STM32CubeIDE/workspace_1.9.0/1_AXIS_F091_Send/Core/Inc/tmc/ramp" -I"C:/Users/wesley/OneDrive - TOP-electronics/STM32CubeIDE/workspace_1.9.0/1_AXIS_F091_Send/Core/Inc/tmc/ic" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-tmc-2f-ic-2f-TMC5160

clean-Core-2f-Inc-2f-tmc-2f-ic-2f-TMC5160:
	-$(RM) ./Core/Inc/tmc/ic/TMC5160/TMC5160.d ./Core/Inc/tmc/ic/TMC5160/TMC5160.o ./Core/Inc/tmc/ic/TMC5160/TMC5160.su

.PHONY: clean-Core-2f-Inc-2f-tmc-2f-ic-2f-TMC5160

