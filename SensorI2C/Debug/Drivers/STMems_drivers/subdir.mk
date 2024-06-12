################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STMems_drivers/lis2dw12_reg.c 

OBJS += \
./Drivers/STMems_drivers/lis2dw12_reg.o 

C_DEPS += \
./Drivers/STMems_drivers/lis2dw12_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STMems_drivers/%.o Drivers/STMems_drivers/%.su Drivers/STMems_drivers/%.cyclo: ../Drivers/STMems_drivers/%.c Drivers/STMems_drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303x8 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Liikmed/STM32CubeIDE/workspace_1.10.1/SensorI2C/Drivers/STMems_drivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-STMems_drivers

clean-Drivers-2f-STMems_drivers:
	-$(RM) ./Drivers/STMems_drivers/lis2dw12_reg.cyclo ./Drivers/STMems_drivers/lis2dw12_reg.d ./Drivers/STMems_drivers/lis2dw12_reg.o ./Drivers/STMems_drivers/lis2dw12_reg.su

.PHONY: clean-Drivers-2f-STMems_drivers

