################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/STM32F407VGTX/startup_stm32f407vgtx.s 

OBJS += \
./Core/Startup/STM32F407VGTX/startup_stm32f407vgtx.o 

S_DEPS += \
./Core/Startup/STM32F407VGTX/startup_stm32f407vgtx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/STM32F407VGTX/%.o: ../Core/Startup/STM32F407VGTX/%.s Core/Startup/STM32F407VGTX/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup-2f-STM32F407VGTX

clean-Core-2f-Startup-2f-STM32F407VGTX:
	-$(RM) ./Core/Startup/STM32F407VGTX/startup_stm32f407vgtx.d ./Core/Startup/STM32F407VGTX/startup_stm32f407vgtx.o

.PHONY: clean-Core-2f-Startup-2f-STM32F407VGTX

