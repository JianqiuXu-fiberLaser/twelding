################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f427vgtx.s 

OBJS += \
./Core/Startup/startup_stm32f427vgtx.o 

S_DEPS += \
./Core/Startup/startup_stm32f427vgtx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/workspace/twelding/User/api/inc" -I"C:/workspace/twelding/User/bsp/inc" -I"C:/workspace/twelding/User/api/src" -I"C:/workspace/twelding/User/bsp/src" -I"C:/workspace/twelding/User/driver/inc" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f427vgtx.d ./Core/Startup/startup_stm32f427vgtx.o

.PHONY: clean-Core-2f-Startup

