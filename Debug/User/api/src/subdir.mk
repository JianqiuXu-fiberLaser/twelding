################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/api/src/algorithm.c \
../User/api/src/cmd.c \
../User/api/src/sysinit.c \
../User/api/src/tcp_client.c 

OBJS += \
./User/api/src/algorithm.o \
./User/api/src/cmd.o \
./User/api/src/sysinit.o \
./User/api/src/tcp_client.o 

C_DEPS += \
./User/api/src/algorithm.d \
./User/api/src/cmd.d \
./User/api/src/sysinit.d \
./User/api/src/tcp_client.d 


# Each subdirectory must supply rules for building sources it contributes
User/api/src/%.o: ../User/api/src/%.c User/api/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F427xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I"C:/workspace/twelding/User/api/inc" -I"C:/workspace/twelding/User/bsp/inc" -I"C:/workspace/twelding/User/driver/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-User-2f-api-2f-src

clean-User-2f-api-2f-src:
	-$(RM) ./User/api/src/algorithm.d ./User/api/src/algorithm.o ./User/api/src/cmd.d ./User/api/src/cmd.o ./User/api/src/sysinit.d ./User/api/src/sysinit.o ./User/api/src/tcp_client.d ./User/api/src/tcp_client.o

.PHONY: clean-User-2f-api-2f-src

