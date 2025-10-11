################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_dma.c \
C:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_exti.c \
C:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_gpio.c \
C:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_rcc.c \
C:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_tim.c \
C:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usart.c \
C:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_utils.c 

OBJS += \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_dma.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_exti.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_gpio.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_rcc.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_tim.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_usart.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_utils.o 

C_DEPS += \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_dma.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_exti.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_gpio.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_rcc.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_tim.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_usart.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_dma.o: C:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_dma.c Drivers/STM32F4xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DSTM32F446xx -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_exti.o: C:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_exti.c Drivers/STM32F4xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DSTM32F446xx -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_gpio.o: C:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_gpio.c Drivers/STM32F4xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DSTM32F446xx -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_rcc.o: C:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_rcc.c Drivers/STM32F4xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DSTM32F446xx -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_tim.o: C:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_tim.c Drivers/STM32F4xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DSTM32F446xx -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_usart.o: C:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usart.c Drivers/STM32F4xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DSTM32F446xx -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_utils.o: C:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_utils.c Drivers/STM32F4xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DSTM32F446xx -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/nicoh/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-STM32F4xx_HAL_Driver

clean-Drivers-2f-STM32F4xx_HAL_Driver:
	-$(RM) ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_dma.cyclo ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_dma.d ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_dma.o ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_dma.su ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_exti.cyclo ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_exti.d ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_exti.o ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_exti.su ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_gpio.cyclo ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_gpio.d ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_gpio.o ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_gpio.su ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_rcc.cyclo ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_rcc.d ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_rcc.o ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_rcc.su ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_tim.cyclo ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_tim.d ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_tim.o ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_tim.su ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_usart.cyclo ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_usart.d ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_usart.o ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_usart.su ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_utils.cyclo ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_utils.d ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_utils.o ./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_ll_utils.su

.PHONY: clean-Drivers-2f-STM32F4xx_HAL_Driver

