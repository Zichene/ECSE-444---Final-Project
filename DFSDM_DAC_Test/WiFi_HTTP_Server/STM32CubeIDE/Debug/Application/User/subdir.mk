################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Administrator/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.0/Projects/B-L4S5I-IOT01A/Applications/WiFi/WiFi_HTTP_Server/Src/flash_l4.c \
../Application/User/receiving_main.c \
../Application/User/sending_main.c \
../Application/User/stm32l4xx_hal_msp.c \
C:/Users/Administrator/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.0/Projects/B-L4S5I-IOT01A/Applications/WiFi/WiFi_HTTP_Server/Src/stm32l4xx_it.c \
../Application/User/syscalls.c \
../Application/User/sysmem.c \
C:/Users/Administrator/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.0/Projects/B-L4S5I-IOT01A/Applications/WiFi/WiFi_HTTP_Server/Src/system_stm32l4xx.c 

OBJS += \
./Application/User/flash_l4.o \
./Application/User/receiving_main.o \
./Application/User/sending_main.o \
./Application/User/stm32l4xx_hal_msp.o \
./Application/User/stm32l4xx_it.o \
./Application/User/syscalls.o \
./Application/User/sysmem.o \
./Application/User/system_stm32l4xx.o 

C_DEPS += \
./Application/User/flash_l4.d \
./Application/User/receiving_main.d \
./Application/User/sending_main.d \
./Application/User/stm32l4xx_hal_msp.d \
./Application/User/stm32l4xx_it.d \
./Application/User/syscalls.d \
./Application/User/sysmem.d \
./Application/User/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/flash_l4.o: C:/Users/Administrator/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.0/Projects/B-L4S5I-IOT01A/Applications/WiFi/WiFi_HTTP_Server/Src/flash_l4.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_STM32L4S5I_IOT01 -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../../../../../../../Drivers/BSP/B-L4S5I-IOT01 -I../../../Common/Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/%.o Application/User/%.su Application/User/%.cyclo: ../Application/User/%.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_STM32L4S5I_IOT01 -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../../../../../../../Drivers/BSP/B-L4S5I-IOT01 -I../../../Common/Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32l4xx_it.o: C:/Users/Administrator/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.0/Projects/B-L4S5I-IOT01A/Applications/WiFi/WiFi_HTTP_Server/Src/stm32l4xx_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_STM32L4S5I_IOT01 -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../../../../../../../Drivers/BSP/B-L4S5I-IOT01 -I../../../Common/Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/system_stm32l4xx.o: C:/Users/Administrator/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.0/Projects/B-L4S5I-IOT01A/Applications/WiFi/WiFi_HTTP_Server/Src/system_stm32l4xx.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_STM32L4S5I_IOT01 -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../../../../../../../Drivers/BSP/B-L4S5I-IOT01 -I../../../Common/Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User

clean-Application-2f-User:
	-$(RM) ./Application/User/flash_l4.cyclo ./Application/User/flash_l4.d ./Application/User/flash_l4.o ./Application/User/flash_l4.su ./Application/User/receiving_main.cyclo ./Application/User/receiving_main.d ./Application/User/receiving_main.o ./Application/User/receiving_main.su ./Application/User/sending_main.cyclo ./Application/User/sending_main.d ./Application/User/sending_main.o ./Application/User/sending_main.su ./Application/User/stm32l4xx_hal_msp.cyclo ./Application/User/stm32l4xx_hal_msp.d ./Application/User/stm32l4xx_hal_msp.o ./Application/User/stm32l4xx_hal_msp.su ./Application/User/stm32l4xx_it.cyclo ./Application/User/stm32l4xx_it.d ./Application/User/stm32l4xx_it.o ./Application/User/stm32l4xx_it.su ./Application/User/syscalls.cyclo ./Application/User/syscalls.d ./Application/User/syscalls.o ./Application/User/syscalls.su ./Application/User/sysmem.cyclo ./Application/User/sysmem.d ./Application/User/sysmem.o ./Application/User/sysmem.su ./Application/User/system_stm32l4xx.cyclo ./Application/User/system_stm32l4xx.d ./Application/User/system_stm32l4xx.o ./Application/User/system_stm32l4xx.su

.PHONY: clean-Application-2f-User

