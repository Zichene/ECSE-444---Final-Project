################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Administrator/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.0/Projects/B-L4S5I-IOT01A/Applications/Project_Main/Common/Src/es_wifi.c \
C:/Users/Administrator/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.0/Projects/B-L4S5I-IOT01A/Applications/Project_Main/Common/Src/es_wifi_io.c \
C:/Users/Administrator/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.0/Projects/B-L4S5I-IOT01A/Applications/Project_Main/Common/Src/wifi.c 

OBJS += \
./Application/WiFi/es_wifi.o \
./Application/WiFi/es_wifi_io.o \
./Application/WiFi/wifi.o 

C_DEPS += \
./Application/WiFi/es_wifi.d \
./Application/WiFi/es_wifi_io.d \
./Application/WiFi/wifi.d 


# Each subdirectory must supply rules for building sources it contributes
Application/WiFi/es_wifi.o: C:/Users/Administrator/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.0/Projects/B-L4S5I-IOT01A/Applications/Project_Main/Common/Src/es_wifi.c Application/WiFi/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_STM32L4S5I_IOT01 -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../../../../../../../Drivers/BSP/B-L4S5I-IOT01 -I../../../Common/Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/WiFi/es_wifi_io.o: C:/Users/Administrator/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.0/Projects/B-L4S5I-IOT01A/Applications/Project_Main/Common/Src/es_wifi_io.c Application/WiFi/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_STM32L4S5I_IOT01 -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../../../../../../../Drivers/BSP/B-L4S5I-IOT01 -I../../../Common/Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/WiFi/wifi.o: C:/Users/Administrator/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.0/Projects/B-L4S5I-IOT01A/Applications/Project_Main/Common/Src/wifi.c Application/WiFi/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_STM32L4S5I_IOT01 -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../../../../../../../Drivers/BSP/B-L4S5I-IOT01 -I../../../Common/Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-WiFi

clean-Application-2f-WiFi:
	-$(RM) ./Application/WiFi/es_wifi.cyclo ./Application/WiFi/es_wifi.d ./Application/WiFi/es_wifi.o ./Application/WiFi/es_wifi.su ./Application/WiFi/es_wifi_io.cyclo ./Application/WiFi/es_wifi_io.d ./Application/WiFi/es_wifi_io.o ./Application/WiFi/es_wifi_io.su ./Application/WiFi/wifi.cyclo ./Application/WiFi/wifi.d ./Application/WiFi/wifi.o ./Application/WiFi/wifi.su

.PHONY: clean-Application-2f-WiFi

