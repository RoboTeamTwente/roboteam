################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Util/gpio_util.c \
../Util/tim_util.c 

OBJS += \
./Util/gpio_util.o \
./Util/tim_util.o 

C_DEPS += \
./Util/gpio_util.d \
./Util/tim_util.d 


# Each subdirectory must supply rules for building sources it contributes
Util/%.o: ../Util/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F767xx -I"C:/Users/454b/Desktop/robot3.0/roboteam_microcontroller3.0/Core/Inc" -I"C:/Users/454b/Desktop/robot3.0/roboteam_microcontroller3.0/Util" -I"C:/Users/454b/Desktop/robot3.0/roboteam_microcontroller3.0/Lib" -I"C:/Users/454b/Desktop/robot3.0/roboteam_microcontroller3.0/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/454b/Desktop/robot3.0/roboteam_microcontroller3.0/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/454b/Desktop/robot3.0/roboteam_microcontroller3.0/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/454b/Desktop/robot3.0/roboteam_microcontroller3.0/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


