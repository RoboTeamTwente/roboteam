################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/PuTTY.c \
../Lib/kalman.c \
../Lib/yawCalibration.c 

OBJS += \
./Lib/PuTTY.o \
./Lib/kalman.o \
./Lib/yawCalibration.o 

C_DEPS += \
./Lib/PuTTY.d \
./Lib/kalman.d \
./Lib/yawCalibration.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/%.o: ../Lib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F767xx -I"/home/simen/roboteamtwente/workspace/src/roboteam_microcontroller3.0/Core/Inc" -I"/home/simen/roboteamtwente/workspace/src/roboteam_microcontroller3.0/Util" -I"/home/simen/roboteamtwente/workspace/src/roboteam_microcontroller3.0/Lib" -I"/home/simen/roboteamtwente/workspace/src/roboteam_microcontroller3.0/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/simen/roboteamtwente/workspace/src/roboteam_microcontroller3.0/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/simen/roboteamtwente/workspace/src/roboteam_microcontroller3.0/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"/home/simen/roboteamtwente/workspace/src/roboteam_microcontroller3.0/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


