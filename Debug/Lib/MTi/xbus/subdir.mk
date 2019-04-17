################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/MTi/xbus/xbusmessage.c \
../Lib/MTi/xbus/xbusparser.c \
../Lib/MTi/xbus/xbusutility.c \
../Lib/MTi/xbus/xsdeviceid.c 

OBJS += \
./Lib/MTi/xbus/xbusmessage.o \
./Lib/MTi/xbus/xbusparser.o \
./Lib/MTi/xbus/xbusutility.o \
./Lib/MTi/xbus/xsdeviceid.o 

C_DEPS += \
./Lib/MTi/xbus/xbusmessage.d \
./Lib/MTi/xbus/xbusparser.d \
./Lib/MTi/xbus/xbusutility.d \
./Lib/MTi/xbus/xsdeviceid.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/MTi/xbus/%.o: ../Lib/MTi/xbus/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F767xx -I"C:/RoboTeam/CODE/roboteam_microcontroller3.0/Core/Inc" -I"C:/RoboTeam/CODE/roboteam_microcontroller3.0/Util" -I"C:/RoboTeam/CODE/roboteam_microcontroller3.0/Lib" -I"C:/RoboTeam/CODE/roboteam_microcontroller3.0/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/RoboTeam/CODE/roboteam_microcontroller3.0/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/RoboTeam/CODE/roboteam_microcontroller3.0/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/RoboTeam/CODE/roboteam_microcontroller3.0/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


