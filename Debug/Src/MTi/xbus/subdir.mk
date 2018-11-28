################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/MTi/xbus/xbusmessage.c \
../Src/MTi/xbus/xbusparser.c \
../Src/MTi/xbus/xbusutility.c \
../Src/MTi/xbus/xsdeviceid.c 

OBJS += \
./Src/MTi/xbus/xbusmessage.o \
./Src/MTi/xbus/xbusparser.o \
./Src/MTi/xbus/xbusutility.o \
./Src/MTi/xbus/xsdeviceid.o 

C_DEPS += \
./Src/MTi/xbus/xbusmessage.d \
./Src/MTi/xbus/xbusparser.d \
./Src/MTi/xbus/xbusutility.d \
./Src/MTi/xbus/xsdeviceid.d 


# Each subdirectory must supply rules for building sources it contributes
Src/MTi/xbus/%.o: ../Src/MTi/xbus/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F417xx -I"C:/RoboTeam/CODE/roboteam_microcontroller3.0/Inc" -I"C:/RoboTeam/CODE/roboteam_microcontroller3.0/Inc/Utils" -I"C:/RoboTeam/CODE/roboteam_microcontroller3.0/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/RoboTeam/CODE/roboteam_microcontroller3.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/RoboTeam/CODE/roboteam_microcontroller3.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/RoboTeam/CODE/roboteam_microcontroller3.0/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


