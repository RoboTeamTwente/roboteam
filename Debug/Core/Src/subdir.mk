################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Wireless.c \
../Core/Src/buzzer.c \
../Core/Src/dribbler.c \
../Core/Src/geneva.c \
../Core/Src/main.c \
../Core/Src/shoot.c \
../Core/Src/stateControl.c \
../Core/Src/stateEstimation.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/system_stm32f7xx.c \
../Core/Src/wheels.c 

OBJS += \
./Core/Src/Wireless.o \
./Core/Src/buzzer.o \
./Core/Src/dribbler.o \
./Core/Src/geneva.o \
./Core/Src/main.o \
./Core/Src/shoot.o \
./Core/Src/stateControl.o \
./Core/Src/stateEstimation.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/system_stm32f7xx.o \
./Core/Src/wheels.o 

C_DEPS += \
./Core/Src/Wireless.d \
./Core/Src/buzzer.d \
./Core/Src/dribbler.d \
./Core/Src/geneva.d \
./Core/Src/main.d \
./Core/Src/shoot.d \
./Core/Src/stateControl.d \
./Core/Src/stateEstimation.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/system_stm32f7xx.d \
./Core/Src/wheels.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F767xx '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -I"/home/kjhertenberg/roboteam_microcontroller3.0/Core/Inc" -I"/home/kjhertenberg/roboteam_microcontroller3.0/Util" -I"/home/kjhertenberg/roboteam_microcontroller3.0/Lib" -I"/home/kjhertenberg/roboteam_microcontroller3.0/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/kjhertenberg/roboteam_microcontroller3.0/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/kjhertenberg/roboteam_microcontroller3.0/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"/home/kjhertenberg/roboteam_microcontroller3.0/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


