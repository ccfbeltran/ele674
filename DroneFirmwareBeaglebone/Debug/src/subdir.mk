################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/Attitude.c \
../src/Control.c \
../src/DroneFirmware.c \
../src/Mavlink.c \
../src/Motor.c \
../src/Sensor.c 

OBJS += \
./src/Attitude.o \
./src/Control.o \
./src/DroneFirmware.o \
./src/Mavlink.o \
./src/Motor.o \
./src/Sensor.o 

C_DEPS += \
./src/Attitude.d \
./src/Control.d \
./src/DroneFirmware.d \
./src/Mavlink.d \
./src/Motor.d \
./src/Sensor.d 


# Each subdirectory must supply rules for building sources it contributes
src/Attitude.o: ../src/Attitude.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-linux-gnueabi-gcc -I"/home/ens/AP27240/Bureau/ELE674_LAB2/DroneFirmwareBeaglebone/src" -I"/home/ens/AP27240/Bureau/ELE674_LAB2/DroneFirmwareBeaglebone/include" -I"/home/ens/AP27240/Bureau/ELE674_LAB2/DroneFirmwareBeaglebone/include" -O0 -g3 -ggdb -Wall -pthread -c -fmessage-length=0 -pthread -lm -lrt -lgpiod -MMD -MP -MF"$(@:%.o=%.d)" -MT"src/Attitude.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-linux-gnueabi-gcc -I/usr/src/gcc-linaro-7.5/include -I/usr/src/gcc-linaro-7.5/arm-linux-gnueabi/include -I/usr/src/gcc-linaro-7.5/arm-linux-gnueabi/libc/usr/include -I/usr/src/gcc-linaro-7.5/lib/gcc/arm-linux-gnueabi/7.5.0/include -I/usr/src/gcc-linaro-7.5/lib/gcc/arm-linux-gnueabi/7.5.0/include-fixed -I/usr/src/gcc-linaro-7.5/lib/gcc/arm-linux-gnueabi/7.5.0/plugin/include -I/usr/src/gcc-linaro-7.5/lib/gcc/arm-linux-gnueabi/7.5.0/install-tools/include -I"/home/ens/AP27240/Bureau/ELE674_LAB2/DroneFirmwareBeaglebone/include" -I"/home/ens/AP27240/Bureau/ELE674_LAB2/DroneFirmwareBeaglebone/src" -O0 -g3 -ggdb -Wall -pthread -c -fmessage-length=0 -pthread -lm -lrt -lgpiod -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


