################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: DroneFirmwareBeaglebone

# Tool invocations
DroneFirmwareBeaglebone: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross GCC Linker'
	arm-linux-gnueabi-gcc -L/usr/src/gcc-linaro-7.5/lib -L/usr/src/gcc-linaro-7.5/libexec -L/usr/src/gcc-linaro-7.5/arm-linux-gnueabi/lib -L/usr/src/gcc-linaro-7.5/arm-linux-gnueabi/libc -L/usr/src/gcc-linaro-7.5/arm-linux-gnueabi/libc/lib -L/usr/src/gcc-linaro-7.5/arm-linux-gnueabi/libc/usr/lib -L/usr/src/gcc-linaro-7.5/arm-linux-gnueabi/libc/usr/libexec -pthread -lm -lrt -lgpiod -o "DroneFirmwareBeaglebone" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) DroneFirmwareBeaglebone
	-@echo ' '

.PHONY: all clean dependents

-include ../makefile.targets
