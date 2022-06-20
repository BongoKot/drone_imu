################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/example.c \
../src/functions.c \
../src/init.c \
../src/step.c 

OBJS += \
./src/example.o \
./src/functions.o \
./src/init.o \
./src/step.o 

C_DEPS += \
./src/example.d \
./src/functions.d \
./src/init.d \
./src/step.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


