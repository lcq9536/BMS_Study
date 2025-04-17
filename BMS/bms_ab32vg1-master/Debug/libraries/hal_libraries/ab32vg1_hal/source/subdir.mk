################################################################################
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal.c \
../libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal_dac.c \
../libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal_gpio.c \
../libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal_rcu.c \
../libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal_sd.c \
../libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal_uart.c 

OBJS += \
./libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal.o \
./libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal_dac.o \
./libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal_gpio.o \
./libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal_rcu.o \
./libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal_sd.o \
./libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal_uart.o 

C_DEPS += \
./libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal.d \
./libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal_dac.d \
./libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal_gpio.d \
./libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal_rcu.d \
./libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal_sd.d \
./libraries/hal_libraries/ab32vg1_hal/source/ab32vg1_hal_uart.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/hal_libraries/ab32vg1_hal/source/%.o: ../libraries/hal_libraries/ab32vg1_hal/source/%.c
	riscv64-unknown-elf-gcc  -DDEBUG -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\applications" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\board" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libcpu\cpu" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_drivers\config" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_drivers" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_libraries\ab32vg1_hal\include" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_libraries\ab32vg1_hal" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_libraries\bmsis\include" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_libraries\bmsis" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\packages\at24cxx-latest" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\packages\bluetrum_sdk-latest" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\drivers\include" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\drivers\sensors" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\drivers\spi" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\finsh" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\libc\compilers\common" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\libc\compilers\gcc\newlib" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\include" -isystem"C:\RT-ThreadStudio\workspace\BMS_AB32VG1" -include"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rtconfig_preinc.h" -std=gnu11 -c -mcmodel=medany -march=rv32imc -mabi=ilp32 -msave-restore -Os -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

