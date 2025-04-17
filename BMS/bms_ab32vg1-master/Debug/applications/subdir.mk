################################################################################
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../applications/bms.c \
../applications/cellvt.c \
../applications/ina237.c \
../applications/ltc6811.c \
../applications/ltc681x.c \
../applications/main.c \
../applications/mnt.c \
../applications/my_iic.c \
../applications/my_spi.c \
../applications/my_task.c \
../applications/my_timer.c \
../applications/packetvc.c \
../applications/temperature.c 

OBJS += \
./applications/bms.o \
./applications/cellvt.o \
./applications/ina237.o \
./applications/ltc6811.o \
./applications/ltc681x.o \
./applications/main.o \
./applications/mnt.o \
./applications/my_iic.o \
./applications/my_spi.o \
./applications/my_task.o \
./applications/my_timer.o \
./applications/packetvc.o \
./applications/temperature.o 

C_DEPS += \
./applications/bms.d \
./applications/cellvt.d \
./applications/ina237.d \
./applications/ltc6811.d \
./applications/ltc681x.d \
./applications/main.d \
./applications/mnt.d \
./applications/my_iic.d \
./applications/my_spi.d \
./applications/my_task.d \
./applications/my_timer.d \
./applications/packetvc.d \
./applications/temperature.d 


# Each subdirectory must supply rules for building sources it contributes
applications/%.o: ../applications/%.c
	riscv64-unknown-elf-gcc  -DDEBUG -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\applications" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\board" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libcpu\cpu" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_drivers\config" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_drivers" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_libraries\ab32vg1_hal\include" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_libraries\ab32vg1_hal" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_libraries\bmsis\include" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_libraries\bmsis" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\packages\at24cxx-latest" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\packages\bluetrum_sdk-latest" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\drivers\include" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\drivers\sensors" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\drivers\spi" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\finsh" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\libc\compilers\common" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\libc\compilers\gcc\newlib" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\include" -isystem"C:\RT-ThreadStudio\workspace\BMS_AB32VG1" -include"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rtconfig_preinc.h" -std=gnu11 -c -mcmodel=medany -march=rv32imc -mabi=ilp32 -msave-restore -Os -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

