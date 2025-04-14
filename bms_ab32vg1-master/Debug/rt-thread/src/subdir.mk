################################################################################
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../rt-thread/src/clock.c \
../rt-thread/src/components.c \
../rt-thread/src/device.c \
../rt-thread/src/idle.c \
../rt-thread/src/ipc.c \
../rt-thread/src/irq.c \
../rt-thread/src/kservice.c \
../rt-thread/src/mem.c \
../rt-thread/src/memheap.c \
../rt-thread/src/mempool.c \
../rt-thread/src/object.c \
../rt-thread/src/scheduler.c \
../rt-thread/src/thread.c \
../rt-thread/src/timer.c 

OBJS += \
./rt-thread/src/clock.o \
./rt-thread/src/components.o \
./rt-thread/src/device.o \
./rt-thread/src/idle.o \
./rt-thread/src/ipc.o \
./rt-thread/src/irq.o \
./rt-thread/src/kservice.o \
./rt-thread/src/mem.o \
./rt-thread/src/memheap.o \
./rt-thread/src/mempool.o \
./rt-thread/src/object.o \
./rt-thread/src/scheduler.o \
./rt-thread/src/thread.o \
./rt-thread/src/timer.o 

C_DEPS += \
./rt-thread/src/clock.d \
./rt-thread/src/components.d \
./rt-thread/src/device.d \
./rt-thread/src/idle.d \
./rt-thread/src/ipc.d \
./rt-thread/src/irq.d \
./rt-thread/src/kservice.d \
./rt-thread/src/mem.d \
./rt-thread/src/memheap.d \
./rt-thread/src/mempool.d \
./rt-thread/src/object.d \
./rt-thread/src/scheduler.d \
./rt-thread/src/thread.d \
./rt-thread/src/timer.d 


# Each subdirectory must supply rules for building sources it contributes
rt-thread/src/%.o: ../rt-thread/src/%.c
	riscv64-unknown-elf-gcc  -DDEBUG -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\applications" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\board" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libcpu\cpu" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_drivers\config" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_drivers" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_libraries\ab32vg1_hal\include" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_libraries\ab32vg1_hal" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_libraries\bmsis\include" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_libraries\bmsis" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\packages\at24cxx-latest" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\packages\bluetrum_sdk-latest" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\drivers\include" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\drivers\sensors" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\drivers\spi" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\finsh" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\libc\compilers\common" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\libc\compilers\gcc\newlib" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\include" -isystem"C:\RT-ThreadStudio\workspace\BMS_AB32VG1" -include"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rtconfig_preinc.h" -std=gnu11 -c -mcmodel=medany -march=rv32imc -mabi=ilp32 -msave-restore -Os -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

