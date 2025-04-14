################################################################################
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../board/ab32vg1_hal_msp.c \
../board/board.c 

OBJS += \
./board/ab32vg1_hal_msp.o \
./board/board.o 

C_DEPS += \
./board/ab32vg1_hal_msp.d \
./board/board.d 


# Each subdirectory must supply rules for building sources it contributes
board/%.o: ../board/%.c
	riscv64-unknown-elf-gcc  -DDEBUG -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\applications" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\board" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libcpu\cpu" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_drivers\config" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_drivers" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_libraries\ab32vg1_hal\include" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_libraries\ab32vg1_hal" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_libraries\bmsis\include" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\libraries\hal_libraries\bmsis" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\packages\at24cxx-latest" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\packages\bluetrum_sdk-latest" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\drivers\include" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\drivers\sensors" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\drivers\spi" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\finsh" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\libc\compilers\common" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\components\libc\compilers\gcc\newlib" -I"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rt-thread\include" -isystem"C:\RT-ThreadStudio\workspace\BMS_AB32VG1" -include"C:\RT-ThreadStudio\workspace\BMS_AB32VG1\rtconfig_preinc.h" -std=gnu11 -c -mcmodel=medany -march=rv32imc -mabi=ilp32 -msave-restore -Os -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

