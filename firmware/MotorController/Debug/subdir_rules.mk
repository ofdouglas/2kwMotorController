################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Compiler'
	"/home/odougs/bin/ti/ccsv7/tools/compiler/gcc-arm-none-eabi-6-2017-q1-update/bin/arm-none-eabi-gcc" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DPART_TM4C1294KCPDT -I"/mnt/storage/project/2kwMotorController/firmware/MotorController" -I"/mnt/storage/project/2kwMotorController/firmware/MotorController/include" -I"/mnt/storage/project/2kwMotorController/firmware/xprintf" -I"/mnt/storage/project/2kwMotorController/firmware/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/mnt/storage/project/2kwMotorController/firmware/FreeRTOS/Source/include" -I"/mnt/storage/project/2kwMotorController/firmware/Tivaware" -I"/home/odougs/bin/ti/ccsv7/tools/compiler/gcc-arm-none-eabi-6-2017-q1-update/arm-none-eabi/include" -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -specs="nosys.specs" -MD -std=c99 -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o"$@" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '


