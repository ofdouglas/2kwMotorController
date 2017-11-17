################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /home/odougs/bin/ti/ccsv7/tools/compiler/gcc-arm-none-eabi-6-2017-q1-update

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./startup.o" \
"./FreeRTOS/event_groups.o" \
"./FreeRTOS/heap_1.o" \
"./FreeRTOS/list.o" \
"./FreeRTOS/port.o" \
"./FreeRTOS/queue.o" \
"./FreeRTOS/tasks.o" \
"./FreeRTOS/timers.o" \
"./source/can.o" \
"./source/control.o" \
"./source/debug.o" \
"./source/encoder.o" \
"./source/fir.o" \
"./source/logger.o" \
"./source/main.o" \
"./source/pinconfig.o" \
"./source/sensors.o" \
"./source/system.o" \
"./source/thermistor_lut.o" \
"./source/xprintf.o" \
-Wl,-T"../tm4c1294kcpdt.lds" \
$(GEN_CMDS__FLAG) \
-Wl,--start-group \
-lc \
-ldriver \
-Wl,--end-group \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include FreeRTOS/subdir_vars.mk
-include source/subdir_vars.mk
-include subdir_rules.mk
-include FreeRTOS/subdir_rules.mk
-include source/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
MotorController.out \

EXE_OUTPUTS__QUOTED += \
"MotorController.out" \

BIN_OUTPUTS += \
MotorController.hex \

BIN_OUTPUTS__QUOTED += \
"MotorController.hex" \


# All Target
all: $(OBJS) $(LDS_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "MotorController.out"

# Tool invocations
MotorController.out: $(OBJS) $(LDS_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU Linker'
	"/home/odougs/bin/ti/ccsv7/tools/compiler/gcc-arm-none-eabi-6-2017-q1-update/bin/arm-none-eabi-gcc" -DPART_TM4C1294KCPDT -O3 -ffunction-sections -fdata-sections -fsingle-precision-constant -g -gdwarf-3 -gstrict-dwarf -Wall -specs="nosys.specs" -MD -std=c99 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Wl,-Map,"MotorController.map" -L"/mnt/storage/project/2kwMotorController/firmware/Tivaware/driverlib/gcc" -o"MotorController.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

MotorController.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: GNU Objcopy Utility'
	"/home/odougs/bin/ti/ccsv7/tools/compiler/gcc-arm-none-eabi-6-2017-q1-update/bin/arm-none-eabi-objcopy"  $(EXE_OUTPUTS__QUOTED) "MotorController.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "startup.o" "FreeRTOS/event_groups.o" "FreeRTOS/heap_1.o" "FreeRTOS/list.o" "FreeRTOS/port.o" "FreeRTOS/queue.o" "FreeRTOS/tasks.o" "FreeRTOS/timers.o" "source/can.o" "source/control.o" "source/debug.o" "source/encoder.o" "source/fir.o" "source/logger.o" "source/main.o" "source/pinconfig.o" "source/sensors.o" "source/system.o" "source/thermistor_lut.o" "source/xprintf.o" 
	-$(RM) "startup.d" "FreeRTOS/event_groups.d" "FreeRTOS/heap_1.d" "FreeRTOS/list.d" "FreeRTOS/port.d" "FreeRTOS/queue.d" "FreeRTOS/tasks.d" "FreeRTOS/timers.d" "source/can.d" "source/control.d" "source/debug.d" "source/encoder.d" "source/fir.d" "source/logger.d" "source/main.d" "source/pinconfig.d" "source/sensors.d" "source/system.d" "source/thermistor_lut.d" "source/xprintf.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

