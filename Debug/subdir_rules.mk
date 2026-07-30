################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-345778635: ../empty.syscfg
	@echo 'SysConfig - building file: "$<"'
	"D:/Tools/CCS2050/sysconfig_1.26.2/sysconfig_cli.bat" -s "C:/ti/mspm0_sdk_2_11_00_07/.metadata/product.json" --script "C:/Users/Administrator/workspace_ccstheia/tube/empty.syscfg" -o "." --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-345778635 ../empty.syscfg
device.opt: build-345778635
device.cmd.genlibs: build-345778635
ti_msp_dl_config.c: build-345778635
ti_msp_dl_config.h: build-345778635
Event.dot: build-345778635

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/Tools/CCS2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/Administrator/workspace_ccstheia/tube/userfun" -I"C:/Users/Administrator/workspace_ccstheia/tube/userfun/OLED" -I"C:/Users/Administrator/workspace_ccstheia/tube" -I"C:/Users/Administrator/workspace_ccstheia/tube/Debug" -I"C:/ti/mspm0_sdk_2_11_00_07/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_11_00_07/source" -g -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: C:/ti/mspm0_sdk_2_11_00_07/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/Tools/CCS2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/Administrator/workspace_ccstheia/tube/userfun" -I"C:/Users/Administrator/workspace_ccstheia/tube/userfun/OLED" -I"C:/Users/Administrator/workspace_ccstheia/tube" -I"C:/Users/Administrator/workspace_ccstheia/tube/Debug" -I"C:/ti/mspm0_sdk_2_11_00_07/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_11_00_07/source" -g -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/Tools/CCS2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/Administrator/workspace_ccstheia/tube/userfun" -I"C:/Users/Administrator/workspace_ccstheia/tube/userfun/OLED" -I"C:/Users/Administrator/workspace_ccstheia/tube" -I"C:/Users/Administrator/workspace_ccstheia/tube/Debug" -I"C:/ti/mspm0_sdk_2_11_00_07/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_11_00_07/source" -g -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


