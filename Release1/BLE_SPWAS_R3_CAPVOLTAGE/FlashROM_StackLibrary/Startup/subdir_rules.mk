################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Startup/CC2640R2_LAUNCHXL.obj: ../Startup/CC2640R2_LAUNCHXL.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"D:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/bin/armcl" --cmd_file="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/config/build_components.opt" --cmd_file="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/config/factory_config.opt" --cmd_file="C:/Users/Uni/Documents/ccs_local/project_zero_cc2640r2lp_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/examples/rtos/CC2640R2_LAUNCHXL/blestack/project_zero/src/extra/" --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE/Application" --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE/Startup" --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE/PROFILES" --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE/Include" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/controller/cc26xx_r2/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/rom" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/common/cc26xx" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/examples/rtos/CC2640R2_LAUNCHXL/blestack/project_zero/src/app" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/icall/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/dev_info" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/roles/cc26xx" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/roles" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/simple_profile/cc26xx" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/simple_profile" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/target" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/hal/src/target/_common" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/hal/src/target/_common/cc26xx" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/hal/src/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/heapmgr" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/icall/src/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/osal/src/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/services/src/saddr" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/services/src/sdata" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/devices/cc26x0r2" --include_path="D:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/include" --define=L2CAP_CONN_UPDATE --define=DeviceFamily_CC26X0R2 --define=BOARD_DISPLAY_USE_LCD=0 --define=BOARD_DISPLAY_USE_UART=0 --define=BOARD_DISPLAY_USE_UART_ANSI=0 --define=Display_DISABLE_ALL --define=CC2640R2_LAUNCHXL --define=CC26XX --define=CC26XX_R2 --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=ICALL_MAX_NUM_ENTITIES=6 --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_STACK0_ADDR --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_ICALL --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL --define=UARTLOG_ENABLE --define=UARTLOG_NUM_EVT_BUF=32 --define=uartlog_FILE="\"CC2640R2_LAUNCHXL.c\"" -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="Startup/CC2640R2_LAUNCHXL.d_raw" --obj_directory="Startup" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Startup/UartLog.obj: C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/examples/rtos/CC2640R2_LAUNCHXL/blestack/project_zero/src/extra/uartlog/UartLog.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"D:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/bin/armcl" --cmd_file="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/config/build_components.opt" --cmd_file="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/config/factory_config.opt" --cmd_file="C:/Users/Uni/Documents/ccs_local/project_zero_cc2640r2lp_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/examples/rtos/CC2640R2_LAUNCHXL/blestack/project_zero/src/extra/" --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE/Application" --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE/Startup" --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE/PROFILES" --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE/Include" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/controller/cc26xx_r2/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/rom" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/common/cc26xx" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/examples/rtos/CC2640R2_LAUNCHXL/blestack/project_zero/src/app" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/icall/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/dev_info" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/roles/cc26xx" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/roles" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/simple_profile/cc26xx" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/simple_profile" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/target" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/hal/src/target/_common" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/hal/src/target/_common/cc26xx" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/hal/src/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/heapmgr" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/icall/src/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/osal/src/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/services/src/saddr" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/services/src/sdata" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/devices/cc26x0r2" --include_path="D:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/include" --define=L2CAP_CONN_UPDATE --define=DeviceFamily_CC26X0R2 --define=BOARD_DISPLAY_USE_LCD=0 --define=BOARD_DISPLAY_USE_UART=0 --define=BOARD_DISPLAY_USE_UART_ANSI=0 --define=Display_DISABLE_ALL --define=CC2640R2_LAUNCHXL --define=CC26XX --define=CC26XX_R2 --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=ICALL_MAX_NUM_ENTITIES=6 --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_STACK0_ADDR --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_ICALL --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL --define=UARTLOG_ENABLE --define=UARTLOG_NUM_EVT_BUF=32 --define=uartlog_FILE="\"UartLog.c\"" -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="Startup/UartLog.d_raw" --obj_directory="Startup" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Startup/ccfg_app_ble.obj: ../Startup/ccfg_app_ble.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"D:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/bin/armcl" --cmd_file="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/config/build_components.opt" --cmd_file="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/config/factory_config.opt" --cmd_file="C:/Users/Uni/Documents/ccs_local/project_zero_cc2640r2lp_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/examples/rtos/CC2640R2_LAUNCHXL/blestack/project_zero/src/extra/" --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE/Application" --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE/Startup" --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE/PROFILES" --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE/Include" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/controller/cc26xx_r2/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/rom" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/common/cc26xx" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/examples/rtos/CC2640R2_LAUNCHXL/blestack/project_zero/src/app" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/icall/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/dev_info" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/roles/cc26xx" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/roles" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/simple_profile/cc26xx" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/simple_profile" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/target" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/hal/src/target/_common" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/hal/src/target/_common/cc26xx" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/hal/src/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/heapmgr" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/icall/src/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/osal/src/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/services/src/saddr" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/services/src/sdata" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/devices/cc26x0r2" --include_path="D:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/include" --define=L2CAP_CONN_UPDATE --define=DeviceFamily_CC26X0R2 --define=BOARD_DISPLAY_USE_LCD=0 --define=BOARD_DISPLAY_USE_UART=0 --define=BOARD_DISPLAY_USE_UART_ANSI=0 --define=Display_DISABLE_ALL --define=CC2640R2_LAUNCHXL --define=CC26XX --define=CC26XX_R2 --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=ICALL_MAX_NUM_ENTITIES=6 --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_STACK0_ADDR --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_ICALL --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL --define=UARTLOG_ENABLE --define=UARTLOG_NUM_EVT_BUF=32 --define=uartlog_FILE="\"ccfg_app_ble.c\"" -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="Startup/ccfg_app_ble.d_raw" --obj_directory="Startup" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Startup/main.obj: C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/examples/rtos/CC2640R2_LAUNCHXL/blestack/project_zero/src/app/main.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"D:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/bin/armcl" --cmd_file="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/config/build_components.opt" --cmd_file="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/config/factory_config.opt" --cmd_file="C:/Users/Uni/Documents/ccs_local/project_zero_cc2640r2lp_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/examples/rtos/CC2640R2_LAUNCHXL/blestack/project_zero/src/extra/" --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE/Application" --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE/Startup" --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE/PROFILES" --include_path="C:/Users/Uni/Documents/ccs_local/BLE_SPWAS_R3_CAPVOLTAGE/Include" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/controller/cc26xx_r2/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/rom" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/common/cc26xx" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/examples/rtos/CC2640R2_LAUNCHXL/blestack/project_zero/src/app" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/icall/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/dev_info" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/roles/cc26xx" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/roles" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/simple_profile/cc26xx" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/profiles/simple_profile" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/target" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/hal/src/target/_common" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/hal/src/target/_common/cc26xx" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/hal/src/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/heapmgr" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/icall/src/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/osal/src/inc" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/services/src/saddr" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/blestack/services/src/sdata" --include_path="C:/ti/simplelink_cc2640r2_sdk_3_10_00_15/source/ti/devices/cc26x0r2" --include_path="D:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/include" --define=L2CAP_CONN_UPDATE --define=DeviceFamily_CC26X0R2 --define=BOARD_DISPLAY_USE_LCD=0 --define=BOARD_DISPLAY_USE_UART=0 --define=BOARD_DISPLAY_USE_UART_ANSI=0 --define=Display_DISABLE_ALL --define=CC2640R2_LAUNCHXL --define=CC26XX --define=CC26XX_R2 --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=ICALL_MAX_NUM_ENTITIES=6 --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_STACK0_ADDR --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_ICALL --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL --define=UARTLOG_ENABLE --define=UARTLOG_NUM_EVT_BUF=32 --define=uartlog_FILE="\"main.c\"" -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="Startup/main.d_raw" --obj_directory="Startup" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


