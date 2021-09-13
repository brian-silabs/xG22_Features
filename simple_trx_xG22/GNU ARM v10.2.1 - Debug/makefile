################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include gecko_sdk_3.2.1/platform/service/udelay/src/subdir.mk
-include gecko_sdk_3.2.1/platform/service/system/src/subdir.mk
-include gecko_sdk_3.2.1/platform/service/sleeptimer/src/subdir.mk
-include gecko_sdk_3.2.1/platform/service/power_manager/src/subdir.mk
-include gecko_sdk_3.2.1/platform/service/mpu/src/subdir.mk
-include gecko_sdk_3.2.1/platform/service/hfxo_manager/src/subdir.mk
-include gecko_sdk_3.2.1/platform/service/device_init/src/subdir.mk
-include gecko_sdk_3.2.1/platform/radio/rail_lib/plugin/rail_util_rssi/subdir.mk
-include gecko_sdk_3.2.1/platform/radio/rail_lib/plugin/rail_util_pti/subdir.mk
-include gecko_sdk_3.2.1/platform/radio/rail_lib/plugin/rail_util_protocol/subdir.mk
-include gecko_sdk_3.2.1/platform/radio/rail_lib/plugin/pa-conversions/subdir.mk
-include gecko_sdk_3.2.1/platform/emlib/src/subdir.mk
-include gecko_sdk_3.2.1/platform/emdrv/gpiointerrupt/src/subdir.mk
-include gecko_sdk_3.2.1/platform/driver/leddrv/src/subdir.mk
-include gecko_sdk_3.2.1/platform/driver/button/src/subdir.mk
-include gecko_sdk_3.2.1/platform/common/toolchain/src/subdir.mk
-include gecko_sdk_3.2.1/platform/common/src/subdir.mk
-include gecko_sdk_3.2.1/platform/Device/SiliconLabs/EFR32MG22/Source/GCC/subdir.mk
-include gecko_sdk_3.2.1/platform/Device/SiliconLabs/EFR32MG22/Source/subdir.mk
-include gecko_sdk_3.2.1/hardware/driver/mx25_flash_shutdown/src/sl_mx25_flash_shutdown_usart/subdir.mk
-include gecko_sdk_3.2.1/hardware/board/src/subdir.mk
-include gecko_sdk_3.2.1/app/flex/component/rail/sl_flex_rail_sleep/subdir.mk
-include autogen/subdir.mk
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
all: simple_trx_xG22.axf

# Tool invocations
simple_trx_xG22.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -T "C:\Users\Brian\SimplicityStudio\v5_workspaceZ3Training\simple_trx_xG22/autogen/linkerfile.ld" -Xlinker --gc-sections -Xlinker -Map="simple_trx_xG22.map" -mfpu=fpv5-sp-d16 -mfloat-abi=hard --specs=nano.specs -o simple_trx_xG22.axf -Wl,--start-group "./app_init.o" "./app_process.o" "./main.o" "./autogen/rail_config.o" "./autogen/sl_board_default_init.o" "./autogen/sl_device_init_clocks.o" "./autogen/sl_event_handler.o" "./autogen/sl_power_manager_handler.o" "./autogen/sl_rail_util_callbacks.o" "./autogen/sl_rail_util_init.o" "./autogen/sl_simple_button_instances.o" "./autogen/sl_simple_led_instances.o" "./gecko_sdk_3.2.1/app/flex/component/rail/sl_flex_rail_sleep/sl_flex_rail_sleep.o" "./gecko_sdk_3.2.1/hardware/board/src/sl_board_control_gpio.o" "./gecko_sdk_3.2.1/hardware/board/src/sl_board_init.o" "./gecko_sdk_3.2.1/hardware/driver/mx25_flash_shutdown/src/sl_mx25_flash_shutdown_usart/sl_mx25_flash_shutdown.o" "./gecko_sdk_3.2.1/platform/Device/SiliconLabs/EFR32MG22/Source/GCC/startup_efr32mg22.o" "./gecko_sdk_3.2.1/platform/Device/SiliconLabs/EFR32MG22/Source/system_efr32mg22.o" "./gecko_sdk_3.2.1/platform/common/src/sl_slist.o" "./gecko_sdk_3.2.1/platform/common/toolchain/src/sl_memory.o" "./gecko_sdk_3.2.1/platform/driver/button/src/sl_button.o" "./gecko_sdk_3.2.1/platform/driver/button/src/sl_simple_button.o" "./gecko_sdk_3.2.1/platform/driver/leddrv/src/sl_led.o" "./gecko_sdk_3.2.1/platform/driver/leddrv/src/sl_simple_led.o" "./gecko_sdk_3.2.1/platform/emdrv/gpiointerrupt/src/gpiointerrupt.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_assert.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_burtc.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_cmu.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_core.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_emu.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_gpio.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_prs.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_rtcc.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_system.o" "./gecko_sdk_3.2.1/platform/emlib/src/em_usart.o" "./gecko_sdk_3.2.1/platform/radio/rail_lib/plugin/pa-conversions/pa_conversions_efr32.o" "./gecko_sdk_3.2.1/platform/radio/rail_lib/plugin/pa-conversions/pa_curves_efr32.o" "./gecko_sdk_3.2.1/platform/radio/rail_lib/plugin/rail_util_protocol/sl_rail_util_protocol.o" "./gecko_sdk_3.2.1/platform/radio/rail_lib/plugin/rail_util_pti/sl_rail_util_pti.o" "./gecko_sdk_3.2.1/platform/radio/rail_lib/plugin/rail_util_rssi/sl_rail_util_rssi.o" "./gecko_sdk_3.2.1/platform/service/device_init/src/sl_device_init_dcdc_s2.o" "./gecko_sdk_3.2.1/platform/service/device_init/src/sl_device_init_emu_s2.o" "./gecko_sdk_3.2.1/platform/service/device_init/src/sl_device_init_hfxo_s2.o" "./gecko_sdk_3.2.1/platform/service/device_init/src/sl_device_init_lfxo_s2.o" "./gecko_sdk_3.2.1/platform/service/device_init/src/sl_device_init_nvic.o" "./gecko_sdk_3.2.1/platform/service/hfxo_manager/src/sl_hfxo_manager.o" "./gecko_sdk_3.2.1/platform/service/hfxo_manager/src/sl_hfxo_manager_hal_s2.o" "./gecko_sdk_3.2.1/platform/service/mpu/src/sl_mpu.o" "./gecko_sdk_3.2.1/platform/service/power_manager/src/sl_power_manager.o" "./gecko_sdk_3.2.1/platform/service/power_manager/src/sl_power_manager_debug.o" "./gecko_sdk_3.2.1/platform/service/power_manager/src/sl_power_manager_hal_s2.o" "./gecko_sdk_3.2.1/platform/service/sleeptimer/src/sl_sleeptimer.o" "./gecko_sdk_3.2.1/platform/service/sleeptimer/src/sl_sleeptimer_hal_burtc.o" "./gecko_sdk_3.2.1/platform/service/sleeptimer/src/sl_sleeptimer_hal_prortc.o" "./gecko_sdk_3.2.1/platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.o" "./gecko_sdk_3.2.1/platform/service/system/src/sl_system_init.o" "./gecko_sdk_3.2.1/platform/service/system/src/sl_system_process_action.o" "./gecko_sdk_3.2.1/platform/service/udelay/src/sl_udelay.o" "./gecko_sdk_3.2.1/platform/service/udelay/src/sl_udelay_armv6m_gcc.o" "C:/SiliconLabs/SimplicityStudio/v5_3/developer/sdks/gecko_sdk_suite/v3.2//platform/radio/rail_lib/autogen/librail_release/librail_efr32xg22_gcc_release.a" -lgcc -lc -lm -lnosys -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: simple_trx_xG22.hex'
	arm-none-eabi-objcopy -O ihex "simple_trx_xG22.axf" "simple_trx_xG22.hex"
	@echo ' '

	@echo 'Building bin file: simple_trx_xG22.bin'
	arm-none-eabi-objcopy -O binary "simple_trx_xG22.axf" "simple_trx_xG22.bin"
	@echo ' '

	@echo 'Building s37 file: simple_trx_xG22.s37'
	arm-none-eabi-objcopy -O srec "simple_trx_xG22.axf" "simple_trx_xG22.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "simple_trx_xG22.axf" -A
	@echo ' '

# Other Targets
clean:
	-@$(foreach FILE, $(EXECUTABLES), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(OBJS), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(C_DEPS), $(shell $(RM) $(FILE)) )
	-$(RM) simple_trx_xG22.axf
	-@echo ' '

.PHONY: all clean dependents

-include ../makefile.targets
