################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include emlib/subdir.mk
-include Graphics/subdir.mk
-include Drivers/subdir.mk
-include CMSIS/EFM32PG12B/subdir.mk
-include BSP/subdir.mk
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
all: IR\ Temperature\ Sensing.axf

# Tool invocations
IR\ Temperature\ Sensing.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "IR Temperature Sensing.ld" -Xlinker --gc-sections -Xlinker -Map="IR Temperature Sensing.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o "IR Temperature Sensing.axf" "./BSP/bsp_bcc.o" "./BSP/bsp_stk.o" "./CMSIS/EFM32PG12B/startup_efm32pg12b.o" "./CMSIS/EFM32PG12B/system_efm32pg12b.o" "./Drivers/display.o" "./Drivers/displayls013b7dh03.o" "./Drivers/displaypalemlib.o" "./Drivers/i2cspm.o" "./Drivers/rtcdriver.o" "./Drivers/si7013.o" "./Drivers/udelay.o" "./Graphics/dmd_display.o" "./Graphics/glib.o" "./Graphics/glib_bitmap.o" "./Graphics/glib_circle.o" "./Graphics/glib_font_narrow_6x8.o" "./Graphics/glib_font_normal_8x8.o" "./Graphics/glib_line.o" "./Graphics/glib_rectangle.o" "./Graphics/glib_string.o" "./emlib/em_adc.o" "./emlib/em_assert.o" "./emlib/em_cmu.o" "./emlib/em_core.o" "./emlib/em_emu.o" "./emlib/em_gpio.o" "./emlib/em_i2c.o" "./emlib/em_leuart.o" "./emlib/em_rtcc.o" "./emlib/em_system.o" "./emlib/em_usart.o" "./src/graphics.o" "./src/humitemp.o" -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: IR Temperature Sensing.hex'
	arm-none-eabi-objcopy -O ihex "IR Temperature Sensing.axf" "IR Temperature Sensing.hex"
	@echo ' '

	@echo 'Building bin file: IR Temperature Sensing.bin'
	arm-none-eabi-objcopy -O binary "IR Temperature Sensing.axf" "IR Temperature Sensing.bin"
	@echo ' '

	@echo 'Building s37 file: IR Temperature Sensing.s37'
	arm-none-eabi-objcopy -O srec "IR Temperature Sensing.axf" "IR Temperature Sensing.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "IR Temperature Sensing.axf" -A
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) "IR Temperature Sensing.axf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
