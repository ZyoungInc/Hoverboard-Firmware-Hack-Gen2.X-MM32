# GCC build for the MindMotion hoverboard firmware.
#   make                  main firmware build/spin05/hoverboard.{elf,bin,hex} for the MM32SPIN05
#   make MCU=F031         another MCU: SPIN05 (default), SPIN06, SPIN25, F031; output in build/<mcu>/
#   make pinfinder        PinFinder build/<mcu>/pinfinder.{elf,bin,hex}
#   make both             main firmware and PinFinder for MCU
#   make targets          main firmware and PinFinder for every MCU
#   make release          targets, then the images copied to ../release/<mcu>/
#   make CROSS=/path/to/arm-none-eabi-    use a toolchain that is not on PATH
#   make flash            flash over SWD with pyOCD (../tools/mm32flash.py, keeps the board config)
#   make flash-pinfinder
#   make clean            this MCU's build;  make clean-all: every MCU and ../release
#   make help

CROSS   ?= arm-none-eabi-
BOARD   ?= GENERIC
CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
SIZE    := $(CROSS)size

# All MCUs share the SPIN05 register headers and HAL: their peripherals have the same register layout
# (compared in the vendor SVD files). They differ in the pin tables (hardware.c), the clock (SPIN25: PLL)
# and the op-amps (SPIN25: enabled per configured pin). Every image uses the same 32 KB layout (31 KB
# code, the config in the last 1 KB page) and 4 KB of RAM, the smallest of the four, so the tools and the
# config address are the same on every board.
MCUS    := SPIN05 SPIN06 SPIN25 F031
MCU     ?= SPIN05
ifeq ($(filter $(MCU),$(MCUS)),)
$(error MCU=$(MCU): one of $(MCUS))
endif
mcu_dir  = $(shell echo $(1) | tr A-Z a-z)
BUILD   := build/$(call mcu_dir,$(MCU))
TARGET  := $(BUILD)/hoverboard
LDSCRIPT := Device/MM32SPIN0x/Source/mm32spin05.ld

SRC := $(filter-out Src/i2c.c,$(wildcard Src/*.c)) \
       $(wildcard Library/MM32SPIN05/HAL_Lib/Src/*.c) \
       Device/MM32SPIN0x/Source/system_mm32spin05.c \
       Device/MM32SPIN0x/Source/startup_mm32spin0x_gcc.c
ASRC := $(wildcard Src/*.S)

# vendor and CMSIS headers as system headers: their warnings are not ours to fix
INC := -IInc -isystem Library/MM32SPIN05/HAL_Lib/Inc -isystem Device/MM32SPIN0x/Include -isystem Device/CMSIS/Include

ARCH    := -mcpu=cortex-m0 -mthumb
# -fno-strict-aliasing: the vendor HAL type-puns peripheral pointers (*(u32*)&gpio)
# -flto: 3 KB less flash (of 30 KB); the SWD tools still find the globals by name
CFLAGS  := $(ARCH) -Os -flto -g3 -std=gnu11 -ffunction-sections -fdata-sections -fno-common \
           -fno-strict-aliasing -Wall $(INC) -DMCU_$(MCU)
ifeq ($(BOARD),GHOST_GEN242)
# This board profile has no configured IMU; leave its serial endpoints reporting IMU_OFF.
CFLAGS  += -DGHOST_GEN242
else ifneq ($(BOARD),GENERIC)
$(error BOARD=$(BOARD): use GENERIC or GHOST_GEN242)
endif
# make TRACE=1: the FOC capture buffer for SWD tools (foc.c, 768 bytes of RAM)
ifeq ($(TRACE),1)
CFLAGS  += -DFOC_TRACE
endif
# stricter for our own code (Src/, startup); the vendor HAL sources keep -Wall
WARN    := -Wextra -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wundef
$(BUILD)/Src/%.o $(BUILD)/Device/MM32SPIN0x/Source/startup_mm32spin0x_gcc.o: CFLAGS += $(WARN)
# -nostartfiles: startup_mm32spin0x_gcc.c is the startup; newlib's crt0 pulled in exit() and stdio
LDFLAGS := $(ARCH) -Os -flto -T$(LDSCRIPT) --specs=nano.specs --specs=nosys.specs -nostartfiles -Wl,--gc-sections \
           -Wl,-Map=$(TARGET).map -Wl,--print-memory-usage

OBJ := $(addprefix $(BUILD)/,$(SRC:.c=.o) $(ASRC:.S=.o))
DEP := $(OBJ:.o=.d)

all: $(TARGET).bin $(TARGET).hex

# objects and the link depend on this Makefile and on the flags used (TRACE=1, CROSS on the command line):
# a flag change rebuilds everything
FLAGS_STAMP := $(BUILD)/flags.txt
$(shell mkdir -p $(BUILD); echo '$(CC) $(CFLAGS)' | cmp -s - $(FLAGS_STAMP) || echo '$(CC) $(CFLAGS)' > $(FLAGS_STAMP))
$(BUILD)/%.o: %.c Makefile $(FLAGS_STAMP)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/%.o: %.S Makefile $(FLAGS_STAMP)
	@mkdir -p $(dir $@)
	$(CC) $(ARCH) -c $< -o $@

$(TARGET).elf: $(OBJ) $(LDSCRIPT) Makefile $(FLAGS_STAMP)
	$(CC) $(LDFLAGS) $(OBJ) -lm -o $@
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

PYTHON ?= python3
flash: $(TARGET).bin
	$(PYTHON) ../tools/mm32flash.py $<

# PinFinder: a separate image (the main firmware has no room for it) built from PinFinder/ and the shared
# hardware, config, uart and startup code. make pinfinder / make flash-pinfinder
PF       := $(BUILD)/pinfinder
PF_SRC   := Src/hardware.c Src/initialize.c Src/sim_eeprom.c Src/uart.c Src/softfloat.c Src/i2c.c Src/imumath.c \
            $(wildcard Library/MM32SPIN05/HAL_Lib/Src/*.c) \
            Device/MM32SPIN0x/Source/system_mm32spin05.c Device/MM32SPIN0x/Source/startup_mm32spin0x_gcc.c
# PinFinder's own sources are compiled from copies with the messages compressed (../tools/pfstrings.py,
# expanded by out() and eq()); the config names too (the copy of cfgnames.h there is found first); pfdict.c
# is the pair table
PF_C     := $(wildcard PinFinder/*.c)
PFZ      := $(BUILD)/pfz
PFZ_SRC  := $(addprefix $(PFZ)/,$(notdir $(PF_C)) pfdict.c)
PF_OBJ   := $(addprefix $(BUILD)/,$(PF_SRC:.c=.o) Src/hwdiv.o) $(PFZ_SRC:.c=.o)
$(PFZ)/%.o: CFLAGS += $(WARN) -IPinFinder

$(PFZ)/stamp: $(PF_C) PinFinder/cfgnames.h ../tools/pfstrings.py
	$(PYTHON) ../tools/pfstrings.py $(PFZ) $(PF_C) PinFinder/cfgnames.h
	@touch $@
$(PFZ_SRC): $(PFZ)/stamp ;

$(PFZ)/%.o: $(PFZ)/%.c Makefile $(FLAGS_STAMP)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

pinfinder: $(PF).bin $(PF).hex

both: all pinfinder

$(PF).elf: $(PF_OBJ) $(LDSCRIPT) Makefile $(FLAGS_STAMP)
	$(CC) $(subst $(TARGET).map,$(PF).map,$(LDFLAGS)) $(PF_OBJ) -o $@
	$(SIZE) $@

$(PF).bin: $(PF).elf
	$(OBJCOPY) -O binary $< $@

$(PF).hex: $(PF).elf
	$(OBJCOPY) -O ihex $< $@

flash-pinfinder: $(PF).bin
	$(PYTHON) ../tools/mm32flash.py $<

# Ghost Gen2.4.2 motor alignment: PinFinder's field/leadScore code, with only
# the three already configured Hall pins. No GPIO scan, TUI, config save or
# automatic voltage increase. Separate image because full PinFinder exceeds
# the SPIN05 31 KB firmware region with this toolchain.
PFH      := $(BUILD)/pinfinder-hall
PFH_C    := PinFinder/pf_cfg.c PinFinder/pf_cmd.c PinFinder/pf_detect.c \
            PinFinder/pf_hw.c PinFinder/pf_io.c PinFinder/pf_main.c
PFH_Z    := $(BUILD)/pfh
PFH_ZSRC := $(addprefix $(PFH_Z)/,$(notdir $(PFH_C)) pfdict.c)
PFH_OBJ  := $(addprefix $(BUILD)/,$(PF_SRC:.c=.o) Src/hwdiv.o) $(PFH_ZSRC:.c=.o)
$(PFH_Z)/%.o: CFLAGS += $(WARN) -IPinFinder -DPF_HALL_ONLY
$(PFH_Z)/stamp: $(PFH_C) PinFinder/cfgnames.h ../tools/pfstrings.py
	$(PYTHON) ../tools/pfstrings.py $(PFH_Z) $(PFH_C) PinFinder/cfgnames.h
	@touch $@
$(PFH_ZSRC): $(PFH_Z)/stamp ;
$(PFH_Z)/%.o: $(PFH_Z)/%.c Makefile $(FLAGS_STAMP)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@
pinfinder-hall: $(PFH).bin $(PFH).hex
$(PFH).elf: $(PFH_OBJ) $(LDSCRIPT) Makefile $(FLAGS_STAMP)
	$(CC) $(subst $(TARGET).map,$(PFH).map,$(LDFLAGS)) $(PFH_OBJ) -o $@
	$(SIZE) $@
$(PFH).bin: $(PFH).elf
	$(OBJCOPY) -O binary $< $@
$(PFH).hex: $(PFH).elf
	$(OBJCOPY) -O ihex $< $@

# Fixed Ghost motor calibration: known Hall and PA2/PA6/PA7 analog pins only.
# The current routine is the author's phase-mapping part of DetectCurrent,
# bounded to 200..500 mV for phase mapping, or 600/1400 mV for the author's
# R004 gain fit (its active vector reaches at most 1500/3500 mV). No automatic
# voltage increase or config save.
PFC      := $(BUILD)/pinfinder-ghost-cal
PFC_Z    := $(BUILD)/pfc
PFC_C    := $(PFH_C) PinFinder/pf_calib.c
PFC_ZSRC := $(addprefix $(PFC_Z)/,$(notdir $(PFC_C)) pfdict.c)
PFC_OBJ  := $(addprefix $(BUILD)/,$(PF_SRC:.c=.o) Src/hwdiv.o) $(PFC_ZSRC:.c=.o)
$(PFC_Z)/%.o: CFLAGS += $(WARN) -IPinFinder -DPF_FIXED_CAL
$(PFC_Z)/stamp: $(PFC_C) PinFinder/cfgnames.h ../tools/pfstrings.py
	$(PYTHON) ../tools/pfstrings.py $(PFC_Z) $(PFC_C) PinFinder/cfgnames.h
	@touch $@
$(PFC_ZSRC): $(PFC_Z)/stamp ;
$(PFC_Z)/%.o: $(PFC_Z)/%.c Makefile $(FLAGS_STAMP)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@
pinfinder-ghost-cal: $(PFC).bin $(PFC).hex
$(PFC).elf: $(PFC_OBJ) $(LDSCRIPT) Makefile $(FLAGS_STAMP)
	$(CC) $(subst $(TARGET).map,$(PFC).map,$(LDFLAGS)) $(PFC_OBJ) -o $@
	$(SIZE) $@
$(PFC).bin: $(PFC).elf
	$(OBJCOPY) -O binary $< $@
$(PFC).hex: $(PFC).elf
	$(OBJCOPY) -O ihex $< $@

clean:
	rm -rf $(BUILD)

clean-all:
	rm -rf build ../release

targets:
	@set -e; for m in $(MCUS); do echo "== $$m"; $(MAKE) --no-print-directory MCU=$$m both; done

RELEASE := ../release
release: targets
	@set -e; for m in $(MCUS); do d=$(RELEASE)/$$(echo $$m | tr A-Z a-z); mkdir -p $$d; \
	  cp build/$$(echo $$m | tr A-Z a-z)/hoverboard.bin build/$$(echo $$m | tr A-Z a-z)/hoverboard.hex \
	     build/$$(echo $$m | tr A-Z a-z)/pinfinder.bin build/$$(echo $$m | tr A-Z a-z)/pinfinder.hex $$d/; done
	@ls -l $(RELEASE)/*

help:
	@sed -n '2,/^$$/{s/^# \{0,1\}//;p}' Makefile

# Standalone read-only board diagnostic. It links no motor-control sources or TIM1 startup code.
DIAG := $(BUILD)/diagnostic
DIAG_SRC := Diagnostics/main.c Diagnostics/startup.c Device/MM32SPIN0x/Source/system_mm32spin05.c
DIAG_OBJ := $(addprefix $(BUILD)/,$(DIAG_SRC:.c=.o))
diagnostic: $(DIAG).bin $(DIAG).hex
$(DIAG).elf: $(DIAG_OBJ) $(LDSCRIPT) Makefile $(FLAGS_STAMP)
	$(CC) $(subst $(TARGET).map,$(DIAG).map,$(LDFLAGS)) $(DIAG_OBJ) -o $@
	$(SIZE) $@
$(DIAG).bin: $(DIAG).elf
	$(OBJCOPY) -O binary $< $@
$(DIAG).hex: $(DIAG).elf
	$(OBJCOPY) -O ihex $< $@

.PHONY: all both clean clean-all flash pinfinder flash-pinfinder pinfinder-hall pinfinder-ghost-cal targets release help diagnostic
-include $(PF_OBJ:.o=.d)
-include $(PFH_OBJ:.o=.d)
-include $(PFC_OBJ:.o=.d)
-include $(DIAG_OBJ:.o=.d)
-include $(DEP)
