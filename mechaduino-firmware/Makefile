
APPL = mechaduino_appl

OBJ_DIR := ./build
BIN_DIR := ./bin

ASF_PATH := asf_mechaduino

APPL_LINKER_SCRIPT = samd21g18a_flash.ld

# Toolchain 
# GNU_INSTALL_ROOT = $(HOME)/Applications/ARM-Toolchain/gcc-arm-none-eabi-7-2018-q2-update
# GNU_INSTALL_ROOT := $(HOME)/Applications/ARM-Toolchain/gcc-arm-none-eabi-7-2017-q4-major
GNU_INSTALL_ROOT := $(HOME)/Applications/ARM-Toolchain/gcc-arm-none-eabi-6-2017-q2-update
# GNU_INSTALL_ROOT = $(HOME)/Applications/ARM-Toolchain/gcc-arm-none-eabi-5_4-2016q3
GNU_PREFIX = arm-none-eabi
CC      := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc"
CPP      := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-cpp"
CXX     := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-c++"
AS      := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as"
AR      := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar" -r
LD      := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld"
NM      := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm"
OBJDUMP := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump"
OBJCOPY := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy"
SIZE    := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size"


SRC_PATHS += ./src
SRC_PATHS += $(ASF_PATH)/hal/src
SRC_PATHS += $(ASF_PATH)/hal/utils/src
SRC_PATHS += $(ASF_PATH)/persistent_storage
SRC_PATHS += $(ASF_PATH)/usb
SRC_PATHS += $(wildcard $(ASF_PATH)/hpl/*)


C_FILTERED += 

C_SRCS += $(filter-out $(C_FILTERED), $(foreach dir,$(SRC_PATHS),$(wildcard $(dir)/*.c)))
C_SRCS += $(ASF_PATH)/atmel_start.c
C_SRCS += $(ASF_PATH)/driver_init.c
C_SRCS += $(ASF_PATH)/persistent_storage_start.c
C_SRCS += $(ASF_PATH)/usb_start.c
C_SRCS += $(ASF_PATH)/usb/class/cdc/device/cdcdf_acm.c
C_SRCS += $(ASF_PATH)/usb/device/usbdc.c
# C_SRCS += $(ASF_PATH)/samd21a/gcc/gcc/startup_samd21.c
C_SRCS += $(ASF_PATH)/samd21a/gcc/system_samd21.c

vpath %.c $(SRC_PATHS) $(ASF_PATH) $(ASF_PATH)/samd21a/gcc $(ASF_PATH)/usb/device/ $(ASF_PATH)/usb/class/cdc/device


INC_FOLDERS += $(ASF_PATH)
INC_FOLDERS += $(ASF_PATH)/samd21a/include
INC_FOLDERS += $(ASF_PATH)/hri
INC_FOLDERS += $(wildcard $(ASF_PATH)/hpl/*)
INC_FOLDERS += $(ASF_PATH)/hal/include
INC_FOLDERS += $(ASF_PATH)/hal/utils/include
INC_FOLDERS += $(ASF_PATH)/config
INC_FOLDERS += $(ASF_PATH)/persistent_storage
INC_FOLDERS += $(ASF_PATH)/usb
INC_FOLDERS += $(ASF_PATH)/usb/device
INC_FOLDERS += $(ASF_PATH)/usb/class/cdc
INC_FOLDERS += $(ASF_PATH)/usb/class/cdc/device
INC_FOLDERS += $(ASF_PATH)/CMSIS/core/include
INC_FOLDERS += src
INCLUDES = $(INC_FOLDERS:%=-I%)

CPPFLAGS += -D__SAMD21G18A__
CPPFLAGS += -DDEBUG
# CPPFLAGS += -DSYSTICK_MODE
# make sure clock setup done by bootloader is not undone 
# looks like everything is ok as long as we don't touch XOSC32K
CPPFLAGS += -DCONF_XOSC32K_CONFIG=0
# CPPFLAGS += -DCONF_DFLL_CONFIG=0
# CPPFLAGS += -DCONF_GCLK_GENERATOR_0_CONFIG=0
# CPPFLAGS += -DCONF_GCLK_GENERATOR_1_CONFIG=0

CFLAGS += $(CPPFLAGS)
CFLAGS += -mcpu=cortex-m0plus
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -mfloat-abi=soft

# C flags common to all targets
CFLAGS +=  -Wall -Os -g3
CFLAGS +=  -std=gnu11
# keep every function in separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin 
# CFLAGS += --short-enums 

CFLAGS += -Werror=incompatible-pointer-types
CFLAGS += -Werror=discarded-qualifiers
CFLAGS += -Werror=implicit-function-declaration
CFLAGS += -Werror=int-conversion
CFLAGS += -Wno-pointer-sign
# CFLAGS += -flto

DEP_FLAGS = -MD -MF 

# Linker flags
LDFLAGS += -mcpu=cortex-m0plus
LDFLAGS += -mthumb -mabi=aapcs
# LDFLAGS += -L $(BUILD_SCRIPTS_PATH)
# let linker to dump unused sections
LDFLAGS += -Wl,-g
LDFLAGS += -g
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
# LDFLAGS += --specs=rdimon.specs -lc -lnosys 
LDFLAGS += --specs=nano.specs -lc -lnosys
# LDFLAGS += -u _printf_float
# LDFLAGS += -flto

APPL_LDFLAGS += $(LDFLAGS)
APPL_LDFLAGS += -T$(APPL_LINKER_SCRIPT)


SRC_OBJ += $(patsubst %.c,$(OBJ_DIR)/%.o,$(notdir $(C_SRCS))) 
# SRC_OBJ += $(patsubst %.s,$(OBJ_DIR)/%.o,$(notdir $(ASM_SRCS)))

APPL_SRC_OBJ = $(SRC_OBJ)

OPENOCD := /usr/local/bin/openocd -f interface/jlink.cfg -c"transport select swd ; adapter_khz 500 ; adapter_nsrst_delay 10 ; set WORKAREASIZE 0" -f target/nrf51.cfg




.PHONY: $(TARGET_NAME) default all clean help appl program


all: appl

appl: $(BIN_DIR)/$(APPL).elf $(BIN_DIR)/$(APPL).bin $(BIN_DIR)/$(APPL).hex


$(OBJ_DIR) $(BIN_DIR):
	mkdir -p $@


$(OBJ_DIR)/%.o: %.c | $(OBJ_DIR)
	@echo compiling $< 
	@$(CC) $(DEP_FLAGS) $(@:%.o=%.d) $(CFLAGS) $(C_INCLUDES) $(INCLUDES) -c $< -o $@

$(OBJ_DIR)/%.o: %.s | $(OBJ_DIR)
	@$(CC) -MP -MD -std=c99 -c $(ASMFLAGS) $< -o $@



$(BIN_DIR)/$(APPL).out: $(APPL_SRC_OBJ) $(APPL_LINKER_SCRIPT) $(BIN_DIR)
	@echo linking $@ with $(APPL_LINKER_SCRIPT)
	@$(CC) -o $@ $(APPL_SRC_OBJ) $(APPL_LDFLAGS)

$(BIN_DIR)/$(BOOT).out: $(BOOT_SRC_OBJ) $(BOOT_LINKER_SCRIPT) $(BIN_DIR)
	@echo linking $@ with $(BOOT_LINKER_SCRIPT)
	@$(CC) -o $@ $(BOOT_SRC_OBJ) $(BOOT_LDFLAGS)

$(BIN_DIR)/%.bin: $(BIN_DIR)/%.out
	@echo Creating binary: $< $@
	@$(OBJCOPY) -O binary $< $@

# Create binary .hex file from the .out file
$(BIN_DIR)/%.hex: $(BIN_DIR)/%.out
	@echo Creating HEX: $@
	@$(OBJCOPY) -O ihex $< $@

# Create binary .hex file from the .out file
$(BIN_DIR)/%.elf: $(BIN_DIR)/%.out
	@echo Creating ELF: $@
	@cp $< $@
	@$(SIZE) -t $@


program: $(BIN_DIR)/$(APPL).hex openocd.cfg
	$(OPENOCD) --file openocd.cfg --command "program $(BIN_DIR)/$(APPL).hex verify reset exit"

bossa: $(BIN_DIR)/$(APPL).bin
	/Users/dog/Documents/embedded/projects/BOSSA/bin/bossac -w -v -b --offset 0x2000 -p /dev/tty.usbmodem14201 $(BIN_DIR)/$(APPL).bin

.PHONY: rst
rst: 
	$(OPENOCD) --command "init; reset run; exit"


GDB      := "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gdb"
GDB_COMMAND_SCRIPT=$(BIN_DIR)/gdbinit
$(GDB_COMMAND_SCRIPT):
	echo "target remote localhost:3333    \n\
        monitor reset halt              \n\
				monitor arm semihosting enable  \n\
        file $(BIN_DIR)/$(APPL).out  \n\
        load                            \n\
        b main                          \n\
        monitor reset                   \n\
        continue" > $(GDB_COMMAND_SCRIPT)

gdbserver:
	$(OPENOCD) --command "gdb_port"

gdb: $(GDB_COMMAND_SCRIPT)
	$(GDB) -x $(GDB_COMMAND_SCRIPT)

# debug:
# 	$(GDB) --eval-command="target remote localhost:3333" --eval-command="monitor reset halt" $(BIN_DIR)/$(APPL).out
debug:
	$(GDB) --eval-command="target extended-remote /dev/tty.usbmodem79A760B81" --eval-command="monitor swdp_scan"  --eval-command="attach 1" --eval-command="set mem inaccessible-by-default off" $(BIN_DIR)/$(APPL).out

flashboot:
	$(GDB) --eval-command="target extended-remote /dev/tty.usbmodem79A760B81" --eval-command="monitor swdp_scan"  --eval-command="attach 1"   --eval-command="load sam_ba_bootloader.elf"

# 	$(GDB) --eval-command="target extended-remote /dev/tty.usbmodem79A760B81" --eval-command="monitor swdp_scan"  --eval-command="attach 1"   --eval-command="load /Users/dog/Library/Arduino15/packages/arduino/hardware/samd/1.8.3/bootloaders/zero/samd21_sam_ba.hex 0x0"
flashapp:
	$(GDB) --eval-command="target extended-remote /dev/tty.usbmodem79A760B81" --eval-command="monitor swdp_scan"  --eval-command="attach 1"   --eval-command="load $(BIN_DIR)/$(APPL).out" $(BIN_DIR)/$(APPL).out


clean:
	rm -rf $(BIN_DIR) $(OBJ_DIR)

-include $(wildcard $(OBJ_DIR)/*.d)

