SHELL := bash
.ONESHELL:
.SHELLFLAGS := -eu -o pipefail -c
.DELETE_ON_ERROR:
MAKEFLAGS += --warn-undefined-variables
MAKEFLAGS += --no-builtin-rules

UNITY_DIR = ../../../Unity

# CUBE_DIR = ../../firmware48
BUILD_DIR = build
TEST_SRC_DIR = .
SRC_DIR = ../src

TEST_SOURCES = $(wildcard $(TEST_SRC_DIR)/*_test.c)
TEST_MODULES = $(patsubst $(TEST_SRC_DIR)/%_test.c,%,$(TEST_SOURCES))
TEST_OBJECTS = $(patsubst %,%.o,$(BUILD_DIR)/$(TEST_MODULES).o)
TEST_EXECUTABLES = $(patsubst %,%.test,$(BUILD_DIR)/$(TEST_MODULES))
TEST_RESULTS = $(patsubst %,%.result,$(BUILD_DIR)/$(TEST_MODULES))

CC=/opt/homebrew/bin/gcc-11


OPT = -Os

# C_INCLUDES += -I$(CUBE_DIR)/Inc
# C_INCLUDES += -I$(CUBE_DIR)/Drivers/STM32G4xx_HAL_Driver/Inc
# C_INCLUDES += -I$(CUBE_DIR)/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
# C_INCLUDES += -I$(CUBE_DIR)/Drivers/CMSIS/Device/ST/STM32G4xx/Include
# C_INCLUDES += -I$(CUBE_DIR)/Drivers/CMSIS/Include
C_INCLUDES += -I$(UNITY_DIR)/src
C_INCLUDES += -I../src
C_INCLUDES += -I./

C_DEFS += -DUSE_HAL_DRIVER
C_DEFS += -DSTM32G431xx
# C_DEFS += -DSTM32G431 
# C_DEFS += -DSTM32 
C_DEFS += -DPERF_MONITOR_ENABLED
# Hardware version 0.1.0
C_DEFS += -DSSF_HARDWARE_VERSION=0x000600

CFLAGS = $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

-include $(wildcard $(TEST_SRC_DIR)/*_test.mk)

define unpath
$(subst .,_,$(subst /,_,$(1)))
endef

define objpath
$(BUILD_DIR)/$(2)_$(call unpath,$(1)).o
endef

# add sources for the test, the corresponding sourcefile, if exists
# and the unit testing module
# $(1)_TEST_SOURCES+=$$(wildcard $(SRC_DIR)/$(1).c)
# $(1)_TEST_SOURCES+=$(TEST_SRC_DIR)/$(1)_test.c
# $(1)_TEST_SOURCES+=$(UNITY_DIR)/src/unity.c

define moduleobj_def
$(1)_TEST_SOURCES+=$$(wildcard $(SRC_DIR)/$(1).c)
$(1)_TEST_SOURCES+=$(TEST_SRC_DIR)/$(1)_test.c
$(1)_TEST_SOURCES+=$(UNITY_DIR)/src/unity.c
$(1)_TEST_OBJECTS=$$(foreach src,$$($(1)_TEST_SOURCES),$$(call objpath,$$(src),$(1)))
endef

# how to build an object file
# $$(eval $$(info objpath $(call objpath,$(1),$(2))))

# $(eval $(info objrule $(call objpath,$(1),$(2)): $(1) Makefile | $$(BUILD_DIR) \
# 	$$(CC) -c $$(CFLAGS) -Wa,-a,-ad,-alms=$$(BUILD_DIR)/$$(notdir $$(<:.c=.lst)) $$< -o $$@))


define objrule
$(call objpath,$(1),$(2)): $(1) Makefile | $$(BUILD_DIR)
	$$(CC) -c $$(CFLAGS) -Wa,-a,-ad,-alms=$$(BUILD_DIR)/$$(notdir $$(<:.c=.lst)) $$< -o $$@
endef

# $(eval $(info name $(1)_TEST_SOURCES))
# $(eval $(info content $(call $(1)_TEST_SOURCES)))
# $(eval $(info module_objrule_src $(call $(1)_TEST_SOURCES)))
# $(eval $(info module_objrule_obj $(call $(1)_TEST_OBJECTS)))
# $(eval $(info module_objrule $(foreach obj,$(call $(1)_TEST_SOURCES), $(call objrule,$(obj),$(1)))))

define module_objrule
$(foreach obj,$(call $(1)_TEST_SOURCES),$(eval $(call objrule,$(obj),$(1))))
endef
# $(info module_objrule $(foreach obj,$(call $(1)_TEST_SOURCES),$(eval $(call objrule,$(obj)))))

# define test build rules
define testrule
$(1): $$(BUILD_DIR)/$(1).result

$$(BUILD_DIR)/$(1).test: $$($(1)_TEST_OBJECTS)
	$$(CC) $$($(1)_TEST_OBJECTS) $$(LDFLAGS) -o $$@
endef
# $$(eval $$(info $(1)_TEST_OBJECTS are $$($(1)_TEST_OBJECTS)))
# $$(eval $$(info $(1)_TEST_SOURCES are $$($(1)_TEST_SOURCES)))

.PHONY: all
all: $(TEST_MODULES)
	echo $(TEST_MODULES)
# 	echo "$(foreach module,$(TEST_MODULES),$(call testrule,$(module)))"

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

.PHONY: $(TEST_MODULES)
# $(TEST_MODULES): $(TEST_EXECUTABLES)
# 	$(BUILD_DIR)/$@.test



# additional source modules definitions
$(foreach module,$(TEST_MODULES),$(eval $(call moduleobj_def,$(module))))

# $(foreach module, $(TEST_MODULES), $(eval $(call testmodules, $(module))))

# build rules
$(foreach module,$(TEST_MODULES),$(call module_objrule,$(module)))

# $(foreach module,$(TEST_MODULES),$(eval $(call module_objrule,$(module))))
$(foreach module,$(TEST_MODULES),$(eval $(call testrule,$(module))))


$(BUILD_DIR)/%.test: $(BUILD_DIR)/%.o Makefile
	$(CC) $*.o $(LDFLAGS) -o $@

$(BUILD_DIR)/%.result: $(BUILD_DIR)/%.test
	./$^ >> $@
	cat $@

# $(BUILD_DIR)/%.o: $(TEST_SRC_DIR)/%.c Makefile $(BUILD_DIR) 
# 	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@


$(BUILD_DIR):
	mkdir -p $@

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)
