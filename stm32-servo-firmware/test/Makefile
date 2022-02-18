SHELL := bash
.ONESHELL:
.SHELLFLAGS := -eu -o pipefail -c
.DELETE_ON_ERROR:
MAKEFLAGS += --warn-undefined-variables
MAKEFLAGS += --no-builtin-rules


CUBE_DIR = ../../firmware48
BUILD_DIR = build
TEST_SRC_DIR = .
SRC_DIR = ../src

TEST_SOURCES = $(wildcard $(TEST_SRC_DIR)/*_test.c)
TEST_MODULES = $(patsubst $(TEST_SRC_DIR)/%_test.c,%,$(TEST_SOURCES))
TEST_OBJECTS = $(patsubst %,%.o,$(BUILD_DIR)/$(TEST_MODULES).o)
TEST_EXECUTABLES = $(patsubst %,%.test,$(BUILD_DIR)/$(TEST_MODULES))

CC=/opt/homebrew/bin/gcc-11


OPT = -Os


CFLAGS = $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

define unpath
$(subst .,_,$(subst /,_,$(1)))
endef

define objpath
$(BUILD_DIR)/$(call unpath,$(1)).o
endef


define moduleobj_def
$(1)_TEST_SOURCES+=$(TEST_SRC_DIR)/$(1)_test.c
$(1)_TEST_OBJECTS=$$(foreach src,$$($(1)_TEST_SOURCES),$$(call objpath,$$(src)))
endef

define objrule
$(call objpath,$(1)): $(1) Makefile | $$(BUILD_DIR)
	$$(CC) -c $$(CFLAGS) -Wa,-a,-ad,-alms=$$(BUILD_DIR)/$$(notdir $$(<:.c=.lst)) $$< -o $$@
endef

define module_objrule
$(foreach obj,$($(1)_TEST_SOURCES),$(call objrule,$(obj)))
$(info module_objrule $(foreach obj,$($(1)_TEST_SOURCES),$(call objrule,$(obj))))
endef

# define test build rules
define testrule
$(1): $(BUILD_DIR)/$(1).test
$(BUILD_DIR)/$(1).test: $$($(1)_TEST_OBJECTS)
	$$(CC) $$($(1)_TEST_OBJECTS) $$(LDFLAGS) -o $$@
$(eval $$(info $(1)_TEST_OBJECTS $$($(1)_TEST_OBJECTS)))
$(eval $$(info $(1)_TEST_SOURCES $$($(1)_TEST_SOURCES)))
endef

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
$(foreach module,$(TEST_MODULES),$(eval $(call module_objrule,$(module))))
$(foreach module,$(TEST_MODULES),$(eval $(call testrule,$(module))))


$(BUILD_DIR)/%.test: $(BUILD_DIR)/%.o Makefile | $(BUILD_DIR) 
	$(CC) $*.o $(LDFLAGS) -o $@

# $(BUILD_DIR)/%.o: $(TEST_SRC_DIR)/%.c Makefile $(BUILD_DIR) 
# 	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@


$(BUILD_DIR):
	mkdir -p $@		

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)
-include $(wildcard $(TEST_SOURCES)/*_test.mk)
