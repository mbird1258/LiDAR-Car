HOME_TREE := ../../
MODULE_NAME := rplidar

BUILD_ROOT        = $(HOME_TREE)
BUILD_OUTPUT_ROOT = $(BUILD_ROOT)/output/$(BUILD_TARGET_PLATFORM)/$(OUTPUT_BUILD_PREFIX)
BUILD_OBJ_ROOT    = $(BUILD_ROOT)/obj/$(BUILD_TARGET_PLATFORM)/$(OUTPUT_BUILD_PREFIX)
RMDIR             = rm -rf
RM                = rm -f

include $(HOME_TREE)/mak_def.inc

CXXSRC += lidar_driver.cpp rplidar_module.cpp
C_INCLUDES += -I$(CURDIR)/../../sdk/include -I$(CURDIR)/../../sdk/src -I$(CURDIR)

CXX := g++
CXXFLAGS += -fPIC -shared $(shell python3 -m pybind11 --includes) $(shell python3-config --includes)
LDFLAGS += -shared
LD_LIBS += -L$(BUILD_OUTPUT_ROOT) -lsl_lidar_sdk $(shell python3-config --embed --ldflags) -lstdc++ -lpthread

APP_TARGET := $(BUILD_OUTPUT_ROOT)/$(MODULE_NAME)$(shell python3-config --extension-suffix)

$(APP_TARGET): $(CXXSRC) $(SDK_TARGET)
	@$(MKDIR) $(dir $@)
	$(CXX) $(CXXFLAGS) $(C_INCLUDES) $^ -o $@ $(LDFLAGS) $(LD_LIBS)

all: build_sdk build_pymodule

build_pymodule: $(APP_TARGET)

clean_app:
	$(RMDIR) $(BUILD_OBJ_ROOT)/$(MODULE_NAME)
	$(RM) $(APP_TARGET)

clean: clean_app