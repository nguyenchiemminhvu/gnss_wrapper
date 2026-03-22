# =============================================================================
# Makefile — sample_usage (gnss_wrapper example)
#
# Usage:
#   make                        — build with NMEA parser enabled (default)
#   make NMEA_PARSER_ENABLED=0  — build without NMEA parser (pre-NMEA behaviour)
#   make clean                  — remove build artefacts and binary
#   make rebuild                — clean then build
# =============================================================================

CXX      := g++
CXXFLAGS := -std=c++14 -Wall -Wextra -O2
LDFLAGS  := -lpthread

TARGET   := sample_usage
BUILDDIR := build

# ---------------------------------------------------------------------------
# NMEA parser feature toggle
# Set to 1 (default) to build with NMEA integration; 0 to disable.
# ---------------------------------------------------------------------------
NMEA_PARSER_ENABLED ?= 1

ifeq ($(NMEA_PARSER_ENABLED),1)
CXXFLAGS += -DNMEA_PARSER_ENABLED
endif

# ---------------------------------------------------------------------------
# Root of the gnss_wrapper tree (one level up from this Makefile)
# ---------------------------------------------------------------------------
GNSS_ROOT := ..

# ---------------------------------------------------------------------------
# Include paths
# ---------------------------------------------------------------------------
INCLUDES := \
    -I$(GNSS_ROOT) \
    -I$(GNSS_ROOT)/ext_libs \
    -I$(GNSS_ROOT)/ext_libs/nmea_parser/include \
    -I$(GNSS_ROOT)/ext_libs/nmea_parser/src \
    -I$(GNSS_ROOT)/ext_libs/nmea_parser/src/internal \
    -I$(GNSS_ROOT)/ext_libs/ubx_parser/include \
    -I$(GNSS_ROOT)/ext_libs/ubx_parser/src \
    -I$(GNSS_ROOT)/ext_libs/ubx_parser/src/internal \
    -I$(GNSS_ROOT)/ext_libs/serial_helper \
    -I$(GNSS_ROOT)/ext_libs/ini_parser \
    -I$(GNSS_ROOT)/ext_libs/lib_event \
    -I$(GNSS_ROOT)/ext_libs/logging

ifeq ($(NMEA_PARSER_ENABLED),1)
INCLUDES += \
    -I$(GNSS_ROOT)/ext_libs/nmea_parser/include \
    -I$(GNSS_ROOT)/ext_libs/nmea_parser/src
endif

CXXFLAGS += $(INCLUDES)

# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------

# The example itself
EXAMPLE_SRCS := sample_usage.cpp

# gnss_wrapper library sources
LIB_SRCS := \
    $(GNSS_ROOT)/ext_libs/global_constants.cpp \
    $(GNSS_ROOT)/ext_libs/serial_helper/serial_port.cpp \
    $(GNSS_ROOT)/ext_libs/serial_helper/serial_wrapper.cpp \
    $(GNSS_ROOT)/adapter/ubx_base_database.cpp \
    $(GNSS_ROOT)/shared_buffer/ubx_database_wrapper.cpp \
    $(GNSS_ROOT)/adapter/control_adapter.cpp \
    $(GNSS_ROOT)/gnss_receiver/gnss_receiver.cpp \
    $(GNSS_ROOT)/gnss_controller/gnss_controller.cpp \
    $(GNSS_ROOT)/gnss_controller/ini_config_provider_impl.cpp \
    $(GNSS_ROOT)/gnss_controller/serial_transport_impl.cpp \

# ubx_parser sources (all .cpp files under ext_libs/ubx_parser/src/)
UBX_SRCS := $(shell find $(GNSS_ROOT)/ext_libs/ubx_parser/src -name '*.cpp')

ifeq ($(NMEA_PARSER_ENABLED),1)
# nmea_parser library sources
NMEA_SRCS         := $(shell find $(GNSS_ROOT)/ext_libs/nmea_parser/src -name '*.cpp')
# nmea_database_wrapper glue code
NMEA_WRAPPER_SRCS := \
    $(GNSS_ROOT)/adapter/nmea_base_database.cpp \
    $(GNSS_ROOT)/shared_buffer/nmea_database_wrapper.cpp
else
NMEA_SRCS         :=
NMEA_WRAPPER_SRCS :=
endif

ALL_SRCS := $(EXAMPLE_SRCS) $(LIB_SRCS) $(UBX_SRCS) $(NMEA_SRCS) $(NMEA_WRAPPER_SRCS)

# ---------------------------------------------------------------------------
# Object files — placed under BUILDDIR, preserving directory structure
# ---------------------------------------------------------------------------
OBJS := $(patsubst %.cpp,$(BUILDDIR)/%.o,$(ALL_SRCS))

# ---------------------------------------------------------------------------
# Rules
# ---------------------------------------------------------------------------
.PHONY: all clean rebuild

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)
	@echo "Built: $(TARGET)"

# Generic pattern rule: compile any .cpp -> BUILDDIR/<path>.o
$(BUILDDIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILDDIR) $(TARGET)

rebuild: clean all
