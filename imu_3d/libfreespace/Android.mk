# This file is part of libfreespace.
# Copyright (c) 2009-2013 Hillcrest Laboratories, Inc.
# libfreespace is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
#
#
# Libfreespace requires the Linux Kernel hidraw headers to be
# present at <linux/hidraw.h>
# This project can easily be dropped in to an Android OS build since
# most Android kernels export hidraw.h
#
# To specify additional include paths (for NDK builds) set the following var:
#

#
# Exports
#	LIBFREESPACE_MSG_GEN - target for message script generation
#	LIBFREESPACE_C_INCLUDES - include directories for libfreespace

LOCAL_PATH := $(call my-dir)/

include $(CLEAR_VARS)

LOCAL_MODULE := libfreespace

LOCAL_MODULE_CLASS := STATIC_LIBRARIES

LOCAL_MODULE_TAGS := optional

ifdef NDK_ROOT
	LIBFREESPACE_GEN_DIR := $(LOCAL_PATH)../../obj/libfreespace
else
    LIBFREESPACE_GEN_DIR := $(call local-intermediates-dir)
endif

# Generate codecs. Setup rules to generate codecs
LIBFREESPACE_MSG_GEN_SCRIPT := $(LOCAL_PATH)/common/messageCodeGenerator.py
LIBFREESPACE_MSG_GEN_MSG_FILE := $(LOCAL_PATH)/common/setupMessages.py
# phony target
LIBFREESPACE_MSG_GEN := $(LIBFREESPACE_GEN_DIR)/.msg

$(LIBFREESPACE_MSG_GEN) : $(LIBFREESPACE_MSG_GEN_SCRIPT) $(LIBFREESPACE_MSG_GEN_MSG_FILE)  $(LIBFREESPACE_EXTRA_MSG_FILE)
	@echo "libfreespace <= Generating Freespace Messages"
	@python $(LIBFREESPACE_MSG_GEN_SCRIPT) -I $(LIBFREESPACE_GEN_DIR)/include/ -s $(LIBFREESPACE_GEN_DIR) $(LIBFREESPACE_MSG_GEN_MSG_FILE)  $(LIBFREESPACE_EXTRA_MSG_FILE)
	@touch $(LIBFREESPACE_MSG_GEN)

# Outputs:
LIBFREESPACE_MSG_GEN_SRCS := \
	$(LIBFREESPACE_GEN_DIR)/freespace_printers.c \
	$(LIBFREESPACE_GEN_DIR)/freespace_codecs.c \
	$(LIBFREESPACE_GEN_DIR)/include/freespace_printers.h \
	$(LIBFREESPACE_GEN_DIR)/include/freespace_codecs.h

$(LIBFREESPACE_MSG_GEN_SRCS) : $(LIBFREESPACE_MSG_GEN)

# Generate a config.h file.
LIBFREESPACE_CONF_FILE := $(LIBFREESPACE_GEN_DIR)/include/freespace_config.h

# Depend on msg gen for directory structure and Android.mk to catch changes to config file
$(LIBFREESPACE_CONF_FILE) : $(LOCAL_PATH)/Android.mk $(LIBFREESPACE_MSG_GEN)
	@echo "libfreespace <= Creating Config File"
	@echo "#define LIBFREESPACE_VERSION \"0.7.0\"	" > $@

LOCAL_SRC_FILES := linux/freespace_hidraw.c common/freespace_deviceTable.c

ifndef NDK_ROOT
LOCAL_GENERATED_SOURCES := $(LIBFREESPACE_CONF_FILE) $(LIBFREESPACE_MSG_GEN_SRCS)

## Headers have to be generated before sources
$(LOCAL_SRC_FILES) : $(LIBFREESPACE_MSG_GEN) $(LIBFREESPACE_CONF_FILE)
else

## NDK is 'special' and doesn't understand LOCAL_GENERATED_SOURCES
## and deps need full paths... so the nice clean rules in the if clause don't work

LOCAL_SRC_FILES += \
	../../obj/libfreespace/freespace_printers.c \
	../../obj/libfreespace/freespace_codecs.c

$(addprefix $(LOCAL_PATH)/, $(LOCAL_SRC_FILES)) : $(LIBFREESPACE_MSG_GEN) $(LIBFREESPACE_CONF_FILE)

endif

LOCAL_CFLAGS += -DFREESPACE_LITTLE_ENDIAN

LOCAL_C_INCLUDES += \
	$(LOCAL_PATH)/include \
	$(LIBFREESPACE_GEN_DIR)/include/ \
	$(LIBFREESPACE_ADDITIONAL_INCLUDES)

LIBFREESPACE_C_INCLUDES := \
	$(LOCAL_PATH)/include \
	$(LIBFREESPACE_GEN_DIR)/include/

LOCAL_EXPORT_C_INCLUDES := $(LIBFREESPACE_C_INCLUDES)

include $(BUILD_STATIC_LIBRARY)
