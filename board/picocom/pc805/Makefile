# SPDX-License-Identifier: GPL-2.0+
#
# Copyright (C) 2019 Picocom Technology Corporation.

obj-y	:= pc805_evb.o

ifdef CONFIG_SPL_BUILD
obj-y	+= pc805_spl.o

ifeq ($(PICO_VFW_LIB_PATH),)
PICO_VFW_LIB_PATH = /build/tmp/work/riscv32-poky-linux/vfw4spl/1.0-r0
$(warning no define PICO_VFW_LIB_PATH environment parameter, default path enabled and need users confirm valid: $(PICO_VFW_LIB_PATH))
endif

KBUILD_CFLAGS += -I$(PICO_VFW_LIB_PATH)
KBUILD_LDFLAGS += --whole-archive  $(PICO_VFW_LIB_PATH)/libpc805_vfw4spl.a  --no-whole-archive
endif
