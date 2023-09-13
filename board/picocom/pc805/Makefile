# SPDX-License-Identifier: GPL-2.0+
#
# Copyright (C) 2019 Picocom Technology Corporation.

obj-y	:= pc805_evb.o

ifdef CONFIG_SPL_BUILD
obj-y	+= pc805_spl.o

ifeq ($(PICO_VFW_LIB_PATH),)
$(warning no define PICO_VFW_LIB_PATH environment parameter, please config it firstly)
endif

KBUILD_CFLAGS += -I/build/tmp/work/riscv32-poky-linux/vfw4spl/1.0-r0
KBUILD_LDFLAGS += --whole-archive  /build/tmp/work/riscv32-poky-linux/vfw4spl/1.0-r0/libpc805_vfw4spl.a  --no-whole-archive
endif
