# SPDX-License-Identifier: GPL-2.0+
#
# Copyright (C) 2019 Picocom Technology Corporation.

obj-y	:= pc805_evb.o

ifdef CONFIG_SPL_BUILD
obj-y	+= pc805_spl.o

ifeq ($(PICO_VFW_LIB_PATH),)
$(error no define PICO_VFW_LIB_PATH environment parameter, please config it firstly)
endif

KBUILD_CFLAGS += -I$(PICO_VFW_LIB_PATH)/target/riscv32gc-unknown-none-elf/release/include/
KBUILD_LDFLAGS += --whole-archive  $(PICO_VFW_LIB_PATH)/target/riscv32gc-unknown-none-elf/release/libpc805_vfw4spl.a  --no-whole-archive
endif
