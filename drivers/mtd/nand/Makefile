# SPDX-License-Identifier: GPL-2.0+

ifneq ($(CONFIG_MTD_NAND_CORE),)
nandcore-objs := core.o bbt.o
endif

obj-$(CONFIG_MTD_RAW_NAND) += raw/
obj-$(CONFIG_MTD_SPI_NAND) += spi/

obj-$(CONFIG_MTD_NAND_CORE) += nandcore.o
