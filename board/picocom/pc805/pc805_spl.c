// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019 Picocom Technology Corporation
 * Wei Zheng, Picocom Technology Corporation <wei.zheng@picocom.com>
 */

#include <common.h>
#include <cpu_func.h>
#include <hang.h>
#include <init.h>
#include <log.h>
#include <spl.h>
#include <mtd.h>
#include <dm/uclass.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <stdio.h>
#include "pc805_address.h"
#include "pc805_vfw4spl.h"

static struct mtd_info *mtd;

void board_spinand_init(void)
{
	struct udevice *dev;
	int ret;
	printf("pc805_spl.c");
	ret = uclass_get_device_by_driver(UCLASS_MTD,
					  DM_GET_DRIVER(spinand), &dev);
	if (ret && ret != -ENODEV) {
		pr_err("Failed to initialize %s. (error %d)\n", "spinand", ret);
		return;
	}

	mtd = dev_get_uclass_priv(dev);
}

extern int riscv_get_time(u64 *time);
#define PC805_PLMT_CYCLES_HZ	(30720000)
#define PC805_PLMT_CYCLES_MS	(PC805_PLMT_CYCLES_HZ / 1000)
#define PC805_PLMT_CYCLES_US	(PC805_PLMT_CYCLES_MS / 1000)
/* Returns time in milliseconds */
ulong get_timer(ulong base)
{
	u64 time_cnt = 0;

	riscv_get_time(&time_cnt);
	return ((time_cnt / PC805_PLMT_CYCLES_MS) - base);
}

void __udelay(unsigned long usec)
{
	u64 start = 0;
	u64 stop = 0;
	u64 intval = usec * PC805_PLMT_CYCLES_US;

	riscv_get_time(&start);
	riscv_get_time(&stop);
	while ((stop - start) < intval) {
		riscv_get_time(&stop);
	}
}

#if CONFIG_SPL_LPDDR4_ISSI_ECC_ERR_ON
#define LPDDR4_ISSI_REG_MR33    33
#define LPDDR4_ISSI_REG_MR34    34
#endif

unsigned char lpddr4_mr_reg_read(unsigned char addr)
{
    unsigned int rdata;

    //loop waiting if write busy. mrstat reg bit0
    while ((readl((void __iomem *)(DDR_UMCTRL_ADDR + UMCTRL_MRSTAT_REG_OFFSET)) & 0x1) != 0);

    //setting mr reg addr
    writel(0xff00 & (addr << 8), (void __iomem *)(DDR_UMCTRL_ADDR + UMCTRL_MRCTRL1_REG_OFFSET));

    //trigger read operation. bit0: 0 write; 1 read. bit4 for rank, pc805 has 1 rank. bit31 0->1 to trigger
    writel((0x1 | (1<<4)), (void __iomem *)(DDR_UMCTRL_ADDR + UMCTRL_MRCTRL0_REG_OFFSET));
    writel((0x1 | (1<<4) | (1<<31)), (void __iomem *)(DDR_UMCTRL_ADDR + UMCTRL_MRCTRL0_REG_OFFSET));

    //loop waiting until complete
    while ((readl((void __iomem *)(DDR_UMCTRL_ADDR + UMCTRL_MRCTRL0_REG_OFFSET)) >> 31) == 0x1);

    //read mr reg data
    rdata = readl((void __iomem *)(DDR_SYS_CTRL_ADDR + MRR_DATA0_REG_OFFSET));

    return (rdata & 0xff);
}

void lpddr4_mr_reg_write(unsigned char addr, unsigned char val)
{
    //loop waiting if write busy. mrstat reg bit0
    while ((readl((void __iomem *)(DDR_UMCTRL_ADDR + UMCTRL_MRSTAT_REG_OFFSET)) & 0x1) != 0);

    //setting mr reg addr and value
    writel((0xff00 & (addr << 8)) | (0xff & val), (void __iomem *)(DDR_UMCTRL_ADDR + UMCTRL_MRCTRL1_REG_OFFSET));

    //trigger read operation. bit0: 0 write; 1 read. bit4 for rank, pc805 has 1 rank. bit31 0->1 to trigger
    writel((1<<4), (void __iomem *)(DDR_UMCTRL_ADDR + UMCTRL_MRCTRL0_REG_OFFSET));
    writel(((1<<4) | (1<<31)), (void __iomem *)(DDR_UMCTRL_ADDR + UMCTRL_MRCTRL0_REG_OFFSET));

    //loop waiting until complete
    while ((readl((void __iomem *)(DDR_UMCTRL_ADDR + UMCTRL_MRCTRL0_REG_OFFSET)) >> 31) == 0x1);
}

pc805_ddr_rank_size_t board_get_lpddr4_size_by_conf(void)
{
    pc805_ddr_rank_size_t ddr_size;

    switch (CONFIG_SPL_LPDDR4_TOTAL_SIZE) {
        case 128:
            ddr_size = DDR_RANK_128MBYTE;
            break;
        case 256:
            ddr_size = DDR_RANK_256MBYTE;
            break;
        case 384:
            ddr_size = DDR_RANK_384MBYTE;
            break;
        case 768:
            ddr_size = DDR_RANK_768MBYTE;
            break;
        case 1024:
            ddr_size = DDR_RANK_1024MBYTE;
            break;
        case 1536:
            ddr_size = DDR_RANK_1536MBYTE;
            break;
        case 2048:
            ddr_size = DDR_RANK_2048MBYTE;
            break;
        default: //512MB
            ddr_size = DDR_RANK_512MBYTE;
            break;
    }

    return ddr_size;
}

void board_init_f(ulong dummy)
{
    int ret;
    pc805_ddr_rank_size_t ddr_size;
    bool inline_ecc = false;

    ddr_size = board_get_lpddr4_size_by_conf();
#if CONFIG_SPL_LPDDR4_INLINE_ECC_ENABLE
    inline_ecc = true;
#endif

    vfw_cpu_crg_init();
    vfw_sys_crg_init();
    vfw_init_ddr(ddr_size, inline_ecc);

#if CONFIG_SPL_LPDDR4_ISSI_ECC_ERR_ON
    //default on-chip ecc enabled
    //consider how to handle err indication signal on the board and open it as requirement
    vfw_ddr_mr_write(LPDDR4_ISSI_REG_MR33, 0xc0); //Enable ECCON and ERRON
#endif

    ret = spl_early_init();
    if (ret) {
        panic("spl_early_init() failed: %d\n", ret);
    }

    arch_cpu_init_dm();

    preloader_console_init();

    printf("lpddr4 size %d, inline ecc %d\n", ddr_size, inline_ecc);
#if CONFIG_SPL_LPDDR4_ISSI_ECC_ERR_ON
    printf("lpddr4 ISSI. MR33 0x%x, MR34 0x%x\n", lpddr4_mr_reg_read(LPDDR4_ISSI_REG_MR33), lpddr4_mr_reg_read(LPDDR4_ISSI_REG_MR34));
#endif
}

/* nand_init() - initialize data to make nand usable by SPL */
void nand_init(void)
{
    board_spinand_init();
}

void nand_deselect(void)
{
	printf("call %s\n", __func__);
}

int nand_spl_load_image(uint32_t offs, unsigned int size, void *dst)
{
	int ret = 0;
	unsigned int off = offs;
	unsigned int lim = offs + size;
	unsigned int remaining = size;
	unsigned int column;

	struct mtd_oob_ops io_op = {};

    if (IS_ERR_OR_NULL(mtd))
    {
        pr_err("%s mtd is not avaiable.\n", __func__);
        return -ENODEV;
    }

	io_op.mode = MTD_OPS_AUTO_OOB;
	io_op.len = min(remaining, mtd->writesize);
	io_op.ooblen = 0;
	io_op.datbuf = dst;
	io_op.oobbuf = NULL;

	/* Loop over to do the actual read/write */
	while (remaining) {
		if (off + remaining > lim) {
			pr_err("Limit reached 0x%x while reading at offset 0x%x\n",
				lim, off);
			return -EIO;
		}
		/* Skip the block if it is bad */
		if (mtd_block_isbad(mtd, off)) {
			off += mtd->erasesize;
			printf("off ot aligned %d or bad block\n", off);
			continue;
		}

		column = off % mtd->writesize;
		if (column) {
			/* Offset not aligned with page.
			   Read whole page and then pick data */
			off &= ~(mtd->writesize - 1);
			io_op.len = mtd->writesize;
			debug("offs=0x%x, column=%d, align to off=0x%x\n",
				offs, column, off);
		}

		ret = mtd_read_oob(mtd, off, &io_op);

		if (ret) {
			printf("Failure while reading at offset 0x%x, err = %d\n", off, ret);
			return -EIO;
		}

		off += io_op.retlen;

		if (unlikely(column)) {
			/* Partial page read */
			io_op.retlen = mtd->writesize - column;
			memmove(io_op.datbuf, io_op.datbuf + column, io_op.retlen);
			column = 0;
		}

		remaining -= io_op.retlen;
		io_op.datbuf += io_op.retlen;
		io_op.len = min(remaining, mtd->writesize);
	}
	return ret;
}

/**
 * nand_spl_adjust_offset - Adjust offset from a starting sector
 * @sector:	Address of the sector
 * @offs:	Offset starting from @sector
 *
 * If one or more bad blocks are in the address space between @sector
 * and @sector + @offs, @offs is increased by the NAND block size for
 * each bad block found.
 */
u32 nand_spl_adjust_offset(u32 sector, u32 offs)
{
	unsigned int block, lastblock;

    if (IS_ERR_OR_NULL(mtd))
    {
        pr_err("%s mtd is not avaiable.\n", __func__);
        return -ENODEV;
    }

	block = sector / mtd->erasesize;
	lastblock = (sector + offs) / mtd->erasesize;

	while (block <= lastblock) {
		if (mtd_block_isbad(mtd, block)) {
			offs += mtd->erasesize;
			lastblock++;
		}

		block++;
	}
	return offs;
}

#ifdef CONFIG_SPL
bool pc805_spl_is_uartboot(void)
{
	/* read gpio31 value to determine boot type:
	 * 0. UART boot
	 * 1. NAND boot
	 */
	u32 val = 0;
	writel((0x1 << 31), (void __iomem *)0x8202018);
	mdelay(1);
	val = readl((void __iomem *)0x8202028);
	printf("Boot select detect: register val = 0x%x\n", val);

	return (val & (0x01 << 31)) ? false : true;
}

void board_boot_order(u32 *spl_boot_list)
{
	u8 i;
	u32 boot_devices[] = {
#ifdef CONFIG_SPL_RAM_SUPPORT
		BOOT_DEVICE_RAM,
#endif
#ifdef CONFIG_SPL_MMC_SUPPORT
		BOOT_DEVICE_MMC1,
#endif
#ifdef CONFIG_SPL_NAND_SUPPORT
		BOOT_DEVICE_NAND,
#endif
#ifdef CONFIG_SPL_YMODEM_SUPPORT
		BOOT_DEVICE_UART,
#endif
	};

	if(pc805_spl_is_uartboot())
	{
		spl_boot_list[0] = BOOT_DEVICE_UART;
		return;
	}
	for (i = 0; i < ARRAY_SIZE(boot_devices); i++)
		spl_boot_list[i] = boot_devices[i];
}
#endif

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	/* boot using first FIT config */
	return 0;
}
#endif
