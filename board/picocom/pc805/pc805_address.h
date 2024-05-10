/*
 * Copyright (C) 2019 Picocom Technology Corporation
 * Wei Zheng, Picocom Technology Corporation <wei.zheng@picocom.com>
 */


#define DDR_CFG_ADDR            0x09000000


#define DDR_UMCTRL_OFFSET       0x0
#define DDR_UMCTRL_ADDR         (DDR_CFG_ADDR + DDR_UMCTRL_OFFSET)

#define DDR_SYS_CTRL_OFFSET     0x200000
#define DDR_SYS_CTRL_ADDR       (DDR_CFG_ADDR + DDR_SYS_CTRL_OFFSET)


//ddr sys ctrl umctrl registers
#define UMCTRL_MRSTAT_REG_OFFSET    0x18
#define UMCTRL_MRCTRL1_REG_OFFSET   0x14
#define UMCTRL_MRCTRL0_REG_OFFSET   0x10


//ddr sys ctrl registers
#define MRR_DATA0_REG_OFFSET        0x20
