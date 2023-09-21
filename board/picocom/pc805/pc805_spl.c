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

#include "pc805_vfw4spl.h"

void board_init_f(ulong dummy)
{
    int ret;
    void (*entry)(unsigned int, unsigned int);

#if 1
    vfw_cpu_crg_init();
    vfw_sys_crg_init();
    vfw_init_ddr();
#endif

    ret = spl_early_init();
    if (ret) {
        panic("spl_early_init() failed: %d\n", ret);
    }

    arch_cpu_init_dm();

    preloader_console_init();
}

