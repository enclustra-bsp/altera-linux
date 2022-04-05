/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2012 Pavel Machek <pavel@denx.de>
 * Copyright (C) 2012-2015 Altera Corporation
 */

#ifndef __MACH_CORE_H
#define __MACH_CORE_H

#define SOCFPGA_RSTMGR_CTRL	0x04
#define SOCFPGA_RSTMGR_MODMPURST	0x10
#define SOCFPGA_RSTMGR_MODPERRST	0x14
#define SOCFPGA_RSTMGR_BRGMODRST	0x1c

#define SOCFPGA_A10_RSTMGR_CTRL		0xC
#define SOCFPGA_A10_RSTMGR_MODMPURST	0x20

/* A10 System Manager Boot ROM code control register */
#define SOCFPGA_A10_SYSMGR_ROMCODE_CTRL	0x204

/* Boot ROM code control register bits */
#define ROMCODE_CTRL_WARMRSTCFGPINMUX	0x1	/* Reset pinmux on warm reset */
#define ROMCODE_CTRL_WARMRSTCFGIO	0x2	/* Reset IO config on warm reset */

/* System Manager bits */
#define RSTMGR_CTRL_SWCOLDRSTREQ	0x1	/* Cold Reset */
#define RSTMGR_CTRL_SWWARMRSTREQ	0x2	/* Warm Reset */

#define RSTMGR_MPUMODRST_CPU1		0x2     /* CPU1 Reset */

void socfpga_init_l2_ecc(void);
void socfpga_init_ocram_ecc(void);
void socfpga_init_arria10_l2_ecc(void);
void socfpga_init_arria10_ocram_ecc(void);
#define SYSMGR_SILICON_ID1_OFFSET 0x0
#define SYSMGR_SILICON_ID1_REV_SHIFT 0
#define SYSMGR_SILICON_ID1_REV_MASK 0x0000FFFF
#define SYSMGR_SILICON_ID1_ID_SHIFT 16
#define SYSMGR_SILICON_ID1_ID_MASK 0xFFFF0000

extern void __iomem *sys_manager_base_addr;
extern void __iomem *rst_manager_base_addr;
extern void __iomem *sdr_ctl_base_addr;

u32 socfpga_sdram_self_refresh(u32 sdr_base);
extern unsigned int socfpga_sdram_self_refresh_sz;

extern char secondary_trampoline[], secondary_trampoline_end[];

extern unsigned long socfpga_cpu1start_addr;

#define SOCFPGA_SCU_VIRT_BASE   0xfee00000

/* Clock manager defines */
#define SOCFPGA_ENABLE_PLL_REG	0xA0

#endif
