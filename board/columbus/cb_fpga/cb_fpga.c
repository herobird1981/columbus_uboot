/*
 * columbus.c - board file for britesemi's columbus board
 *
 *
 * Copyright (C) 2011-2012 8D Technologies inc.
 * Copyright (C) 2009 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <asm/io.h>
#include <asm/mach-types.h>

DECLARE_GLOBAL_DATA_PTR;

int board_early_init_f(void)
{
	return 0;
}

int board_init(void)
{
	return 0;
}

int dram_init(void)
{
	gd->ram_size = CONFIG_SYS_SDRAM_SIZE;
	return 0;
}