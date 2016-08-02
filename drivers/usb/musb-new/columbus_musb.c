/*
 * Allwinner SUNXI "glue layer"
 *
 * Copyright © 2015 Hans de Goede <hdegoede@redhat.com>
 * Copyright © 2013 Jussi Kivilinna <jussi.kivilinna@iki.fi>
 *
 * Based on the sw_usb "Allwinner OTG Dual Role Controller" code.
 *  Copyright 2007-2012 (C) Allwinner Technology Co., Ltd.
 *  javen <javen@allwinnertech.com>
 *
 * Based on the DA8xx "glue layer" code.
 *  Copyright (c) 2008-2009 MontaVista Software, Inc. <source@mvista.com>
 *  Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */
#include <common.h>
#include <dm/lists.h>
#include <dm/root.h>
#include <linux/usb/musb.h>
#include "linux-compat.h"
#include "musb_core.h"
#include "musb_uboot.h"

static int columbus_musb_init(struct musb *musb)
{
	return 0;
}

static const struct musb_platform_ops columbus_musb_ops = {
	.init		= columbus_musb_init,
};

static struct musb_hdrc_config musb_config = {
	.multipoint     = 1,
	.dyn_fifo       = 1,
	.num_eps        = 16,
	.ram_bits       = 12,
};

static struct musb_hdrc_platform_data musb_plat = {
	.mode		= MUSB_PERIPHERAL,
	.config         = &musb_config,
	.power          = 250,
	.platform_ops	= &columbus_musb_ops,
};

void columbus_musb_board_init(void)
{
	musb_register(&musb_plat, NULL, (void *)0x7B000000);
}
