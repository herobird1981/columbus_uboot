/*
 * TI OMAP timer driver
 *
 * Copyright (C) 2015, Texas Instruments, Incorporated
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <timer.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

/* Timer register bits */

struct columbus_timer_regs {
	u32	trig;
	u32	load;
	u32	count;
	u32	cap1;
	u32	cap2;
	u32	compare;
	u32	ctrl;
	u32	irq_status;
	u32	irq_raw;
	u32	irq_en;
};

struct columbus_timer_priv {
	struct columbus_timer_regs *regs;
	u32 pclk;
	u32 freq;
};

static int columbus_timer_get_count(struct udevice *dev, u64 *count)
{
	struct columbus_timer_priv *priv = dev_get_priv(dev);
	u32 val;

	val = readl(&priv->regs->count);
	*count = timer_conv_64(val);
	return 0;
}

static int columbus_timer_probe(struct udevice *dev)
{
	struct timer_dev_priv *uc_priv = dev_get_uclass_priv(dev);
	struct columbus_timer_priv *priv = dev_get_priv(dev);
	int i;
	u32 div;
	u32 ctrl;

	/*calculate best div*/
	div = priv->pclk / priv->freq;
	if ( div > 256)
		div = 256;
	else if (div==0)
		div = 1;
	/*convert div to register config val*/
	for (i=0;i<=8;i++)
		if ((div >= (1<<i)) && (div < (1<<(i+1))))
			break;
	div = i;

	uc_priv->clock_rate = priv->pclk / (1<<div);

	/*reset*/
	writel(0x8000, &priv->regs->ctrl);
	for (i=0;i<1000;i++) {
		readl(&priv->regs->count);
	}
	writel(0x0000, &priv->regs->ctrl);

	/*configure default*/
	writel(0x0, &priv->regs->load);
	writel(0x0, &priv->regs->count);
	writel(0x0, &priv->regs->irq_en);
	writel(0x7, &priv->regs->irq_raw);
	writel(0x7, &priv->regs->irq_status);

	/*count mode: up ; run mode: auto; mode: usr;  start timer*/
	ctrl = (1<<12) |(1<<11) |((div & 0xf) <<1)|0x1;
	writel(ctrl, &priv->regs->ctrl);

	return 0;
}

static int columbus_timer_ofdata_to_platdata(struct udevice *dev)
{
	struct columbus_timer_priv *priv = dev_get_priv(dev);

	priv->regs = (struct columbus_timer_regs *)dev_get_addr(dev);

	priv->pclk = fdtdec_get_int(gd->fdt_blob, dev->of_offset,
		"clock-frequency", DEFAULT_PCLK);
	priv->freq = fdtdec_get_int(gd->fdt_blob, dev->of_offset,
		"clock-freq", 1000000);
	return 0;
}

static const struct timer_ops columbus_timer_ops = {
	.get_count = columbus_timer_get_count,
};

static const struct udevice_id columbus_timer_ids[] = {
	{ .compatible = "columbus,timer" },
	{}
};

U_BOOT_DRIVER(columbus_timer) = {
	.name	= "columbus_timer",
	.id	= UCLASS_TIMER,
	.of_match = columbus_timer_ids,
	.ofdata_to_platdata = columbus_timer_ofdata_to_platdata,
	.priv_auto_alloc_size = sizeof(struct columbus_timer_priv),
	.probe = columbus_timer_probe,
	.ops	= &columbus_timer_ops,
	.flags = DM_FLAG_PRE_RELOC,
};
