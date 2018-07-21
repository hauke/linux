/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2010 John Crispin <john@phrozen.org>
 *  Copyright (C) 2017 Hauke Mehrtens <hauke@hauke-m.de>
 *  Based on EP93xx wdt driver
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/of_platform.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <lantiq_soc.h>

#define LTQ_XRX_RCU_RST_STAT		0x0014
#define LTQ_XRX_RCU_RST_STAT_WDT	BIT(31)

/* CPU0 Reset Source Register */
#define LTQ_FALCON_SYS1_CPU0RS		0x0060
/* reset cause mask */
#define LTQ_FALCON_SYS1_CPU0RS_MASK	0x0007
#define LTQ_FALCON_SYS1_CPU0RS_WDT	0x02

/*
 * Section 3.4 of the datasheet
 * The password sequence protects the WDT control register from unintended
 * write actions, which might cause malfunction of the WDT.
 *
 * essentially the following two magic passwords need to be written to allow
 * IO access to the WDT core
 */
#define LTQ_WDT_CR_PW1		0x00BE0000
#define LTQ_WDT_CR_PW2		0x00DC0000

#define LTQ_WDT_CR		0x0		/* watchdog control register */
#define  LTQ_WDT_CR_GEN		BIT(31)		/* enable bit */
#define  LTQ_WDT_CR_PWL		(0x3 << 26)	/* Pre-warning limit set to 1/16 of max WDT period */
#define  LTQ_WDT_CR_CLKDIV	(0x3 << 24)	/* set clock divider to 0x40000 */
#define  LTQ_WDT_CR_PW_MASK	GENMASK(23, 16)	/* Password field */
#define  LTQ_WDT_CR_RELOAD_MASK	GENMASK(15, 0)	/* Reload value */
#define LTQ_WDT_SR		0x8	/* watchdog status register */
#define  LTQ_WDT_SR_VALUE_MASK	GENMASK(15, 0)	/* Timer value */

#define LTQ_WDT_DIVIDER		0x40000ll
#define LTQ_MAX_TIMEOUT		((1 << 16) - 1)	/* the reload field is 16 bit */

static bool nowayout = WATCHDOG_NOWAYOUT;

struct ltq_wdt_priv {
	struct watchdog_device wdt;
	void __iomem *membase;
	unsigned long clk_rate;
};

static u32 ltq_wdt_r32(struct ltq_wdt_priv *priv, u32 offset)
{
	return __raw_readl(priv->membase + offset);
}

static void ltq_wdt_w32(struct ltq_wdt_priv *priv, u32 val, u32 offset)
{
	return __raw_writel(val, priv->membase + offset);
}

static void ltq_wdt_mask(struct ltq_wdt_priv *priv, u32 clear, u32 set,
			 u32 offset)
{
	u32 val = ltq_wdt_r32(priv, offset);

	val &= ~(clear);
	val |= set;
	ltq_wdt_w32(priv, val, offset);
}

static struct ltq_wdt_priv *ltq_wdt_get_priv(struct watchdog_device *wdt)
{
	return container_of(wdt, struct ltq_wdt_priv, wdt);
}

static struct watchdog_info ltq_wdt_info = {
	.options = WDIOF_MAGICCLOSE | WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING |
		   WDIOF_CARDRESET,
	.identity = "ltq_wdt",
};

static int ltq_wdt_start(struct watchdog_device *wdt)
{
	struct ltq_wdt_priv *priv = ltq_wdt_get_priv(wdt);
	u32 timeout;

	timeout = wdt->timeout * (priv->clk_rate / LTQ_WDT_DIVIDER);

	ltq_wdt_mask(priv, LTQ_WDT_CR_PW_MASK, LTQ_WDT_CR_PW1, LTQ_WDT_CR);
	/* write the second magic plus the configuration and new timeout */
	ltq_wdt_mask(priv, LTQ_WDT_CR_PW_MASK | LTQ_WDT_CR_RELOAD_MASK,
		     LTQ_WDT_CR_GEN | LTQ_WDT_CR_PWL | LTQ_WDT_CR_CLKDIV |
		     LTQ_WDT_CR_PW2 | timeout,
		     LTQ_WDT_CR);

	return 0;
}

static int ltq_wdt_stop(struct watchdog_device *wdt)
{
	struct ltq_wdt_priv *priv = ltq_wdt_get_priv(wdt);

	ltq_wdt_mask(priv, LTQ_WDT_CR_PW_MASK, LTQ_WDT_CR_PW1, LTQ_WDT_CR);
	ltq_wdt_mask(priv, LTQ_WDT_CR_GEN | LTQ_WDT_CR_PW_MASK,
		     LTQ_WDT_CR_PW2, LTQ_WDT_CR);

	return 0;
}

static int ltq_wdt_ping(struct watchdog_device *wdt)
{
	struct ltq_wdt_priv *priv = ltq_wdt_get_priv(wdt);
	u32 timeout;

	timeout = wdt->timeout * (priv->clk_rate / LTQ_WDT_DIVIDER);

	ltq_wdt_mask(priv, LTQ_WDT_CR_PW_MASK, LTQ_WDT_CR_PW1, LTQ_WDT_CR);
	/* write the second magic plus the configuration and new timeout */
	ltq_wdt_mask(priv, LTQ_WDT_CR_PW_MASK | LTQ_WDT_CR_RELOAD_MASK,
		     LTQ_WDT_CR_PW2 | timeout, LTQ_WDT_CR);

	return 0;
}

static const struct watchdog_ops ltq_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= ltq_wdt_start,
	.stop		= ltq_wdt_stop,
	.ping		= ltq_wdt_ping,
};

typedef int (*ltq_wdt_bootstatus_get)(struct platform_device *pdev);

static int ltq_wdt_bootstatus_xrx(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap *rcu_regmap;
	u32 val;
	int err;

	rcu_regmap = syscon_regmap_lookup_by_phandle(dev->of_node, "regmap");
	if (IS_ERR(rcu_regmap))
		return PTR_ERR(rcu_regmap);

	err = regmap_read(rcu_regmap, LTQ_XRX_RCU_RST_STAT, &val);
	if (err)
		return err;

	if (val & LTQ_XRX_RCU_RST_STAT_WDT)
		return WDIOF_CARDRESET;

	return 0;
}

static int ltq_wdt_bootstatus_falcon(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap *rcu_regmap;
	u32 val;
	int err;

	rcu_regmap = syscon_regmap_lookup_by_phandle(dev->of_node,
						     "lantiq,rcu");
	if (IS_ERR(rcu_regmap))
		return PTR_ERR(rcu_regmap);

	err = regmap_read(rcu_regmap, LTQ_FALCON_SYS1_CPU0RS, &val);
	if (err)
		return err;

	if ((val & LTQ_FALCON_SYS1_CPU0RS_MASK) == LTQ_FALCON_SYS1_CPU0RS_WDT)
		return WDIOF_CARDRESET;

	return 0;
}

static int ltq_wdt_probe(struct platform_device *pdev)
{
	u64 max_clocks = LTQ_MAX_TIMEOUT * LTQ_WDT_DIVIDER;
	struct ltq_wdt_priv *priv;
	struct watchdog_device *wdt;
	struct resource *res;
	struct clk *clk;
	ltq_wdt_bootstatus_get ltq_wdt_bootstatus_get;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	wdt = &priv->wdt;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->membase))
		return PTR_ERR(priv->membase);

	/* we do not need to enable the clock as it is always running */
	clk = clk_get_io();
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "Failed to get clock\n");
		return -ENOENT;
	}
	priv->clk_rate = clk_get_rate(clk);
	clk_put(clk);

	wdt->info		= &ltq_wdt_info;
	wdt->ops		= &ltq_wdt_ops;
	wdt->status		= 0;
	wdt->min_timeout	= 1;
	wdt->max_timeout	= div_u64(max_clocks, priv->clk_rate);
	wdt->timeout		= wdt->max_timeout;
	wdt->parent		= &pdev->dev;

	ltq_wdt_bootstatus_get = of_device_get_match_data(&pdev->dev);
	if (ltq_wdt_bootstatus_get) {
		ret = ltq_wdt_bootstatus_get(pdev);
		if (ret < 0)
			return ret;
		wdt->bootstatus = ret;
	}

	watchdog_set_nowayout(wdt, nowayout);
	platform_set_drvdata(pdev, priv);

	ltq_wdt_stop(wdt);

	ret = watchdog_register_device(wdt);
	if (ret)
		return ret;

	return 0;
}

static int ltq_wdt_remove(struct platform_device *pdev)
{
	struct ltq_wdt_priv *priv = platform_get_drvdata(pdev);

	watchdog_unregister_device(&priv->wdt);

	return 0;
}

static const struct of_device_id ltq_wdt_match[] = {
	{ .compatible = "lantiq,wdt", .data = NULL},
	{ .compatible = "lantiq,xrx100-wdt", .data = ltq_wdt_bootstatus_xrx },
	{ .compatible = "lantiq,falcon-wdt", .data = ltq_wdt_bootstatus_falcon },
	{},
};
MODULE_DEVICE_TABLE(of, ltq_wdt_match);

static struct platform_driver ltq_wdt_driver = {
	.probe = ltq_wdt_probe,
	.remove = ltq_wdt_remove,
	.driver = {
		.name = "wdt",
		.of_match_table = ltq_wdt_match,
	},
};

module_platform_driver(ltq_wdt_driver);

module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started");
MODULE_AUTHOR("John Crispin <john@phrozen.org>");
MODULE_DESCRIPTION("Lantiq SoC Watchdog");
MODULE_LICENSE("GPL");
