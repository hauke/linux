/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2011-2015 John Crispin <blogic@openwrt.org>
 *  Copyright (C) 2015 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 */

#include <linux/ioport.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/regmap.h>

#include <lantiq_soc.h>

#define XBAR_ALWAYS_LAST	0x430
#define XBAR_FPI_BURST_EN	BIT(1)
#define XBAR_AHB_BURST_EN	BIT(2)

#define RCU_VR9_BE_AHB1S	0x00000008

static int ltq_xbar_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource res_xbar;
	struct regmap *rcu_regmap;
	void __iomem *xbar_membase;
	u32 rcu_ahb_endianness_reg_offset;
	u32 rcu_ahb_endianness_val;
	int ret;

	ret = of_address_to_resource(np, 0, &res_xbar);
	if (ret) {
		dev_err(dev, "Failed to get xbar resources");
		return ret;
	}

	if (!devm_request_mem_region(dev, res_xbar.start,
				     resource_size(&res_xbar),
		res_xbar.name)) {
		dev_err(dev, "Failed to get xbar resources");
		return -ENODEV;
	}

	xbar_membase = devm_ioremap_nocache(dev, res_xbar.start,
						resource_size(&res_xbar));
	if (!xbar_membase) {
		dev_err(dev, "Failed to remap xbar resources");
		return -ENODEV;
	}

	/* RCU configuration is optional */
	rcu_regmap = syscon_regmap_lookup_by_phandle(np, "lantiq,rcu-syscon");
	if (!IS_ERR_OR_NULL(rcu_regmap)) {
		if (of_property_read_u32_index(np, "lantiq,rcu-syscon", 1,
			&rcu_ahb_endianness_reg_offset)) {
			dev_err(&pdev->dev, "Failed to get RCU reg offset\n");
			return -EINVAL;
		}

		if (of_device_is_big_endian(np))
			rcu_ahb_endianness_val = RCU_VR9_BE_AHB1S;
		else
			rcu_ahb_endianness_val = 0;

		if (regmap_update_bits(rcu_regmap,
					rcu_ahb_endianness_reg_offset,
					RCU_VR9_BE_AHB1S,
					rcu_ahb_endianness_val))
			dev_warn(&pdev->dev,
				"Failed to configure RCU AHB endianness\n");
	}

	/* disable fpi burst */
	ltq_w32_mask(XBAR_FPI_BURST_EN, 0,
		     xbar_membase + XBAR_ALWAYS_LAST);

	return 0;
}

static const struct of_device_id xbar_match[] = {
	{ .compatible = "lantiq,xbar-xway" },
	{},
};
MODULE_DEVICE_TABLE(of, xbar_match);

static struct platform_driver xbar_driver = {
	.probe = ltq_xbar_probe,
	.driver = {
		.name = "xbar-xway",
		.of_match_table = xbar_match,
	},
};

builtin_platform_driver(xbar_driver);
