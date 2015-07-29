/*
 * Lantiq XWAY SoC RCU module based USB 1.1/2.0 PHY driver
 *
 * Copyright (C) 2016 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 * Copyright (C) 2017 Hauke Mehrtens <hauke@hauke-m.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#define MAX_VBUS_GPIO		2

/* Transmitter HS Pre-Emphasis Enable */
#define RCU_CFG1_TX_PEE		BIT(0)
/* Disconnect Threshold */
#define RCU_CFG1_DIS_THR_MASK	0x00038000
#define RCU_CFG1_DIS_THR_SHIFT	15

struct ltq_rcu_usb2_bits {
	u8 hostmode;
	u8 slave_endianness;
	u8 host_endianness;
	bool have_ana_cfg;
};

struct ltq_rcu_usb2_priv {
	struct regmap			*regmap;
	u32				phy_reg_offset;
	u32				ana_cfg1_reg_offset;
	const struct ltq_rcu_usb2_bits	*reg_bits;
	struct device			*dev;
	struct gpio_desc		*gpiod_vbus[MAX_VBUS_GPIO];
	struct phy			*phy;
	struct clk			*ctrl_gate_clk;
	struct clk			*phy_gate_clk;
	struct reset_control		*ctrl_reset;
	struct reset_control		*phy_reset;
};

static const struct ltq_rcu_usb2_bits xway_rcu_usb2_reg_bits = {
	.hostmode = 11,
	.slave_endianness = 9,
	.host_endianness = 10,
	.have_ana_cfg = false,
};

static const struct ltq_rcu_usb2_bits xrx100_rcu_usb2_reg_bits = {
	.hostmode = 11,
	.slave_endianness = 17,
	.host_endianness = 10,
	.have_ana_cfg = false,
};

static const struct ltq_rcu_usb2_bits xrx200_rcu_usb2_reg_bits = {
	.hostmode = 11,
	.slave_endianness = 9,
	.host_endianness = 10,
	.have_ana_cfg = true,
};

static const struct of_device_id ltq_rcu_usb2_phy_of_match[] = {
	{
		.compatible = "lantiq,ase-rcu-usb2-phy",
		.data = &xway_rcu_usb2_reg_bits,
	},
	{
		.compatible = "lantiq,danube-rcu-usb2-phy",
		.data = &xway_rcu_usb2_reg_bits,
	},
	{
		.compatible = "lantiq,xrx100-rcu-usb2-phy",
		.data = &xrx100_rcu_usb2_reg_bits,
	},
	{
		.compatible = "lantiq,xrx200-rcu-usb2-phy",
		.data = &xrx200_rcu_usb2_reg_bits,
	},
	{
		.compatible = "lantiq,xrx300-rcu-usb2-phy",
		.data = &xrx200_rcu_usb2_reg_bits,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, ltq_rcu_usb2_phy_of_match);

static void ltq_rcu_usb2_set_vbus_gpio_value(struct gpio_desc **gpiods,
						int value)
{
	int i;

	for (i = 0; i < MAX_VBUS_GPIO; i++)
		if (!IS_ERR_OR_NULL(gpiods[i]))
			gpiod_set_value(gpiods[i], value);
}

static int ltq_rcu_usb2_phy_power_on(struct phy *phy)
{
	struct ltq_rcu_usb2_priv *priv = phy_get_drvdata(phy);

	if (priv->phy_reset)
		reset_control_deassert(priv->phy_reset);

	/* enable the port-specific VBUS GPIOs if available */
	ltq_rcu_usb2_set_vbus_gpio_value(priv->gpiod_vbus, 1);

	return 0;
}

static int ltq_rcu_usb2_phy_power_off(struct phy *phy)
{
	struct ltq_rcu_usb2_priv *priv = phy_get_drvdata(phy);

	/*
	 * only disable the port-specific VBUS GPIO here (if available), the
	 * shared VBUS GPIO might still be used by another port
	 */
	ltq_rcu_usb2_set_vbus_gpio_value(priv->gpiod_vbus, 0);

	if (priv->phy_reset)
		reset_control_assert(priv->phy_reset);

	return 0;
}

static struct phy_ops ltq_rcu_usb2_phy_ops = {
	.power_on	= ltq_rcu_usb2_phy_power_on,
	.power_off	= ltq_rcu_usb2_phy_power_off,
	.owner		= THIS_MODULE,
};

static void ltq_rcu_usb2_start_cores(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ltq_rcu_usb2_priv *priv = dev_get_drvdata(dev);

	/* Power on the USB core. */
	if (clk_prepare_enable(priv->ctrl_gate_clk)) {
		dev_err(dev, "failed to enable CTRL gate\n");
		return;
	}

	/*
	 * Power on the USB PHY. We have to do it early because
	 * otherwise the second core won't turn on properly.
	 */
	if (clk_prepare_enable(priv->phy_gate_clk)) {
		dev_err(dev, "failed to enable PHY gate\n");
		return;
	}

	if (priv->reg_bits->have_ana_cfg) {
		regmap_update_bits(priv->regmap, priv->ana_cfg1_reg_offset,
			RCU_CFG1_TX_PEE, RCU_CFG1_TX_PEE);
		regmap_update_bits(priv->regmap, priv->ana_cfg1_reg_offset,
			RCU_CFG1_DIS_THR_MASK, 7 << RCU_CFG1_DIS_THR_SHIFT);
	}

	/* Configure core to host mode */
	regmap_update_bits(priv->regmap, priv->phy_reg_offset,
			   BIT(priv->reg_bits->hostmode), 0);

	/* Select DMA endianness (Host-endian: big-endian) */
	regmap_update_bits(priv->regmap, priv->phy_reg_offset,
		BIT(priv->reg_bits->slave_endianness), 0);
	regmap_update_bits(priv->regmap, priv->phy_reg_offset,
		BIT(priv->reg_bits->host_endianness),
		BIT(priv->reg_bits->host_endianness));

	/* Reset USB core throgh reset controller */
	reset_control_deassert(priv->ctrl_reset);

	if (priv->phy_reset)
		reset_control_assert(priv->phy_reset);
}

static int ltq_rcu_usb2_get_vbus_gpios(struct device *dev,
					  struct gpio_desc **gpios)
{
	int i;

	for (i = 0; i < MAX_VBUS_GPIO; i++) {
		gpios[i] = devm_gpiod_get_index_optional(dev, "vbus", i,
							 GPIOD_OUT_LOW);
		if (IS_ERR(gpios[i]))
			return PTR_ERR(gpios[i]);
	}

	return 0;
}

static int ltq_rcu_usb2_of_probe(struct device_node *phynode,
				    struct ltq_rcu_usb2_priv *priv)
{
	struct device *dev = priv->dev;
	const struct of_device_id *match =
		of_match_node(ltq_rcu_usb2_phy_of_match, phynode);
	int ret;

	if (!match) {
		dev_err(dev, "Not a compatible Lantiq RCU USB PHY\n");
		return -EINVAL;
	}

	priv->reg_bits = match->data;

	priv->regmap = syscon_regmap_lookup_by_phandle(phynode,
						       "lantiq,rcu-syscon");
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "Failed to lookup RCU regmap\n");
		return PTR_ERR(priv->regmap);
	}

	ret = ltq_rcu_usb2_get_vbus_gpios(dev, priv->gpiod_vbus);
	if (ret) {
		dev_err(dev, "failed to request shared USB VBUS GPIO\n");
		return ret;
	}

	priv->ctrl_gate_clk = devm_clk_get(dev, "ctrl");
	if (IS_ERR(priv->ctrl_gate_clk)) {
		dev_err(dev, "Unable to get USB ctrl gate clk\n");
		return PTR_ERR(priv->ctrl_gate_clk);
	}

	priv->phy_gate_clk = devm_clk_get(dev, "phy");
	if (IS_ERR(priv->phy_gate_clk)) {
		dev_err(dev, "Unable to get USB phy gate clk\n");
		return PTR_ERR(priv->phy_gate_clk);
	}

	priv->ctrl_reset = devm_reset_control_get_shared(dev, "ctrl");
	if (IS_ERR(priv->ctrl_reset)) {
		dev_err(dev, "failed to get 'ctrl' reset\n");
		return PTR_ERR(priv->ctrl_reset);
	}

	priv->phy_reset = devm_reset_control_get_optional(dev, "phy");
	if (IS_ERR(priv->phy_reset)) {
		if (PTR_ERR(priv->phy_reset) == -EPROBE_DEFER)
			return PTR_ERR(priv->phy_reset);
		priv->phy_reset = NULL;
	}

	ret = of_property_read_u32_index(phynode, "lantiq,rcu-syscon", 1,
					 &priv->phy_reg_offset);
	if (ret) {
		dev_err(dev, "Failed to get RCU PHY reg offset\n");
		return ret;
	}

	if (priv->reg_bits->have_ana_cfg) {
		ret = of_property_read_u32_index(phynode, "lantiq,rcu-syscon",
						 2, &priv->ana_cfg1_reg_offset);
		if (ret) {
			dev_dbg(dev, "Failed to get RCU ANA CFG1 reg offset\n");
			return ret;
		}
	}

	return 0;
}

static int ltq_rcu_usb2_phy_probe(struct platform_device *pdev)
{
	struct device_node *child, *np = pdev->dev.of_node;
	struct ltq_rcu_usb2_priv *priv;
	struct phy_provider *provider;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv),
				       GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	dev_set_drvdata(priv->dev, priv);

	ret = ltq_rcu_usb2_of_probe(np, priv);
	if (ret)
		return ret;

	priv->phy = devm_phy_create(&pdev->dev, child,
					 &ltq_rcu_usb2_phy_ops);
	if (IS_ERR(priv->phy)) {
		dev_err(&pdev->dev, "failed to create PHY\n");
		return PTR_ERR(priv->phy);
	}

	phy_set_drvdata(priv->phy, priv);

	ltq_rcu_usb2_start_cores(pdev);

	provider = devm_of_phy_provider_register(&pdev->dev,
						 of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static struct platform_driver ltq_rcu_usb2_phy_driver = {
	.probe	= ltq_rcu_usb2_phy_probe,
	.driver = {
		.name	= "lantiq-rcu-usb2-phy",
		.of_match_table	= ltq_rcu_usb2_phy_of_match,
	}
};
module_platform_driver(ltq_rcu_usb2_phy_driver);

MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Lantiq XWAY USB2 PHY driver");
MODULE_LICENSE("GPL v2");
