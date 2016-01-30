/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2012 John Crispin <blogic@phrozen.org>
 *  Copyright (C) 2016 Martin Blumenstingl <martin.blumenstingl@googlemail.com>
 *  Copyright (C) 2017 Hauke Mehrtens <hauke@hauke-m.de>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#include <dt-bindings/mips/lantiq_rcu_gphy.h>

#include <lantiq_soc.h>

#define XRX200_GPHY_FW_ALIGN	(16 * 1024)

struct xway_gphy_priv {
	struct clk *gphy_clk_gate;
	struct reset_control *gphy_reset;
	struct reset_control *gphy_reset2;
	struct regmap *regmap;
	struct notifier_block gphy_reboot_nb;
	u32 reg_offset;
	char *fw_name;
	dma_addr_t dma_addr;
	void *fw_addr;
	size_t size;
};

struct xway_gphy_match_data {
	char *fe_firmware_name;
	char *ge_firmware_name;
};

static const struct xway_gphy_match_data xrx200a1x_gphy_data = {
	.fe_firmware_name = "lantiq/xrx200_phy22f_a14.bin",
	.ge_firmware_name = "lantiq/xrx200_phy11g_a14.bin",
};

static const struct xway_gphy_match_data xrx200a2x_gphy_data = {
	.fe_firmware_name = "lantiq/xrx200_phy22f_a22.bin",
	.ge_firmware_name = "lantiq/xrx200_phy11g_a22.bin",
};

static const struct xway_gphy_match_data xrx300_gphy_data = {
	.fe_firmware_name = "lantiq/xrx300_phy22f_a21.bin",
	.ge_firmware_name = "lantiq/xrx300_phy11g_a21.bin",
};

static const struct of_device_id xway_gphy_match[] = {
	{ .compatible = "lantiq,xrx200a1x-rcu-gphy", .data = &xrx200a1x_gphy_data },
	{ .compatible = "lantiq,xrx200a2x-rcu-gphy", .data = &xrx200a2x_gphy_data },
	{ .compatible = "lantiq,xrx300-rcu-gphy", .data = &xrx300_gphy_data },
	{ .compatible = "lantiq,xrx330-rcu-gphy", .data = &xrx300_gphy_data },
	{},
};
MODULE_DEVICE_TABLE(of, xway_gphy_match);

static struct xway_gphy_priv *to_xway_gphy_priv(struct notifier_block *nb)
{
	return container_of(nb, struct xway_gphy_priv, gphy_reboot_nb);
}

static int xway_gphy_reboot_notify(struct notifier_block *reboot_nb,
				   unsigned long code, void *unused)
{
	struct xway_gphy_priv *priv = to_xway_gphy_priv(reboot_nb);

	if (priv) {
		reset_control_assert(priv->gphy_reset);
		reset_control_assert(priv->gphy_reset2);
	}

	return NOTIFY_DONE;
}

static int xway_gphy_load(struct platform_device *pdev,
			  struct xway_gphy_priv *priv,
			  dma_addr_t *dev_addr)
{
	const struct firmware *fw;
	void *fw_addr_align;
	int ret;

	ret = request_firmware(&fw, priv->fw_name, &pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to load firmware: %s, error: %i\n",
			priv->fw_name, ret);
		return ret;
	}

	/*
	 * GPHY cores need the firmware code in a persistent and contiguous
	 * memory area with a 16 kB boundary aligned start address.
	 */
	priv->size = fw->size + XRX200_GPHY_FW_ALIGN;

	priv->fw_addr = dma_alloc_coherent(&pdev->dev, priv->size,
					   &priv->dma_addr, GFP_KERNEL);
	if (priv->fw_addr) {
		fw_addr_align = PTR_ALIGN(priv->fw_addr, XRX200_GPHY_FW_ALIGN);
		*dev_addr = ALIGN(priv->dma_addr, XRX200_GPHY_FW_ALIGN);
		memcpy(fw_addr_align, fw->data, fw->size);
	} else {
		dev_err(&pdev->dev, "failed to alloc firmware memory\n");
		ret = -ENOMEM;
	}

	release_firmware(fw);

	return ret;
}

static int xway_gphy_of_probe(struct platform_device *pdev,
			      struct xway_gphy_priv *priv)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match = of_match_node(xway_gphy_match,
							 dev->of_node);
	const struct xway_gphy_match_data *gphy_fw_name_cfg;
	u32 gphy_mode;
	int ret;

	gphy_fw_name_cfg = match->data;

	priv->gphy_clk_gate = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->gphy_clk_gate)) {
		dev_err(&pdev->dev, "Failed to lookup gate clock\n");
		return PTR_ERR(priv->gphy_clk_gate);
	}

	priv->regmap = syscon_regmap_lookup_by_phandle(dev->of_node, "regmap");
	if (IS_ERR(priv->regmap)) {
		dev_err(&pdev->dev, "Failed to lookup RCU regmap\n");
		return PTR_ERR(priv->regmap);
	}

	ret = device_property_read_u32(dev, "offset", &priv->reg_offset);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get RCU reg offset\n");
		return -EINVAL;
	}

	priv->gphy_reset = devm_reset_control_get(&pdev->dev, "gphy");
	if (IS_ERR(priv->gphy_reset)) {
		if (PTR_ERR(priv->gphy_reset) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to lookup gphy reset\n");
		return PTR_ERR(priv->gphy_reset);
	}

	priv->gphy_reset2 = devm_reset_control_get_optional(&pdev->dev,
							    "gphy2");
	if (IS_ERR(priv->gphy_reset2))
		return PTR_ERR(priv->gphy_reset2);

	ret = device_property_read_u32(dev, "lantiq,gphy-mode", &gphy_mode);
	/* Default to GE mode */
	if (ret)
		gphy_mode = GPHY_MODE_GE;

	switch (gphy_mode) {
	case GPHY_MODE_FE:
		priv->fw_name = gphy_fw_name_cfg->fe_firmware_name;
		break;
	case GPHY_MODE_GE:
		priv->fw_name = gphy_fw_name_cfg->ge_firmware_name;
		break;
	default:
		dev_err(&pdev->dev, "Unknown GPHY mode %d\n", gphy_mode);
		return -EINVAL;
	}

	return 0;
}

static int xway_gphy_probe(struct platform_device *pdev)
{
	struct xway_gphy_priv *priv;
	dma_addr_t fw_addr = 0;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = xway_gphy_of_probe(pdev, priv);
	if (ret)
		return ret;

	ret = clk_prepare_enable(priv->gphy_clk_gate);
	if (ret)
		return ret;

	ret = xway_gphy_load(pdev, priv, &fw_addr);
	if (ret)
		return ret;

	reset_control_assert(priv->gphy_reset);
	reset_control_assert(priv->gphy_reset2);

	ret = regmap_write(priv->regmap, priv->reg_offset, fw_addr);
	if (ret) {
		dev_err(&pdev->dev, "Failed to configure FW addr\n");
		return ret;
	}

	reset_control_deassert(priv->gphy_reset);
	reset_control_deassert(priv->gphy_reset2);

	/* assert the gphy reset because it can hang after a reboot: */
	priv->gphy_reboot_nb.notifier_call = xway_gphy_reboot_notify;
	priv->gphy_reboot_nb.priority = -1;

	ret = register_reboot_notifier(&priv->gphy_reboot_nb);
	if (ret)
		dev_warn(&pdev->dev, "Failed to register reboot notifier\n");

	platform_set_drvdata(pdev, priv);

	return ret;
}

static int xway_gphy_remove(struct platform_device *pdev)
{
	struct xway_gphy_priv *priv = platform_get_drvdata(pdev);
	int ret;

	reset_control_assert(priv->gphy_reset);
	reset_control_assert(priv->gphy_reset2);

	ret = regmap_write(priv->regmap, priv->reg_offset, 0);
	if (ret)
		dev_err(&pdev->dev, "Failed to configure FW addr\n");

	clk_disable_unprepare(priv->gphy_clk_gate);

	dma_free_coherent(&pdev->dev, priv->size, priv->fw_addr,
			  priv->dma_addr);

	ret = unregister_reboot_notifier(&priv->gphy_reboot_nb);
	if (ret)
		dev_warn(&pdev->dev, "Failed to unregister reboot notifier\n");

	return 0;
}

static struct platform_driver xway_gphy_driver = {
	.probe = xway_gphy_probe,
	.remove = xway_gphy_remove,
	.driver = {
		.name = "xway-rcu-gphy",
		.of_match_table = xway_gphy_match,
	},
};

module_platform_driver(xway_gphy_driver);

MODULE_FIRMWARE("lantiq/xrx300_phy11g_a21.bin");
MODULE_FIRMWARE("lantiq/xrx300_phy22f_a21.bin");
MODULE_FIRMWARE("lantiq/xrx200_phy11g_a14.bin");
MODULE_FIRMWARE("lantiq/xrx200_phy11g_a22.bin");
MODULE_FIRMWARE("lantiq/xrx200_phy22f_a14.bin");
MODULE_FIRMWARE("lantiq/xrx200_phy22f_a22.bin");
MODULE_AUTHOR("Martin Blumenstingl <martin.blumenstingl@googlemail.com>");
MODULE_DESCRIPTION("Lantiq XWAY GPHY Firmware Loader");
MODULE_LICENSE("GPL");
