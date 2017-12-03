/*
 *   This program is free software; you can redistribute it and/or modify it
 *   under the terms of the GNU General Public License version 2 as published
 *   by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA.
 *
 *   Copyright (C) 2010 Lantiq Deutschland
 *   Copyright (C) 2012 John Crispin <blogic@openwrt.org>
 */

#include <linux/switch.h>
#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/if_vlan.h>
#include <asm/delay.h>

#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

#include <xway_dma.h>
#include <lantiq_soc.h>

#include "lantiq_pce.h"
#include "lantiq_xrx200_sw.h"

#define SW_POLLING
#define SW_ROUTING

#ifdef SW_ROUTING
#define XRX200_MAX_DEV		2
#else
#define XRX200_MAX_DEV		1
#endif

#define XRX200_MAX_VLAN		64
#define XRX200_PCE_ACTVLAN_IDX	0x01
#define XRX200_PCE_VLANMAP_IDX	0x02

#define XRX200_MAX_PORT		7
#define XRX200_MAX_DMA		8

#define XRX200_HEADROOM		4

#define XRX200_TX_TIMEOUT	(10 * HZ)

/* port type */
#define XRX200_PORT_TYPE_PHY	1
#define XRX200_PORT_TYPE_MAC	2

/* DMA */
#define XRX200_DMA_DATA_LEN	0x600
#define XRX200_DMA_IRQ		INT_NUM_IM2_IRL0
#define XRX200_DMA_RX		0
#define XRX200_DMA_TX		1
#define XRX200_DMA_TX_2		3
#define XRX200_DMA_IS_TX(x)	(x%2)
#define XRX200_DMA_IS_RX(x)	(!XRX200_DMA_IS_TX(x))

/* fetch / store dma */
#define FDMA_PCTRL0		0x2A00
#define FDMA_PCTRLx(x)		(FDMA_PCTRL0 + (x * 0x18))
#define SDMA_PCTRL0		0x2F00
#define SDMA_PCTRLx(x)		(SDMA_PCTRL0 + (x * 0x18))

/* buffer management */
#define BM_PCFG0		0x200
#define BM_PCFGx(x)		(BM_PCFG0 + (x * 8))

/* MDIO */
#define MDIO_GLOB		0x0000
#define MDIO_CTRL		0x0020
#define MDIO_READ		0x0024
#define MDIO_WRITE		0x0028
#define MDIO_PHY0		0x0054
#define MDIO_PHY(x)		(0x0054 - (x * sizeof(unsigned)))
#define MDIO_CLK_CFG0		0x002C
#define MDIO_CLK_CFG1		0x0030

#define MDIO_GLOB_ENABLE	0x8000
#define MDIO_BUSY		BIT(12)
#define MDIO_RD			BIT(11)
#define MDIO_WR			BIT(10)
#define MDIO_MASK		0x1f
#define MDIO_ADDRSHIFT		5
#define MDIO1_25MHZ		9

#define MDIO_PHY_LINK_DOWN	0x4000
#define MDIO_PHY_LINK_UP	0x2000

#define MDIO_PHY_SPEED_M10	0x0000
#define MDIO_PHY_SPEED_M100	0x0800
#define MDIO_PHY_SPEED_G1	0x1000

#define MDIO_PHY_FDUP_EN	0x0200
#define MDIO_PHY_FDUP_DIS	0x0600

#define MDIO_PHY_LINK_MASK	0x6000
#define MDIO_PHY_SPEED_MASK	0x1800
#define MDIO_PHY_FDUP_MASK	0x0600
#define MDIO_PHY_ADDR_MASK	0x001f
#define MDIO_UPDATE_MASK	MDIO_PHY_ADDR_MASK | MDIO_PHY_LINK_MASK | \
					MDIO_PHY_SPEED_MASK | MDIO_PHY_FDUP_MASK

/* MII */
#define MII_CFG(p)		(p * 8)

#define MII_CFG_EN		BIT(14)

#define MII_CFG_MODE_MIIP	0x0
#define MII_CFG_MODE_MIIM	0x1
#define MII_CFG_MODE_RMIIP	0x2
#define MII_CFG_MODE_RMIIM	0x3
#define MII_CFG_MODE_RGMII	0x4
#define MII_CFG_MODE_MASK	0xf

#define MII_CFG_RATE_M2P5	0x00
#define MII_CFG_RATE_M25	0x10
#define MII_CFG_RATE_M125	0x20
#define MII_CFG_RATE_M50	0x30
#define MII_CFG_RATE_AUTO	0x40
#define MII_CFG_RATE_MASK	0x70

/* cpu port mac */
#define PMAC_HD_CTL		0x0000
#define PMAC_RX_IPG		0x0024
#define PMAC_EWAN		0x002c

#define PMAC_IPG_MASK		0xf
#define PMAC_HD_CTL_AS		0x0008
#define PMAC_HD_CTL_AC		0x0004
#define PMAC_HD_CTL_RC		0x0010
#define PMAC_HD_CTL_RXSH	0x0040
#define PMAC_HD_CTL_AST		0x0080
#define PMAC_HD_CTL_RST		0x0100

/* PCE */
#define PCE_TBL_KEY(x)		(0x1100 + ((7 - x) * 4))
#define PCE_TBL_MASK		0x1120
#define PCE_TBL_VAL(x)		(0x1124 + ((4 - x) * 4))
#define PCE_TBL_ADDR		0x1138
#define PCE_TBL_CTRL		0x113c
#define PCE_PMAP1		0x114c
#define PCE_PMAP2		0x1150
#define PCE_PMAP3		0x1154
#define PCE_GCTRL_REG(x)	(0x1158 + (x * 4))
#define PCE_PCTRL_REG(p, x)	(0x1200 + (((p * 0xa) + x) * 4))

#define PCE_TBL_BUSY		BIT(15)
#define PCE_TBL_CFG_ADDR_MASK	0x1f
#define PCE_TBL_CFG_ADWR	0x20
#define PCE_TBL_CFG_ADWR_MASK	0x60
#define PCE_INGRESS		BIT(11)

/* MAC */
#define MAC_FLEN_REG		(0x2314)
#define MAC_CTRL_REG(p, x)	(0x240c + (((p * 0xc) + x) * 4))

/* buffer management */
#define BM_PCFG(p)		(0x200 + (p * 8))

/* special tag in TX path header */
#define SPID_SHIFT		24
#define DPID_SHIFT		16
#define DPID_ENABLE		1
#define SPID_CPU_PORT		2
#define PORT_MAP_SEL		BIT(15)
#define PORT_MAP_EN		BIT(14)
#define PORT_MAP_SHIFT		1
#define PORT_MAP_MASK		0x3f

#define SPPID_MASK		0x7
#define SPPID_SHIFT		4

/* MII regs not yet in linux */
#define MDIO_DEVAD_NONE		(-1)
#define ADVERTIZE_MPD		(1 << 10)

struct gswip_priv {
	__iomem void *gswip;
	struct dsa_switch *ds;
	
};

static u32 gswip_switch_r32(struct gswip_priv *priv, u32 offset)
{
	return __raw_readl(priv->gswip + offset);
}

static void gswip_switch_w32(struct gswip_priv *priv, u32 val, u32 offset)
{
	return __raw_writel(val, priv->gswip + offset);
}

static void gswip_switch_w32_mask(struct gswip_priv *priv, u32 clear, u32 set, u32 offset)
{
	u32 val = gswip_switch_r32(priv, offset);

	val &= ~(clear);
	val |= set;
	gswip_switch_w32(priv, val, offset);
}

static void xrx200_hw_init(struct gswip_priv *priv)
{
	int i;

	/* enable clock gate */
	clk_enable(hw->clk);

	gswip_switch_w32(priv, 1, 0);
	mdelay(100);
	gswip_switch_w32(priv, 0, 0);
	/*
	 * TODO: we should really disbale all phys/miis here and explicitly
	 * enable them in the device secific init function
	 */

	/* disable port fetch/store dma */
	for (i = 0; i < 7; i++ ) {
		gswip_switch_w32(priv, 0, FDMA_PCTRLx(i));
		gswip_switch_w32(priv, 0, SDMA_PCTRLx(i));
	}

	/* Default unknown Broadcat/Multicast/Unicast port maps */
	gswip_switch_w32(priv, 0x40, PCE_PMAP1);
	gswip_switch_w32(priv, 0x40, PCE_PMAP2);
	gswip_switch_w32(priv, 0x40, PCE_PMAP3);

	/* RMON Counter Enable for all physical ports */
	for (i = 0; i < 7; i++)
		gswip_switch_w32(priv, 0x1, BM_PCFG(i));

	/* enable port statistic counters */
	for (i = 0; i < 7; i++)
		gswip_switch_w32(priv, 0x1, BM_PCFGx(i));

	/* enable port fetch/store dma & VLAN Modification */
	for (i = 0; i < 7; i++ ) {
		gswip_switch_w32_mask(priv, 0, 0x19, FDMA_PCTRLx(i));
		gswip_switch_w32_mask(priv, 0, 0x01, SDMA_PCTRLx(i));
		gswip_switch_w32_mask(priv, 0, PCE_INGRESS, PCE_PCTRL_REG(i, 0));
	}

	/* enable special tag insertion on cpu port */
	gswip_switch_w32_mask(priv, 0, 0x02, FDMA_PCTRLx(6));
	gswip_switch_w32_mask(priv, 0, PCE_INGRESS, PCE_PCTRL_REG(6, 0));
	gswip_switch_w32_mask(priv, 0, BIT(3), MAC_CTRL_REG(6, 2));
	gswip_switch_w32(priv, 1518 + 8 + 4 * 2, MAC_FLEN_REG);
	xrx200sw_write_x(1, XRX200_BM_QUEUE_GCTRL_GL_MOD, 0);

	for (i = 0; i < XRX200_MAX_VLAN; i++)
		hw->vlan_vid[i] = i;
}


static enum dsa_tag_protocol gswip_get_tag_protocol(struct dsa_switch *ds)
{
	return DSA_TAG_PROTO_GSWIP;
}

static const struct dsa_switch_ops gswip_switch_ops = {
	.get_tag_protocol	= gswip_get_tag_protocol,
/*
	.setup			= qca8k_setup,
	.get_strings		= qca8k_get_strings,
	.phy_read		= qca8k_phy_read,
	.phy_write		= qca8k_phy_write,
	.get_ethtool_stats	= qca8k_get_ethtool_stats,
	.get_sset_count		= qca8k_get_sset_count,
	.get_mac_eee		= qca8k_get_mac_eee,
	.set_mac_eee		= qca8k_set_mac_eee,
	.port_enable		= qca8k_port_enable,
	.port_disable		= qca8k_port_disable,
	.port_stp_state_set	= qca8k_port_stp_state_set,
	.port_bridge_join	= qca8k_port_bridge_join,
	.port_bridge_leave	= qca8k_port_bridge_leave,
	.port_fdb_add		= qca8k_port_fdb_add,
	.port_fdb_del		= qca8k_port_fdb_del,
	.port_fdb_dump		= qca8k_port_fdb_dump,
*/
};

static int xrx200_probe(struct platform_device *pdev)
{
	struct gswip_priv *priv;
	struct resource *gswip_res;
	u32 id;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	gswip_res = platform_get_resource(pdev, IORESOURCE_MEM, i);
	priv->gswip = devm_ioremap_resource(&pdev->dev, gswip_res);
	if (!priv->gswip)
		return -ENOMEM;

	priv->ds = dsa_switch_alloc(&pdev->dev, DSA_MAX_PORTS);
	if (!priv->ds)
		return -ENOMEM;

	priv->ds->priv = priv;
	priv->ds->ops = &gswip_switch_ops;

	/* bring up the dma engine and IP core */
	xrx200_hw_init(&priv);

	platform_set_drvdata(pdev, priv);

	return dsa_register_switch(priv->ds);
}

static int xrx200_remove(struct platform_device *pdev)
{
	struct gswip_priv *priv = platform_get_drvdata(pdev);

	if (!priv)
		return 0;

	dsa_unregister_switch(priv->ds);

	return 0;
}

static const struct of_device_id gswip_match[] = {
	{ .compatible = "intel,gswip20" },
	{},
};
MODULE_DEVICE_TABLE(of, xrx200_match);

static struct platform_driver gswip_driver = {
	.probe = gswip_probe,
	.remove = gswip_remove,
	.driver = {
		.name = "gswip",
		.of_match_table = gswip_of_match,
	},
};

module_platform_driver(gswip_driver);

MODULE_AUTHOR("Hauke Mehrtens <hauke@hauke-m.de>");
MODULE_DESCRIPTION("Intel / Lantiq GSWIP driver");
MODULE_LICENSE("GPL v2");
