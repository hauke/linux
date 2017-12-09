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

#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/if_vlan.h>
#include <linux/delay.h>
#include <net/dsa.h>

#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>


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


#define GSWIP_BM_QUEUE_GCTRL		0x0128
#define  GSWIP_BM_QUEUE_GCTRL_GL_MOD	BIT(10)

struct gswip_priv {
	__iomem void *gswip;
	__iomem void *mdio;
	__iomem void *mii;
	struct dsa_switch *ds;
	struct device *dev;	
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

static u32 gswip_mdio_r32(struct gswip_priv *priv, u32 offset)
{
	return __raw_readl(priv->mdio + offset);
}

static void gswip_mdio_w32(struct gswip_priv *priv, u32 val, u32 offset)
{
	return __raw_writel(val, priv->mdio + offset);
}

static void gswip_mdio_w32_mask(struct gswip_priv *priv, u32 clear, u32 set, u32 offset)
{
	u32 val = gswip_mdio_r32(priv, offset);

	val &= ~(clear);
	val |= set;
	gswip_mdio_w32(priv, val, offset);
}

static u32 gswip_mii_r32(struct gswip_priv *priv, u32 offset)
{
	return __raw_readl(priv->mii + offset);
}

static void gswip_mii_w32(struct gswip_priv *priv, u32 val, u32 offset)
{
	return __raw_writel(val, priv->mii + offset);
}

static void gswip_mii_w32_mask(struct gswip_priv *priv, u32 clear, u32 set, u32 offset)
{
	u32 val = gswip_mii_r32(priv, offset);

	val &= ~(clear);
	val |= set;
	gswip_mii_w32(priv, val, offset);
}


static inline int xrx200_mdio_poll(struct gswip_priv *priv)
{
	unsigned cnt = 10000;

	while (likely(cnt--)) {
		unsigned ctrl = gswip_mdio_r32(priv, MDIO_CTRL);
		if ((ctrl & MDIO_BUSY) == 0)
			return 0;
	}

	return 1;
}

static int xrx200_mdio_wr(struct mii_bus *bus, int addr, int reg, u16 val)
{
	struct gswip_priv *priv = bus->priv;

	if (xrx200_mdio_poll(priv))
		return 1;

	gswip_mdio_w32(priv, val, MDIO_WRITE);
	gswip_mdio_w32(priv, MDIO_BUSY | MDIO_WR |
		((addr & MDIO_MASK) << MDIO_ADDRSHIFT) |
		(reg & MDIO_MASK),
		MDIO_CTRL);

	return 0;
}

static int xrx200_mdio_rd(struct mii_bus *bus, int addr, int reg)
{
	struct gswip_priv *priv = bus->priv;

	if (xrx200_mdio_poll(priv))
		return -1;

	gswip_mdio_w32(priv, MDIO_BUSY | MDIO_RD |
		((addr & MDIO_MASK) << MDIO_ADDRSHIFT) |
		(reg & MDIO_MASK),
		MDIO_CTRL);

	if (xrx200_mdio_poll(priv))
		return -1;

	return gswip_mdio_r32(priv, MDIO_READ);
}

static int gswip_mdio(struct gswip_priv *priv, struct device_node *mdio_np)
{
	struct dsa_switch *ds = priv->ds;

	ds->slave_mii_bus = devm_mdiobus_alloc(priv->dev);
	if (!ds->slave_mii_bus)
		return -ENOMEM;

	ds->slave_mii_bus->priv = priv;
	ds->slave_mii_bus->read = xrx200_mdio_rd;
	ds->slave_mii_bus->write = xrx200_mdio_wr;
	ds->slave_mii_bus->name = "lantiq,xrx200-mdio";
	snprintf(ds->slave_mii_bus->id, MII_BUS_ID_SIZE, "%x", 0);
	ds->slave_mii_bus->parent = priv->dev;
	ds->slave_mii_bus->phy_mask = ~ds->phys_mii_mask;

	if (of_mdiobus_register(ds->slave_mii_bus, mdio_np))
		return -ENXIO;

	return 0;
}

static int gswip_setup(struct dsa_switch *ds)
{
	struct gswip_priv *priv = (struct gswip_priv *)ds->priv;
	int i;

	gswip_switch_w32(priv, 1, 0);
	msleep(100);
	gswip_switch_w32(priv, 0, 0);
	/*
	 * TODO: we should really disbale all phys/miis here and explicitly
	 * enable them in the device secific init function
	 */

	/* disable port fetch/store dma */
	for (i = 0; i < 7; i++ ) {
		gswip_switch_w32_mask(priv, 1, 0, FDMA_PCTRLx(i));
		gswip_switch_w32_mask(priv, 1, 0, SDMA_PCTRLx(i));
	}

	/* enable Switch */
	gswip_mdio_w32_mask(priv, 0, MDIO_GLOB_ENABLE, MDIO_GLOB);

	/* Default unknown Broadcat/Multicast/Unicast port maps */
	gswip_switch_w32(priv, 0x40, PCE_PMAP1);
	gswip_switch_w32(priv, 0x40, PCE_PMAP2);
	gswip_switch_w32(priv, 0x40, PCE_PMAP3);

	/* disable auto polling */
	gswip_mdio_w32(priv, 0x0, MDIO_CLK_CFG0);

	/* RMON Counter Enable for all physical ports */
	gswip_switch_w32(priv, 0x1, BM_PCFG(6));

	/* enable port statistic counters */
	gswip_switch_w32(priv, 0x1, BM_PCFGx(6));

	/* enable port fetch/store dma & VLAN Modification */
	gswip_switch_w32_mask(priv, 0, 0x19, FDMA_PCTRLx(6));
	gswip_switch_w32_mask(priv, 0, 0x01, SDMA_PCTRLx(6));
	gswip_switch_w32_mask(priv, 0, PCE_INGRESS, PCE_PCTRL_REG(6, 0));

	/* enable special tag insertion on cpu port */
	gswip_switch_w32_mask(priv, 0, 0x02, FDMA_PCTRLx(6));
	gswip_switch_w32_mask(priv, 0, PCE_INGRESS, PCE_PCTRL_REG(6, 0));
	gswip_switch_w32_mask(priv, 0, BIT(3), MAC_CTRL_REG(6, 2));
	gswip_switch_w32(priv, 1518 + 8 + 4 * 2, MAC_FLEN_REG);
	gswip_switch_w32_mask(priv, 0, GSWIP_BM_QUEUE_GCTRL_GL_MOD, GSWIP_BM_QUEUE_GCTRL);

	return 0;
}

static void gswip_adjust_link(struct dsa_switch *ds, int port, struct phy_device *phydev)
{
	struct gswip_priv *priv = (struct gswip_priv *)ds->priv;
	u16 phyaddr = phydev->mdio.addr & MDIO_PHY_ADDR_MASK;
	u16 miirate = 0;
	u16 miimode;

	/* do not run this for the CPU port 6 */
	if (port > 5)
		return;

	miimode = gswip_mdio_r32(priv, MII_CFG(port)) & MII_CFG_MODE_MASK;

	switch (phydev->speed) {
	case SPEED_1000:
		phyaddr |= MDIO_PHY_SPEED_G1;
		miirate = MII_CFG_RATE_M125;
		break;

	case SPEED_100:
		phyaddr |= MDIO_PHY_SPEED_M100;
		switch (miimode) {
		case MII_CFG_MODE_RMIIM:
		case MII_CFG_MODE_RMIIP:
			miirate = MII_CFG_RATE_M50;
			break;
		default:
			miirate = MII_CFG_RATE_M25;
			break;
		}
		break;

	default:
		phyaddr |= MDIO_PHY_SPEED_M10;
		miirate = MII_CFG_RATE_M2P5;
		break;
	}

	if (phydev->link)
		phyaddr |= MDIO_PHY_LINK_UP;
	else
		phyaddr |= MDIO_PHY_LINK_DOWN;

	if (phydev->duplex == DUPLEX_FULL)
		phyaddr |= MDIO_PHY_FDUP_EN;
	else
		phyaddr |= MDIO_PHY_FDUP_DIS;

	gswip_mdio_w32_mask(priv, MDIO_UPDATE_MASK, phyaddr, MDIO_PHY(port));
	gswip_mii_w32_mask(priv, MII_CFG_RATE_MASK, miirate, MII_CFG(port));
}

static int gswip_port_enable(struct dsa_switch *ds, int port, struct phy_device *phy)
{
	struct gswip_priv *priv = (struct gswip_priv *)ds->priv;

	/* RMON Counter Enable for all physical ports */
	gswip_switch_w32(priv, 0x1, BM_PCFG(port));

	/* enable port statistic counters */
	gswip_switch_w32(priv, 0x1, BM_PCFGx(port));

	/* enable port fetch/store dma & VLAN Modification */
	gswip_switch_w32_mask(priv, 0, 0x19, FDMA_PCTRLx(port));
	gswip_switch_w32_mask(priv, 0, 0x01, SDMA_PCTRLx(port));
	gswip_switch_w32_mask(priv, 0, PCE_INGRESS, PCE_PCTRL_REG(port, 0));

	return 0;
}

static void gswip_port_disable(struct dsa_switch *ds, int port, struct phy_device *phy)
{
	struct gswip_priv *priv = (struct gswip_priv *)ds->priv;
}

static enum dsa_tag_protocol gswip_get_tag_protocol(struct dsa_switch *ds)
{
	return DSA_TAG_PROTO_GSWIP;
}

static const struct dsa_switch_ops gswip_switch_ops = {
	.get_tag_protocol	= gswip_get_tag_protocol,
	.setup			= gswip_setup,
	.adjust_link		= gswip_adjust_link,
	.port_enable		= gswip_port_enable,
	.port_disable		= gswip_port_disable,
/*

	.get_strings		= qca8k_get_strings,
	.phy_read		= qca8k_phy_read,
	.phy_write		= qca8k_phy_write,
	.get_ethtool_stats	= qca8k_get_ethtool_stats,
	.get_sset_count		= qca8k_get_sset_count,
	.get_mac_eee		= qca8k_get_mac_eee,
	.set_mac_eee		= qca8k_set_mac_eee,
	.port_stp_state_set	= qca8k_port_stp_state_set,
	.port_bridge_join	= qca8k_port_bridge_join,
	.port_bridge_leave	= qca8k_port_bridge_leave,
	.port_fdb_add		= qca8k_port_fdb_add,
	.port_fdb_del		= qca8k_port_fdb_del,
	.port_fdb_dump		= qca8k_port_fdb_dump,
*/
};

static int gswip_probe(struct platform_device *pdev)
{
	struct gswip_priv *priv;
	struct resource *gswip_res, *mdio_res, *mii_res;
	struct device_node *mdio_np;
	int err;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	gswip_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->gswip = devm_ioremap_resource(&pdev->dev, gswip_res);
	if (!priv->gswip)
		return -ENOMEM;

	mdio_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	priv->mdio = devm_ioremap_resource(&pdev->dev, mdio_res);
	if (!priv->mdio)
		return -ENOMEM;

	mii_res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	priv->mii = devm_ioremap_resource(&pdev->dev, mii_res);
	if (!priv->mii)
		return -ENOMEM;

	priv->ds = dsa_switch_alloc(&pdev->dev, DSA_MAX_PORTS);
	if (!priv->ds)
		return -ENOMEM;

	priv->ds->priv = priv;
	priv->ds->ops = &gswip_switch_ops;
	priv->dev = &pdev->dev;

	/* bring up the mdio bus */
	mdio_np = of_find_compatible_node(pdev->dev.of_node, NULL,
				"lantiq,xrx200-mdio");
	if (mdio_np)
		if (gswip_mdio(priv, mdio_np))
			dev_err(&pdev->dev, "mdio probe failed\n");

	platform_set_drvdata(pdev, priv);

	err = dsa_register_switch(priv->ds);
	if (err && mdio_np)
		mdiobus_unregister(priv->ds->slave_mii_bus);

	return err;
}

static int gswip_remove(struct platform_device *pdev)
{
	struct gswip_priv *priv = platform_get_drvdata(pdev);

	if (!priv)
		return 0;

	/* disable the switch */
	gswip_mdio_w32_mask(priv, MDIO_GLOB_ENABLE, 0, MDIO_GLOB);

	dsa_unregister_switch(priv->ds);

	mdiobus_unregister(priv->ds->slave_mii_bus);

	return 0;
}

static const struct of_device_id gswip_of_match[] = {
	{ .compatible = "lantiq,xrx200-gswip" },
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
