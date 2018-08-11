// SPDX-License-Identifier: GPL-2.0
/*
 * Lantiq / Intel GSWIP switch driver for VRX200 SoCs
 *
 * Copyright (C) 2010 Lantiq Deutschland
 * Copyright (C) 2012 John Crispin <john@phrozen.org>
 * Copyright (C) 2017 - 2018 Hauke Mehrtens <hauke@hauke-m.de>
 */

#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/if_vlan.h>
#include <linux/if_bridge.h>
#include <net/dsa.h>

#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/of_platform.h>
#include <linux/version.h>

#include "lantiq_pce.h"


/* GSWIP MDIO Registers */
#define GSWIP_MDIO_GLOB			0x00
#define  GSWIP_MDIO_GLOB_ENABLE		BIT(15)
#define GSWIP_MDIO_CTRL			0x08
#define  GSWIP_MDIO_CTRL_BUSY		BIT(12)
#define  GSWIP_MDIO_CTRL_RD		BIT(11)
#define  GSWIP_MDIO_CTRL_WR		BIT(10)
#define  GSWIP_MDIO_CTRL_PHYAD_MASK	0x1f
#define  GSWIP_MDIO_CTRL_PHYAD_SHIFT	5
#define  GSWIP_MDIO_CTRL_REGAD_MASK	0x1f
#define GSWIP_MDIO_READ			0x09
#define GSWIP_MDIO_WRITE		0x0A
#define GSWIP_MDIO_MDC_CFG0		0x0B
#define GSWIP_MDIO_MDC_CFG1		0x0C
#define GSWIP_MDIO_PHYp(p)		(0x15 - (p))
#define  GSWIP_MDIO_PHY_LINK_DOWN	0x4000
#define  GSWIP_MDIO_PHY_LINK_UP		0x2000
#define  GSWIP_MDIO_PHY_SPEED_M10	0x0000
#define  GSWIP_MDIO_PHY_SPEED_M100	0x0800
#define  GSWIP_MDIO_PHY_SPEED_G1	0x1000
#define  GSWIP_MDIO_PHY_FDUP_EN		0x0200
#define  GSWIP_MDIO_PHY_FDUP_DIS	0x0600
#define  GSWIP_MDIO_PHY_FCONTX_EN	0x0100
#define  GSWIP_MDIO_PHY_FCONTX_DIS	0x0180
#define  GSWIP_MDIO_PHY_FCONRX_EN	0x0020
#define  GSWIP_MDIO_PHY_FCONRX_DIS	0x0060
#define  GSWIP_MDIO_PHY_LINK_MASK	0x6000
#define  GSWIP_MDIO_PHY_SPEED_MASK	0x1800
#define  GSWIP_MDIO_PHY_FDUP_MASK	0x0600
#define  GSWIP_MDIO_PHY_FCONTX_MASK	0x0180
#define  GSWIP_MDIO_PHY_FCONRX_MASK	0x0060
#define  GSWIP_MDIO_PHY_ADDR_MASK	0x001f
#define  GSWIP_MDIO_PHY_MASK		(GSWIP_MDIO_PHY_ADDR_MASK | \
					 GSWIP_MDIO_PHY_FCONRX_MASK | \
					 GSWIP_MDIO_PHY_FCONTX_MASK | \
					 GSWIP_MDIO_PHY_LINK_MASK | \
					 GSWIP_MDIO_PHY_SPEED_MASK | \
					 GSWIP_MDIO_PHY_FDUP_MASK)

/* GSWIP MII Registers */
#define GSWIP_MII_CFGp(p)		((p) * 2)
#define  GSWIP_MII_CFG_EN		BIT(14)
#define  GSWIP_MII_CFG_MODE_MIIP	0x0
#define  GSWIP_MII_CFG_MODE_MIIM	0x1
#define  GSWIP_MII_CFG_MODE_RMIIP	0x2
#define  GSWIP_MII_CFG_MODE_RMIIM	0x3
#define  GSWIP_MII_CFG_MODE_RGMII	0x4
#define  GSWIP_MII_CFG_MODE_MASK	0xf
#define  GSWIP_MII_CFG_RATE_M2P5	0x00
#define  GSWIP_MII_CFG_RATE_M25	0x10
#define  GSWIP_MII_CFG_RATE_M125	0x20
#define  GSWIP_MII_CFG_RATE_M50	0x30
#define  GSWIP_MII_CFG_RATE_AUTO	0x40
#define  GSWIP_MII_CFG_RATE_MASK	0x70

/* GSWIP Core Registers */
#define GSWIP_ETHSW_SWRES		0x000
#define  GSWIP_ETHSW_SWRES_R1		BIT(1)	/* GSWIP Software reset */
#define  GSWIP_ETHSW_SWRES_R0		BIT(0)	/* GSWIP Hardware reset */

#define GSWIP_BM_RAM_VAL(x)		(0x043 - (x))
#define GSWIP_BM_RAM_ADDR		0x044
#define GSWIP_BM_RAM_CTRL		0x045
#define  GSWIP_BM_RAM_CTRL_BAS		BIT(15)
#define  GSWIP_BM_RAM_CTRL_OPMOD	BIT(5)
#define  GSWIP_BM_RAM_CTRL_ADDR_MASK	GENMASK(4, 0)
#define GSWIP_BM_QUEUE_GCTRL		0x04A
#define  GSWIP_BM_QUEUE_GCTRL_GL_MOD	BIT(10)
/* buffer management Port Configuration Register */
#define GSWIP_BM_PCFGp(p)		(0x080 + ((p) * 2))
#define  GSWIP_BM_PCFG_CNTEN		BIT(0)	/* RMON Counter Enable */
#define  GSWIP_BM_PCFG_IGCNT		BIT(1)	/* Ingres Special Tag RMON count */
/* buffer management Port Control Register */
#define GSWIP_BM_RMON_CTRLp(p)		(0x81 + ((p) * 2))
#define  GSWIP_BM_CTRL_RMON_RAM1_RES	BIT(0)	/* Software Reset for RMON RAM 1 */
#define  GSWIP_BM_CTRL_RMON_RAM2_RES	BIT(1)	/* Software Reset for RMON RAM 2 */

/* PCE */
#define GSWIP_PCE_TBL_KEY(x)		(0x447 - (x))
#define GSWIP_PCE_TBL_MASK		0x448
#define GSWIP_PCE_TBL_VAL(x)		(0x44D - (x))
#define GSWIP_PCE_TBL_ADDR		0x44E
#define GSWIP_PCE_TBL_CTRL		0x44F
#define  GSWIP_PCE_TBL_CTRL_BAS		BIT(15)
#define  GSWIP_PCE_TBL_CTRL_TYPE	BIT(13)
#define  GSWIP_PCE_TBL_CTRL_VLD		BIT(12)
#define  GSWIP_PCE_TBL_CTRL_KEYFORM	BIT(11)
#define  GSWIP_PCE_TBL_CTRL_GMAP_MASK	GENMASK(10, 7)
#define  GSWIP_PCE_TBL_CTRL_OPMOD_MASK	GENMASK(6, 5)
#define  GSWIP_PCE_TBL_CTRL_OPMOD_ADRD	0x00
#define  GSWIP_PCE_TBL_CTRL_OPMOD_ADWR	0x20
#define  GSWIP_PCE_TBL_CTRL_OPMOD_KSRD	0x40
#define  GSWIP_PCE_TBL_CTRL_OPMOD_KSWR	0x60
#define  GSWIP_PCE_TBL_CTRL_ADDR_MASK	GENMASK(4, 0)
#define GSWIP_PCE_PMAP1			0x453	/* Monitoring port map */
#define GSWIP_PCE_PMAP2			0x454	/* Default Multicast port map */
#define GSWIP_PCE_PMAP3			0x455	/* Default Unknown Unicast port map */
#define GSWIP_PCE_GCTRL_0		0x456
#define  GSWIP_PCE_GCTRL_0_MC_VALID	BIT(3)
#define  GSWIP_PCE_GCTRL_0_VLAN		BIT(14) /* VLAN aware Switching */
#define GSWIP_PCE_GCTRL_1		0x457
#define  GSWIP_PCE_GCTRL_1_MAC_GLOCK	BIT(2)	/* MAC Address table lock */
#define  GSWIP_PCE_GCTRL_1_MAC_GLOCK_MOD	BIT(3) /* Mac address table lock forwarding mode */
#define GSWIP_PCE_PCTRL_0p(p)		(0x480 + ((p) * 0xA))
#define  GSWIP_PCE_PCTRL_0_INGRESS	BIT(11)
#define  GSWIP_PCE_PCTRL_0_PSTATE_LISTEN	0x0
#define  GSWIP_PCE_PCTRL_0_PSTATE_RX		0x1
#define  GSWIP_PCE_PCTRL_0_PSTATE_TX		0x2
#define  GSWIP_PCE_PCTRL_0_PSTATE_LEARNING	0x3
#define  GSWIP_PCE_PCTRL_0_PSTATE_FORWARDING	0x7
#define  GSWIP_PCE_PCTRL_0_PSTATE_MASK	GENMASK(2, 0)

#define GSWIP_MAC_FLEN			0x8C5
#define GSWIP_MAC_CTRL_2p(p)		(0x905 + ((p) * 0xC))
#define GSWIP_MAC_CTRL_2_MLEN		BIT(3) /* Maximum Untagged Frame Lnegth */

/* Ethernet Switch Fetch DMA Port Control Register */
#define GSWIP_FDMA_PCTRLp(p)		(0xA80 + ((p) * 0x6))
#define  GSWIP_FDMA_PCTRL_EN		BIT(0)	/* FDMA Port Enable */
#define  GSWIP_FDMA_PCTRL_STEN		BIT(1)	/* Special Tag Insertion Enable */
#define  GSWIP_FDMA_PCTRL_VLANMOD_MASK	GENMASK(4, 3)	/* VLAN Modification Control */
#define  GSWIP_FDMA_PCTRL_VLANMOD_SHIFT	3	/* VLAN Modification Control */
#define  GSWIP_FDMA_PCTRL_VLANMOD_DIS	(0x0 << GSWIP_FDMA_PCTRL_VLANMOD_SHIFT)
#define  GSWIP_FDMA_PCTRL_VLANMOD_PRIO	(0x1 << GSWIP_FDMA_PCTRL_VLANMOD_SHIFT)
#define  GSWIP_FDMA_PCTRL_VLANMOD_ID	(0x2 << GSWIP_FDMA_PCTRL_VLANMOD_SHIFT)
#define  GSWIP_FDMA_PCTRL_VLANMOD_BOTH	(0x3 << GSWIP_FDMA_PCTRL_VLANMOD_SHIFT)

/* Ethernet Switch Store DMA Port Control Register */
#define GSWIP_SDMA_PCTRLp(p)		(0xBC0 + ((p) * 0x6))
#define  GSWIP_SDMA_PCTRL_EN		BIT(0)	/* SDMA Port Enable */
#define  GSWIP_SDMA_PCTRL_FCEN		BIT(1)	/* Flow Control Enable */
#define  GSWIP_SDMA_PCTRL_PAUFWD	BIT(1)	/* Pause Frame Forwarding */

struct gswip_hw_info {
	int max_ports;
	int cpu_port;
};

struct gswip_priv {
	__iomem void *gswip;
	__iomem void *mdio;
	__iomem void *mii;
	struct gswip_hw_info *hw_info;
	struct dsa_switch *ds;
	struct device *dev;
};

struct gswip_rmon_cnt_desc {
	unsigned int size;
	unsigned int offset;
	const char *name;
};

#define MIB_DESC(_size, _offset, _name) {.size = _size, .offset = _offset, .name = _name}

static const struct gswip_rmon_cnt_desc gswip_rmon_cnt[] = {
	MIB_DESC(1, 0x1F, "RxGoodPkts"),
	MIB_DESC(1, 0x23, "RxUnicastPkts"),
	MIB_DESC(1, 0x22, "RxMulticastPkts"),
	MIB_DESC(1, 0x21, "RxFCSErrorPkts"),
	MIB_DESC(1, 0x1D, "RxUnderSizeGoodPkts"),
	MIB_DESC(1, 0x1E, "RxUnderSizeErrorPkts"),
	MIB_DESC(1, 0x1B, "RxOversizeGoodPkts"),
	MIB_DESC(1, 0x1C, "RxOversizeErrorPkts"),
	MIB_DESC(1, 0x20, "RxGoodPausePkts"),
	MIB_DESC(1, 0x1A, "RxAlignErrorPkts"),
	MIB_DESC(1, 0x12, "Rx64BytePkts"),
	MIB_DESC(1, 0x13, "Rx127BytePkts"),
	MIB_DESC(1, 0x14, "Rx255BytePkts"),
	MIB_DESC(1, 0x15, "Rx511BytePkts"),
	MIB_DESC(1, 0x16, "Rx1023BytePkts"),
	MIB_DESC(1, 0x17, "RxMaxBytePkts"),
	MIB_DESC(1, 0x18, "RxDroppedPkts"),
	MIB_DESC(1, 0x19, "RxFilteredPkts"),
	MIB_DESC(2, 0x24, "RxGoodBytes"),
	MIB_DESC(2, 0x26, "RxBadBytes"),
	MIB_DESC(1, 0x11, "TxAcmDroppedPkts"),
	MIB_DESC(1, 0x0C, "TxGoodPkts"),
	MIB_DESC(1, 0x06, "TxUnicastPkts"),
	MIB_DESC(1, 0x07, "TxMulticastPkts"),
	MIB_DESC(1, 0x00, "Tx64BytePkts"),
	MIB_DESC(1, 0x01, "Tx127BytePkts"),
	MIB_DESC(1, 0x02, "Tx255BytePkts"),
	MIB_DESC(1, 0x03, "Tx511BytePkts"),
	MIB_DESC(1, 0x04, "Tx1023BytePkts"),
	MIB_DESC(1, 0x05, "TxMaxBytePkts"),
	MIB_DESC(1, 0x08, "TxSingleCollCount"),
	MIB_DESC(1, 0x09, "TxMultCollCount"),
	MIB_DESC(1, 0x0A, "TxLateCollCount"),
	MIB_DESC(1, 0x0B, "TxExcessCollCount"),
	MIB_DESC(1, 0x0D, "TxPauseCount"),
	MIB_DESC(1, 0x10, "TxDroppedPkts"),
	MIB_DESC(2, 0x0E, "TxGoodBytes"),
};

static u32 gswip_switch_r(struct gswip_priv *priv, u32 offset)
{
	return __raw_readl(priv->gswip + (offset * 4));
}

static void gswip_switch_w(struct gswip_priv *priv, u32 val, u32 offset)
{
	return __raw_writel(val, priv->gswip + (offset * 4));
}

static void gswip_switch_mask(struct gswip_priv *priv, u32 clear, u32 set,
			      u32 offset)
{
	u32 val = gswip_switch_r(priv, offset);

	val &= ~(clear);
	val |= set;
	gswip_switch_w(priv, val, offset);
}

static u32 gswip_switch_r_timeout(struct gswip_priv *priv, u32 offset, cleared)
{
	u32 val;

	return readx_poll_timeout(__raw_readl, priv->gswip + (offset * 4), val,
				  (val & cleared) == 0, 20, 50000)
}

static u32 gswip_mdio_r(struct gswip_priv *priv, u32 offset)
{
	return __raw_readl(priv->mdio + (offset * 4));
}

static void gswip_mdio_w(struct gswip_priv *priv, u32 val, u32 offset)
{
	return __raw_writel(val, priv->mdio + (offset * 4));
}

static void gswip_mdio_mask(struct gswip_priv *priv, u32 clear, u32 set,
			    u32 offset)
{
	u32 val = gswip_mdio_r(priv, offset);

	val &= ~(clear);
	val |= set;
	gswip_mdio_w(priv, val, offset);
}

static u32 gswip_mii_r(struct gswip_priv *priv, u32 offset)
{
	return __raw_readl(priv->mii + (offset * 4));
}

static void gswip_mii_w(struct gswip_priv *priv, u32 val, u32 offset)
{
	return __raw_writel(val, priv->mii + (offset * 4));
}

static void gswip_mii_mask(struct gswip_priv *priv, u32 clear, u32 set,
			   u32 offset)
{
	u32 val = gswip_mii_r(priv, offset);

	val &= ~(clear);
	val |= set;
	gswip_mii_w(priv, val, offset);
}

static int gswip_mdio_poll(struct gswip_priv *priv)
{
	int cnt = 10000;

	while (likely(cnt--)) {
		u32 ctrl = gswip_mdio_r(priv, GSWIP_MDIO_CTRL);

		if ((ctrl & GSWIP_MDIO_CTRL_BUSY) == 0)
			return 0;
		cpu_relax();
	}

	return -ETIMEDOUT;
}

static int gswip_mdio_wr(struct mii_bus *bus, int addr, int reg, u16 val)
{
	struct gswip_priv *priv = bus->priv;
	int err;

	err = gswip_mdio_poll(priv);
	if (err)
		return err;

	gswip_mdio_w(priv, val, GSWIP_MDIO_WRITE);
	gswip_mdio_w(priv, GSWIP_MDIO_CTRL_BUSY | GSWIP_MDIO_CTRL_WR |
		((addr & GSWIP_MDIO_CTRL_PHYAD_MASK) << GSWIP_MDIO_CTRL_PHYAD_SHIFT) |
		(reg & GSWIP_MDIO_CTRL_REGAD_MASK),
		GSWIP_MDIO_CTRL);

	return 0;
}

static int gswip_mdio_rd(struct mii_bus *bus, int addr, int reg)
{
	struct gswip_priv *priv = bus->priv;
	int err;

	err = gswip_mdio_poll(priv);
	if (err)
		return err;

	gswip_mdio_w(priv, GSWIP_MDIO_CTRL_BUSY | GSWIP_MDIO_CTRL_RD |
		((addr & GSWIP_MDIO_CTRL_PHYAD_MASK) << GSWIP_MDIO_CTRL_PHYAD_SHIFT) |
		(reg & GSWIP_MDIO_CTRL_REGAD_MASK),
		GSWIP_MDIO_CTRL);

	err = gswip_mdio_poll(priv);
	if (err)
		return err;

	return gswip_mdio_r(priv, GSWIP_MDIO_READ);
}

static int gswip_mdio(struct gswip_priv *priv, struct device_node *mdio_np)
{
	struct dsa_switch *ds = priv->ds;

	ds->slave_mii_bus = devm_mdiobus_alloc(priv->dev);
	if (!ds->slave_mii_bus)
		return -ENOMEM;

	ds->slave_mii_bus->priv = priv;
	ds->slave_mii_bus->read = gswip_mdio_rd;
	ds->slave_mii_bus->write = gswip_mdio_wr;
	ds->slave_mii_bus->name = "lantiq,xrx200-mdio";
	snprintf(ds->slave_mii_bus->id, MII_BUS_ID_SIZE, "%s-mii",
		 dev_name(priv->dev));
	ds->slave_mii_bus->parent = priv->dev;
	ds->slave_mii_bus->phy_mask = ~ds->phys_mii_mask;

	return of_mdiobus_register(ds->slave_mii_bus, mdio_np);
}

static int gswip_wait_pce_tbl_ready(struct gswip_priv *priv)
{
	int cnt = 10000;

static u32 gswip_switch_r_timeout(priv, GSWIP_PCE_TBL_CTRL, GSWIP_PCE_TBL_CTRL_BAS);

	return -ETIMEDOUT;
}

static int gswip_port_enable(struct dsa_switch *ds, int port,
			     struct phy_device *phy)
{
	struct gswip_priv *priv = ds->priv;

	/* RMON Counter Enable for port */
	gswip_switch_w(priv, GSWIP_BM_PCFG_CNTEN, GSWIP_BM_PCFGp(port));

	/* enable port fetch/store dma & VLAN Modification */
	gswip_switch_mask(priv, 0, GSWIP_FDMA_PCTRL_EN |
				   GSWIP_FDMA_PCTRL_VLANMOD_BOTH,
			 GSWIP_FDMA_PCTRLp(port));
	gswip_switch_mask(priv, 0, GSWIP_SDMA_PCTRL_EN,
			  GSWIP_SDMA_PCTRLp(port));
	gswip_switch_mask(priv, 0, GSWIP_PCE_PCTRL_0_INGRESS,
			  GSWIP_PCE_PCTRL_0p(port));

	return 0;
}

static void gswip_port_disable(struct dsa_switch *ds, int port,
			       struct phy_device *phy)
{
	struct gswip_priv *priv = ds->priv;

	gswip_switch_mask(priv, GSWIP_FDMA_PCTRL_EN, 0,
			  GSWIP_FDMA_PCTRLp(port));
	gswip_switch_mask(priv, GSWIP_SDMA_PCTRL_EN, 0,
			  GSWIP_SDMA_PCTRLp(port));
}

static int gswip_pce_microcode(struct gswip_priv *priv)
{
	int i;
	int err;

	gswip_switch_mask(priv, GSWIP_PCE_TBL_CTRL_ADDR_MASK |
				GSWIP_PCE_TBL_CTRL_OPMOD_MASK,
			  GSWIP_PCE_TBL_CTRL_OPMOD_ADWR, GSWIP_PCE_TBL_CTRL);
	gswip_switch_w(priv, 0, GSWIP_PCE_TBL_MASK);

	for (i = 0; i < ARRAY_SIZE(gswip_pce_microcode); i++) {
		gswip_switch_w(priv, i, GSWIP_PCE_TBL_ADDR);
		gswip_switch_w(priv, gswip_pce_microcode[i].val_0,
			       GSWIP_PCE_TBL_VAL(0));
		gswip_switch_w(priv, gswip_pce_microcode[i].val_1,
			       GSWIP_PCE_TBL_VAL(1));
		gswip_switch_w(priv, gswip_pce_microcode[i].val_2,
			       GSWIP_PCE_TBL_VAL(2));
		gswip_switch_w(priv, gswip_pce_microcode[i].val_3,
			       GSWIP_PCE_TBL_VAL(3));

		/* start the table access: */
		gswip_switch_mask(priv, 0, GSWIP_PCE_TBL_CTRL_BAS,
				  GSWIP_PCE_TBL_CTRL);
		err = gswip_switch_r_timeout(priv, GSWIP_PCE_TBL_CTRL,
					     GSWIP_PCE_TBL_CTRL_BAS);
		if (err)
			return err;
	}

	/* tell the switch that the microcode is loaded */
	gswip_switch_mask(priv, 0, GSWIP_PCE_GCTRL_0_MC_VALID,
			  GSWIP_PCE_GCTRL_0);
}

static int gswip_setup(struct dsa_switch *ds)
{
	struct gswip_priv *priv = ds->priv;
	int i;
	unsigned int cpu_port = priv->hw_info->cpu_port;

	gswip_switch_w(priv, GSWIP_ETHSW_SWRES_R0, GSWIP_ETHSW_SWRES);
	usleep_range(5000, 10000);
	gswip_switch_w(priv, 0, GSWIP_ETHSW_SWRES);

	/* disable port fetch/store dma on all ports */
	for (i = 0; i < priv->hw_info->max_ports; i++) {
		gswip_port_disable(ds, i, NULL);
	}

	/* enable Switch */
	gswip_mdio_mask(priv, 0, GSWIP_MDIO_GLOB_ENABLE, GSWIP_MDIO_GLOB);

	err = gswip_pce_microcode(priv);
	if (err) {
		dev_err(priv->dev, "writing PCE microcode failed, %i", err);
		return err;
	}

	/* Default unknown Broadcast/Multicast/Unicast port maps */
	gswip_switch_w(priv, BIT(cpu_port), GSWIP_PCE_PMAP1);
	gswip_switch_w(priv, BIT(cpu_port), GSWIP_PCE_PMAP2);
	gswip_switch_w(priv, BIT(cpu_port), GSWIP_PCE_PMAP3);

	/* disable auto polling */
	gswip_mdio_w(priv, 0x0, GSWIP_MDIO_MDC_CFG0);

	/* enable special tag insertion on cpu port */
	gswip_switch_mask(priv, 0, GSWIP_FDMA_PCTRL_STEN,
			  GSWIP_FDMA_PCTRLp(cpu_port));

	gswip_switch_mask(priv, 0, GSWIP_MAC_CTRL_2_MLEN,
			  GSWIP_MAC_CTRL_2p(cpu_port));
	gswip_switch_w(priv, VLAN_ETH_FRAME_LEN + 8, GSWIP_MAC_FLEN);
	gswip_switch_mask(priv, 0, GSWIP_BM_QUEUE_GCTRL_GL_MOD,
			  GSWIP_BM_QUEUE_GCTRL);

	/* VLAN aware Switching */
	gswip_switch_mask(priv, 0, GSWIP_PCE_GCTRL_0_VLAN, GSWIP_PCE_GCTRL_0);

	/* Mac Address Table Lock */
	gswip_switch_mask(priv, 0, GSWIP_PCE_GCTRL_1_MAC_GLOCK |
				   GSWIP_PCE_GCTRL_1_MAC_GLOCK_MOD,
			  GSWIP_PCE_GCTRL_1);

	gswip_port_enable(ds, cpu_port, NULL);
	return 0;
}

static void gswip_adjust_link(struct dsa_switch *ds, int port,
			      struct phy_device *phydev)
{
	struct gswip_priv *priv = ds->priv;
	u16 macconf = phydev->mdio.addr & GSWIP_MDIO_PHY_ADDR_MASK;
	u16 miirate = 0;
	u16 miimode;
	u16 lcl_adv = 0, rmt_adv = 0;
	u8 flowctrl;

	/* do not run this for the CPU port */
	if (dsa_is_cpu_port(ds, port))
		return;

	miimode = gswip_mdio_r(priv, GSWIP_MII_CFGp(port));
	miimode &= GSWIP_MII_CFG_MODE_MASK;

	switch (phydev->speed) {
	case SPEED_1000:
		macconf |= GSWIP_MDIO_PHY_SPEED_G1;
		miirate = GSWIP_MII_CFG_RATE_M125;
		break;

	case SPEED_100:
		macconf |= GSWIP_MDIO_PHY_SPEED_M100;
		switch (miimode) {
		case GSWIP_MII_CFG_MODE_RMIIM:
		case GSWIP_MII_CFG_MODE_RMIIP:
			miirate = GSWIP_MII_CFG_RATE_M50;
			break;
		default:
			miirate = GSWIP_MII_CFG_RATE_M25;
			break;
		}
		break;

	default:
		macconf |= GSWIP_MDIO_PHY_SPEED_M10;
		miirate = GSWIP_MII_CFG_RATE_M2P5;
		break;
	}

	if (phydev->link)
		macconf |= GSWIP_MDIO_PHY_LINK_UP;
	else
		macconf |= GSWIP_MDIO_PHY_LINK_DOWN;

	if (phydev->duplex == DUPLEX_FULL)
		macconf |= GSWIP_MDIO_PHY_FDUP_EN;
	else
		macconf |= GSWIP_MDIO_PHY_FDUP_DIS;

	if (phydev->pause)
		rmt_adv = LPA_PAUSE_CAP;
	if (phydev->asym_pause)
		rmt_adv |= LPA_PAUSE_ASYM;

	if (phydev->advertising & ADVERTISED_Pause)
		lcl_adv |= ADVERTISE_PAUSE_CAP;
	if (phydev->advertising & ADVERTISED_Asym_Pause)
		lcl_adv |= ADVERTISE_PAUSE_ASYM;

	flowctrl = mii_resolve_flowctrl_fdx(lcl_adv, rmt_adv);

	if (flowctrl & FLOW_CTRL_TX)
		macconf |= GSWIP_MDIO_PHY_FCONTX_EN;
	else
		macconf |= GSWIP_MDIO_PHY_FCONTX_DIS;
	if (flowctrl & FLOW_CTRL_RX)
		macconf |= GSWIP_MDIO_PHY_FCONRX_EN;
	else
		macconf |= GSWIP_MDIO_PHY_FCONRX_DIS;

	gswip_mdio_mask(priv, GSWIP_MDIO_PHY_MASK, macconf,
			GSWIP_MDIO_PHYp(port));
	gswip_mii_mask(priv, GSWIP_MII_CFG_RATE_MASK, miirate,
		       GSWIP_MII_CFGp(port));
}

static enum dsa_tag_protocol gswip_get_tag_protocol(struct dsa_switch *ds,
						    int port)
{
	return DSA_TAG_PROTO_GSWIP;
}

static void gswip_get_strings(struct dsa_switch *ds, int port, u32 stringset,
			      uint8_t *data)
{
	int i;

	if (stringset != ETH_SS_STATS)
		return;

	for (i = 0; i < ARRAY_SIZE(gswip_rmon_cnt); i++)
		strncpy(data + i * ETH_GSTRING_LEN, gswip_rmon_cnt[i].name,
			ETH_GSTRING_LEN);
}

static u32 gswip_bcm_ram_entry_read(struct gswip_priv *priv, u32 table,
				    u32 index)
{
	u32 result;

	gswip_switch_w(priv, index, GSWIP_BM_RAM_ADDR);
	gswip_switch_mask(priv, GSWIP_BM_RAM_CTRL_ADDR_MASK |
				GSWIP_BM_RAM_CTRL_OPMOD,
			      table | GSWIP_BM_RAM_CTRL_BAS,
			      GSWIP_BM_RAM_CTRL);

	err = gswip_switch_r_timeout(priv, GSWIP_BM_RAM_CTRL,
				     GSWIP_BM_RAM_CTRL_BAS);
	if (err) {
		dev_err(priv->dev, "timeout while reading table: %u, index: %u",
			table, index);
		return 0;
	}

	result = gswip_switch_r(priv, GSWIP_BM_RAM_VAL(0));
	result |= gswip_switch_r(priv, GSWIP_BM_RAM_VAL(1)) << 16;

	return result;
}

static void gswip_get_ethtool_stats(struct dsa_switch *ds, int port,
				    uint64_t *data)
{
	struct gswip_priv *priv = ds->priv;
	const struct gswip_rmon_cnt_desc *rmon_cnt;
	int i;
	u64 high;

	for (i = 0; i < ARRAY_SIZE(gswip_rmon_cnt); i++) {
		rmon_cnt = &gswip_rmon_cnt[i];

		data[i] = gswip_bcm_ram_entry_read(priv, port,
						   rmon_cnt->offset);
		if (rmon_cnt->size == 2) {
			high = gswip_bcm_ram_entry_read(priv, port,
							rmon_cnt->offset + 1);
			data[i] |= high << 32;
		}
	}
}

static int gswip_get_sset_count(struct dsa_switch *ds, int port, int sset)
{
	if (sset != ETH_SS_STATS)
		return 0;

	return ARRAY_SIZE(gswip_rmon_cnt);
}

static const struct dsa_switch_ops gswip_switch_ops = {
	.get_tag_protocol	= gswip_get_tag_protocol,
	.setup			= gswip_setup,
	.adjust_link		= gswip_adjust_link,
	.port_enable		= gswip_port_enable,
	.port_disable		= gswip_port_disable,
	.get_strings		= gswip_get_strings,
	.get_ethtool_stats	= gswip_get_ethtool_stats,
	.get_sset_count		= gswip_get_sset_count,
};

static int gswip_probe(struct platform_device *pdev)
{
	struct gswip_priv *priv;
	struct resource *gswip_res, *mdio_res, *mii_res;
	struct device_node *mdio_np;
	struct device *dev = &pdev->dev;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	gswip_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->gswip = devm_ioremap_resource(dev, gswip_res);
	if (!priv->gswip)
		return -ENOMEM;

	mdio_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	priv->mdio = devm_ioremap_resource(dev, mdio_res);
	if (!priv->mdio)
		return -ENOMEM;

	mii_res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	priv->mii = devm_ioremap_resource(dev, mii_res);
	if (!priv->mii)
		return -ENOMEM;

	priv->hw_info = of_device_get_match_data(&pdev->dev);
	if (!priv->hw_info)
		return -EINVAL;

	priv->ds = dsa_switch_alloc(dev, priv->hw_info->max_ports);
	if (!priv->ds)
		return -ENOMEM;

	priv->ds->priv = priv;
	priv->ds->ops = &gswip_switch_ops;
	priv->dev = dev;
	if (ds->dst->cpu_dp->index != priv->hw_info->cpu_port) {
		dev_err(dev, "wrong CPU port defined, HW only supports port: %i",
			priv->hw_info->cpu_port);
		return -EINVAL;
	}

	/* bring up the mdio bus */
	mdio_np = of_find_compatible_node(pdev->dev.of_node, NULL,
					  "lantiq,xrx200-mdio");
	if (mdio_np) {
		err = gswip_mdio(priv, mdio_np);
		if (err) {
			dev_err(dev, "mdio probe failed\n");
			return err;
		}
	}

	platform_set_drvdata(pdev, priv);

	err = dsa_register_switch(priv->ds);
	if (err) {
		dev_err(dev, "dsa switch register failed: %i\n", err);
		if (mdio_np)
			mdiobus_unregister(priv->ds->slave_mii_bus);
	}

	return err;
}

static int gswip_remove(struct platform_device *pdev)
{
	struct gswip_priv *priv = platform_get_drvdata(pdev);

	if (!priv)
		return 0;

	/* disable the switch */
	gswip_mdio_mask(priv, GSWIP_MDIO_GLOB_ENABLE, 0, GSWIP_MDIO_GLOB);

	dsa_unregister_switch(priv->ds);

	if (priv->ds->slave_mii_bus)
		mdiobus_unregister(priv->ds->slave_mii_bus);

	return 0;
}

static const struct gswip_hw_info gswip_xrx200 = {
	max_ports = 7;
	cpu_port = 6;
};

static const struct of_device_id gswip_of_match[] = {
	{ .compatible = "lantiq,xrx200-gswip", data = &gswip_xrx200 },
	{},
};
MODULE_DEVICE_TABLE(of, gswip_of_match);

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
