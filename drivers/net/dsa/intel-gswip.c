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
 *   Copyright (C) 2017 - 2018 Hauke Mehrtens <hauke@hauke-m.de>
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


#define XRX200_MAX_VLAN		64
#define XRX200_PCE_ACTVLAN_IDX	0x01
#define XRX200_PCE_VLANMAP_IDX	0x02

/* GSWIP MDIO Registers */
#define GSWIP_MDIO_GLOB			0x0000
#define  GSWIP_MDIO_GLOB_ENABLE		BIT(15)
#define GSWIP_MDIO_CTRL			0x0020
#define  GSWIP_MDIO_CTRL_BUSY		BIT(12)
#define  GSWIP_MDIO_CTRL_RD		BIT(11)
#define  GSWIP_MDIO_CTRL_WR		BIT(10)
#define  GSWIP_MDIO_CTRL_PHYAD_MASK	0x1f
#define  GSWIP_MDIO_CTRL_PHYAD_SHIFT	5
#define  GSWIP_MDIO_CTRL_REGAD_MASK	0x1f
#define GSWIP_MDIO_READ			0x0024
#define GSWIP_MDIO_WRITE		0x0028
#define GSWIP_MDIO_PHY(x)		(0x0054 - (x * 4))
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
#define  GSWIP_MDIO_PHY_MASK		GSWIP_MDIO_PHY_ADDR_MASK | \
					GSWIP_MDIO_PHY_FCONRX_MASK | \
					GSWIP_MDIO_PHY_FCONTX_MASK | \
					GSWIP_MDIO_PHY_LINK_MASK | \
					GSWIP_MDIO_PHY_SPEED_MASK | \
					GSWIP_MDIO_PHY_FDUP_MASK
#define GSWIP_MDIO_CLK_CFG0		0x002C
#define GSWIP_MDIO_CLK_CFG1		0x0030


/* GSWIP MII Registers */
#define GSWIP_MII_CFG(p)		(p * 8)
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

#define GSWIP_BM_RAM_VAL(x)		(0x10C - ((x) * 4))
#define GSWIP_BM_RAM_ADDR		0x110
#define GSWIP_BM_RAM_CTRL		0x114
#define  GSWIP_BM_RAM_CTRL_BAS		BIT(15)
#define  GSWIP_BM_RAM_CTRL_OPMOD	BIT(5)
#define  GSWIP_BM_RAM_CTRL_ADDR_MASK	GENMASK(4, 0)
#define GSWIP_BM_QUEUE_GCTRL		0x0128
#define  GSWIP_BM_QUEUE_GCTRL_GL_MOD	BIT(10)
/* buffer management Port Configuration Register */
#define GSWIP_BM_PCFGx(p)		(0x200 + (p * 8))
#define  GSWIP_BM_PCFG_CNTEN		BIT(0)	/* RMON Counter Enable */
#define  GSWIP_BM_PCFG_IGCNT		BIT(1)	/* Ingres Special Tag RMON count */
/* buffer management Port Control Register */
#define GSWIP_BM_CTRLx(p)		(0x204 + (p * 8))
#define  GSWIP_BM_CTRL_RAM1_RES		BIT(0)	/* Software Reset for RMON RAM 1 */
#define  GSWIP_BM_CTRL_RAM2_RES		BIT(1)	/* Software Reset for RMON RAM 2 */

/* PCE */
#define GSWIP_PCE_TBL_KEY(x)		(0x111C - ((x) * 4))
#define GSWIP_PCE_TBL_MASK		0x1120
#define GSWIP_PCE_TBL_VAL(x)		(0x1134 - ((x) * 4))
#define GSWIP_PCE_TBL_ADDR		0x1138
#define GSWIP_PCE_TBL_CTRL		0x113c
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
#define GSWIP_PCE_PMAP1			0x114c	/* Monitoring port map */
#define GSWIP_PCE_PMAP2			0x1150	/* Default Multicast port map */
#define GSWIP_PCE_PMAP3			0x1154	/* Default Unknown Unicast port map */
#define GSWIP_PCE_GCTRL_0		0x1158
#define  GSWIP_PCE_GCTRL_0_MC_VALID	BIT(3)
#define  GSWIP_PCE_GCTRL_0_VLAN		BIT(14) /* VLAN aware Switching */
#define GSWIP_PCE_GCTRL_1		0x115C
#define  GSWIP_PCE_GCTRL_1_MAC_GLOCK	BIT(2)	/* MAC Address table lock */
#define  GSWIP_PCE_GCTRL_1_MAC_GLOCK_MOD	BIT(3) /* Mac address table lock forwarding mode */
#define GSWIP_PCE_PCTRL_0p(p)		(0x1200 + (p * 0x28))
#define  GSWIP_PCE_PCTRL_0_INGRESS	BIT(11)
#define  GSWIP_PCE_PCTRL_0_PSTATE_LISTEN	0x0
#define  GSWIP_PCE_PCTRL_0_PSTATE_RX		0x1
#define  GSWIP_PCE_PCTRL_0_PSTATE_TX		0x2
#define  GSWIP_PCE_PCTRL_0_PSTATE_LEARNING 	0x3
#define  GSWIP_PCE_PCTRL_0_PSTATE_FORWARDING 	0x7
#define  GSWIP_PCE_PCTRL_0_PSTATE_MASK 	GENMASK(2,0)

#define GSWIP_MAC_FLEN			0x2314
#define GSWIP_MAC_CTRL_2p(p)		(0x2414 + (p * 0x30))
#define GSWIP_MAC_CTRL_2_MLEN		BIT(3) /* Maximum Untagged Frame Lnegth */

/* Ethernet Switch Fetch DMA Port Control Register */
#define GSWIP_FDMA_PCTRLx(p)		(0x2A00 + ((p) * 0x18))
#define  GSWIP_FDMA_PCTRL_EN		BIT(0)	/* FDMA Port Enable */
#define  GSWIP_FDMA_PCTRL_STEN		BIT(1)	/* Special Tag Insertion Enable */
#define  GSWIP_FDMA_PCTRL_VLANMOD_MASK	GENMASK(4,3)	/* VLAN Modification Control */
#define  GSWIP_FDMA_PCTRL_VLANMOD_SHIFT	3	/* VLAN Modification Control */
#define  GSWIP_FDMA_PCTRL_VLANMOD_DIS	(0x0 << GSWIP_FDMA_PCTRL_VLANMOD_SHIFT)
#define  GSWIP_FDMA_PCTRL_VLANMOD_PRIO	(0x1 << GSWIP_FDMA_PCTRL_VLANMOD_SHIFT)
#define  GSWIP_FDMA_PCTRL_VLANMOD_ID	(0x2 << GSWIP_FDMA_PCTRL_VLANMOD_SHIFT)
#define  GSWIP_FDMA_PCTRL_VLANMOD_BOTH	(0x3 << GSWIP_FDMA_PCTRL_VLANMOD_SHIFT)

/* Ethernet Switch Store DMA Port Control Register */
#define GSWIP_SDMA_PCTRLx(p)		(0x2F00 + ((p) * 0x18))
#define  GSWIP_SDMA_PCTRL_EN		BIT(0)	/* SDMA Port Enable */
#define  GSWIP_SDMA_PCTRL_FCEN		BIT(1)	/* Flow Control Enable */
#define  GSWIP_SDMA_PCTRL_PAUFWD	BIT(1)	/* Pause Frame Forwarding */


struct gswip_vlan {
	struct net_device *bridge;
	u16 id;
	u8 fid;
	bool used;
	u16 port_map;
	u16 tag_map;
};

struct gswip_priv {
	__iomem void *gswip;
	__iomem void *mdio;
	__iomem void *mii;
	int cpu_port;
	struct dsa_switch *ds;
	struct device *dev;
	struct gswip_vlan vlan[64];
};

struct gswip_rmon_cnt_desc {
	unsigned int size;
	unsigned int offset;
	const char* name;
};

#define MIB_DESC(_size, _offset, _name) {.size = _size, .offset = _offset, .name = _name}

static const struct gswip_rmon_cnt_desc gswip_rmon_cnt[] = {
	/** Receive Packet Count (only packets that are accepted and not discarded). */
	MIB_DESC(1, 0x1F, "RxGoodPkts"),
	/** Receive Unicast Packet Count. */
	MIB_DESC(1, 0x23, "RxUnicastPkts"),
	/** Receive Multicast Packet Count. */
	MIB_DESC(1, 0x22, "RxMulticastPkts"),
	/** Receive FCS Error Packet Count. */
	MIB_DESC(1, 0x21, "RxFCSErrorPkts"),
	/** Receive Undersize Good Packet Count. */
	MIB_DESC(1, 0x1D, "RxUnderSizeGoodPkts"),
	/** Receive Undersize Error Packet Count. */
	MIB_DESC(1, 0x1E, "RxUnderSizeErrorPkts"),
	/** Receive Oversize Good Packet Count. */
	MIB_DESC(1, 0x1B, "RxOversizeGoodPkts"),
	/** Receive Oversize Error Packet Count. */
	MIB_DESC(1, 0x1C, "RxOversizeErrorPkts"),
	/** Receive Good Pause Packet Count. */
	MIB_DESC(1, 0x20, "RxGoodPausePkts"),
	/** Receive Align Error Packet Count. */
	MIB_DESC(1, 0x1A, "RxAlignErrorPkts"),
	/** Receive Size 64 Packet Count. */
	MIB_DESC(1, 0x12, "Rx64BytePkts"),
	/** Receive Size 65-127 Packet Count. */
	MIB_DESC(1, 0x13, "Rx127BytePkts"),
	/** Receive Size 128-255 Packet Count. */
	MIB_DESC(1, 0x14, "Rx255BytePkts"),
	/** Receive Size 256-511 Packet Count. */
	MIB_DESC(1, 0x15, "Rx511BytePkts"),
	/** Receive Size 512-1023 Packet Count. */
	MIB_DESC(1, 0x16, "Rx1023BytePkts"),
	/** Receive Size 1024-1522 (or more, if configured) Packet Count. */
	MIB_DESC(1, 0x17, "RxMaxBytePkts"),
	/** Receive Dropped Packet Count. */
	MIB_DESC(1, 0x18, "RxDroppedPkts"),
	/** Filtered Packet Count. */
	MIB_DESC(1, 0x19, "RxFilteredPkts"),
	/** Receive Good Byte Count (64 bit). */
	MIB_DESC(2, 0x24, "RxGoodBytes"),
	/** Receive Bad Byte Count (64 bit). */
	MIB_DESC(2, 0x26, "RxBadBytes"),
	/** Transmit Dropped Packet Count, based on Congestion Management. */
	MIB_DESC(1, 0x11, "TxAcmDroppedPkts"),
	/** Transmit Packet Count. */
	MIB_DESC(1, 0x0C, "TxGoodPkts"),
	/** Transmit Unicast Packet Count. */
	MIB_DESC(1, 0x06, "TxUnicastPkts"),
	/** Transmit Multicast Packet Count. */
	MIB_DESC(1, 0x07, "TxMulticastPkts"),
	/** Transmit Size 64 Packet Count. */
	MIB_DESC(1, 0x00, "Tx64BytePkts"),
	/** Transmit Size 65-127 Packet Count. */
	MIB_DESC(1, 0x01, "Tx127BytePkts"),
	/** Transmit Size 128-255 Packet Count. */
	MIB_DESC(1, 0x02, "Tx255BytePkts"),
	/** Transmit Size 256-511 Packet Count. */
	MIB_DESC(1, 0x03, "Tx511BytePkts"),
	/** Transmit Size 512-1023 Packet Count. */
	MIB_DESC(1, 0x04, "Tx1023BytePkts"),
	/** Transmit Size 1024-1522 (or more, if configured) Packet Count. */
	MIB_DESC(1, 0x05, "TxMaxBytePkts"),
	/** Transmit Single Collision Count. */
	MIB_DESC(1, 0x08, "TxSingleCollCount"),
	/** Transmit Multiple Collision Count. */
	MIB_DESC(1, 0x09, "TxMultCollCount"),
	/** Transmit Late Collision Count. */
	MIB_DESC(1, 0x0A, "TxLateCollCount"),
	/** Transmit Excessive Collision Count. */
	MIB_DESC(1, 0x0B, "TxExcessCollCount"),
	/** Transmit Pause Packet Count. */
	MIB_DESC(1, 0x0D, "TxPauseCount"),
	/** Transmit Drop Packet Count. */
	MIB_DESC(1, 0x10, "TxDroppedPkts"),
	/** Transmit Good Byte Count (64 bit). */
	MIB_DESC(2, 0x0E, "TxGoodBytes"),
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

static int xrx200_mdio_poll(struct gswip_priv *priv)
{
	unsigned cnt = 10000;

	while (likely(cnt--)) {
		unsigned ctrl = gswip_mdio_r32(priv, GSWIP_MDIO_CTRL);
		if ((ctrl & GSWIP_MDIO_CTRL_BUSY) == 0)
			return 0;
		cpu_relax();
	}

	return 1;
}

static int xrx200_mdio_wr(struct mii_bus *bus, int addr, int reg, u16 val)
{
	struct gswip_priv *priv = bus->priv;

	if (xrx200_mdio_poll(priv))
		return -EIO;

	gswip_mdio_w32(priv, val, GSWIP_MDIO_WRITE);
	gswip_mdio_w32(priv, GSWIP_MDIO_CTRL_BUSY | GSWIP_MDIO_CTRL_WR |
		((addr & GSWIP_MDIO_CTRL_PHYAD_MASK) << GSWIP_MDIO_CTRL_PHYAD_SHIFT) |
		(reg & GSWIP_MDIO_CTRL_REGAD_MASK),
		GSWIP_MDIO_CTRL);

	return 0;
}

static int xrx200_mdio_rd(struct mii_bus *bus, int addr, int reg)
{
	struct gswip_priv *priv = bus->priv;

	if (xrx200_mdio_poll(priv))
		return -EIO;

	gswip_mdio_w32(priv, GSWIP_MDIO_CTRL_BUSY | GSWIP_MDIO_CTRL_RD |
		((addr & GSWIP_MDIO_CTRL_PHYAD_MASK) << GSWIP_MDIO_CTRL_PHYAD_SHIFT) |
		(reg & GSWIP_MDIO_CTRL_REGAD_MASK),
		GSWIP_MDIO_CTRL);

	if (xrx200_mdio_poll(priv))
		return -EIO;

	return gswip_mdio_r32(priv, GSWIP_MDIO_READ);
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

	return of_mdiobus_register(ds->slave_mii_bus, mdio_np);
}

struct xrx200_pce_table_entry {
	u16 index;	// PCE_TBL_ADDR.ADDR = pData->table_index
	u16 table; 	// PCE_TBL_CTRL.ADDR = pData->table
	u16 key[8];
	u16 val[5];
	u16 mask;
	u8 gmap;
	bool type;
	bool valid;
};

static void gswip_wait_pce_tbl_ready(struct gswip_priv *priv)
{
	while (gswip_switch_r32(priv, GSWIP_PCE_TBL_CTRL) & GSWIP_PCE_TBL_CTRL_BAS)
		cond_resched();
}

static void xrx200_pce_table_entry_read(struct gswip_priv *priv,
				        struct xrx200_pce_table_entry *tbl)
{
	int i;
	u16 crtl;

	gswip_wait_pce_tbl_ready(priv);

	gswip_switch_w32(priv, tbl->index, GSWIP_PCE_TBL_ADDR);
	gswip_switch_w32_mask(priv, GSWIP_PCE_TBL_CTRL_ADDR_MASK | GSWIP_PCE_TBL_CTRL_OPMOD_MASK,
			      tbl->table | GSWIP_PCE_TBL_CTRL_OPMOD_ADRD | GSWIP_PCE_TBL_CTRL_BAS,
			      GSWIP_PCE_TBL_CTRL);

	gswip_wait_pce_tbl_ready(priv);

	for (i = 0; i < ARRAY_SIZE(tbl->key); i++)
		tbl->key[i] = gswip_switch_r32(priv, GSWIP_PCE_TBL_KEY(i));

	for (i = 0; i < ARRAY_SIZE(tbl->val); i++)
		tbl->val[i] = gswip_switch_r32(priv, GSWIP_PCE_TBL_VAL(i));

	tbl->mask = gswip_switch_r32(priv, GSWIP_PCE_TBL_MASK);

	crtl = gswip_switch_r32(priv, GSWIP_PCE_TBL_CTRL);

	tbl->type = !!(crtl & GSWIP_PCE_TBL_CTRL_TYPE);
	tbl->valid = !!(crtl & GSWIP_PCE_TBL_CTRL_VLD);
	tbl->gmap = (crtl & GSWIP_PCE_TBL_CTRL_GMAP_MASK) >> 7;
}

static void xrx200_pce_table_entry_write(struct gswip_priv *priv, struct xrx200_pce_table_entry *tbl)
{
	int i;
	u16 crtl;

	gswip_wait_pce_tbl_ready(priv);

	gswip_switch_w32(priv, tbl->index, GSWIP_PCE_TBL_ADDR);
	gswip_switch_w32_mask(priv, GSWIP_PCE_TBL_CTRL_ADDR_MASK | GSWIP_PCE_TBL_CTRL_OPMOD_MASK,
			      tbl->table | GSWIP_PCE_TBL_CTRL_OPMOD_ADWR,
			      GSWIP_PCE_TBL_CTRL);

	for (i = 0; i < ARRAY_SIZE(tbl->key); i++)
		gswip_switch_w32(priv, tbl->key[i], GSWIP_PCE_TBL_KEY(i));

	for (i = 0; i < ARRAY_SIZE(tbl->val); i++)
		gswip_switch_w32(priv, tbl->val[i], GSWIP_PCE_TBL_VAL(i));

	gswip_switch_w32_mask(priv, GSWIP_PCE_TBL_CTRL_ADDR_MASK | GSWIP_PCE_TBL_CTRL_OPMOD_MASK,
			      tbl->table | GSWIP_PCE_TBL_CTRL_OPMOD_ADWR,
			      GSWIP_PCE_TBL_CTRL);

	gswip_switch_w32(priv, tbl->mask, GSWIP_PCE_TBL_MASK);

	crtl = gswip_switch_r32(priv, GSWIP_PCE_TBL_CTRL);
	crtl &= ~(GSWIP_PCE_TBL_CTRL_TYPE | GSWIP_PCE_TBL_CTRL_VLD | GSWIP_PCE_TBL_CTRL_GMAP_MASK);
	if (tbl->type)
		crtl |= GSWIP_PCE_TBL_CTRL_TYPE;
	if (tbl->valid)
		crtl |= GSWIP_PCE_TBL_CTRL_VLD;
	crtl |= (tbl->gmap << 7) & GSWIP_PCE_TBL_CTRL_GMAP_MASK;
	crtl |= GSWIP_PCE_TBL_CTRL_BAS;
	gswip_switch_w32(priv, crtl, GSWIP_PCE_TBL_CTRL);

	gswip_wait_pce_tbl_ready(priv);
}

static void xrx200_pci_microcode(struct gswip_priv *priv)
{
	int i;

	gswip_switch_w32_mask(priv, GSWIP_PCE_TBL_CTRL_ADDR_MASK | GSWIP_PCE_TBL_CTRL_OPMOD_MASK,
			      GSWIP_PCE_TBL_CTRL_OPMOD_ADWR, GSWIP_PCE_TBL_CTRL);
	gswip_switch_w32(priv, 0, GSWIP_PCE_TBL_MASK);

	for (i = 0; i < ARRAY_SIZE(pce_microcode); i++) {
		gswip_switch_w32(priv, i, GSWIP_PCE_TBL_ADDR);
		gswip_switch_w32(priv, pce_microcode[i].val[3], GSWIP_PCE_TBL_VAL(0));
		gswip_switch_w32(priv, pce_microcode[i].val[2], GSWIP_PCE_TBL_VAL(1));
		gswip_switch_w32(priv, pce_microcode[i].val[1], GSWIP_PCE_TBL_VAL(2));
		gswip_switch_w32(priv, pce_microcode[i].val[0], GSWIP_PCE_TBL_VAL(3));

		// start the table access:
		gswip_switch_w32_mask(priv, 0, GSWIP_PCE_TBL_CTRL_BAS, GSWIP_PCE_TBL_CTRL);
		gswip_wait_pce_tbl_ready(priv);
	}

	/* tell the switch that the microcode is loaded */
	gswip_switch_w32_mask(priv, 0, GSWIP_PCE_GCTRL_0_MC_VALID, GSWIP_PCE_GCTRL_0);
}

static int gswip_setup(struct dsa_switch *ds)
{
	struct gswip_priv *priv = ds->priv;
	int i;

	gswip_switch_w32(priv, GSWIP_ETHSW_SWRES_R0, GSWIP_ETHSW_SWRES);
	msleep(10);
	gswip_switch_w32(priv, 0, GSWIP_ETHSW_SWRES);

	/* disable port fetch/store dma */
	for (i = 0; i < 7; i++ ) {
		gswip_switch_w32_mask(priv, GSWIP_FDMA_PCTRL_EN, 0, GSWIP_FDMA_PCTRLx(i));
		gswip_switch_w32_mask(priv, GSWIP_SDMA_PCTRL_EN, 0, GSWIP_SDMA_PCTRLx(i));
	}

	/* enable Switch */
	gswip_mdio_w32_mask(priv, 0, GSWIP_MDIO_GLOB_ENABLE, GSWIP_MDIO_GLOB);

	xrx200_pci_microcode(priv);

	/* Default unknown Broadcat/Multicast/Unicast port maps */
	gswip_switch_w32(priv, BIT(priv->cpu_port), GSWIP_PCE_PMAP1);
	gswip_switch_w32(priv, BIT(priv->cpu_port), GSWIP_PCE_PMAP2);
	gswip_switch_w32(priv, BIT(priv->cpu_port), GSWIP_PCE_PMAP3);

	/* disable auto polling */
	gswip_mdio_w32(priv, 0x0, GSWIP_MDIO_CLK_CFG0);

	/* RMON Counter Enable CPU port */
	gswip_switch_w32(priv, GSWIP_BM_PCFG_CNTEN, GSWIP_BM_PCFGx(priv->cpu_port));

	/* enable port fetch/store dma & VLAN Modification */
	gswip_switch_w32_mask(priv, 0, GSWIP_FDMA_PCTRL_EN | GSWIP_FDMA_PCTRL_VLANMOD_BOTH, GSWIP_FDMA_PCTRLx(priv->cpu_port));
	gswip_switch_w32_mask(priv, 0, GSWIP_SDMA_PCTRL_EN, GSWIP_SDMA_PCTRLx(priv->cpu_port));
	gswip_switch_w32_mask(priv, 0, GSWIP_PCE_PCTRL_0_INGRESS, GSWIP_PCE_PCTRL_0p(priv->cpu_port));

	/* enable special tag insertion on cpu port */
	gswip_switch_w32_mask(priv, 0, GSWIP_FDMA_PCTRL_STEN, GSWIP_FDMA_PCTRLx(priv->cpu_port));
	gswip_switch_w32_mask(priv, 0, GSWIP_PCE_PCTRL_0_INGRESS, GSWIP_PCE_PCTRL_0p(priv->cpu_port));
	gswip_switch_w32_mask(priv, 0, GSWIP_MAC_CTRL_2_MLEN, GSWIP_MAC_CTRL_2p(priv->cpu_port));
	gswip_switch_w32(priv, VLAN_ETH_FRAME_LEN + 8, GSWIP_MAC_FLEN);
	gswip_switch_w32_mask(priv, 0, GSWIP_BM_QUEUE_GCTRL_GL_MOD, GSWIP_BM_QUEUE_GCTRL);

	/* VLAN aware Switching */
	gswip_switch_w32_mask(priv, 0, GSWIP_PCE_GCTRL_0_VLAN, GSWIP_PCE_GCTRL_0);

	/* Mac Address Table Lock */
	gswip_switch_w32_mask(priv, 0, GSWIP_PCE_GCTRL_1_MAC_GLOCK | GSWIP_PCE_GCTRL_1_MAC_GLOCK_MOD, GSWIP_PCE_GCTRL_1);
	return 0;
}

static void gswip_adjust_link(struct dsa_switch *ds, int port, struct phy_device *phydev)
{
	struct gswip_priv *priv = (struct gswip_priv *)ds->priv;
	u16 phyaddr = phydev->mdio.addr & GSWIP_MDIO_PHY_ADDR_MASK;
	u16 miirate = 0;
	u16 miimode;
	u16 lcl_adv = 0, rmt_adv = 0;
	u8 flowctrl;

	/* do not run this for the CPU port 6 */
	if (port >= priv->cpu_port)
		return;

	miimode = gswip_mdio_r32(priv, GSWIP_MII_CFG(port)) & GSWIP_MII_CFG_MODE_MASK;

	switch (phydev->speed) {
	case SPEED_1000:
		phyaddr |= GSWIP_MDIO_PHY_SPEED_G1;
		miirate = GSWIP_MII_CFG_RATE_M125;
		break;

	case SPEED_100:
		phyaddr |= GSWIP_MDIO_PHY_SPEED_M100;
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
		phyaddr |= GSWIP_MDIO_PHY_SPEED_M10;
		miirate = GSWIP_MII_CFG_RATE_M2P5;
		break;
	}

	if (phydev->link)
		phyaddr |= GSWIP_MDIO_PHY_LINK_UP;
	else
		phyaddr |= GSWIP_MDIO_PHY_LINK_DOWN;

	if (phydev->duplex == DUPLEX_FULL)
		phyaddr |= GSWIP_MDIO_PHY_FDUP_EN;
	else
		phyaddr |= GSWIP_MDIO_PHY_FDUP_DIS;

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
		phyaddr |= GSWIP_MDIO_PHY_FCONTX_EN;
	else
		phyaddr |= GSWIP_MDIO_PHY_FCONTX_DIS;
	if (flowctrl & FLOW_CTRL_RX)
		phyaddr |= GSWIP_MDIO_PHY_FCONRX_EN;
	else
		phyaddr |= GSWIP_MDIO_PHY_FCONRX_DIS;

	gswip_mdio_w32_mask(priv, GSWIP_MDIO_PHY_MASK, phyaddr, GSWIP_MDIO_PHY(port));
	gswip_mii_w32_mask(priv, GSWIP_MII_CFG_RATE_MASK, miirate, GSWIP_MII_CFG(port));
}

static int gswip_port_enable(struct dsa_switch *ds, int port, struct phy_device *phy)
{
	struct gswip_priv *priv = (struct gswip_priv *)ds->priv;

	/* RMON Counter Enable for port */
	gswip_switch_w32(priv, GSWIP_BM_PCFG_CNTEN, GSWIP_BM_PCFGx(port));

	/* enable port fetch/store dma & VLAN Modification */
	gswip_switch_w32_mask(priv, 0, GSWIP_FDMA_PCTRL_EN | GSWIP_FDMA_PCTRL_VLANMOD_BOTH, GSWIP_FDMA_PCTRLx(port));
	gswip_switch_w32_mask(priv, 0, GSWIP_SDMA_PCTRL_EN, GSWIP_SDMA_PCTRLx(port));
	gswip_switch_w32_mask(priv, 0, GSWIP_PCE_PCTRL_0_INGRESS, GSWIP_PCE_PCTRL_0p(port));

	return 0;
}

static void gswip_port_disable(struct dsa_switch *ds, int port, struct phy_device *phy)
{
	struct gswip_priv *priv = (struct gswip_priv *)ds->priv;

	gswip_switch_w32_mask(priv, GSWIP_FDMA_PCTRL_EN, 0, GSWIP_FDMA_PCTRLx(port));
	gswip_switch_w32_mask(priv, GSWIP_SDMA_PCTRL_EN, 0, GSWIP_SDMA_PCTRLx(port));
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,15,0)
static enum dsa_tag_protocol gswip_get_tag_protocol(struct dsa_switch *ds,
						    int port)
#else
static enum dsa_tag_protocol gswip_get_tag_protocol(struct dsa_switch *ds)
#endif
{
	return DSA_TAG_PROTO_GSWIP;
}

static void gswip_stp_state_set(struct dsa_switch *ds, int port, u8 state)
{
	struct gswip_priv *priv = ds->priv;
	u32 stp_state;

	switch (state) {
	case BR_STATE_DISABLED:
		gswip_switch_w32_mask(priv, GSWIP_SDMA_PCTRL_EN, 0, GSWIP_SDMA_PCTRLx(port));
		return;
	case BR_STATE_BLOCKING:
	case BR_STATE_LISTENING:
		stp_state = GSWIP_PCE_PCTRL_0_PSTATE_LISTEN;
		break;
	case BR_STATE_LEARNING:
		stp_state = GSWIP_PCE_PCTRL_0_PSTATE_LEARNING;
		break;
	case BR_STATE_FORWARDING:
	default:
		stp_state = GSWIP_PCE_PCTRL_0_PSTATE_FORWARDING;
		break;
	}

	gswip_switch_w32_mask(priv, 0, GSWIP_SDMA_PCTRL_EN, GSWIP_SDMA_PCTRLx(port));
	gswip_switch_w32_mask(priv, GSWIP_PCE_PCTRL_0_PSTATE_MASK, stp_state, GSWIP_PCE_PCTRL_0p(port));
}

static struct gswip_vlan *gswip_vlan_entry(struct gswip_priv *priv, int vlan)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(priv->vlan); i++) {
		if (priv->vlan[i].id == vlan && priv->vlan[i].used)
			return &priv->vlan[i];
	}
	return NULL;
}

/*
 * The switch has a table with the active VLANs (XRX200_PCE_ACTVLAN_IDX)
 * with 64 possible entries and an other table (XRX200_PCE_VLANMAP_IDX)
 * where ports can be added to such an VLAN.
 * The driver first searches if there is already an entry for this bridge
 * and if not it searches for a free index and a unused VLAN and add a new
 * VLAN entry.  At the end this port will be added to the VLAN map. */
static int
gswip_port_bridge_join(struct dsa_switch *ds, int port,
			struct net_device *bridge)
{
	struct gswip_priv *priv = ds->priv;
	struct gswip_vlan *vlan_entry = NULL;
	struct xrx200_pce_table_entry tbl = {0,};
	int vlan;
	int vlan_idx = -1;
	int i;

	for (i = 0; i < ARRAY_SIZE(priv->vlan); i++) {
		if (priv->vlan[i].bridge == bridge) {
			vlan_entry = &priv->vlan[i];
			break;
		}
	}

	/* Find a VLAN ID in the switch from 1000 to 1064 which is not
	 * used by trying all the 64 VLAN IDs and then checking all the
	 * 64 possible positions. The range was choose randomly. */
	if (!vlan_entry) {
		for (vlan = 1000; vlan < 1064; vlan++) {
			/* check if the VLAN is used */
			if (gswip_vlan_entry(priv, vlan))
				continue;

			for (i = 0; i < ARRAY_SIZE(priv->vlan); i++) {
				if (!priv->vlan[i].used) {
					vlan_entry = &priv->vlan[i];
					vlan_idx = i;
					break;
				}
			}

			if (!vlan_entry)
				return -EIO;

			vlan_entry->id = vlan;
			vlan_entry->bridge = bridge;
			vlan_entry->used = true;
			vlan_entry->fid = vlan_idx + 1;

			tbl.index = vlan_idx;
			tbl.table = XRX200_PCE_ACTVLAN_IDX;
			tbl.key[0] = vlan_entry->id;
			tbl.val[0] = vlan_entry->fid;
			tbl.valid = true;
			xrx200_pce_table_entry_write(priv, &tbl);
			memset(&tbl, 0x0, sizeof(tbl));

			vlan_entry->port_map |= BIT(priv->cpu_port);
			vlan_entry->tag_map |= BIT(priv->cpu_port);
			break;
		}
	}

	if (!vlan_entry)
		return -EIO;

	vlan_entry->port_map |= BIT(port);

	tbl.index = vlan_idx;
	tbl.table = XRX200_PCE_VLANMAP_IDX;
	tbl.key[0] = vlan_entry->id;
	tbl.val[0] = vlan_entry->port_map;
	tbl.val[1] = vlan_entry->tag_map;
	tbl.valid = true;
	xrx200_pce_table_entry_write(priv, &tbl);
	return 0;
}

static void
gswip_port_bridge_leave(struct dsa_switch *ds, int port,
			 struct net_device *bridge)
{
	struct gswip_priv *priv = ds->priv;
	struct gswip_vlan *vlan_entry;
	struct xrx200_pce_table_entry tbl = {0,};
	int i;

	for (i = 0; i < ARRAY_SIZE(priv->vlan); i++) {
		if (priv->vlan[i].bridge == bridge) {
			vlan_entry = &priv->vlan[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(priv->vlan)) {
		dev_err(priv->dev, "can not remove unknown brigde for port %i\n",
			port);
		return;
	}

	vlan_entry->port_map &= ~BIT(port);

	tbl.index = i;
	tbl.table = XRX200_PCE_VLANMAP_IDX;
	tbl.key[0] = vlan_entry->id;
	tbl.val[0] = vlan_entry->port_map;
	tbl.val[1] = vlan_entry->tag_map;
	if (vlan_entry->port_map)
		tbl.valid = true;
	xrx200_pce_table_entry_write(priv, &tbl);

	if (!vlan_entry->port_map) {
		memset(&tbl, 0x0, sizeof(tbl));
		tbl.index = i;
		tbl.table = XRX200_PCE_ACTVLAN_IDX;
		tbl.key[0] = vlan_entry->id;
		tbl.valid = false;
		xrx200_pce_table_entry_write(priv, &tbl);
	}
}

static void gswip_get_strings(struct dsa_switch *ds, int port, u32 stringset, uint8_t *data)
{
	int i;

	if (stringset != ETH_SS_STATS)
		return;

	for (i = 0; i < ARRAY_SIZE(gswip_rmon_cnt); i++)
		strncpy(data + i * ETH_GSTRING_LEN, gswip_rmon_cnt[i].name,
			ETH_GSTRING_LEN);
}

static u32 gswip_bcm_ram_entry_read(struct gswip_priv *priv, u32 table, u32 index)
{
	u32 result;

	gswip_switch_w32(priv, index, GSWIP_BM_RAM_ADDR);
	gswip_switch_w32_mask(priv, GSWIP_BM_RAM_CTRL_ADDR_MASK | GSWIP_BM_RAM_CTRL_OPMOD,
			      table | GSWIP_BM_RAM_CTRL_BAS,
			      GSWIP_BM_RAM_CTRL);

	while (gswip_switch_r32(priv, GSWIP_BM_RAM_CTRL) & GSWIP_BM_RAM_CTRL_BAS)
		cond_resched();

	result = gswip_switch_r32(priv, GSWIP_BM_RAM_VAL(0));
	result |= gswip_switch_r32(priv, GSWIP_BM_RAM_VAL(1)) << 16;

	return result;
}

static void gswip_get_ethtool_stats(struct dsa_switch *ds, int port, uint64_t *data)
{
	struct gswip_priv *priv = ds->priv;
	const struct gswip_rmon_cnt_desc *rmon_cnt;
	int i;
	u64 high;

	for (i = 0; i < ARRAY_SIZE(gswip_rmon_cnt); i++) {
		rmon_cnt = &gswip_rmon_cnt[i];

		data[i] = gswip_bcm_ram_entry_read(priv, port, rmon_cnt->offset);
		if (rmon_cnt->size == 2) {
			high = gswip_bcm_ram_entry_read(priv, port, rmon_cnt->offset + 1);
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
	.port_stp_state_set	= gswip_stp_state_set,
	.port_bridge_join	= gswip_port_bridge_join,
	.port_bridge_leave	= gswip_port_bridge_leave,
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

	priv->ds = dsa_switch_alloc(dev, DSA_MAX_PORTS);
	if (!priv->ds)
		return -ENOMEM;

	priv->ds->priv = priv;
	priv->ds->ops = &gswip_switch_ops;
	priv->dev = dev;
	priv->cpu_port = 6;

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
	gswip_mdio_w32_mask(priv, GSWIP_MDIO_GLOB_ENABLE, 0, GSWIP_MDIO_GLOB);

	dsa_unregister_switch(priv->ds);

	if (priv->ds->slave_mii_bus)
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
