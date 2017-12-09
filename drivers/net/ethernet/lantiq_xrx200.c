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

struct xrx200_port {
	u8 num;
	u8 phy_addr;
	u16 flags;
	phy_interface_t phy_if;

	int link;
	int gpio;
	enum of_gpio_flags gpio_flags;

	struct phy_device *phydev;
	struct device_node *phy_node;
};

struct xrx200_chan {
	int idx;
	int refcount;
	int tx_free;

	struct net_device *devs;

	struct tasklet_struct tasklet;
	struct napi_struct napi;
	struct ltq_dma_channel dma;
	struct sk_buff *skb[LTQ_DESC_NUM];

	spinlock_t lock;
};

struct xrx200_hw {
	struct clk *clk;
	struct mii_bus *mii_bus;

	struct xrx200_chan chan[XRX200_MAX_DMA];

	struct net_device *devs;
};

struct xrx200_priv {
	struct net_device_stats stats;

	struct xrx200_port port[XRX200_MAX_PORT];
	int num_port;
	bool sw;
	unsigned short port_map;
	unsigned char mac[6];

	struct xrx200_hw *hw;
};

static __iomem void *xrx200_pmac_membase;

#define ltq_pmac_r32(x)		ltq_r32(xrx200_pmac_membase + (x))
#define ltq_pmac_w32(x, y)	ltq_w32(x, xrx200_pmac_membase + (y))
#define ltq_pmac_w32_mask(x, y, z) \
			ltq_w32_mask(x, y, xrx200_pmac_membase + (z))

#define XRX200_GLOBAL_REGATTR(reg) \
	.id = reg, \
	.type = SWITCH_TYPE_INT, \
	.set = xrx200_set_global_attr, \
	.get = xrx200_get_global_attr

#define XRX200_PORT_REGATTR(reg) \
	.id = reg, \
	.type = SWITCH_TYPE_INT, \
	.set = xrx200_set_port_attr, \
	.get = xrx200_get_port_attr

static int xrx200_open(struct net_device *dev)
{
	struct xrx200_priv *priv = netdev_priv(dev);
	int i;

	for (i = 0; i < XRX200_MAX_DMA; i++) {
		if (!priv->hw->chan[i].dma.irq)
			continue;
		spin_lock_bh(&priv->hw->chan[i].lock);
		if (!priv->hw->chan[i].refcount) {
			if (XRX200_DMA_IS_RX(i))
				napi_enable(&priv->hw->chan[i].napi);
			ltq_dma_open(&priv->hw->chan[i].dma);
		}
		priv->hw->chan[i].refcount++;
		spin_unlock_bh(&priv->hw->chan[i].lock);
	}
	for (i = 0; i < priv->num_port; i++)
		if (priv->port[i].phydev)
			phy_start(priv->port[i].phydev);
	netif_wake_queue(dev);

	return 0;
}

static int xrx200_close(struct net_device *dev)
{
	struct xrx200_priv *priv = netdev_priv(dev);
	int i;

	netif_stop_queue(dev);

	for (i = 0; i < priv->num_port; i++)
		if (priv->port[i].phydev)
			phy_stop(priv->port[i].phydev);

	for (i = 0; i < XRX200_MAX_DMA; i++) {
		if (!priv->hw->chan[i].dma.irq)
			continue;

		priv->hw->chan[i].refcount--;
		if (!priv->hw->chan[i].refcount) {
			if (XRX200_DMA_IS_RX(i))
				napi_disable(&priv->hw->chan[i].napi);
			spin_lock_bh(&priv->hw->chan[i].lock);
			ltq_dma_close(&priv->hw->chan[XRX200_DMA_RX].dma);
			spin_unlock_bh(&priv->hw->chan[i].lock);
		}
	}

	return 0;
}

static int xrx200_alloc_skb(struct xrx200_chan *ch)
{
#define DMA_PAD	(NET_IP_ALIGN + NET_SKB_PAD)
	ch->skb[ch->dma.desc] = dev_alloc_skb(XRX200_DMA_DATA_LEN + DMA_PAD);
	if (!ch->skb[ch->dma.desc])
		goto skip;

	skb_reserve(ch->skb[ch->dma.desc], NET_SKB_PAD);
	ch->dma.desc_base[ch->dma.desc].addr = dma_map_single(NULL,
		ch->skb[ch->dma.desc]->data, XRX200_DMA_DATA_LEN,
			DMA_FROM_DEVICE);
	ch->dma.desc_base[ch->dma.desc].addr =
		CPHYSADDR(ch->skb[ch->dma.desc]->data);
	skb_reserve(ch->skb[ch->dma.desc], NET_IP_ALIGN);

skip:
	ch->dma.desc_base[ch->dma.desc].ctl =
		LTQ_DMA_OWN | LTQ_DMA_RX_OFFSET(NET_IP_ALIGN) |
		XRX200_DMA_DATA_LEN;

	return 0;
}

static void xrx200_hw_receive(struct xrx200_chan *ch)
{
	struct net_device *dev = ch->devs;
	struct xrx200_priv *priv = netdev_priv(dev);
	struct ltq_dma_desc *desc = &ch->dma.desc_base[ch->dma.desc];
	struct sk_buff *skb = ch->skb[ch->dma.desc];
	int len = (desc->ctl & LTQ_DMA_SIZE_MASK);
	int ret;

	ret = xrx200_alloc_skb(ch);

	ch->dma.desc++;
	ch->dma.desc %= LTQ_DESC_NUM;

	if (ret) {
		netdev_err(dev,
			"failed to allocate new rx buffer\n");
		return;
	}

	skb_put(skb, len);
	skb->dev = dev;
	skb->protocol = eth_type_trans(skb, dev);
	netif_receive_skb(skb);
	priv->stats.rx_packets++;
	priv->stats.rx_bytes+=len;
}

static int xrx200_poll_rx(struct napi_struct *napi, int budget)
{
	struct xrx200_chan *ch = container_of(napi,
				struct xrx200_chan, napi);
	int rx = 0;
	int complete = 0;

	while ((rx < budget) && !complete) {
		struct ltq_dma_desc *desc = &ch->dma.desc_base[ch->dma.desc];
		if ((desc->ctl & (LTQ_DMA_OWN | LTQ_DMA_C)) == LTQ_DMA_C) {
			xrx200_hw_receive(ch);
			rx++;
		} else {
			complete = 1;
		}
	}

	if (complete || !rx) {
		napi_complete(&ch->napi);
		ltq_dma_enable_irq(&ch->dma);
	}

	return rx;
}

static void xrx200_tx_housekeeping(unsigned long ptr)
{
	struct xrx200_chan *ch = (struct xrx200_chan *) ptr;
	int pkts = 0;

	spin_lock_bh(&ch->lock);
	ltq_dma_ack_irq(&ch->dma);
	while ((ch->dma.desc_base[ch->tx_free].ctl & (LTQ_DMA_OWN | LTQ_DMA_C)) == LTQ_DMA_C) {
		struct sk_buff *skb = ch->skb[ch->tx_free];

		pkts++;
		ch->skb[ch->tx_free] = NULL;
		dev_kfree_skb(skb);
		memset(&ch->dma.desc_base[ch->tx_free], 0,
			sizeof(struct ltq_dma_desc));
		ch->tx_free++;
		ch->tx_free %= LTQ_DESC_NUM;
	}
	ltq_dma_enable_irq(&ch->dma);
	spin_unlock_bh(&ch->lock);

	if (!pkts)
		return;

	netif_wake_queue(ch->devs);
}

static struct net_device_stats *xrx200_get_stats (struct net_device *dev)
{
	struct xrx200_priv *priv = netdev_priv(dev);

	return &priv->stats;
}

static void xrx200_tx_timeout(struct net_device *dev)
{
	struct xrx200_priv *priv = netdev_priv(dev);

	printk(KERN_ERR "%s: transmit timed out, disable the dma channel irq\n", dev->name);

	priv->stats.tx_errors++;
	netif_wake_queue(dev);
}

static int xrx200_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct xrx200_priv *priv = netdev_priv(dev);
	struct xrx200_chan *ch;
	struct ltq_dma_desc *desc;
	u32 byte_offset;
	int ret = NETDEV_TX_OK;
	int len;

	ch = &priv->hw->chan[XRX200_DMA_TX];

	desc = &ch->dma.desc_base[ch->dma.desc];

	skb->dev = dev;
	len = skb->len < ETH_ZLEN ? ETH_ZLEN : skb->len;

	/* dma needs to start on a 16 byte aligned address */
	byte_offset = CPHYSADDR(skb->data) % 16;

	spin_lock_bh(&ch->lock);
	if ((desc->ctl & (LTQ_DMA_OWN | LTQ_DMA_C)) || ch->skb[ch->dma.desc]) {
		netdev_err(dev, "tx ring full\n");
		netif_stop_queue(dev);
		ret = NETDEV_TX_BUSY;
		goto out;
	}

	ch->skb[ch->dma.desc] = skb;

	netif_trans_update(dev);

	desc->addr = ((unsigned int) dma_map_single(NULL, skb->data, len,
						DMA_TO_DEVICE)) - byte_offset;
	wmb();
	desc->ctl = LTQ_DMA_OWN | LTQ_DMA_SOP | LTQ_DMA_EOP |
		LTQ_DMA_TX_OFFSET(byte_offset) | (len & LTQ_DMA_SIZE_MASK);
	ch->dma.desc++;
	ch->dma.desc %= LTQ_DESC_NUM;
	if (ch->dma.desc == ch->tx_free)
		netif_stop_queue(dev);


	priv->stats.tx_packets++;
	priv->stats.tx_bytes+=len;

out:
	spin_unlock_bh(&ch->lock);

	return ret;
}

static irqreturn_t xrx200_dma_irq(int irq, void *priv)
{
	struct xrx200_hw *hw = priv;
	int chnr = irq - XRX200_DMA_IRQ;
	struct xrx200_chan *ch = &hw->chan[chnr];

	ltq_dma_disable_irq(&ch->dma);
	ltq_dma_ack_irq(&ch->dma);

	if (chnr % 2)
		tasklet_schedule(&ch->tasklet);
	else
		napi_schedule(&ch->napi);

	return IRQ_HANDLED;
}

static int xrx200_dma_init(struct xrx200_hw *hw)
{
	int i, err = 0;

	ltq_dma_init_port(DMA_PORT_ETOP);

	for (i = 0; i < 8 && !err; i++) {
		int irq = XRX200_DMA_IRQ + i;
		struct xrx200_chan *ch = &hw->chan[i];

		spin_lock_init(&ch->lock);

		ch->idx = ch->dma.nr = i;

		if (i == XRX200_DMA_TX) {
			ltq_dma_alloc_tx(&ch->dma);
			err = request_irq(irq, xrx200_dma_irq, 0, "vrx200_tx", hw);
		} else if (i == XRX200_DMA_TX_2) {
			ltq_dma_alloc_tx(&ch->dma);
			err = request_irq(irq, xrx200_dma_irq, 0, "vrx200_tx_2", hw);
		} else if (i == XRX200_DMA_RX) {
			ltq_dma_alloc_rx(&ch->dma);
			for (ch->dma.desc = 0; ch->dma.desc < LTQ_DESC_NUM;
					ch->dma.desc++)
				if (xrx200_alloc_skb(ch))
					err = -ENOMEM;
			ch->dma.desc = 0;
			err = request_irq(irq, xrx200_dma_irq, 0, "vrx200_rx", hw);
		} else
			continue;

		if (!err)
			ch->dma.irq = irq;
		else
			pr_err("net-xrx200: failed to request irq %d\n", irq);
	}

	return err;
}

static int xrx200_init(struct net_device *dev)
{
	struct xrx200_priv *priv = netdev_priv(dev);
	struct sockaddr mac;
	int err;

#ifndef SW_POLLING
	unsigned int reg = 0;

	/* enable auto polling */
	for (i = 0; i < priv->num_port; i++)
		reg |= BIT(priv->port[i].num);
	ltq_mdio_w32(reg, MDIO_CLK_CFG0);
	ltq_mdio_w32(MDIO1_25MHZ, MDIO_CLK_CFG1);
#endif

	memcpy(&mac.sa_data, priv->mac, ETH_ALEN);
	if (!is_valid_ether_addr(mac.sa_data)) {
		pr_warn("net-xrx200: invalid MAC, using random\n");
		eth_random_addr(mac.sa_data);
		dev->addr_assign_type |= NET_ADDR_RANDOM;
	}

	err = eth_mac_addr(dev, &mac);
	if (err)
		goto err_netdev;


	return 0;

err_netdev:
	unregister_netdev(dev);
	free_netdev(dev);
	return err;
}

static void xrx200_hw_init(struct xrx200_hw *hw)
{
	/* enable clock gate */
	clk_enable(hw->clk);

	/* set IPG to 12 */
	ltq_pmac_w32_mask(PMAC_IPG_MASK, 0xb, PMAC_RX_IPG);

	/* enable status header, enable CRC */
	ltq_pmac_w32_mask(0,
		PMAC_HD_CTL_RST | PMAC_HD_CTL_AST | PMAC_HD_CTL_RXSH | PMAC_HD_CTL_AS | PMAC_HD_CTL_AC | PMAC_HD_CTL_RC,
		PMAC_HD_CTL);
}

static void xrx200_hw_cleanup(struct xrx200_hw *hw)
{
	int i;



	/* free the channels and IRQs */
	for (i = 0; i < 2; i++) {
		ltq_dma_free(&hw->chan[i].dma);
		if (hw->chan[i].dma.irq)
			free_irq(hw->chan[i].dma.irq, hw);
	}

	/* free the allocated RX ring */
	for (i = 0; i < LTQ_DESC_NUM; i++)
		dev_kfree_skb_any(hw->chan[XRX200_DMA_RX].skb[i]);

	/* release the clock */
	clk_disable(hw->clk);
	clk_put(hw->clk);
}


static const struct net_device_ops xrx200_netdev_ops = {
	.ndo_init		= xrx200_init,
	.ndo_open		= xrx200_open,
	.ndo_stop		= xrx200_close,
	.ndo_start_xmit		= xrx200_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_get_stats		= xrx200_get_stats,
	.ndo_tx_timeout		= xrx200_tx_timeout,
};

static void xrx200_of_iface(struct xrx200_hw *hw, struct device *dev)
{
	struct device_node *iface = dev->of_node;
	struct xrx200_priv *priv;
	const u8 *mac;

	/* alloc the network device */
	hw->devs = alloc_etherdev(sizeof(struct xrx200_priv));
	if (!hw->devs)
		return;

	/* setup the network device */
	strcpy(hw->devs->name, "eth%d");
	hw->devs->netdev_ops = &xrx200_netdev_ops;
	hw->devs->watchdog_timeo = XRX200_TX_TIMEOUT;
	hw->devs->needed_headroom = XRX200_HEADROOM;
	SET_NETDEV_DEV(hw->devs, dev);

	/* setup our private data */
	priv = netdev_priv(hw->devs);
	priv->hw = hw;

	mac = of_get_mac_address(iface);
	if (mac)
		memcpy(priv->mac, mac, ETH_ALEN);

	/* register the actual device */
	register_netdev(hw->devs);
}

static struct xrx200_hw xrx200_hw;

static int xrx200_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device_node *phy_np;
	struct of_phandle_iterator it;
	int err;

	/* load the memory ranges */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get resources\n");
		return -ENOENT;
	}

	xrx200_pmac_membase = devm_ioremap_resource(&pdev->dev, res);
	if (!xrx200_pmac_membase) {
		dev_err(&pdev->dev, "failed to request and remap io ranges \n");
		return -ENOMEM;
	}

	of_for_each_phandle(&it, err, pdev->dev.of_node, "lantiq,phys", NULL, 0) {
		phy_np = it.node;
		if (phy_np) {
			struct platform_device *phy = of_find_device_by_node(phy_np);
	
			of_node_put(phy_np);
			if (!platform_get_drvdata(phy))
				return -EPROBE_DEFER;
		}
	}

	/* get the clock */
	xrx200_hw.clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(xrx200_hw.clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		return PTR_ERR(xrx200_hw.clk);
	}

	/* bring up the dma engine and IP core */
	xrx200_dma_init(&xrx200_hw);
	xrx200_hw_init(&xrx200_hw);
	tasklet_init(&xrx200_hw.chan[XRX200_DMA_TX].tasklet, xrx200_tx_housekeeping, (u32) &xrx200_hw.chan[XRX200_DMA_TX]);
	tasklet_init(&xrx200_hw.chan[XRX200_DMA_TX_2].tasklet, xrx200_tx_housekeeping, (u32) &xrx200_hw.chan[XRX200_DMA_TX_2]);

	xrx200_of_iface(&xrx200_hw, &pdev->dev);

	xrx200_hw.chan[XRX200_DMA_RX].devs = xrx200_hw.devs;
	xrx200_hw.chan[XRX200_DMA_TX].devs = xrx200_hw.devs;
	xrx200_hw.chan[XRX200_DMA_TX_2].devs = xrx200_hw.devs;

	/* setup NAPI */
	netif_napi_add(xrx200_hw.chan[XRX200_DMA_RX].devs,
			&xrx200_hw.chan[XRX200_DMA_RX].napi, xrx200_poll_rx, 32);

	platform_set_drvdata(pdev, &xrx200_hw);

	return 0;
}

static int xrx200_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct xrx200_priv *priv;

	if (!dev)
		return 0;

	priv = netdev_priv(dev);

	/* free stack related instances */
	netif_stop_queue(dev);
	netif_napi_del(&xrx200_hw.chan[XRX200_DMA_RX].napi);

	/* shut down hardware */
	xrx200_hw_cleanup(&xrx200_hw);

	/* remove the actual device */
	unregister_netdev(dev);
	free_netdev(dev);

	return 0;
}

static const struct of_device_id xrx200_match[] = {
	{ .compatible = "lantiq,xrx200-net" },
	{},
};
MODULE_DEVICE_TABLE(of, xrx200_match);

static struct platform_driver xrx200_driver = {
	.probe = xrx200_probe,
	.remove = xrx200_remove,
	.driver = {
		.name = "lantiq,xrx200-net",
		.of_match_table = xrx200_match,
		.owner = THIS_MODULE,
	},
};

module_platform_driver(xrx200_driver);

MODULE_AUTHOR("John Crispin <blogic@openwrt.org>");
MODULE_DESCRIPTION("Lantiq SoC XRX200 ethernet");
MODULE_LICENSE("GPL");
