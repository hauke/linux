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

/* MAC */
#define MAC_FLEN_REG		(0x2314)
#define MAC_CTRL_REG(p, x)	(0x240c + (((p * 0xc) + x) * 4))


struct xrx200_chan {
	int tx_free;

	struct tasklet_struct tasklet;
	struct napi_struct napi;
	struct ltq_dma_channel dma;
	struct sk_buff *skb[LTQ_DESC_NUM];

	spinlock_t lock;
	struct xrx200_priv *priv;
};

struct xrx200_priv {
	struct net_device_stats stats;

	struct clk *clk;

	struct xrx200_chan chan[XRX200_MAX_DMA];

	struct net_device *net_dev;
	struct device *dev;

	__iomem void *pmac_reg;
};

static u32 xrx200_pmac_r32(struct xrx200_priv *priv, u32 offset)
{
	return __raw_readl(priv->pmac_reg + offset);
}

static void xrx200_pmac_w32(struct xrx200_priv *priv, u32 val, u32 offset)
{
	return __raw_writel(val, priv->pmac_reg + offset);
}

static void xrx200_switch_w32_mask(struct xrx200_priv *priv, u32 clear, u32 set, u32 offset)
{
	u32 val = xrx200_pmac_r32(priv, offset);

	val &= ~(clear);
	val |= set;
	xrx200_pmac_w32(priv, val, offset);
}

static int xrx200_open(struct net_device *dev)
{
	struct xrx200_priv *priv = netdev_priv(dev);
	int i;

	for (i = 0; i < XRX200_MAX_DMA; i++) {
		if (!priv->chan[i].dma.irq)
			continue;
		spin_lock_bh(&priv->chan[i].lock);
		if (XRX200_DMA_IS_RX(i))
			napi_enable(&priv->chan[i].napi);
		ltq_dma_open(&priv->chan[i].dma);
		spin_unlock_bh(&priv->chan[i].lock);
	}
	netif_wake_queue(dev);

	return 0;
}

static int xrx200_close(struct net_device *dev)
{
	struct xrx200_priv *priv = netdev_priv(dev);
	int i;

	netif_stop_queue(dev);

	for (i = 0; i < XRX200_MAX_DMA; i++) {
		if (!priv->chan[i].dma.irq)
			continue;

		if (XRX200_DMA_IS_RX(i))
			napi_disable(&priv->chan[i].napi);
		spin_lock_bh(&priv->chan[i].lock);
		ltq_dma_close(&priv->chan[XRX200_DMA_RX].dma);
		spin_unlock_bh(&priv->chan[i].lock);
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
	ch->dma.desc_base[ch->dma.desc].addr = dma_map_single(ch->priv->dev,
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
	struct xrx200_priv *priv = ch->priv;
	struct ltq_dma_desc *desc = &ch->dma.desc_base[ch->dma.desc];
	struct sk_buff *skb = ch->skb[ch->dma.desc];
	int len = (desc->ctl & LTQ_DMA_SIZE_MASK);
	int ret;

	ret = xrx200_alloc_skb(ch);

	ch->dma.desc++;
	ch->dma.desc %= LTQ_DESC_NUM;

	if (ret) {
		netdev_err(priv->net_dev,
			"failed to allocate new rx buffer\n");
		return;
	}

	skb_put(skb, len);
	skb->dev = priv->net_dev;
	skb->protocol = eth_type_trans(skb, priv->net_dev);
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

	netif_wake_queue(ch->priv->net_dev);
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

	ch = &priv->chan[XRX200_DMA_TX];

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

	desc->addr = ((unsigned int) dma_map_single(priv->dev, skb->data, len,
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

static irqreturn_t xrx200_dma_irq(int irq, void *ptr)
{
	struct xrx200_priv *priv = ptr;
	int chnr = irq - XRX200_DMA_IRQ;
	struct xrx200_chan *ch = &priv->chan[chnr];

	ltq_dma_disable_irq(&ch->dma);
	ltq_dma_ack_irq(&ch->dma);

	if (chnr % 2)
		tasklet_schedule(&ch->tasklet);
	else
		napi_schedule(&ch->napi);

	return IRQ_HANDLED;
}

static int xrx200_dma_init(struct xrx200_priv *priv)
{
	int i, err = 0;

	ltq_dma_init_port(DMA_PORT_ETOP);

	for (i = 0; i < 8 && !err; i++) {
		int irq = XRX200_DMA_IRQ + i;
		struct xrx200_chan *ch = &priv->chan[i];

		spin_lock_init(&ch->lock);

		ch->dma.nr = i;
		ch->priv = priv;

		if (i == XRX200_DMA_TX) {
			ltq_dma_alloc_tx(&ch->dma);
			err = request_irq(irq, xrx200_dma_irq, 0, "vrx200_tx", priv);
		} else if (i == XRX200_DMA_TX_2) {
			ltq_dma_alloc_tx(&ch->dma);
			err = request_irq(irq, xrx200_dma_irq, 0, "vrx200_tx_2", priv);
		} else if (i == XRX200_DMA_RX) {
			ltq_dma_alloc_rx(&ch->dma);
			for (ch->dma.desc = 0; ch->dma.desc < LTQ_DESC_NUM;
					ch->dma.desc++)
				if (xrx200_alloc_skb(ch))
					err = -ENOMEM;
			ch->dma.desc = 0;
			err = request_irq(irq, xrx200_dma_irq, 0, "vrx200_rx", priv);
		} else
			continue;

		if (!err)
			ch->dma.irq = irq;
		else
			pr_err("net-xrx200: failed to request irq %d\n", irq);
	}

	return err;
}

static void xrx200_hw_init(struct xrx200_priv *priv)
{
	/* enable clock gate */
	clk_enable(priv->clk);

	/* set IPG to 12 */
	xrx200_switch_w32_mask(priv, PMAC_IPG_MASK, 0xb, PMAC_RX_IPG);

	/* enable status header, enable CRC */
	xrx200_switch_w32_mask(priv, 0,
		PMAC_HD_CTL_RST | PMAC_HD_CTL_AST | PMAC_HD_CTL_RXSH | PMAC_HD_CTL_AS | PMAC_HD_CTL_AC | PMAC_HD_CTL_RC,
		PMAC_HD_CTL);
}

static void xrx200_hw_cleanup(struct xrx200_priv *priv)
{
	int i;

	/* free the channels and IRQs */
	for (i = 0; i < 2; i++) {
		ltq_dma_free(&priv->chan[i].dma);
		if (priv->chan[i].dma.irq)
			free_irq(priv->chan[i].dma.irq, priv);
	}

	/* free the allocated RX ring */
	for (i = 0; i < LTQ_DESC_NUM; i++)
		dev_kfree_skb_any(priv->chan[XRX200_DMA_RX].skb[i]);

	/* release the clock */
	clk_disable(priv->clk);
	clk_put(priv->clk);
}


static const struct net_device_ops xrx200_netdev_ops = {
	.ndo_open		= xrx200_open,
	.ndo_stop		= xrx200_close,
	.ndo_start_xmit		= xrx200_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_get_stats		= xrx200_get_stats,
	.ndo_tx_timeout		= xrx200_tx_timeout,
};

static int xrx200_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *res;
	struct device_node *phy_np;
	struct of_phandle_iterator it;
	struct xrx200_priv *priv;
	struct net_device *net_dev;
	const u8 *mac;
	int err;

	/* alloc the network device */
	net_dev = devm_alloc_etherdev(dev, sizeof(struct xrx200_priv));
	if (!net_dev)
		return -ENOMEM;

	priv = netdev_priv(net_dev);
	priv->net_dev = net_dev;
	priv->dev = dev;

	net_dev->netdev_ops = &xrx200_netdev_ops;
	net_dev->watchdog_timeo = XRX200_TX_TIMEOUT;
	net_dev->needed_headroom = XRX200_HEADROOM;
	SET_NETDEV_DEV(net_dev, dev);

	/* load the memory ranges */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get resources\n");
		return -ENOENT;
	}

	priv->pmac_reg = devm_ioremap_resource(&pdev->dev, res);
	if (!priv->pmac_reg) {
		dev_err(&pdev->dev, "failed to request and remap io ranges \n");
		return -ENOMEM;
	}

	of_for_each_phandle(&it, err, np, "lantiq,phys", NULL, 0) {
		phy_np = it.node;
		if (phy_np) {
			struct platform_device *phy = of_find_device_by_node(phy_np);
	
			of_node_put(phy_np);
			if (!platform_get_drvdata(phy))
				return -EPROBE_DEFER;
		}
	}

	/* get the clock */
	priv->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "failed to get clock\n");
		return PTR_ERR(priv->clk);
	}

	mac = of_get_mac_address(np);
	if (mac)
		ether_addr_copy(net_dev->dev_addr, mac);
	else
		eth_hw_addr_random(net_dev);

	/* bring up the dma engine and IP core */
	xrx200_dma_init(priv);
	xrx200_hw_init(priv);
	tasklet_init(&priv->chan[XRX200_DMA_TX].tasklet, xrx200_tx_housekeeping, (u32) &priv->chan[XRX200_DMA_TX]);
	tasklet_init(&priv->chan[XRX200_DMA_TX_2].tasklet, xrx200_tx_housekeeping, (u32) &priv->chan[XRX200_DMA_TX_2]);

	/* setup NAPI */
	netif_napi_add(net_dev, &priv->chan[XRX200_DMA_RX].napi, xrx200_poll_rx, 32);

	platform_set_drvdata(pdev, priv);

	return register_netdev(net_dev);
}

static int xrx200_remove(struct platform_device *pdev)
{
	struct xrx200_priv *priv = platform_get_drvdata(pdev);
	struct net_device *net_dev = priv->net_dev;

	/* free stack related instances */
	netif_stop_queue(net_dev);
	netif_napi_del(&priv->chan[XRX200_DMA_RX].napi);

	/* shut down hardware */
	xrx200_hw_cleanup(priv);

	/* remove the actual device */
	unregister_netdev(net_dev);

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
	},
};

module_platform_driver(xrx200_driver);

MODULE_AUTHOR("John Crispin <blogic@openwrt.org>");
MODULE_DESCRIPTION("Lantiq SoC XRX200 ethernet");
MODULE_LICENSE("GPL");
