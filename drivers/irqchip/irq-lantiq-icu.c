/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 * Copyright (C) 2010 John Crispin <john@phrozen.org>
 * Copyright (C) 2010 Thomas Langer <thomas.langer@lantiq.com>
 */

#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/sched.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <asm/irq_cpu.h>

/* register definitions - internal irqs */
#define LTQ_ICU_IM0_ISR		0x0000
#define LTQ_ICU_IM0_IER		0x0008
#define LTQ_ICU_IM0_IOSR	0x0010
#define LTQ_ICU_IM0_IRSR	0x0018
#define LTQ_ICU_IM0_IMR		0x0020
#define LTQ_ICU_IM1_ISR		0x0028

/* we have a cascade of 8 irqs */
#define MIPS_CPU_IRQ_CASCADE		8

struct ltq_icu_data {
	void __iomem *membase;
	struct irq_domain *domain;
	int irq;
};

static u32 ltq_icu_r32(struct ltq_icu_data *priv, u32 offset)
{
	return __raw_readl(priv->membase + offset);
}

static void ltq_icu_w32(struct ltq_icu_data *priv, u32 val, u32 offset)
{
	return __raw_writel(val, priv->membase + offset);
}

static void ltq_icu_mask(struct ltq_icu_data *priv, u32 clear, u32 set,
			 u32 offset)
{
	u32 val = ltq_icu_r32(priv, offset);

	val &= ~(clear);
	val |= set;
	ltq_icu_w32(priv, val, offset);
}

static void ltq_disable_irq(struct irq_data *d)
{
	struct ltq_icu_data *priv = irq_data_get_irq_chip_data(d);

	ltq_icu_mask(priv, BIT(d->hwirq), 0, LTQ_ICU_IM0_IER);
}

static void ltq_mask_and_ack_irq(struct irq_data *d)
{
	struct ltq_icu_data *priv = irq_data_get_irq_chip_data(d);

	ltq_icu_mask(priv, BIT(d->hwirq), 0 , LTQ_ICU_IM0_IER);
	ltq_icu_w32(priv, BIT(d->hwirq), LTQ_ICU_IM0_ISR);
}

static void ltq_ack_irq(struct irq_data *d)
{
	struct ltq_icu_data *priv = irq_data_get_irq_chip_data(d);

	ltq_icu_w32(priv, BIT(d->hwirq), LTQ_ICU_IM0_ISR);
}

static void ltq_enable_irq(struct irq_data *d)
{
	struct ltq_icu_data *priv = irq_data_get_irq_chip_data(d);

	ltq_icu_mask(priv, 0, BIT(d->hwirq), LTQ_ICU_IM0_IER);
}

static struct irq_chip ltq_irq_type = {
	.name = "icu",
	.irq_enable = ltq_enable_irq,
	.irq_disable = ltq_disable_irq,
	.irq_unmask = ltq_enable_irq,
	.irq_ack = ltq_ack_irq,
	.irq_mask = ltq_disable_irq,
	.irq_mask_ack = ltq_mask_and_ack_irq,
};

static void icu_intc_irq_handler(struct irq_desc *desc)
{
	struct ltq_icu_data *priv = irq_desc_get_handler_data(desc);
	u32 pending;

	pending = ltq_icu_r32(priv, LTQ_ICU_IM0_IOSR);

	if (pending) {
		generic_handle_irq(irq_find_mapping(priv->domain, __ffs(pending)));
	} else {
		spurious_interrupt();
	}
}

static int icu_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw)
{
	irq_set_chip_and_handler(irq, &ltq_irq_type, handle_level_irq);
	irq_set_chip_data(irq, d->host_data);

	return 0;
}

static const struct irq_domain_ops irq_domain_ops = {
	.xlate = irq_domain_xlate_onetwocell,
	.map = icu_map,
};

static int __init icu_of_init(struct device_node *node, struct device_node *parent)
{
	struct resource res;
	int i;
	int start = MIPS_CPU_IRQ_CASCADE;
	struct ltq_icu_data *priv;

	priv = kmalloc_array(MAX_IM, sizeof(struct ltq_icu_data), __GFP_ZERO | GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	for (i = 0; i < MAX_IM; i++) {
		if (of_address_to_resource(node, i, &res))
			panic("Failed to get icu memory range");

		if (!request_mem_region(res.start, resource_size(&res),
					res.name))
			pr_err("Failed to request icu memory");

		priv[i].membase = ioremap_nocache(res.start, resource_size(&res));
		if (!priv[i].membase)
			panic("Failed to remap icu memory");

		priv[i].irq = irq_of_parse_and_map(node, i);
		if (!priv[i].irq)
			panic("Failed to get INTC IRQ");
	}

	/* turn off all irqs by default */
	for (i = 0; i < MAX_IM; i++) {
		/* make sure all irqs are turned off by default */
		ltq_icu_w32(&priv[i], 0, LTQ_ICU_IM0_IER);
		/* clear all possibly pending interrupts */
		ltq_icu_w32(&priv[i], ~0, LTQ_ICU_IM0_ISR);
	}

	for (i = 0; i < MAX_IM; i++) {
		priv[i].domain = irq_domain_add_legacy(node, INT_NUM_IM_OFFSET, start, 0, &irq_domain_ops, &priv[i]);
		if (!priv[i].domain)
			panic("Failed to add irqdomain");

		irq_set_chained_handler_and_data(priv[i].irq, icu_intc_irq_handler, &priv[i]);
		start += INT_NUM_IM_OFFSET;
	}

	return 0;
}

IRQCHIP_DECLARE(icu_intc, "lantiq,icu", icu_of_init);
