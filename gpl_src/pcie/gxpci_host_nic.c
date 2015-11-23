/*
 * Copyright 2014 Tilera Corporation. All Rights Reserved.
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation, version 2.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 *   NON INFRINGEMENT.  See the GNU General Public License for
 *   more details.
 *
 * Tilera Gx PCIe driver that provides a NIC interface to the kernel.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/vmalloc.h>
#include <linux/rtnetlink.h>
#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <linux/ethtool.h>
#ifdef GXPCI_NETLIB_VNIC
#include <linux/if_vlan.h>
#endif
#include <linux/prefetch.h>
#include <linux/delay.h>

#include <stdbool.h>

#include <asm/io.h>

#include "tilegxpci.h"
#include "tilegxpci_host.h"

#include "gxpci_host_nic.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
#define ETH_FCS_LEN		4
#endif

#define DRV_NAME		"gxpci_host_nic"

#if 0
#define GXPCI_DUMP_PACKETS
#define GXPCI_DEBUG
#endif

#if 0
#define GXPCI_RX_DROP_PACKETS
#endif

#if 0
#define GXPCI_TX_DROP_PACKETS
#endif

#if 0
#define GXPCI_ECHO_PACKETS
#endif

#ifdef GXPCI_DEBUG
#define GXPCI_TRACE(FMT, ...)	printk(FMT, ## __VA_ARGS__)
#else
#define GXPCI_TRACE(FMT, ...)
#endif

#ifndef GXPCI_PACKET_PROTOCOL
#include <linux/etherdevice.h>
#endif

#ifndef pr_err
#define pr_err(fmt, ...)	\
	printk(KERN_ERR fmt, ## __VA_ARGS__)
#endif

#ifndef pr_warning
#define pr_warning(fmt, ...)	\
	printk(KERN_WARNING fmt, ## __VA_ARGS__)
#endif

static int rx_copybreak = 200;

const char gxpci_driver_name[] = "gxpci_host_nic";

const char gxpci_driver_version[] = "3.0-4.2.0";

const char gxpci_fw_version[] = "1.0.GA";

#if GXPCI_HOST_NIC_QUEUE_DEBUG
struct timer_list dump_timer[GXPCI_HOST_NIC_COUNT];

static void dump(unsigned long arg)
{
	int i;
	struct gxpci_nic *nic = (struct gxpci_nic*) arg;

	for (i = 0; i < nic->num_tx_queues; i++) {
		struct gxpci_ring *tx_ring = &nic->tx_ring[i];
        	printk("nic%d-tx_ring[%d]: cmds_posted %d, consumed %d\n",
		       nic->if_num, i, tx_ring->commands_posted,
		       readl(&tx_ring->regs->cmds_consumed_count));
	}

	for (i = 0; i < nic->num_rx_queues; i++) {
		struct gxpci_ring *rx_ring = &nic->rx_ring[i];
		printk("nic%d-rx_ring[%d] posted %d, consumed %d\n",
		       nic->if_num, i, rx_ring->commands_posted,
		       readl(&rx_ring->regs->cmds_consumed_count));
	}

	mod_timer(&dump_timer[nic->if_num], jiffies + 30 * HZ);
}
#endif

#ifdef GXPCI_DUMP_PACKETS
/**
 * Dump a packet.
 **/
static void dump_packet(unsigned char *data, unsigned long length, char *s)
{
	unsigned long i;
	static unsigned int count = 0;

	printk("dump_packet(data %p, length %lx %s count %x)\n",
	       data, length, s, count++);

	printk("\n");

	for (i = 0; i < length; i++) {
		if ((i & 0xf) == 0) {
			printk("%8.8lx:", i);
		}
		printk(" %2.2x", data[i]);
		if (0xf == (i & 0xf)) {
			printk("\n");
		}
	}

	printk("\n\n");
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
static inline struct sk_buff *
gxpci_netdev_alloc_skb_ip_align(struct net_device *dev, unsigned int length)
{
	struct sk_buff *skb;
	skb = __alloc_skb(length + NET_SKB_PAD + NET_IP_ALIGN, GFP_ATOMIC, 0,
			  numa_node_id());
	if (likely(skb)) {
		skb_reserve(skb, NET_SKB_PAD);
		skb->dev = dev;
	}

	if (NET_IP_ALIGN && skb)
		skb_reserve(skb, NET_IP_ALIGN);
	return skb;
}
#endif

#ifdef GXPCI_NETLIB_VNIC
/**
 * gxpci_host_nic_reg_wr - Post command to Gx side for writing MAC register.
 **/
static inline void 
gxpci_host_nic_reg_wr(struct gxpci_nic *nic, uint32_t cmd, uint64_t value)
{
	writel(cmd, &nic->netlib_regs->cmd);
	writeq(value, &nic->netlib_regs->data);
	writel(GXPCI_NETLIB_VNIC_WRITE, &nic->netlib_regs->ops);

	/* Wait until Gx side read register value. */
	while (readl(&nic->netlib_regs->ops) != GXPCI_NETLIB_VNIC_NOP)
		;
}

/**
 * gxpci_host_nic_reg_rd - Post command to Gx side for reading MAC register.
 **/
static inline uint64_t gxpci_host_nic_reg_rd(struct gxpci_nic *nic,
					     uint32_t cmd)
{
	uint64_t rnt = 0;

	writel(cmd, &nic->netlib_regs->cmd);
	writel(GXPCI_NETLIB_VNIC_READ, &nic->netlib_regs->ops);

	/* Wait until Gx side set register value. */
	while (readl(&nic->netlib_regs->ops) != GXPCI_NETLIB_VNIC_NOP)
		;

	rnt = readq(&nic->netlib_regs->data);

	return rnt;	
}
#endif

/**
 * gxpci_desc_unused - calculate if we have unused descriptors.
 **/
static inline int gxpci_desc_unused(struct gxpci_ring *ring)
{
	return ring->count - (ring->commands_posted - ring->commands_consumed);
}

/**
 * gxpci_alloc_rx_buffers - Alloc new or replace used receive buffers.
 * @nic: address of board private structure
 **/
static void gxpci_alloc_rx_buffers(struct gxpci_nic *nic,
				   struct gxpci_ring *rx_ring,
				   int cleaned_count)
{
	struct net_device *netdev = nic->netdev;
	struct pci_dev *pdev = nic->pci_dev;
	struct gxpci_buffer *buffer_info;
	struct sk_buff *skb;
	unsigned int i, cmd_cnt;
	tile_nic_dma_cmd_t *dma;

	cmd_cnt = rx_ring->commands_posted;

	while (cleaned_count--) {
		i = cmd_cnt & rx_ring->count_mask;
		buffer_info = &rx_ring->buffer_infos[i];
		dma = &rx_ring->desc[i];

		/*
 		 * Reuse buffers that received small ingress packets, see
 		 * rx_copybreak.
 		 */
		if (buffer_info->skb)
			goto skip_allocation;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
		skb = gxpci_netdev_alloc_skb_ip_align(netdev,
			nic->rx_buffer_len);
#else
		skb = dev_alloc_skb(nic->rx_buffer_len);
#endif
		if (unlikely(!skb)) {
			GXPCI_TRACE("gxpci_alloc_rx_buffers alloc_skb"
				    " failure\n");
			break;
		}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
		skb->dev = netdev;
#endif
		buffer_info->skb = skb;
		buffer_info->dma = pci_map_single(pdev, skb->data,
			nic->rx_buffer_len, PCI_DMA_FROMDEVICE);
		if (unlikely(!buffer_info->dma)) {
			pr_err("gxpci_alloc_rx_buffers: RX DMA map failed\n");
			break;
		}

		writeq(buffer_info->dma, &dma->host_addr);
skip_allocation:
		cmd_cnt++;
	}

	/*
	 * Make sure all the host_addr are visible before we update
	 * cmds_posted_count.
	 */
	wmb();

	if (likely(rx_ring->commands_posted != cmd_cnt)) {
		rx_ring->commands_posted = cmd_cnt;

		writel(rx_ring->commands_posted,
		       &rx_ring->regs->cmds_posted_count);
	}
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
/**
 * gxpci_irq_vector_enable - Enable interrupt generation
 *			     for a vector.
 **/
static inline void gxpci_irq_vector_enable(struct gxpci_nic *nic,
					   struct gxpci_queue_vector *vector)
{
#ifndef USE_MSI_ONLY
	int ring_index = vector->ring;

	if (vector->tx_only) {
		writel(1, &nic->tx_ring[ring_index].regs->interrupt_enable);
	} else if (vector->rx_only) {
		writel(1, &nic->rx_ring[ring_index].regs->interrupt_enable);
	} else {
		writel(1, &nic->tx_ring[ring_index].regs->interrupt_enable);
		writel(1, &nic->rx_ring[ring_index].regs->interrupt_enable);
	}
#else
	int ring_index;

	for (ring_index = 0; ring_index < nic->num_tx_queues; ring_index++) {
		writel(1, &nic->tx_ring[ring_index].regs->interrupt_enable);
		writel(1, &nic->rx_ring[ring_index].regs->interrupt_enable);
	}
#endif
}

/**
 * gxpci_irq_vector_disable - Mask off interrupt generation
 *			      for a vector.
 **/
static inline void gxpci_irq_vector_disable(struct gxpci_nic *nic,
					    struct gxpci_queue_vector *vector)
{
#ifndef USE_MSI_ONLY
	int ring_index = vector->ring;

	if (vector->tx_only) {
		writel(0, &nic->tx_ring[ring_index].regs->interrupt_enable);
	} else if (vector->rx_only) {
		writel(0, &nic->rx_ring[ring_index].regs->interrupt_enable);
	} else {
		writel(0, &nic->tx_ring[ring_index].regs->interrupt_enable);
		writel(0, &nic->rx_ring[ring_index].regs->interrupt_enable);
	}
#else
	int ring_index;

	for (ring_index = 0; ring_index < nic->num_tx_queues; ring_index++) {
		writel(0, &nic->tx_ring[ring_index].regs->interrupt_enable);
		writel(0, &nic->rx_ring[ring_index].regs->interrupt_enable);
	}
#endif
}

/**
 * gxpci_irq_enable - Enable interrupt generation for all vectors.
 **/
static inline void gxpci_irq_enable(struct gxpci_nic *nic)
{
	int i;

        for (i = 0; i < nic->int_vectors; i++) {
                struct gxpci_queue_vector *vec = &nic->vectors[i];

                gxpci_irq_vector_enable(nic, vec);
	}
}

/**
 * gxpci_irq_disable - Mask off interrupt generation on the NIC.
 **/
static inline void gxpci_irq_disable(struct gxpci_nic *nic)
{
	int i;

        for (i = 0; i < nic->int_vectors; i++) {
                struct gxpci_queue_vector *vec = &nic->vectors[i];

                gxpci_irq_vector_disable(nic, vec);
	}
}

/**
 * gxpci_tx_intr - Interrupt handler for a Tx-only vector.
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 **/
static irqreturn_t gxpci_tx_intr(int irq, void *data)
{
	struct gxpci_queue_vector *vector = data;
	struct gxpci_nic *nic = vector->nic;

	if (unlikely(test_bit(__GXPCI_DOWN, &nic->state)))
		return IRQ_NONE;

	if (napi_schedule_prep(&vector->napi)) {
		/* Disable interrupts and register for poll. */
		gxpci_irq_vector_disable(nic, vector);
		__napi_schedule(&vector->napi);
	}

	return IRQ_HANDLED;
}

/**
 * gxpci_rx_intr - Interrupt handler for a Rx-only vector.
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 **/
static irqreturn_t gxpci_rx_intr(int irq, void *data)
{
	struct gxpci_queue_vector *vector = data;
	struct gxpci_nic *nic = vector->nic;

	if (unlikely(test_bit(__GXPCI_DOWN, &nic->state)))
		return IRQ_NONE;

	if (napi_schedule_prep(&vector->napi)) {
		/* Disable interrupts and register for poll. */
		gxpci_irq_vector_disable(nic, vector);
		__napi_schedule(&vector->napi);
	}

	return IRQ_HANDLED;
}

#else

/**
 * gxpci_irq_enable - Enable interrupt generation.
 **/
static inline void gxpci_irq_enable(struct gxpci_nic *nic)
{
	writel(1, &nic->tx_ring->regs->interrupt_enable);
	writel(1, &nic->rx_ring->regs->interrupt_enable);
}

/**
 * gxpci_irq_disable - Mask off interrupt generation on the NIC.
 **/
static inline void gxpci_irq_disable(struct gxpci_nic *nic)
{
	writel(0, &nic->tx_ring->regs->interrupt_enable);
	writel(0, &nic->rx_ring->regs->interrupt_enable);
}

#endif

/**
 * gxpci_intr - Interrupt Handler for Tx-Rx vector.
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 **/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t gxpci_intr(int irq, void *data, struct pt_regs *regs)
#else
static irqreturn_t gxpci_intr(int irq, void *data)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	struct gxpci_queue_vector *vector = data;
	struct gxpci_nic *nic = vector->nic;
	struct napi_struct *napi = &vector->napi;
#else
	struct net_device *netdev = data;
	struct gxpci_nic *nic = netdev_priv(netdev);
#endif

	if (unlikely(test_bit(__GXPCI_DOWN, &nic->state)))
		return IRQ_NONE;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	if (napi_schedule_prep(napi)) {
		/* Disable interrupts and register for poll. */
		gxpci_irq_vector_disable(nic, vector);
		__napi_schedule(napi);
	}
#else
	if (netif_rx_schedule_prep(netdev)) {
		/* Disable interrupts and register for poll. */
		gxpci_irq_disable(nic);
		__netif_rx_schedule(netdev);
	}
#endif

	return IRQ_HANDLED;
}

/*
 * gxpci_rx_copybreak - If the packet size is small, copy it to a new buffer
 * because the memory copy would cost less than the DMA mapping/unmapping
 * overhead that is associated with a newly allocated receive buffer.
 *
 * @nic: Driver private structure.
 * @skb: Packet to copy.
 * @pkt_len: Length of the packet.
 */
static struct sk_buff *gxpci_rx_copybreak(struct gxpci_nic *nic,
					  struct sk_buff *skb, int pkt_len)
{
	struct sk_buff *new_skb;

	if (likely(pkt_len >= rx_copybreak))
		return NULL;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
	new_skb = netdev_alloc_skb_ip_align(nic->netdev, pkt_len);
#else
	new_skb = dev_alloc_skb(pkt_len);
#endif

	if (unlikely(!new_skb))
		return NULL;

	/* Copy skbuff by skipping our own custom header. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31) 
	skb_copy_to_linear_data_offset(new_skb, -NET_IP_ALIGN,
		skb->data + sizeof(tile_nic_tile_desc_t) - NET_IP_ALIGN,
		pkt_len + NET_IP_ALIGN);
#else
	memcpy(new_skb->data, skb->data + sizeof(tile_nic_tile_desc_t),
	       pkt_len);
#endif
	/* Add new data. */
	skb_put(new_skb, pkt_len);

	return new_skb;
}

/**
 * gxpci_rx_irq - Send received data up the network stack.
 * @nic: board private structure
 * @ring: descriptor ring to transact packets on
 * @work_done: amount of packets that have been processed
 * @work_to_do: amount of packets to process
 **/
static void gxpci_rx_irq(struct gxpci_nic *nic, struct gxpci_ring *ring,
			 int *work_done, int work_to_do)
{
	struct net_device *netdev = nic->netdev;
	struct pci_dev *pdev = nic->pci_dev;
	struct gxpci_buffer *buffer_info;
	struct sk_buff *new_skb;
	struct sk_buff *skb;
#ifndef GXPCI_PACKET_PROTOCOL
#ifdef GXPCI_NETLIB_VNIC 
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
	struct ethhdr *eth;
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31) */
#endif /* GXPCI_NETLIB_VNIC */
#endif /* GXPCI_PACKET_PROTOCOL */
	unsigned int rx_packets = 0;
	unsigned int rx_bytes = 0;
	unsigned int last, new;
	unsigned int pkt_len;
	unsigned int count;
	unsigned int index;
	int cleaned_count = 0;

	/*
	 * If the interface is down, drop the packet.
	 * This is needed here because the posted receive commands would
	 * complete with reset bit set after the interface is brought down.
	 */
	if (unlikely(test_bit(__GXPCI_DOWN, &nic->state))) {
		return;
	}

	new = readl(&ring->regs->cmds_consumed_count);
	last = ring->commands_consumed;

	for (; last != new; last++) {
	 	tile_nic_tile_desc_t *tile_rx_header;

		if (*work_done >= work_to_do)
			break;
		(*work_done)++;

		index = last & ring->count_mask;
		buffer_info = &ring->buffer_infos[index];

		skb = buffer_info->skb;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
		prefetch(skb->data - NET_IP_ALIGN);
#else
		prefetch(skb->data);
#endif
		prefetch(buffer_info + 1);

		/* Get the packet size from our own custom header. */
		tile_rx_header = (tile_nic_tile_desc_t *)skb->data;
		pkt_len = tile_rx_header->size;

		new_skb = gxpci_rx_copybreak(nic, skb, pkt_len);
		if (new_skb) {
			skb = new_skb;
		} else {
			pci_unmap_single(pdev, buffer_info->dma,
					 nic->rx_buffer_len,
					 PCI_DMA_FROMDEVICE);
			buffer_info->dma = 0;
			buffer_info->skb = NULL;

			/* Add new data, along with our own custom header. */
			skb_put(skb, pkt_len + sizeof(tile_nic_tile_desc_t));

			/* Skip our own custom header. */
			skb_pull(skb, sizeof(tile_nic_tile_desc_t));
		}

		skb->dev = netdev;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
		skb_record_rx_queue(skb, ring->index);
#endif

#ifdef GXPCI_PACKET_PROTOCOL
		skb->protocol = GXPCI_PACKET_PROTOCOL;
#else
		if (unlikely(nic->rx_csum == 0))
			skb->ip_summed = CHECKSUM_NONE;
		else
			skb->ip_summed = CHECKSUM_UNNECESSARY;
#ifdef GXPCI_NETLIB_VNIC 
		if (!tile_rx_header->csum_t2h)
			skb->ip_summed = CHECKSUM_NONE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
		/* Do eth_type_trans() with pkt_type offload from Tile. */
		skb_reset_mac_header(skb);
		__skb_pull(skb, ETH_HLEN);
		eth = eth_hdr(skb);
		skb->pkt_type = tile_rx_header->msg.pkt_type;
		if (ntohs(eth->h_proto) >= 1536)
			skb->protocol = eth->h_proto;
		else 
			skb->protocol = htons(ETH_P_802_3);
#else
		skb->protocol = eth_type_trans(skb, netdev);
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31) */
#else
		skb->protocol = eth_type_trans(skb, netdev);
#endif /* GXPCI_NETLIB_VNIC */
#endif /* GXPCI_PACKET_PROTOCOL */

#ifdef GXPCI_DUMP_PACKETS
		dump_packet(skb->data, skb->len, "rx");
#endif

#ifdef GXPCI_RX_DROP_PACKETS
		dev_kfree_skb_irq(skb);
#else
#ifdef GXPCI_ECHO_PACKETS
		dev_queue_xmit(skb);
#else
		netif_receive_skb(skb);
#endif
#endif
		netdev->last_rx = jiffies;

		rx_packets++;
		rx_bytes += pkt_len;

		ring->stats.packets++;

		cleaned_count++;

		/* Replenish some receive buffers, one at a time is too slow. */
		if (unlikely(cleaned_count > GXPCI_RXD_REFILL_THRESHOLD)) {
			gxpci_alloc_rx_buffers(nic, ring, cleaned_count);
			cleaned_count = 0;
		}
	}

	ring->commands_consumed = last;

	/* Replenish some receive buffers again. */
	count = gxpci_desc_unused(ring);
	if (count > GXPCI_RXD_REFILL_THRESHOLD >> 1)
		gxpci_alloc_rx_buffers(nic, ring, count);

	nic->stats.rx_packets += rx_packets;
	nic->stats.rx_bytes += rx_bytes;

	return;
}

/**
 * gxpci_tx_irq - Reclaim resources after transmit completes.
 * @nic: board private structure
 * @ring: descriptor ring to transact packets on
 * @budget: amount of packets driver is allowed to process in this poll
 *
 * Return true if tx ring is cleaned up completely, otherwise false
 **/
static bool gxpci_tx_irq(struct gxpci_nic *nic, struct gxpci_ring *ring,
			 int budget)
{
	struct net_device *netdev = nic->netdev;
	struct pci_dev *pdev = nic->pci_dev;
	struct gxpci_buffer *buffer_info;
	unsigned int tx_packets = 0;
	unsigned int tx_bytes = 0;
	unsigned int last, new;
	unsigned int tx_cleaned = 0;

	if (unlikely(test_bit(__GXPCI_DOWN, &nic->state)))
		return true;

	new = readl(&ring->regs->cmds_consumed_count);
	last = ring->commands_consumed;

	tx_cleaned = new - last;
	if (unlikely(tx_cleaned == 0))
		return true;

	for (; last != new;) {
		unsigned int index = last++ & ring->count_mask;

		buffer_info = &ring->buffer_infos[index];

		if (buffer_info->dma) {
			pci_unmap_single(pdev, buffer_info->dma,
			       buffer_info->length, PCI_DMA_TODEVICE);
			buffer_info->dma = 0;
		}
		if (buffer_info->skb) {
			dev_kfree_skb_irq(buffer_info->skb);
			buffer_info->skb = NULL;
		}

		tx_packets++;
		tx_bytes += buffer_info->length;

		prefetch(buffer_info + 1);

		ring->stats.packets++;
		if (--budget == 0)
			break;
	}

	ring->commands_consumed = last;

	if (netif_carrier_ok(netdev) &&
	    gxpci_desc_unused(ring) >= GXPCI_WAKE_THRESHOLD) {
		smp_mb();

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
		if (__netif_subqueue_stopped(netdev, ring->index) &&
#else
		if (netif_queue_stopped(netdev) &&
#endif
		    !(test_bit(__GXPCI_DOWN, &nic->state))) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
			netif_wake_subqueue(netdev, ring->index);
#else
			netif_wake_queue(netdev);
#endif
		}
	}

	nic->stats.tx_packets += tx_packets;
	nic->stats.tx_bytes += tx_bytes;

	return !!budget;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
#ifndef USE_MSI_ONLY
/**
 * gxpci_poll_tx_only - NAPI polling callback for a Tx-only queue.
 * @napi: struct associated with this polling callback
 * @budget: amount of packets driver is allowed to process in this poll
 **/
static int gxpci_poll_tx_only(struct napi_struct *napi, int budget)
{
	struct gxpci_queue_vector *vector =
		container_of(napi, struct gxpci_queue_vector, napi);
	struct gxpci_nic *nic = vector->nic;
	struct gxpci_ring *ring = &nic->tx_ring[vector->ring];
	bool tx_clean_complete = true;

	tx_clean_complete &= gxpci_tx_irq(nic, ring, GXPCI_DEFAULT_TX_BUDGET);

	/*
	 * If all xmit work done, reenable the interrupt and
	 * exit the polling mode.
	 */
	if (tx_clean_complete) {
		napi_complete(napi);
		if (!test_bit(__GXPCI_DOWN, &nic->state))
			gxpci_irq_vector_enable(nic, vector);

		return 0;
	}

	/* If all work not complete, return budget and keep polling. */
	return budget;
}

/**
 * gxpci_poll_rx_only - NAPI polling callback for a Rx-only queue.
 * @napi: struct associated with this polling callback
 * @budget: amount of packets driver is allowed to process in this poll
 **/
static int gxpci_poll_rx_only(struct napi_struct *napi, int budget)
{
	struct gxpci_queue_vector *vector =
		container_of(napi, struct gxpci_queue_vector, napi);
	struct gxpci_nic *nic = vector->nic;
	struct gxpci_ring *ring = &nic->rx_ring[vector->ring];
	int work_done = 0;

	gxpci_rx_irq(nic, ring, &work_done, budget);

	/*
	 * If budget not fully consumed, reenable the interrupt and
	 * exit the polling mode.
	 */
	if (work_done < budget) {
		napi_complete(napi);
		if (!test_bit(__GXPCI_DOWN, &nic->state)) {
			gxpci_irq_vector_enable(nic, vector);
		}
	}

	return work_done;
}

/**
 * gxpci_poll - NAPI Rx polling callback.
 * @napi: struct associated with this polling callback
 * @budget: amount of packets driver is allowed to process in this poll
 **/
static int gxpci_poll(struct napi_struct *napi, int budget)
{
	struct gxpci_queue_vector *vector =
		container_of(napi, struct gxpci_queue_vector, napi);
	struct gxpci_nic *nic = vector->nic;
	struct gxpci_ring *tx_ring;
	struct gxpci_ring *rx_ring;
	bool tx_clean_complete = true;
	int work_done = 0;

	tx_ring = &nic->tx_ring[vector->ring];
	tx_clean_complete &= gxpci_tx_irq(nic, tx_ring,
					  GXPCI_DEFAULT_TX_BUDGET);

	rx_ring = &nic->rx_ring[vector->ring];
	gxpci_rx_irq(nic, rx_ring, &work_done, budget);

	/*
	 * If budget not fully consumed, reenable the interrupt and
	 * exit the polling mode.
	 */
	if (work_done < budget && tx_clean_complete) {
		napi_complete(napi);
		if (!test_bit(__GXPCI_DOWN, &nic->state))
			gxpci_irq_vector_enable(nic, vector);
	}

	return work_done;
}
#else
/**
 * gxpci_poll_msi - NAPI Rx polling callback when MSI is used.
 * @napi: struct associated with this polling callback
 * @budget: amount of packets driver is allowed to process in this poll
 **/
static int gxpci_poll_msi(struct napi_struct *napi, int budget)
{
	struct gxpci_queue_vector *vector =
		container_of(napi, struct gxpci_queue_vector, napi);
	struct gxpci_nic *nic = vector->nic;
	struct gxpci_ring *ring;
	int work_done = 0;
	int i;

	/* Loop through all the queues. */
	for (i = 0; i < nic->num_tx_queues; i++) {
		ring = &nic->tx_ring[i];
		gxpci_tx_irq(nic, ring, ring->count);
	}

	for (i = 0; i < nic->num_rx_queues; i++) {
		ring = &nic->rx_ring[i];
		gxpci_rx_irq(nic, ring, &work_done, ring->count);
	}

	/*
	 * Since we process all the packets, reenable the interrupt and
	 * exit the polling mode.
	 */
	napi_complete(napi);
	if (!test_bit(__GXPCI_DOWN, &nic->state))
		gxpci_irq_vector_enable(nic, vector);

	return work_done;
}
#endif
#else
/**
 * gxpci_poll - NAPI Rx polling callback.
 * Return value;
 *	0: exit polling mode
 *	1: continue in polling mode, meanwhile decrease the number
 *	   of "budget" by "work_done".
 **/
static int gxpci_poll(struct net_device *poll_dev, int *budget)
{
	struct gxpci_nic *nic;
	int work_to_do = min(*budget, poll_dev->quota);
	bool tx_clean_complete = true;
	int work_done = 0;

	/* Must NOT use netdev_priv macro here. */
	nic = poll_dev->priv;

	tx_clean_complete &= gxpci_tx_irq(nic, nic->tx_ring,
					  GXPCI_DEFAULT_TX_BUDGET);

	gxpci_rx_irq(nic, nic->rx_ring, &work_done, work_to_do);

	*budget -= work_done;
	poll_dev->quota -= work_done;

	/* If no Tx and not enough Rx work done, exit polling mode. */
	if ((tx_clean_complete && work_done == 0) || !netif_running(poll_dev)) {
		netif_rx_complete(poll_dev);
		if (!test_bit(__GXPCI_DOWN, &nic->state))
			gxpci_irq_enable(nic);

		return 0;
	}

	return 1;
}	
#endif

/**
 * gxpci_clean_rx_ring - Free Rx Buffers per Queue.
 * @nic: board private structure
 * @rx_ring: rx descriptor ring to transact packets on
 **/
static void gxpci_clean_rx_ring(struct gxpci_nic *nic,
				struct gxpci_ring *rx_ring)
{
	struct pci_dev *pdev = nic->pci_dev;
	struct gxpci_buffer *buffer_info;
	unsigned int i, size;

	if (rx_ring->buffer_infos == NULL)
		return;

	/* Free all the Rx ring sk_buffs. */
	for (i = 0; i < rx_ring->count; i++) {
		buffer_info = &rx_ring->buffer_infos[i];
		if (buffer_info->dma) {
			pci_unmap_single(pdev,
					 buffer_info->dma,
					 nic->rx_buffer_len,
					 PCI_DMA_FROMDEVICE);
			buffer_info->dma = 0;
		}

		if (buffer_info->skb) {
			dev_kfree_skb(buffer_info->skb);
			buffer_info->skb = NULL;
		}
	}

	size = sizeof(struct gxpci_buffer) * rx_ring->count;
	memset(rx_ring->buffer_infos, 0, size);

	rx_ring->commands_posted = 0;
	rx_ring->commands_consumed = 0;
}

/**
 * gxpci_request_irq - initialize interrupts.
 *
 * Attempts to configure interrupts using the best available
 * capabilities of the hardware and kernel, i.e. MSI-X or MSI.
 **/
static int gxpci_request_irq(struct gxpci_nic *nic)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	struct net_device *netdev = nic->netdev;
#endif
	struct pci_dev *pdev = nic->pci_dev;
	struct tlr_pcie_dev *tlr = nic->tlr;
	int err = 0;

	if (tlr->msix_vectors) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
		irqreturn_t (*handler)(int, void *, struct pt_regs *);
#else
		irqreturn_t (*handler)(int, void *);
#endif
		int msix_table_index;
		char *irq_name;
		void *irq_arg;
		int msix_index;
		int v;

		msix_table_index = tlr->msix_host_nic_intr_vec_base +
			nic->if_num * nic->int_vectors;
		for (v = 0; v < nic->int_vectors; v++) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
			struct gxpci_queue_vector *vector = &nic->vectors[v];

			if (vector->tx_only)
				handler = &gxpci_tx_intr;
			else if (vector->rx_only)
				handler = &gxpci_rx_intr;
			else
				handler = &gxpci_intr;
			irq_name = vector->name;
			irq_arg = vector;
#else
			handler = &gxpci_intr;
			irq_name = netdev->name;
			irq_arg = netdev;
#endif
			msix_index = v + msix_table_index;
			err = request_irq(tlr->msix_entries[msix_index].vector,
					  handler, 0, irq_name, irq_arg);
			if (err) {
				pr_err("%s: MSI-X gxpci_request_irq failure: "
				       "%d\n", irq_name, err);
			}
		}
	} else {
		char *irq_name;
		void *irq_arg;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
		struct gxpci_queue_vector *vector = &nic->vectors[0];
		irq_name = vector->name;
		irq_arg = vector;
#else
		irq_name = netdev->name;
		irq_arg = netdev;
#endif
		err = request_irq(pdev->irq, gxpci_intr, IRQF_SHARED,
				  irq_name, irq_arg);
		if (err) {
			pr_err("%s: MSI gxpci_request_irq failure: %d\n",
			       irq_name, err);
		}
	}

	return err;
}

static void gxpci_free_irq(struct gxpci_nic *nic)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	struct net_device *netdev = nic->netdev;
#endif
	struct tlr_pcie_dev *tlr = nic->tlr;
	int msix_table_index;

	if (tlr->msix_vectors) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
		int v;
		int msix_index;
		struct gxpci_queue_vector *vector;
		msix_table_index = tlr->msix_host_nic_intr_vec_base +
	        	nic->if_num * nic->int_vectors;
		for (v = 0; v < nic->int_vectors; v++) {
			vector = &nic->vectors[v];
			msix_index = v + msix_table_index;
			free_irq(tlr->msix_entries[msix_index].vector, vector);
		}
#else
		msix_table_index = tlr->msix_host_nic_intr_vec_base +
			nic->if_num;
		free_irq(tlr->msix_entries[msix_table_index].vector, netdev);
#endif
	} else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
		struct gxpci_queue_vector *vector = &nic->vectors[0];
		free_irq(nic->pci_dev->irq, vector);
#else
		free_irq(nic->pci_dev->irq, netdev);
#endif
	}
}

/**
 * gxpci_setup_tx_resources - allocate Tx resources (Descriptors).
 * @nic: board private structure
 *
 * Return 0 on success, negative on failure
 **/
static int gxpci_setup_tx_resources(struct gxpci_nic *nic)
{
	int err = -ENOMEM;
	int tx;

	for (tx = 0; tx < nic->num_tx_queues; tx++) {
		struct gxpci_ring *tx_ring = &nic->tx_ring[tx];
		int size;

		tx_ring->index = tx;

		size = sizeof(struct gxpci_buffer) * tx_ring->count;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,38)
		tx_ring->buffer_infos = vzalloc(size);
#else
		tx_ring->buffer_infos = vmalloc(size);
#endif
		if (!tx_ring->buffer_infos)
			goto err_buffer_infos;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
		memset(tx_ring->buffer_infos, 0, size);
#endif

		tx_ring->desc = nic->nic_desc->h2t_desc[tx].dma_cmds;

		tx_ring->commands_posted = 0;
		tx_ring->commands_consumed = 0;
	}

	return 0;

err_buffer_infos:
	while (tx--) {
		struct gxpci_ring *tx_ring = &nic->tx_ring[tx];

		vfree(tx_ring->buffer_infos);
	}
	pr_err(DRV_NAME ": Unable to allocate memory "
	       "for the transmit descriptor ring\n");
	return err;
}

/**
 * gxpci_setup_rx_resources - allocate Rx resources (Descriptors).
 * @nic: board private structure
 *
 * Returns 0 on success, negative on failure
 **/
static int gxpci_setup_rx_resources(struct gxpci_nic *nic)
{
	int err = -ENOMEM;
	int rx;

	for (rx = 0; rx < nic->num_rx_queues; rx++) {
		struct gxpci_ring *rx_ring = &nic->rx_ring[rx];
		int size;

		rx_ring->index = rx;

		size = sizeof(struct gxpci_buffer) * rx_ring->count;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,38)
		rx_ring->buffer_infos = vzalloc(size);
#else
		rx_ring->buffer_infos = vmalloc(size);
#endif
		if (!rx_ring->buffer_infos)
			goto err_buffer_infos;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
		memset(rx_ring->buffer_infos, 0, size);
#endif

		rx_ring->desc = nic->nic_desc->t2h_desc[rx].dma_cmds;

		rx_ring->commands_posted = 0;
		rx_ring->commands_consumed = 0;
	}

	return 0;

err_buffer_infos:
	while (rx--) {
		struct gxpci_ring *rx_ring = &nic->rx_ring[rx];

		vfree(rx_ring->buffer_infos);
	}
	pr_err(DRV_NAME ": Unable to allocate memory "
	       "for the receive descriptor ring\n");
	return err;
}

/**
 * gxpci_clean_tx_ring - Free Tx Buffers.
 * @nic: board private structure
 **/
static void gxpci_clean_tx_ring(struct gxpci_nic *nic,
				struct gxpci_ring *tx_ring)
{
	struct pci_dev *pdev = nic->pci_dev;
	struct gxpci_buffer *buffer_info;
	unsigned long size;
	unsigned int i;

	if (tx_ring->buffer_infos == NULL)
		return;

	for (i = 0; i < tx_ring->count; i++) {
		buffer_info = &tx_ring->buffer_infos[i];

		if (buffer_info->dma) {
			pci_unmap_single(pdev, buffer_info->dma,
				buffer_info->length, PCI_DMA_TODEVICE);
			buffer_info->dma = 0;
		}
		if (buffer_info->skb) {
			dev_kfree_skb_any(buffer_info->skb);
			buffer_info->skb = NULL;
		}
	}

	size = sizeof(struct gxpci_buffer) * tx_ring->count;
	memset(tx_ring->buffer_infos, 0, size);

	tx_ring->commands_posted = 0;
	tx_ring->commands_consumed = 0;
}

/**
 * gxpci_free_tx_resources - Free Tx Resources per Queue.
 * @nic: board private structure
 *
 * Free all transmit software resources
 **/
static void gxpci_free_tx_resources(struct gxpci_nic *nic)
{
	int tx;

	for (tx = 0; tx < nic->num_tx_queues; tx++) {
		struct gxpci_ring *tx_ring = &nic->tx_ring[tx];

		gxpci_clean_tx_ring(nic, tx_ring);

		if (tx_ring->buffer_infos)
			vfree(tx_ring->buffer_infos);

		tx_ring->buffer_infos = NULL;
		tx_ring->desc = NULL;
	}
}

/**
 * gxpci_free_rx_resources - Free Rx Resources.
 * @nic: board private structure
 *
 * Free all receive software resources
 **/
static void gxpci_free_rx_resources(struct gxpci_nic *nic)
{
	int rx;

	for (rx = 0; rx < nic->num_rx_queues; rx++) {
		struct gxpci_ring *rx_ring = &nic->rx_ring[rx];

		gxpci_clean_rx_ring(nic, rx_ring);

		if (rx_ring->buffer_infos)
			vfree(rx_ring->buffer_infos);

		rx_ring->buffer_infos = NULL;
		rx_ring->desc = NULL;
	}
}

#ifdef GXPCI_NETLIB_VNIC
static void gxpci_nic_mpipe_sync_mac(struct gxpci_nic *nic)
{
	struct net_device *netdev = nic->netdev;
	uint64_t mac_addr;
	int i, bit;

	mac_addr = gxpci_host_nic_reg_rd(nic, GXPCI_TILE_MPIPE_MAC_ADDR);

	for (i = ETH_ALEN - 1, bit = 0; i >= 0; i--, bit += 8)
		netdev->dev_addr[i] = mac_addr >> bit;
}

static int gxpci_nic_mpipe_up(struct gxpci_nic *nic)
{
	uint64_t rnt;

	rnt = gxpci_host_nic_reg_rd(nic, GXPCI_TILE_MPIPE_LINK_STATUS);
	/* Gx side link is already up. */
	if (rnt)
		return 0;

	/* Tell Gx side to link up. */
	gxpci_host_nic_reg_wr(nic, GXPCI_TILE_MPIPE_LINK_UP, 0);

	return 0;
}
#endif

static void gxpci_up(struct gxpci_nic *nic, int reset)
{
	int i;

	for (i = 0; i < nic->num_rx_queues; i++) {
		struct gxpci_ring *rx_ring = &nic->rx_ring[i];

		gxpci_alloc_rx_buffers(nic, rx_ring,
				       gxpci_desc_unused(rx_ring));
	}

	/* Leave carrier/operstate OFF/DOWN if it is called by channel reset. */
	if (!reset) {
		clear_bit(__GXPCI_DOWN, &nic->state);
		netif_carrier_on(nic->netdev);
	}
}

/**
 * Send the channel reset event to all the Tx/Rx queues on Tile side.
 **/
#if GXPCI_HOST_NIC_CHAN_RESET
static void reset_tile_queues(struct gxpci_nic *nic)
{
	int i;

	for (i = 0; i < nic->num_tx_queues; i++) {
		struct gxpci_ring *tx_ring = &nic->tx_ring[i];

		writel(GXPCI_CHAN_RESET, &tx_ring->regs->queue_status);
	}

	for (i = 0; i < nic->num_rx_queues; i++) {
		struct gxpci_ring *rx_ring = &nic->rx_ring[i];

		writel(GXPCI_CHAN_RESET, &rx_ring->regs->queue_status);
	}
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
static void gxpci_napi_disable(struct gxpci_nic *nic)
{
	int i;

        for (i = 0; i < nic->int_vectors; i++) {
                struct gxpci_queue_vector *vec = &nic->vectors[i];

                napi_disable(&vec->napi);
        }
}
#endif

#ifdef GXPCI_NETLIB_VNIC
static void gxpci_nic_mpipe_down(struct gxpci_nic *nic)
{
	uint64_t rnt;

	rnt = gxpci_host_nic_reg_rd(nic, GXPCI_TILE_MPIPE_LINK_STATUS);
	/* Gx side link is already down. */
	if (rnt == 0)
		return;

	/* Tell Gx side to link down. */
	gxpci_host_nic_reg_wr(nic, GXPCI_TILE_MPIPE_LINK_DOWN, 0);
}
#endif

static void gxpci_down(struct gxpci_nic *nic)
{
	struct net_device *netdev = nic->netdev;
#ifdef GXPCI_NETLIB_VNIC
	int sleep_msecs = 0;
#endif
#if GXPCI_HOST_NIC_CHAN_RESET
	int i;
#endif

	/*
	 * Signal that we're down so the interrupt handler does not
	 * reschedule our watchdog timer.
	 */
	set_bit(__GXPCI_DOWN, &nic->state);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	netif_tx_stop_all_queues(netdev);
#else
	netif_stop_queue(netdev);
#endif

#if GXPCI_HOST_NIC_CHAN_RESET
	/* Disable transfers by resetting PCIe channels. */
	reset_tile_queues(nic);
#endif

#ifdef GXPCI_NETLIB_VNIC
	while (1) {
		/* Timeout. */
		if (sleep_msecs >= GXPCI_RELEASE_WAIT_MAX_MSECS)
			break;

		/* Exiting due to a signal. */
		if (msleep_interruptible(GXPCI_QUEUE_MONITOR_INTERVAL))
			break;

		sleep_msecs += GXPCI_QUEUE_MONITOR_INTERVAL;
	}
#else
	msleep(10);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	gxpci_napi_disable(nic);
#else
	netif_poll_disable(netdev);
#endif

	gxpci_irq_disable(nic);

	netif_carrier_off(netdev);

#if GXPCI_HOST_NIC_QUEUE_DEBUG
	del_timer(&dump_timer[nic->if_num]);
#endif

#if GXPCI_HOST_NIC_CHAN_RESET
	for (i = 0; i < nic->num_tx_queues; i++) {
		struct gxpci_ring *tx_ring = &nic->tx_ring[i];

		gxpci_clean_tx_ring(nic, tx_ring);
	}
	for (i = 0; i < nic->num_rx_queues; i++) {
		struct gxpci_ring *rx_ring = &nic->rx_ring[i];

		gxpci_clean_rx_ring(nic, rx_ring);
	}
#endif
}

/**
 * This checks if the Tile side app has initialized all the Tx/Rx queues.
 * If any queue on the Tile side is not ready, return failure.
 **/
static int tile_is_ready(struct gxpci_nic *nic)
{
#ifndef GXPCI_HOST_NIC_P2P
	int i;

	for (i = 0; i < nic->num_tx_queues; i++) {
		struct gxpci_ring *ring = &nic->tx_ring[i];

		if (readl(&ring->regs->queue_status) != GXPCI_TILE_CHAN_READY)
			return 0;
	}

	for (i = 0; i < nic->num_rx_queues; i++) {
		struct gxpci_ring *ring = &nic->rx_ring[i];

		if (readl(&ring->regs->queue_status) != GXPCI_TILE_CHAN_READY)
			return 0;
	}

	return 1;
#else
	/* Gxpci point-to-point driver does not need handshake. */
	return 1;
#endif /* !GXPCI_HOST_NIC_P2P */
}

/**
 * This notifies the Tile queues that host is ready.
 **/
static void host_is_ready(struct gxpci_nic *nic)
{
#ifndef GXPCI_HOST_NIC_P2P
	int i;

	for (i = 0; i < nic->num_tx_queues; i++) {
		struct gxpci_ring *ring = &nic->tx_ring[i];

		writel(GXPCI_HOST_CHAN_READY, &ring->regs->queue_status);
	}

	for (i = 0; i < nic->num_rx_queues; i++) {
		struct gxpci_ring *ring = &nic->rx_ring[i];

		writel(nic->rx_buffer_len, &ring->regs->host_rx_buf_len);
		wmb();

		writel(GXPCI_HOST_CHAN_READY, &ring->regs->queue_status);
	}
#else
	/* Gxpci point-to-point driver does not need handshake. */
	return;
#endif /* !GXPCI_HOST_NIC_P2P */
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
static void gxpci_napi_enable(struct gxpci_nic *nic)
{
	int i;

	for (i = 0; i < nic->int_vectors; i++) {
		struct gxpci_queue_vector *vec = &nic->vectors[i];

		napi_enable(&vec->napi);
	}
}
#endif

#ifdef GXPCI_HOST_SDNIC
/*
 * gxpci_sdnic_set_mac - Change the Ethernet Address of the NIC.
 * @netdev: network interface device structure
 * @p: pointer to an address structure
 *
 * Returns 0 on success, negative on failure
 **/
static int gxpci_sdnic_set_mac(struct net_device *netdev, void *p)
{
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);

	return tlr_ifconfig_req_rsp(netdev, SIOCSIFHWADDR,
				    addr->sa_data, netdev->addr_len, NULL, 0);
}
#else
/**
 * gxpci_set_mac - Change the Ethernet Address of the NIC.
 * @netdev: network interface device structure
 * @p: pointer to an address structure
 *
 * Returns 0 on success, negative on failure
 **/
static int gxpci_set_mac(struct net_device *netdev, void *p)
{
	struct sockaddr *addr = p;
#ifdef GXPCI_NETLIB_VNIC
	struct gxpci_nic *nic = netdev_priv(netdev);
	uint64_t mac_addr = 0;
#endif

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);

#ifdef GXPCI_NETLIB_VNIC
	memcpy(&mac_addr, addr->sa_data, netdev->addr_len);
	gxpci_host_nic_reg_wr(nic, GXPCI_TILE_MPIPE_MAC_ADDR, mac_addr);
#endif

	return 0;
}
#endif /* GXPCI_HOST_SDNIC */

#ifdef GXPCI_HOST_NIC_P2P
void gxpci_reset(struct gxpci_nic *nic)
{
	GXPCI_TRACE("%s: Reset channel\n", nic->netdev->name);

	/* First, do partial close work. */
	gxpci_down(nic);

	gxpci_free_tx_resources(nic);

	gxpci_free_rx_resources(nic);

	/* Second, do partial open work. */
	gxpci_setup_tx_resources(nic);

	gxpci_setup_rx_resources(nic);

	gxpci_up(nic, 1);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	gxpci_napi_enable(nic);

	netif_tx_start_all_queues(nic->netdev);
#else
	netif_poll_enable(nic->netdev);

	netif_start_queue(nic->netdev);
#endif
	gxpci_irq_enable(nic);
}
#endif

/**
 * gxpci_open - Called when a network interface is made active.
 * @netdev: network interface device structure
 *
 * Returns 0 on success, negative value on failure
 *
 * The open entry point is called when a network interface is made
 * active by the system (IFF_UP).  At this point all resources needed
 * for transmit and receive operations are allocated, the interrupt
 * handler is registered with the OS, the watchdog timer is started,
 * and the stack is notified that the interface is ready.
 **/
static int gxpci_open(struct net_device *netdev)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	struct tlr_pcie_dev *tlr = nic->tlr;
	int err;

	spin_lock(&tlr->is_ready_lock);
	if (!tlr->is_ready) {
		spin_unlock(&tlr->is_ready_lock);
		return -EBUSY;
	}
	spin_unlock(&tlr->is_ready_lock);

	down(&reboot_mutex);
	tlr->board->host_nic_cnt++;
	up(&reboot_mutex);

	/* Disallow open during test. */
	if (test_bit(__GXPCI_TESTING, &nic->state))
		return -EBUSY;

	netif_carrier_off(netdev);

#if GXPCI_HOST_NIC_CHAN_RESET
	/* If the Tile side SW is not ready, return failure. */
	if (!tile_is_ready(nic))
		return -EBUSY;

	/* Allocate transmit descriptors. */
	err = gxpci_setup_tx_resources(nic);
	if (err)
		return err;

	/* Allocate receive descriptors. */
	err = gxpci_setup_rx_resources(nic);
	if (err) {
		gxpci_free_tx_resources(nic);
		return err;
	}
#else
	if (nic->gxpci_nic_status == GXPCI_HOST_NIC_UNINITIALIZED) {
		/* If the Tile side SW is not ready, return failure. */
		if (!tile_is_ready(nic))
			return -EBUSY;

		/* Allocate transmit descriptors. */
		err = gxpci_setup_tx_resources(nic);
		if (err)
			return err;

		/* Allocate receive descriptors. */
		err = gxpci_setup_rx_resources(nic);
		if (err) {
			gxpci_free_tx_resources(nic);
			return err;
		}
		nic->gxpci_nic_status = GXPCI_HOST_NIC_READY;
	}
#endif

	err = gxpci_request_irq(nic);
	if (err) {
#if GXPCI_HOST_NIC_CHAN_RESET
		gxpci_free_tx_resources(nic);
		gxpci_free_rx_resources(nic);
#else
		if (nic->gxpci_nic_status & GXPCI_HOST_NIC_READY) {
			gxpci_free_tx_resources(nic);
			gxpci_free_rx_resources(nic);
		}
#endif
		return err;
	}

	gxpci_up(nic, 0);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	gxpci_napi_enable(nic);

	gxpci_irq_enable(nic);

	netif_tx_start_all_queues(netdev);
#else
	netif_poll_enable(netdev);

	gxpci_irq_enable(nic);

	netif_start_queue(netdev);
#endif

	/* Notify the Tile side of the host readiness. */
	host_is_ready(nic);

#ifdef GXPCI_NETLIB_VNIC
	/* Sync MAC address from Tile side. */
	if (!(nic->gxpci_nic_status & GXPCI_HOST_NIC_MAC_INITIALIZED)) {
		gxpci_nic_mpipe_sync_mac(nic);
		nic->gxpci_nic_status |= GXPCI_HOST_NIC_MAC_INITIALIZED;
	}

	/* Link up Tile side. */
	err = gxpci_nic_mpipe_up(nic);
	if (err) {
		pr_err("gxpci_nic_mpipe_up failed = %d\n", err);
		return err;
	}
#endif

#ifdef GXPCI_HOST_NIC_P2P
	writel(GXPCI_HOST_CHAN_READY,
	       &nic->nic_regs->queue_pair_status.host_queue_status);
#endif

#if GXPCI_HOST_NIC_QUEUE_DEBUG
	init_timer(&dump_timer[nic->if_num]);
	dump_timer[nic->if_num].data = (unsigned long) nic;
	dump_timer[nic->if_num].function = dump;
	dump_timer[nic->if_num].expires = jiffies + 30 * HZ;
	add_timer(&dump_timer[nic->if_num]);
#endif

	return 0;
}

/**
 * gxpci_close_nic - Disables a network interface.
 * @nic: network interface device structure
 *
 * Returns 0, this is not allowed to fail
 *
 * The close entry point is called when an interface is de-activated
 * by the OS.  The hardware is still under the drivers control, but
 * needs to be disabled.  A global MAC reset is issued to stop the
 * hardware, and all transmit and receive resources are freed.
 **/
static void gxpci_close_nic(struct gxpci_nic *nic)
{
#ifdef GXPCI_HOST_NIC_P2P
	int sleep_msecs = 0;
#endif

	/* Notify the Tile side of the host reset. */
#if GXPCI_HOST_NIC_CHAN_RESET
	struct tlr_pcie_dev *tlr = nic->tlr;
	struct gxpci_queue_pair_status *queue_pair_status =
		&tlr->queue_sts_array->nic_sts[nic->if_num];
#endif

#if GXPCI_HOST_NIC_CHAN_RESET
	queue_pair_status->host_queue_status = GXPCI_CHAN_RESET;
#endif

#ifdef GXPCI_HOST_NIC_P2P
	writel(GXPCI_CHAN_RESET,
	       &nic->nic_regs->queue_pair_status.host_queue_status);
	wmb();

	/* Trigger the Tile side MMI interrupt. */
	writeq(1, nic->intr_regs);

	nic->channel_state = __GXPCI_NIC_DOWN;

	while (readl(&nic->nic_regs->queue_pair_status.host_queue_status) !=
	       GXPCI_CHAN_RESET_ACK) {

		atomic_compiler_barrier();

		if (sleep_msecs >= GXPCI_RELEASE_WAIT_MAX_MSECS)
			break;

		/* Exiting due to a signal. */
		if (msleep_interruptible(GXPCI_QUEUE_MONITOR_INTERVAL))
			break;

		sleep_msecs += GXPCI_QUEUE_MONITOR_INTERVAL;
	}
#endif
}

/**
 * gxpci_close - Disables a network interface.
 * @netdev: network interface device structure
 *
 * Returns 0, this is not allowed to fail
 *
 * The close entry point is called when an interface is de-activated
 * by the OS.  The hardware is still under the drivers control, but
 * needs to be disabled.  A global MAC reset is issued to stop the
 * hardware, and all transmit and receive resources are freed.
 **/
static int gxpci_close(struct net_device *netdev)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	struct tlr_pcie_dev *tlr = nic->tlr;

	WARN_ON(test_bit(__GXPCI_RESETTING, &nic->state));

#ifdef GXPCI_NETLIB_VNIC
	/* Link down Tile side. */
	gxpci_nic_mpipe_down(nic);
#endif

	gxpci_close_nic(nic);

	if (!test_bit(__GXPCI_DOWN, &nic->state)) {
		gxpci_down(nic);
		gxpci_free_irq(nic);
	}

#if GXPCI_HOST_NIC_CHAN_RESET
	gxpci_free_tx_resources(nic);
	gxpci_free_rx_resources(nic);
#endif

	down(&reboot_mutex);
	tlr->board->host_nic_cnt--;
	up(&reboot_mutex);

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
static u16 gxpci_select_queue(struct net_device *netdev, struct sk_buff *skb)
{
#define TX_Q_HASH
#ifndef TX_Q_HASH
	u16 txq = smp_processor_id();
	while (unlikely(txq >= netdev->real_num_tx_queues))
		txq -= netdev->real_num_tx_queues;
	return txq;
#else
	return skb_tx_hash(netdev, skb);
#endif
}
#endif 

void gxpci_net_dev_close(struct tlr_pcie_dev *tlr, int if_num)
{
        struct gxpci_nic *nic = tlr->net_devs[if_num];

	if (nic == NULL)
		return;

	rtnl_lock();
	if (nic->netdev->flags & IFF_UP)
		dev_close(nic->netdev);
	rtnl_unlock();
}

/*
 * The packet transmit routine.
 */
static int gxpci_xmit_frame(struct sk_buff *skb, struct net_device *netdev)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	struct pci_dev *pdev = nic->pci_dev;
	struct gxpci_buffer *buffer_info;
	tile_nic_host_desc_t host_desc = { .desc = 0 };
	unsigned int len = skb->len;
	struct gxpci_ring *tx_ring;
	tile_nic_dma_cmd_t *dma;
	unsigned int index;

	if (test_bit(__GXPCI_DOWN, &nic->state)) {
		dev_kfree_skb_any(skb);
		nic->stats.tx_carrier_errors++;
		nic->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

#ifdef GXPCI_HOST_NIC_P2P
	if (nic->channel_state == __GXPCI_NIC_DOWN) {
		dev_kfree_skb_any(skb);
		nic->stats.tx_carrier_errors++;
		nic->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}
#endif

	if (unlikely(len == 0)) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	if (unlikely(skb_is_nonlinear(skb))) {
		if (net_ratelimit())
			pr_warning("Can not handle packet with fragments\n");
		skb_linearize(skb);
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	tx_ring = &nic->tx_ring[skb->queue_mapping];
#else
	tx_ring = &nic->tx_ring[0];
#endif

	if (unlikely(gxpci_desc_unused(tx_ring) == 0)) {

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
		netif_stop_subqueue(netdev, tx_ring->index);
#else
		netif_stop_queue(netdev);
#endif
		smp_mb();

		/*
 		 * We need to check again in case another CPU has just
 		 * made room available.
 		 */
		if (likely(gxpci_desc_unused(tx_ring) == 0))
			return NETDEV_TX_BUSY;

		/* A reprieve! */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
		netif_start_subqueue(netdev, tx_ring->index);
#else
		netif_start_queue(netdev);
#endif
	}

#ifdef GXPCI_TX_DROP_PACKETS
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
#endif

	index = tx_ring->commands_posted & tx_ring->count_mask;
	buffer_info = &tx_ring->buffer_infos[index];

	buffer_info->length = len;
	buffer_info->skb = skb;
	buffer_info->dma = pci_map_single(pdev, skb->data,
					  len, PCI_DMA_TODEVICE);
	if (!buffer_info->dma) {
		dev_kfree_skb_any(skb);
		nic->stats.tx_errors++;
		nic->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

#ifdef GXPCI_DUMP_PACKETS
	dump_packet(skb->data, len, "tx");
#endif

	GXPCI_TRACE("WRITE size= %d, bus_addr= %#llx\n",
		    buffer_info->length, buffer_info->dma);

	host_desc.size = buffer_info->length;
	host_desc.filled = 1;

#ifdef GXPCI_NETLIB_VNIC
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,2,0)
	if (unlikely(skb_vlan_tag_present(skb))) {
#else
	if (unlikely(vlan_tx_tag_present(skb))) {
#endif
		host_desc.vlan_packet = 1;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,2,0)
		host_desc.vlan_tag = skb_vlan_tag_get(skb);
#else
		host_desc.vlan_tag = vlan_tx_tag_get(skb);
#endif
	}
#endif

#if defined(GXPCI_NETLIB_VNIC) || defined(GXPCI_HOST_SDNIC)

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		unsigned int csum_start = skb_transport_offset(skb);
		host_desc.csum = 1;
		host_desc.csum_start = csum_start;
		host_desc.csum_offset = csum_start + skb->csum_offset;
	}
#endif

#endif

	dma = &tx_ring->desc[index];
	writeq(buffer_info->dma, &dma->host_addr);
	wmb();
	writeq(host_desc.desc, &dma->host_desc);

#ifdef GXPCI_HOST_NIC_P2P
	wmb();

	/* Trigger the Tile side MMI interrupt. */
	writeq(1, nic->intr_regs);
#endif /* GXPCI_HOST_NIC_P2P */

	tx_ring->commands_posted++;
	netdev->trans_start = jiffies;

	return NETDEV_TX_OK;
}

static struct net_device_stats *gxpci_get_stats(struct net_device *netdev)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	return (&nic->stats);
}

static void gxpci_reinit_locked(struct gxpci_nic *nic)
{
        WARN_ON(in_interrupt());
        while (test_and_set_bit(__GXPCI_RESETTING, &nic->state))
                msleep(1);
        gxpci_down(nic);

        gxpci_up(nic, 0);
        clear_bit(__GXPCI_RESETTING, &nic->state);
}

#ifdef GXPCI_HOST_SDNIC
/**
 * gxpci_sdnic_change_mtu - Change the Maximum Transfer Unit.
 * @netdev: network interface device structure
 * @new_mtu: new value for maximum frame size
 *
 * Returns 0 on success, negative on failure
 **/
static int gxpci_sdnic_change_mtu(struct net_device *netdev, int new_mtu)
{
	int max_frame = new_mtu + ETH_HLEN + ETH_FCS_LEN;

	/* MTU < 68 is an error and causes problems on some kernels */
	if (new_mtu < 68 || max_frame > GXPCI_MAX_JUMBO_FRAME_SIZE)
		return -EINVAL;

	pr_info("changing MTU from %d to %d\n", netdev->mtu, new_mtu);

	/* must set new MTU before calling down or up */
	netdev->mtu = new_mtu;

	return tlr_ifconfig_req_rsp(netdev, SIOCSIFMTU, (char *)&new_mtu,
				    sizeof(new_mtu), NULL, 0);
}
#else
/**
 * gxpci_change_mtu - Change the Maximum Transfer Unit.
 * @netdev: network interface device structure
 * @new_mtu: new value for maximum frame size
 *
 * Returns 0 on success, negative on failure
 **/
static int gxpci_change_mtu(struct net_device *netdev, int new_mtu)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	int max_frame = new_mtu + ETH_HLEN + ETH_FCS_LEN;

	/* MTU < 68 is an error and causes problems on some kernels */
	if ((new_mtu < 68) || (max_frame > GXPCI_MAX_JUMBO_FRAME_SIZE))
		return -EINVAL;

	pr_info("changing MTU from %d to %d\n", netdev->mtu, new_mtu);
	/* must set new MTU before calling down or up */
	netdev->mtu = new_mtu;

#ifdef GXPCI_NETLIB_VNIC
	gxpci_host_nic_reg_wr(nic, GXPCI_TILE_MPIPE_JUMBO, new_mtu);
#else
	if (netif_running(netdev))
		gxpci_reinit_locked(nic);
#endif

	return 0;
}
#endif /* GXPCI_HOST_SDNIC */

/**
 * gxpci_tx_timeout - Respond to a Tx Hang.
 * @netdev: network interface device structure
 **/
static void gxpci_tx_timeout(struct net_device *netdev)
{
	struct gxpci_nic *nic = netdev_priv(netdev);

	/* Do the reset outside of interrupt context. */
	schedule_work(&nic->reset_task);
}

static void gxpci_reset_task(struct work_struct *work)
{
	struct gxpci_nic *nic;
	nic = container_of(work, struct gxpci_nic, reset_task);

	/* If we're already down or resetting, just bail. */
	if (test_bit(__GXPCI_DOWN, &nic->state) ||
	    test_bit(__GXPCI_RESETTING, &nic->state))
		return;
	
	pr_info("Reset NIC\n");
	gxpci_reinit_locked(nic);
}

#ifdef GXPCI_HOST_SDNIC
/**
 * gxpci_sdnic_set_rx_mode - Unicast, Multicast and Promiscuous mode set.
 * @netdev: network interface device structure
 *
 * The set_rx_method entry point is called whenever the unicast/multicast
 * address list or the network interface flags are updated.  This routine is
 * responsible for configuring the hardware for proper unicast, multicast and
 * promiscuous mode.
 **/
static void gxpci_sdnic_set_rx_mode(struct net_device *netdev)
{
	tlr_ifconfig_req_rsp(netdev, SIOCSIFFLAGS, (char *)&netdev->flags,
			     sizeof(short int), NULL, 0);
}
#else
/**
 * gxpci_set_rx_mode - Unicast, Multicast and Promiscuous mode set.
 * @netdev: network interface device structure
 *
 * The set_rx_method entry point is called whenever the unicast/multicast
 * address list or the network interface flags are updated.  This routine is
 * responsible for configuring the hardware for proper unicast, multicast and
 * promiscuous mode.
 **/
static void gxpci_set_rx_mode(struct net_device *netdev)
{
#ifdef GXPCI_NETLIB_VNIC
	struct gxpci_nic *nic = netdev_priv(netdev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	//struct dev_addr_list *mc_ptr = netdev->mc_list;
	struct netdev_hw_addr *mc_ptr = netdev->mc.list.next; // -- ertl-liyixiao
	struct netdev_hw_addr *ha;
	uint64_t data;
	int i, exact_match_entries = GXPCI_HOST_NETLIB_MAX_EMR_NUM;
	bool use_uc = false;
#endif

	/* Check for Promiscuous and All Multicast modes. */
	if (netdev->flags & IFF_PROMISC) {
		gxpci_host_nic_reg_wr(nic, GXPCI_TILE_MPIPE_PROMISCUOUS, 0);
	} else {
		if (netdev->flags & IFF_ALLMULTI) {
			gxpci_host_nic_reg_wr(nic, GXPCI_TILE_MPIPE_MULTICAST,
					      0);
		} else {
			gxpci_host_nic_reg_wr(nic, GXPCI_TILE_MPIPE_UNICAST, 0);
		}
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	if (netdev_uc_count(netdev) > exact_match_entries - 1) {
		gxpci_host_nic_reg_wr(nic, GXPCI_TILE_MPIPE_UPE, 0);
	} else if (!(netdev->flags & IFF_PROMISC)) {
		gxpci_host_nic_reg_wr(nic, GXPCI_TILE_MPIPE_UPD, 0);
		use_uc = true;
	}

	/*
	 * Load the first 7 addresses into the exact filters 1-7. Unicast
	 * addresses take precedence to avoid disabling unicast filtering
	 * when possible.
	 *
	 * Exact Match Register 0 is used for the station MAC address
	 * if there are not 7 addresses, go ahead and clear the filters.
	 */
	i = 1;
	if (use_uc)
		netdev_for_each_uc_addr(ha, netdev) {
			if (i == exact_match_entries)
				break;

			data = 0;
			memcpy(&data, ha->addr, ETH_ALEN);

			data = (data << 16) + i++;
			gxpci_host_nic_reg_wr(nic, GXPCI_TILE_MPIPE_SET_EMR,
					      data);
		}

	for (; i < exact_match_entries; i++) {
		if (mc_ptr) {
			data = 0;
			//memcpy(&data, mc_ptr->da_addr, ETH_ALEN);
			memcpy(&data, mc_ptr->addr, ETH_ALEN); // -- ertl-liyixiao

			data = (data << 16) + i;
			gxpci_host_nic_reg_wr(nic, GXPCI_TILE_MPIPE_SET_EMR,
					      data);

			//mc_ptr = mc_ptr->next;
			mc_ptr = mc_ptr->list.next; // -- ertl-liyixiao
		} else {
			data = i;
			gxpci_host_nic_reg_wr(nic, GXPCI_TILE_MPIPE_SET_EMR,
					      data);
		}
	}
#endif
#endif /* GXPCI_NETLIB_VNIC */
}
#endif /* GXPCI_HOST_SDNIC */

static int gxpci_ioctl(struct net_device *netdev, struct ifreq *req, int cmd)
{
	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
#ifdef GXPCI_NETLIB_VNIC
/*
 * The VLAN filter table is maintained as 64 8-byte registers,
 * representing a 4096-bit vector VLAN filter table array. Each bit
 * represents an unique VLAN ID.
 */
static void gxpci_vlan_rx_add_vid(struct net_device *netdev, u16 vid)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	struct tlr_pcie_dev *tlr = nic->tlr;
	uint64_t index, value;

	spin_lock(&tlr->lock);

	/* Get the index to the filter table. */
	index = (vid >> 6) & 0x3F;
	gxpci_host_nic_reg_wr(nic, GXPCI_TILE_VLAN_FILTER_TABLE_INDEX, index);
	value = gxpci_host_nic_reg_rd(nic, GXPCI_TILE_VLAN_FILTER_TABLE_ENTRY);
	value |= (1ULL << (vid & 0x3F));

	/* Add the VLAN ID to the filter table. */
	gxpci_host_nic_reg_wr(nic, GXPCI_TILE_VLAN_FILTER_TABLE_ENTRY_ADD_VID,
			      value);

	spin_unlock(&tlr->lock);
}

static void gxpci_vlan_rx_kill_vid(struct net_device *netdev, u16 vid)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	struct tlr_pcie_dev *tlr = nic->tlr;
	uint64_t index, value;

	spin_lock(&tlr->lock);

	/* Get the index to the filter table. */
	index = (vid >> 6) & 0x3F;
	gxpci_host_nic_reg_wr(nic, GXPCI_TILE_VLAN_FILTER_TABLE_INDEX, index);
	value = gxpci_host_nic_reg_rd(nic, GXPCI_TILE_VLAN_FILTER_TABLE_ENTRY);
	value &= ~(1ULL << (vid & 0x3F));

	/* Delete the VLAN ID from the filter table. */
	gxpci_host_nic_reg_wr(nic, GXPCI_TILE_VLAN_FILTER_TABLE_ENTRY_DEL_VID,
			      value);

	spin_unlock(&tlr->lock);
}
#endif

static const struct net_device_ops gxpci_netdev_ops = {
	.ndo_open		= gxpci_open,
	.ndo_stop		= gxpci_close,
	.ndo_select_queue	= gxpci_select_queue,
	.ndo_start_xmit		= gxpci_xmit_frame,
	.ndo_get_stats		= gxpci_get_stats,
	.ndo_validate_addr      = eth_validate_addr,
	.ndo_tx_timeout         = gxpci_tx_timeout,
	.ndo_do_ioctl           = gxpci_ioctl,
#ifdef GXPCI_NETLIB_VNIC
	.ndo_vlan_rx_add_vid	= gxpci_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid	= gxpci_vlan_rx_kill_vid,
#endif
#ifdef GXPCI_HOST_SDNIC
	.ndo_set_mac_address    = gxpci_sdnic_set_mac,
	.ndo_change_mtu         = gxpci_sdnic_change_mtu,
	.ndo_set_rx_mode        = gxpci_sdnic_set_rx_mode,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
	.ndo_set_multicast_list = gxpci_sdnic_set_rx_mode,
#endif
#else
	.ndo_set_mac_address    = gxpci_set_mac,
	.ndo_change_mtu         = gxpci_change_mtu,
	.ndo_set_rx_mode        = gxpci_set_rx_mode,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
	.ndo_set_multicast_list = gxpci_set_rx_mode,
#endif
#endif
};
#endif

static int gxpci_get_settings(struct net_device *netdev,
                              struct ethtool_cmd *ecmd)
{
        ecmd->supported = SUPPORTED_10000baseT_Full;
        ecmd->advertising |= ADVERTISED_10000baseT_Full;
        ecmd->autoneg = AUTONEG_DISABLE;

	if (netif_carrier_ok(netdev)) {
                ecmd->speed = SPEED_10000;
                ecmd->duplex = DUPLEX_FULL;
        } else {
                ecmd->speed = -1;
                ecmd->duplex = -1;
        }

        return 0;
}

static void gxpci_get_drvinfo(struct net_device *netdev,
                              struct ethtool_drvinfo *drvinfo)
{
#ifdef GXPCI_HOST_SDNIC
	tlr_ethtool_req_rsp(netdev, ETHTOOL_GDRVINFO, NULL, 0,
			    (char *)drvinfo, sizeof(*drvinfo));
#else
        struct gxpci_nic *nic = netdev_priv(netdev);

        strncpy(drvinfo->driver, gxpci_driver_name, sizeof(drvinfo->driver));
        strncpy(drvinfo->version, gxpci_driver_version,
                sizeof(drvinfo->version));
        strncpy(drvinfo->fw_version, gxpci_fw_version,
                sizeof(drvinfo->fw_version));
        strncpy(drvinfo->bus_info, pci_name(nic->pci_dev),
                sizeof(drvinfo->bus_info));
#endif /* GXPCI_HOST_SDNIC */
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
static u32 gxpci_get_rx_csum(struct net_device *netdev)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	return nic->rx_csum;
}

static int gxpci_set_rx_csum(struct net_device *netdev, u32 data)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	nic->rx_csum = data;

	return 0;
}

static u32 gxpci_get_tx_csum(struct net_device *netdev)
{
	return (netdev->features & NETIF_F_HW_CSUM) != 0;
}

static int gxpci_set_tx_csum(struct net_device *netdev, u32 data)
{
	if (data)
		netdev->features |= NETIF_F_HW_CSUM;
	else
		netdev->features &= ~NETIF_F_HW_CSUM;

	return 0;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
static int ethtool_op_get_sset_count(struct net_device *netdev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ((((struct gxpci_nic *) netdev_priv(netdev))->num_tx_queues +
			((struct gxpci_nic *) netdev_priv(netdev))->num_rx_queues) *
			(sizeof(struct gxpci_queue_stats) / sizeof(u64)));
	default:
		return -EOPNOTSUPP;
	}
}

static void ethtool_op_get_strings(struct net_device *netdev, u32 stringset,
				   u8 *data)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	char *p = (char *) data;
	int i = 0;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < nic->num_tx_queues; i++) {
			sprintf(p, "tx_queue_%u_packets", i);
			p += ETH_GSTRING_LEN;
		}

		for (i = 0; i < nic->num_rx_queues; i++) {
			sprintf(p, "rx_queue_%u_packets", i);
			p += ETH_GSTRING_LEN;
		}
		break;
	default:
		break;
	}
}

static void ethtool_op_get_ethtool_stats(struct net_device *netdev,
					 struct ethtool_stats *stats, u64 *data)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	int i = 0, j = 0, k = 0;
	u64 *queue_stat;

	int stat_count = sizeof(struct gxpci_queue_stats) / sizeof(u64);

	for (j = 0; j < nic->num_tx_queues; j++) {
		queue_stat = (u64 *) &nic->tx_ring[j].stats;

		for (k = 0; k < stat_count; k++)
			data[i + k] = queue_stat[k];
		i += k;
	}

	for (j = 0; j < nic->num_rx_queues; j++) {
		queue_stat = (u64 *) &nic->rx_ring[j].stats;

		for (k = 0; k < stat_count; k++)
			data[i + k] = queue_stat[k];
		i += k;
	}
}
#endif

static struct ethtool_ops gxpci_ethtool_ops = {
	.get_settings           = gxpci_get_settings,
	.get_drvinfo            = gxpci_get_drvinfo,
	.get_link               = ethtool_op_get_link,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
        .get_rx_csum            = gxpci_get_rx_csum,
        .set_rx_csum            = gxpci_set_rx_csum,
        .get_tx_csum            = gxpci_get_tx_csum,
        .set_tx_csum            = gxpci_set_tx_csum,
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
	.get_sset_count		= ethtool_op_get_sset_count,
	.get_strings            = ethtool_op_get_strings,
	.get_ethtool_stats      = ethtool_op_get_ethtool_stats,
#endif
};

void gxpci_set_ethtool_ops(struct net_device *netdev)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0) // -- ertl-liyixiao
    netdev->ethtool_ops = &gxpci_ethtool_ops; // -- ertl-liyixiao
#else
        SET_ETHTOOL_OPS(netdev, &gxpci_ethtool_ops);
#endif
}

static void gxpci_setup(struct net_device *netdev)
{
	ether_setup(netdev);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
	netdev->netdev_ops = &gxpci_netdev_ops;
#else
	netdev->open = gxpci_open;
	netdev->stop = gxpci_close;
	netdev->hard_start_xmit = gxpci_xmit_frame;
	netdev->get_stats = gxpci_get_stats;
	netdev->tx_timeout = gxpci_tx_timeout;
	netdev->do_ioctl = gxpci_ioctl;
#ifdef GXPCI_HOST_SDNIC
	netdev->set_mac_address = gxpci_sdnic_set_mac;
	netdev->change_mtu = gxpci_sdnic_change_mtu;
	netdev->set_multicast_list = gxpci_sdnic_set_rx_mode;
#else
	netdev->set_mac_address = gxpci_set_mac;
	netdev->change_mtu = gxpci_change_mtu;
	netdev->set_multicast_list = gxpci_set_rx_mode;
#endif /* GXPCI_HOST_SDNIC */
#endif
  gxpci_set_ethtool_ops(netdev);

	netdev->mtu = GXPCI_NIC_MTU_DEFAULT;

#ifdef GXPCI_PACKET_PROTOCOL
	netdev->flags |= IFF_NOARP;
	netdev->flags &= ~IFF_MULTICAST;
#endif
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
static int gxpci_alloc_int_vectors(struct gxpci_nic *nic)
{
	struct net_device *netdev = nic->netdev;
	int txrx_queues;
	int i;

	nic->vectors = kcalloc(nic->int_vectors,
			       sizeof(struct gxpci_queue_vector), GFP_KERNEL);
	if (!nic->vectors)
		return -ENOMEM;

	txrx_queues = min(nic->num_tx_queues, nic->num_rx_queues);
	BUG_ON(txrx_queues == 0);

	for (i = 0; i < nic->int_vectors; i++) {
		struct gxpci_queue_vector *vec = &nic->vectors[i];
		int (*poll) (struct napi_struct *, int);

		vec->nic = nic;

#ifndef USE_MSI_ONLY
		vec->ring = i;
		if (i < txrx_queues) {
			vec->tx_only = 0;
			vec->rx_only = 0;
			poll = gxpci_poll;
			sprintf(vec->name, "%s-%s-%d", netdev->name,
				"TxRx", i);
		} else if (i >= nic->num_rx_queues) {
			vec->tx_only = 1;
			poll = gxpci_poll_tx_only;
			sprintf(vec->name, "%s-%s-%d", netdev->name,
				"Tx", i);
		} else {
			vec->rx_only = 1;
			poll = gxpci_poll_rx_only;
			sprintf(vec->name, "%s-%s-%d", netdev->name,
				"Rx", i);
		}
#else
		vec->tx_only = 0;
		vec->rx_only = 0;
		poll = gxpci_poll_msi;
		sprintf(vec->name, "%s-%s", netdev->name, "TxRx");
#endif
		netif_napi_add(netdev, &vec->napi, poll,
			       GXPCI_DEFAULT_POLL_WEIGHT);
	}

	return 0;
}

static void gxpci_free_int_vectors(struct gxpci_nic *nic)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
	int i;

	for (i = 0; i < nic->int_vectors; i++) {
		struct gxpci_queue_vector *vec = &nic->vectors[i];

		netif_napi_del(&vec->napi);
	}
#endif
	kfree(nic->vectors);
}
#endif

#ifdef GXPCI_NETLIB_VNIC
static int gxpci_alloc_netlib_regs(struct gxpci_nic *nic)
{
	struct pci_dev *pdev = nic->pci_dev;

	/*
	 * Netlib registers start right behind the nic_regs,
	 * and each VNIC port has a 4KB register memory.
	 */
	uint64_t local_bar_offset = pci_resource_start(pdev, 0) +
				    GXPCI_HOST_NIC_REGS_BAR_OFFSET +
				    nic->if_num * GXPCI_HOST_NIC_REGS_SIZE +
				    sizeof(struct gxpci_host_nic_regs);
	nic->netlib_regs = ioremap(local_bar_offset, 
				   GXPCI_HOST_NETLIB_REGS_MAP_SIZE);
	if (nic->netlib_regs == NULL)
		return -EIO;
	else {
		printk("%s: vnic %d, PCI offset %llx\n", __func__,
		       nic->if_num, (unsigned long long) local_bar_offset);
		return 0;
	}
}

static void gxpci_free_netlib_regs(struct gxpci_nic *nic)
{
	if (nic->netlib_regs) {
		iounmap(nic->netlib_regs);
		nic->netlib_regs = NULL;
	}
}
#endif

static int gxpci_alloc_tx_rings(struct gxpci_nic *nic)
{
	int err;
	int i;

	nic->tx_ring = kcalloc(nic->num_tx_queues, sizeof(struct gxpci_ring),
			       GFP_KERNEL);
	if (!nic->tx_ring)
		return -ENOMEM;

	for (i = 0; i < nic->num_tx_queues; i++) {
		struct gxpci_ring *ring = &nic->tx_ring[i];

		/* Set the default ring size. */
		ring->count = GXPCI_DEFAULT_TXD;
		ring->count_mask = ring->count - 1;

		ring->regs = &nic->nic_regs->h2t_regs[i];
		if (ring->regs == NULL) {
			err = -EIO;
			goto ioremap_error;
		}
	}

	return 0;

ioremap_error:
	while (i--) {
		struct gxpci_ring *ring = &nic->tx_ring[i];
		ring->regs = NULL;
	}
	kfree(nic->tx_ring);
	nic->tx_ring = NULL;

	return err;
}

static void gxpci_free_tx_rings(struct gxpci_nic *nic)
{
        int i;

	for (i = 0; i < nic->num_tx_queues; i++) {
		struct gxpci_ring *ring = &nic->tx_ring[i];

		iounmap(ring->regs);
		ring->regs = NULL;
	}
	kfree(nic->tx_ring);
	nic->tx_ring = NULL;
}

static int gxpci_alloc_rx_rings(struct gxpci_nic *nic)
{
	int err;
	int i;

	nic->rx_ring = kcalloc(nic->num_rx_queues, sizeof(struct gxpci_ring),
			       GFP_KERNEL);
	if (!nic->rx_ring)
		return -ENOMEM;

	for (i = 0; i < nic->num_rx_queues; i++) {
		struct gxpci_ring *ring = &nic->rx_ring[i];

		/* Set the default ring size. */
		ring->count = GXPCI_DEFAULT_RXD;
		ring->count_mask = ring->count - 1;

		ring->regs = &nic->nic_regs->t2h_regs[i];
		if (ring->regs == NULL) {
			err = -EIO;
			goto ioremap_error;
		}
	}

	return 0;

ioremap_error:
	while (i--) {
		struct gxpci_ring *ring = &nic->rx_ring[i];
		ring->regs = NULL;
	}
	kfree(nic->rx_ring);
	nic->rx_ring = NULL;

	return err;
}

static void gxpci_free_rx_rings(struct gxpci_nic *nic)
{
        int i;

	for (i = 0; i < nic->num_rx_queues; i++) {
		struct gxpci_ring *ring = &nic->rx_ring[i];

		iounmap(ring->regs);
		ring->regs = NULL;
	}
	kfree(nic->rx_ring);
	nic->rx_ring = NULL;
}

static void gxpci_nic_io_exit(struct gxpci_nic *nic)
{
	if (nic->nic_regs) {
		iounmap(nic->nic_regs);
		nic->nic_regs = NULL;
	}
	if (nic->nic_desc) {
		iounmap(nic->nic_desc);
		nic->nic_desc = NULL;
	}
#ifdef GXPCI_HOST_NIC_P2P
	if (nic->intr_regs) {
		iounmap(nic->intr_regs);
		nic->intr_regs = NULL;
	}
#endif
}

static int gxpci_nic_io_init(struct gxpci_nic *nic)
{
	struct tlr_pcie_dev *tlr = nic->tlr;
	struct pci_dev *pdev = tlr->pci_dev;

	/* Map the NIC registers in BAR0 space. */
	nic->nic_regs = ioremap(pci_resource_start(pdev, 0) +
				GXPCI_HOST_NIC_REGS_BAR_OFFSET +
				(nic->if_num * GXPCI_HOST_NIC_REGS_SIZE),
				sizeof(struct gxpci_host_nic_regs));
	if (!nic->nic_regs)
		return -ENOMEM;

	/* Map the NIC DMA descriptor arrays in BAR2 space. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	nic->nic_desc = ioremap_wc(pci_resource_start(pdev, 2) +
#else
	nic->nic_desc = ioremap(pci_resource_start(pdev, 2) +
#endif
				tlr->link_index * GXPCI_EP_HUGE_PAGE_SIZE +
				(nic->if_num * GXPCI_HOST_NIC_DMA_RING_SIZE),
				sizeof(struct gxpci_host_nic_desc));
	if (!nic->nic_desc)
		return -ENOMEM;

#ifdef GXPCI_HOST_NIC_P2P
	nic->intr_regs = ioremap(pci_resource_start(pdev, 0) +
				 GXPCI_HOST_NIC_STREAMS_MMI_REGS_OFFSET +
				 nic->if_num * GXPCI_HOST_NIC_REGS_MAP_SIZE,
				 sizeof(uint64_t));
	if (!nic->intr_regs)
		return -ENOMEM;
#endif /* GXPCI_HOST_NIC_P2P */

	return 0;
}

/**
 * gxpci_nic_init - register a virtual network interface.
 *
 * Returns 0 on success, negative on failure
 **/
static int gxpci_nic_init(struct tlr_pcie_dev *tlr, int if_num)
{
	struct pci_dev *pdev = tlr->pci_dev;
        struct net_device *netdev;
        struct gxpci_nic *nic;
	char ifname[IFNAMSIZ];
	int err;

#ifdef GXPCI_HOST_SDNIC
	/* Make it one based in conformance with tile side. */
	sprintf(ifname, "gxpci%d-eth%d", tlr->link_index, (if_num + 1));
#else
	sprintf(ifname, "gxpci%d-%d", tlr->link_index, if_num);
#endif /* GXPCI_HOST_SDNIC */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,17,0) // -- ertl-liyixiao
	netdev = alloc_netdev_mq(sizeof(struct gxpci_nic), ifname,
				 NET_NAME_ENUM, gxpci_setup, tlr->nic_tx_queues); // -- ertl-liyixiao
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
	netdev = alloc_netdev_mq(sizeof(struct gxpci_nic), ifname,
				 gxpci_setup, tlr->nic_tx_queues);
#else
	netdev = alloc_netdev(sizeof(struct gxpci_nic), ifname, gxpci_setup);
#endif
	if (!netdev) {
		err = -ENOMEM;
		goto err_alloc_netdev;
	}

	SET_NETDEV_DEV(netdev, &pdev->dev);

	netdev->irq = pdev->irq;
	nic = netdev_priv(netdev);

	nic->netdev = netdev;
	nic->tlr = tlr;
	nic->if_num = if_num;
	nic->pci_dev = pdev;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
	nic->num_tx_queues = tlr->nic_tx_queues;
	nic->num_rx_queues = tlr->nic_rx_queues;
#ifndef USE_MSI_ONLY
	nic->int_vectors =
		MAX(nic->num_tx_queues, nic->num_rx_queues);
#else
	nic->int_vectors = 1;
#endif
#else
	nic->num_tx_queues = 1;
	nic->num_rx_queues = 1;
	nic->int_vectors = 1;
#endif

	nic->rx_csum = 1;

	/* Set the default receive buffer size. */
	nic->rx_buffer_len = GXPCI_NIC_RX_BUF_LEN;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
	INIT_WORK(&nic->reset_task, gxpci_reset_task);
#else
	INIT_WORK(&nic->reset_task,
		  (void (*)(void *))gxpci_reset_task, netdev);
#endif

#ifdef GXPCI_NETLIB_VNIC
	netdev->features = NETIF_F_HW_CSUM |
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
			   NETIF_F_HW_VLAN_CTAG_TX |
			   NETIF_F_HW_VLAN_CTAG_FILTER;
#else
			   NETIF_F_HW_VLAN_TX |
			   NETIF_F_HW_VLAN_FILTER;
#endif

	err = gxpci_alloc_netlib_regs(nic);
	if (err)
		goto err_netlib_regs;
#endif

#ifdef GXPCI_HOST_SDNIC
	// Enable HW checksum for SDNIC.
	netdev->features |= NETIF_F_HW_CSUM;
#endif

	err = gxpci_nic_io_init(nic);
	if (err)
		goto err_io_init;

	err = gxpci_alloc_tx_rings(nic);
	if (err)
		goto err_tx_rings;

	err = gxpci_alloc_rx_rings(nic);
	if (err)
		goto err_rx_rings;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	err = gxpci_alloc_int_vectors(nic);
	if (err)
		goto err_int_vectors;
#else
	netdev->poll = &gxpci_poll;
	netdev->weight = GXPCI_DEFAULT_POLL_WEIGHT;
#endif

	set_bit(__GXPCI_DOWN, &nic->state);
	
	/* Tell the stack to leave us alone until gxpci_open() is called. */
	netif_carrier_off(netdev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	netif_tx_stop_all_queues(netdev);
#else
	netif_stop_queue(netdev);
#endif

	/* Set a random MAC address instead of leaving it zero. */
	random_ether_addr(netdev->dev_addr);

#ifdef GXPCI_HOST_SDNIC
	/* For SDNIC Override with tile if's mac address if discovered. */
	tlr_netdev_ctrl_populate(netdev);
	if (nic->netdev_ctrl && nic->netdev_ctrl->discovered) {
		tlr_netdev_ctrl_get_mac_address(nic->netdev_ctrl,
						if_num,
						netdev->dev_addr);
	}	
#endif

	err = register_netdev(netdev);
	if (err)
		goto err_register;

	pr_info("Registering network device %s\n", ifname);

	tlr->net_devs[if_num] = nic;


	return 0;

err_register:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	gxpci_free_int_vectors(nic);
err_int_vectors:
#endif
	gxpci_free_rx_rings(nic);
err_rx_rings:
	gxpci_free_tx_rings(nic);
err_tx_rings:
err_io_init:
	gxpci_nic_io_exit(nic);
#ifdef GXPCI_NETLIB_VNIC
	gxpci_free_netlib_regs(nic);
err_netlib_regs:
#endif
	free_netdev(netdev);
err_alloc_netdev:

	return err;
}

/**
 * gxpci_nic_remove_locked - unregister a virtual network interface, must be
 *			     called with rtnl lock held.
 * @tlr: tilegx pci device.
 * @if_num: nic interface index to remove.
 * @force: force removal of in-use interfaces.
 **/
static int gxpci_nic_remove_locked(struct tlr_pcie_dev *tlr, int if_num,
				   bool force)
{
	struct gxpci_nic *nic = tlr->net_devs[if_num];
	struct pci_dev *pdev = tlr->pci_dev;
	struct net_device *netdev;

	if (!nic || !nic->netdev) {
		dev_err(&pdev->dev, "error removing, missing device (if %d)\n",
			if_num);
		return -ENODEV;
	}

	netdev = nic->netdev;
	if (!force && netdev->flags & IFF_UP) {
		dev_err(&pdev->dev, "error removing, if %d is up\n", if_num);
		return -EBUSY;
	}

	if (!test_bit(__GXPCI_DOWN, &nic->state)) {
		gxpci_down(nic);
		gxpci_free_irq(nic);
	}

#if GXPCI_HOST_NIC_CHAN_RESET
	if (nic->gxpci_nic_status & GXPCI_HOST_NIC_READY) {
		gxpci_free_tx_resources(nic);
		gxpci_free_rx_resources(nic);
	}
#endif

	/* flush_scheduled_work() ? */

	unregister_netdevice(netdev);

#ifdef GXPCI_NETLIB_VNIC
	gxpci_free_netlib_regs(nic);
#endif

	gxpci_nic_io_exit(nic);

	if (nic->tx_ring)
		kfree(nic->tx_ring);
	if (nic->rx_ring)
		kfree(nic->rx_ring);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
        gxpci_free_int_vectors(nic);
#endif

	return 0;
}

/**
 * gxpci_nic_free - release a previously unregistered network interface.
 **/
static void gxpci_nic_free(struct tlr_pcie_dev *tlr, int if_num)
{
	struct gxpci_nic *nic = tlr->net_devs[if_num];

	if (nic && nic->netdev)
		free_netdev(nic->netdev);
	tlr->net_devs[if_num] = NULL;
}

/**
 * gxpci_net_devs_init - initialize the virtual network interfaces
 *			 supported on this PCIe link.
 *
 * This is called by the Gx PCIe port driver's probe routine.
 **/
void gxpci_net_devs_init(struct tlr_pcie_dev *tlr)
{
	int i;

	for (i = 0; i < tlr->nic_ports; i++) {

		/* Just give up upon any error in setting up the interface. */
		if (gxpci_nic_init(tlr, i))
			return;
	}
	return;
}

/**
 * gxpci_net_devs_remove - remove the virtual network interfaces
 *			   supported on this PCIe link.
 *
 * This is called by the Gx PCIe port driver's remove routine.
 **/
void gxpci_net_devs_remove(struct tlr_pcie_dev *tlr)
{
	int i;

	rtnl_lock();
	for (i = 0; i < tlr->nic_ports; i++)
		gxpci_nic_remove_locked(tlr, i, true);
	rtnl_unlock();

	/* the free must be outside the rtnl lock */
	for (i = 0; i < tlr->nic_ports; i++)
		gxpci_nic_free(tlr, i);
}
