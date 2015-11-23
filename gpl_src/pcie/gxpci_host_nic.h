/*
 * gxpci_host_nic.h
 *
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
 */
#ifndef __GXPCI_HOST_NIC_H__
#define __GXPCI_HOST_NIC_H__

#include <linux/version.h>
#include <linux/if_vlan.h>
#include <linux/netdevice.h>
#include <linux/pci.h>

#if !defined(TILEPCI_HOST)
typedef struct pcie_host_nic_buffer_cmd gxpci_tx_desc_t;
typedef struct pcie_host_nic_buffer_cmd gxpci_rx_desc_t;
typedef struct pcie_host_nic_completion gxpci_tx_comp_t;
typedef struct pcie_host_nic_completion gxpci_rx_comp_t;
#endif

struct tlr_pcie_dev;

/*
 * Define the huge page size on the TILE EP node.
 * This is used to determine the offset of the host NIC DMA
 * descriptor arrays in the EP port's BAR2 space, i.e.
 * <port link index> * GXPCI_EP_HUGE_PAGE_SIZE.
 */
#define GXPCI_EP_HUGE_PAGE_SIZE		(1 << 24)

/*
 * Define a custom packet protocol number, if this driver is used
 * to deliver ingress packets to a custom packet handler. Undefine it
 * if the packets are delivered to the kernel IP stack.
 */
#if defined(TILEPCI_HOST)
#if (!defined(GXPCI_NETLIB_VNIC) && \
     !defined(GXPCI_HOST_NIC_P2P) && \
     !defined(GXPCI_HOST_SDNIC))
#define GXPCI_PACKET_PROTOCOL           0xABCD
#endif /* !GXPCI_NETLIB_VNIC && !GXPCI_HOST_NIC_P2P */
#endif /* defined(TILEPCI_HOST) */

/* Total waiting time for the reset acknowledgement from the other end. */
#define GXPCI_RELEASE_WAIT_MAX_MSECS	1000

/* MTU size, 4000 bytes by default. */
#define GXPCI_NIC_MTU_DEFAULT 		4000

/*
 * Receive buffer length.
 * MTU + ethernet header + VLAN header + IP header align offset +
 * Tilera own custom header.
 */
#define GXPCI_NIC_RX_BUF_LEN 		\
	(GXPCI_NIC_MTU_DEFAULT + ETH_HLEN + VLAN_HLEN + NET_IP_ALIGN + \
	 sizeof(tile_nic_tile_desc_t))

/* Default number of transmit ring entries. */
#define GXPCI_DEFAULT_TXD		PCIE_CMD_QUEUE_ENTRIES

/* Transmit queue restart threshold. */
#define GXPCI_WAKE_THRESHOLD		(GXPCI_DEFAULT_TXD >> 2)

/* Default number of receive ring entries. */
#define GXPCI_DEFAULT_RXD		PCIE_CMD_QUEUE_ENTRIES

/* Refill threshold for receive ring entries. */
#define GXPCI_RXD_REFILL_THRESHOLD	(GXPCI_DEFAULT_RXD >> 4)

/* Default NAPI poll weight. */
#define GXPCI_DEFAULT_POLL_WEIGHT	64

/* Default Tx budget. */
#define GXPCI_DEFAULT_TX_BUDGET		256 

/* Maximum jumbo frame size. */
#define GXPCI_MAX_JUMBO_FRAME_SIZE      10240 

#define GXPCI_HOST_NIC_CHAN_RESET	0x1

#define GXPCI_HOST_NIC_QUEUE_DEBUG	0x0

#define GXPCI_HOST_NIC_UNINITIALIZED    0x0

#define GXPCI_HOST_NIC_READY            0x1

#define GXPCI_HOST_NIC_MAC_INITIALIZED  0x2

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
/*
 * Normally, an interrupt vector is assigned to a pair of rx/tx queues.
 * When a NIC has unequal number of rx and tx queues, the vector will
 * be used by a single rx or tx queue.
 */
struct gxpci_queue_vector {
	char name[IFNAMSIZ + 16];
	struct gxpci_nic *nic;
	struct napi_struct napi;
	uint32_t ring;
	uint32_t tx_only;
	uint32_t rx_only;
};
#endif

struct gxpci_buffer {
	dma_addr_t dma;
	struct sk_buff *skb;
	uint32_t length;
};

struct gxpci_queue_stats {
	u64 packets;
};

struct gxpci_ring {
	/* The mapped address to the device's queue registers. */
	struct gxpci_host_nic_queue_regs __iomem *regs;

	/* Pointer to tile_nic_dma_cmd_t array on tile. */
	tile_nic_dma_cmd_t __iomem *desc;

	uint32_t index;

	struct gxpci_buffer *buffer_infos;

#if !defined(TILEPCI_HOST)
	/* Length of ring in bytes. */
	unsigned int size;
#endif

	/* Number of descriptors in ring. Must be power of 2. */
	unsigned int count;

	/* Mask of the ring descriptor number. */
	unsigned int count_mask;

	/* Number of commands that are posted by the host. */
	unsigned int commands_posted;

	/* Number of commands that are consumed by the device. */
	unsigned int commands_consumed;

	/* Statistics. */
	struct gxpci_queue_stats stats;
};

typedef enum gxpci_nic_chan_state_e {

	__GXPCI_NIC_DOWN,

	__GXPCI_NIC_UP,

} gxpci_nic_chan_state_t;

/* State object for each PCIe network interface. */
struct gxpci_nic {
	struct net_device *netdev;
	struct tlr_pcie_dev *tlr;
	struct pci_dev *pci_dev;
	struct net_device_stats stats;
	struct work_struct reset_task;

	u32 gxpci_nic_status;

	/*
	 * The network interface number, between 0 and
	 * (GXPCI_HOST_NIC_COUNT - 1).
	 */
	int if_num;

	/* The mapped address to the device's queue registers. */
	struct gxpci_host_nic_regs __iomem *nic_regs;

	/* Pointer to tile_nic_dma_cmd_t array on tile. */
	struct gxpci_host_nic_desc __iomem *nic_desc;

	/* The mapped address to the device's queue registers on an VF. */
	struct gxpci_host_nic_regs_vf __iomem *nic_regs_vf;

	/* Pointer to tile_nic_dma_cmd_t array on tile on an VF. */
	struct gxpci_host_nic_desc_vf __iomem *nic_desc_vf;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	struct gxpci_queue_vector *vectors;
#else
	struct net_device *polling_netdev;
#endif
	int int_vectors;

	u32 rx_buffer_len;

	/* Flag indicating if receive checksum offload is enabled. */
	u32 rx_csum;

	/* Track device up/down state */
	unsigned long state;

	/* Channel state. */
	gxpci_nic_chan_state_t channel_state;

	/* Pointer to array of Tx rings. */
	struct gxpci_ring *tx_ring ____cacheline_aligned_in_smp;
	int num_tx_queues;

	/* Pointer to array of Rx rings. */
	struct gxpci_ring *rx_ring ____cacheline_aligned_in_smp;
	int num_rx_queues;

#if !defined(TILEPCI_HOST)
	/* This is used to monitor the queue status. */
	struct delayed_work ep_queue_monitor_work;
#endif /* !defined(TILEPCI_HOST) */

        /* Address to trigger the card's MMI interrupt. */
        uint64_t __iomem *intr_regs;

#ifdef GXPCI_NETLIB_VNIC
	/* Pointer to netlib registers. */
	struct gxpci_netlib_regs __iomem *netlib_regs;
#endif

#ifdef GXPCI_HOST_NIC_P2P
	struct timer_list intr_timer;
#endif
#ifdef GXPCI_HOST_SDNIC
  tlr_netdev_ctrl_t *netdev_ctrl;
#endif
};

enum gxpci_state_t {
	__GXPCI_TESTING,
	__GXPCI_RESETTING,
	__GXPCI_DOWN
};

extern void gxpci_net_devs_init(struct tlr_pcie_dev *tlr);
extern void gxpci_net_devs_remove(struct tlr_pcie_dev *tlr);
extern void gxpci_net_dev_close(struct tlr_pcie_dev *tlr, int if_num);
#if (defined(GXPCI_HOST_NIC_P2P) || defined(TILEPCI_VF))
extern void gxpci_reset(struct gxpci_nic *nic);
#endif

#endif
