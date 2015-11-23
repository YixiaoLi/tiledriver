/*
 * gxpci_host_shared.h
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
#ifndef __GXPCI_HOST_SHARED_H__
#define __GXPCI_HOST_SHARED_H__

#if defined(TILEPCI_HOST) && (defined(CONFIG_UIO) || defined(CONFIG_UIO_MODULE))
#include <linux/uio.h>
#include <linux/uio_driver.h>
#endif

#include "gxpci_host_nic.h"

/* Number of 8-byte words in the boot buffer. */
#define PCIE_BOOT_FIFO_WORDS  (RSH_BOOT_FIFO_SIZE >> 3)

/*
 * State object for each Tile endpoint PCIe port, used on RC.
 * This is used for both physical and virtual functions, though
 * some fields are not used for the VFs.
 */
struct tlr_pcie_dev {
	struct pci_dev *pci_dev;
	struct cdev cdev;
	dev_t first_dev;
	/* Global link index in the host PCIe domain. */
	int link_index;

	/* Port index within a single chip or board. */
	int port_index;

	/* This is set if the host driver detects driver incompatibility. */
	int drv_mismatch;

	unsigned long long bar0_address;
	unsigned long long bar2_address;
	unsigned int bar0_size;
	unsigned int bar2_size;

	/* This is used to manage the EP device setup. */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
	struct delayed_work ep_setup_work;
#else
	struct work_struct ep_setup_work;
#endif

	/* This is used to monitor the queue status. */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
	struct delayed_work ep_queue_monitor_work;
#else
	struct work_struct ep_queue_monitor_work;
#endif

	/* General per port lock. */
	spinlock_t lock;

	/* Address of the card's MMIO region. */
	struct gxpci_host_regs __iomem *regs;

	/* Address of the rshim registers. */
	u8 __iomem *rshim_regs;

	/* MSI-X entry table. */
	struct msix_entry *msix_entries;

	/* The MSI-X interrupt vector base number of the host NICs. */
	int msix_host_nic_intr_vec_base;

	/* The MSI-X interrupt vector base number of the character streams. */
	int msix_cs_q_intr_vec_base;

	/* The MSI-X interrupt vector base number of the zero-copy queues. */
	int msix_zc_q_intr_vec_base;

	/* Number of MSI-X vectors. */
	unsigned int msix_vectors;

	/* Lock used to make sure endpoint is only initialized once. */
	spinlock_t is_ready_lock;

	/* Flag indicating that the endpoint driver is up. */
	int is_ready;

	/* Serialize access to the 'boot' device file. */
	struct mutex boot_mutex;
	int is_booting;

	int chip_version;
	int chip_width;
	int chip_height;

	int link_speed;			/* Gen 1 or Gen 2. */
	int link_width;

	/* Number of host virtual NIC interfaces. */
	int nic_ports;

	/* Number of TX queues per virtual NIC interface. */
	int nic_tx_queues;

	/* Number of RX queues per virtual NIC interface. */
	int nic_rx_queues;

	/* Number of H2T queues per packet queue interface. */
	int pq_h2t_queues;

	/* Number of T2H queues per packet queue interface. */
	int pq_t2h_queues;

	/* Number of H2T queues per Raw DMA interface. */
	int rd_h2t_queues;

	/* Number of T2H queues per Raw DMA interface. */
	int rd_t2h_queues;

	/* Pointers to the network interfaces per PCIe link. */
	struct gxpci_nic *net_devs[GXPCI_HOST_NIC_COUNT];

#if defined(TILEPCI_HOST)
	/* The link width that is expected based on the switch capability. */
	int expected_link_width;

	/* The link speed that is expected based on the switch capability. */
	int expected_link_speed;

	/* The offset within the BAR of the PQ H2T queue registers. */
	unsigned int pq_h2t_regs_offset;

	/* The size of the host mapping for the PQ driver-visible registers. */
	unsigned int pq_drv_regs_map_size;

	/* The size of the host mapping for the PQ app-visible registers. */
	unsigned int pq_app_regs_map_size;

	struct tlr_board_s *board;

	/*
 	 * Locks to protect the shared DMA ring buffers for H2T and T2H packet
 	 * queues with the same interface number.
 	 */
	struct semaphore pq_mutex[GXPCI_HOST_PQ_MAX_QUEUE_NUM];

	/* Pointers to the char stream interfaces per PCIe link. */
	struct tlr_stream *streams[NUM_CHAR_STREAMS];
	
        /* Pointers to the zero-copy H2T stream interfaces per PCIe link. */
        struct tlr_zc_stream *zc_h2t_streams[NUM_ZC_H2T_CHAN];

        /* Pointers to the zero-copy T2H stream interfaces per PCIe link. */
        struct tlr_zc_stream *zc_t2h_streams[NUM_ZC_T2H_CHAN];

	/* H2T packet queue state information. */
	struct tlr_packet_queue_state pq_h2t[GXPCI_HOST_PQ_MAX_QUEUE_NUM];

	/* T2H packet queue state information. */
	struct tlr_packet_queue_state pq_t2h[GXPCI_HOST_PQ_MAX_QUEUE_NUM];

	/* H2T raw_dma state information. */
	struct tlr_raw_dma_state rd_h2t[GXPCI_RAW_DMA_QUEUE_COUNT];

	/* T2H raw_dma state information. */
	struct tlr_raw_dma_state rd_t2h[GXPCI_RAW_DMA_QUEUE_COUNT];

#if defined(CONFIG_UIO) || defined(CONFIG_UIO_MODULE)
	/* UIO info */
	struct uio_info uio;
#endif

#endif /* defined(TILEPCI_HOST) */

	/* Address of the queue-pair status info array for non-C2C queues. */
	struct gxpci_queue_status_array *queue_sts_array;

	/* PCI address of the queue-pair status array for non-C2C queues. */
	dma_addr_t queue_sts_bus_addr;

	/* Boot buffer. Using stack space could overflow kernel stack. */
	uint64_t boot_buf[PCIE_BOOT_FIFO_WORDS];

	/* Number of instantiated virtual functions per PCIe port. */
	unsigned int num_vfs;
	/* Size of BAR0 space for each VF. */
	unsigned int vf_bar0_size;
	/* Size of BAR2 space for each VF. */
	unsigned int vf_bar2_size;
};

/*
 * We check if the endpoint driver is up and if the port status word
 * can be accessed by looking for the ready flag in the endpoint
 * device's Subsystem Vendor ID Register.
 */
static inline int
gx_ep_not_booted(struct tlr_pcie_dev *tlr)
{
	uint16_t ready;

	pci_read_config_word(tlr->pci_dev, PCI_SUBSYSTEM_VENDOR_ID, &ready);

	return ready == PCIE_STATUS_READY ? 0 : 1;
}

#endif
