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
 * Declares state used by the RC-side chip-to-chip driver.
 */
#ifndef __GXPCI_CHIP_TO_CHIP_H__
#define __GXPCI_CHIP_TO_CHIP_H__

#include <linux/pci.h>

#include <asm/tilegxpci.h>

/* For each C2C queue, this struct defines its state info. */
struct tlr_c2c_state {
	/*
	 * Address of the entry in the C2C queue status array
	 * corresponding to this queue.
	 */
	struct gxpci_c2c_queue_sts *queue_pair_status;

	/*
	 * Address of the queue status buffer that is shared by
	 * the driver and the application running on the RC node.
	 * The buffer is allocated in the driver and mmap'ed to
	 * the application.
	 */
	struct tlr_c2c_status *queue_status;

	/* mmap info. */
	int vmas;

	/* # of VMAs referencing us. */
	struct semaphore mutex;
};

/* The pseudo RC device structure per RC port. */
struct tlr_pcie_rc_c2c_dev {
	struct cdev cdev;
	dev_t first_dev;

	/* PCIe mac index on the TRIO shim. */
	int mac;
	/* Index of TRIO shim that contains the MAC. */
	int trio_index;

	/* PCI domain. */
	int domain;

	/* Address of the C2C queue status array. */
	struct gxpci_c2c_queue_sts_array *c2c_queue_sts_array;

	/* PCI bus address of the C2C queue status array. */
	dma_addr_t c2c_queue_sts_array_bus_addr;

	/* This implements the timer to monitor the C2C queue status. */
	struct delayed_work c2c_queue_monitor_work;

	/* Chip-to-chip send queue state information. */
	struct tlr_c2c_state c2c_send[GXPCI_C2C_QUEUE_COUNT];

	/* Chip-to-chip receive queue state information. */
	struct tlr_c2c_state c2c_recv[GXPCI_C2C_QUEUE_COUNT];
};

#endif	/* !defined(__GXPCI_CHIP_TO_CHIP_H__) */
