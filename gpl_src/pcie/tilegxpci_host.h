/*
 * tilegxpci_host.h
 *
 * Host-driver function and structure declarations.
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
#ifndef __TLR_GXPCI_HOST_H__
#define __TLR_GXPCI_HOST_H__ 

#include <asm/ioctl.h>
#include "linux/list.h"

#include "tilegxpci_shared_code.h"

/** VM_RESERVED is no longer in Linux 3.7.x and up. */
#ifndef VM_RESERVED
#define VM_RESERVED	(VM_DONTEXPAND | VM_DONTDUMP)
#endif

/*
 * The largest this can be set to is the maximum allocation size
 * in the kernel (PAGE_SIZE << (MAX_ORDER-1)); to increase that limit,
 * see the code setting MAX_ORDER in <linux/mmzone.h>.
 * Also see the HOST_PQ_SEGMENT_MAX_SIZE test in tilegxpci_host_common.h.
 */
#if HOST_RD_SEGMENT_MAX_SIZE > (PAGE_SIZE << (MAX_ORDER - 1))
# error HOST_RD_SEGMENT_MAX_SIZE set too large for this configuration
#endif

/** The vendor ID for all Tilera processors. */
#define TILERA_VENDOR_ID		0x1a41

/** The device ID for the Gx36 processor. */
#define TILERA_GX36_DEV_ID		0x0200

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37)
#define DECLARE_MUTEX(_m)  DEFINE_SEMAPHORE(_m)
#endif

/** The device RSHIM byte-access widget registers. */
#define RSH_BYTE_ACC_CTL			0x0490
#define RSH_BYTE_ACC_CTL__SIZE_SHIFT		3
#define RSH_BYTE_ACC_CTL__SIZE_VAL_4BYTE	0x2
#define RSH_BYTE_ACC_CTL__PENDING_MASK		0x20
#define RSH_BYTE_ACC_CTL__READ_TRIG_SHIFT	6
#define RSH_BYTE_ACC_WDAT			0x0498
#define RSH_BYTE_ACC_RDAT			0x04a0
#define RSH_BYTE_ACC_ADDR			0x04a8

/** The device RSHIM Interrupt Binding register. */
#define RSH_SWINT			0x0318
#define RSH_INT_VEC0_RTC__SWINT3_MASK	0x8

#define RSH_GX36_IO_RESET__TRIO_MASK	0x40
#define RSH_GX36_IO_RESET__PCIE0_MASK	0x80
#define RSH_GX36_IO_RESET__PCIE1_MASK	0x100
#define RSH_GX36_IO_RESET__PCIE2_MASK	0x200

#define TRIO_PCIE_MAC_MASK \
	(RSH_GX36_IO_RESET__TRIO_MASK | RSH_GX36_IO_RESET__PCIE0_MASK | \
	RSH_GX36_IO_RESET__PCIE1_MASK | RSH_GX36_IO_RESET__PCIE2_MASK)

/** Max number of Tilera chips in one PCI domain. */
#define MAX_CHIPS_PER_DOMAIN		32

/** Max number of EP ports on a Gx processor, 3 on Gx36, 6 on Gx72. */
#define MAX_PCIE_LINKS_PER_CHIP		6

/** Milliseconds to wait for chip to come back after reset. */
#define TLR_RESET_WAIT_MS		500

/** Milliseconds to wait between boot completion and ready check. */
#define TLR_REBOOT_TO_READY_WAIT_MS	300

/*
 * The host driver must use MSI, not MSI-X, if the VFs need to generate
 * interrupts to the host.
 */
#if 0
#define USE_MSI_ONLY
#endif

typedef int tlr_board_func_t(struct tlr_board_s* board);

/**
 * Each Tilera board is represented by one of these structures.
 * A board is a Tilera chip which has one or more PCIe ports or links.
 */
typedef struct tlr_board_s
{
	struct list_head list;
	tlr_pcie_dev_t *ports[MAX_PCIE_LINKS_PER_CHIP];
	int board_index;
	int num_ports;

	/* State related to reboot / device locking.  Modifying these
	 * fields requires holding the global reboot_lock. */
	int is_rebooting;
	pid_t locked_pid;
	int packet_queue_cnt;
	int raw_dma_cnt;
	int barmem_cnt;
	int host_nic_cnt;

	/* This one can be set at any time - we may want to whack it
	 * via ioctl() if something outside this driver causes the
	 * board to reset. */
	int need_reset;

	/* Returns TRUE if any of the board's PCI links are down. */
	tlr_board_func_t* is_link_down;
	
	/* Returns TRUE if the board has already booted (potentially
	 * via some mechanism other than PCI). */
	tlr_board_func_t* has_booted;

	/* Reset the board. Returns negative error code if we couldn't
	 * reset or if we tried and something went wrong (link didn't
	 * come back up, etc.). */
	tlr_board_func_t* do_reset;

	/* Lock to serialize RSHIM register access under 32-bit OS. */
	spinlock_t rshim_reg_lock;

#ifdef GXPCI_HOST_SDNIC
  struct tlr_tile_capability tile_capability;
  tlr_netdev_ctrl_t netdev_ctrl;
#endif
}
tlr_board_t;

// Functions exported by tilegxpci_host.c

extern void get_link_speed_width(struct pci_dev *dev);
extern u64 rshim_reg_read(struct tlr_pcie_dev *tlr, u32 reg);
extern void rshim_reg_write(struct tlr_pcie_dev *tlr, u32 reg, u64 val);

extern struct semaphore reboot_mutex;

extern int gxpci_reset_all;

// Functions exported by raw_dma.c
extern int tlr_raw_dma_open(struct tlr_pcie_dev *tlr, struct file *filp,
			    struct tlr_raw_dma_state *rd, int channel);
extern void tlr_raw_dma_free(struct tlr_raw_dma_state *rd);

// Functions exported by barmem.c
resource_size_t tlr_barmem_size(struct tlr_pcie_dev* tlr);
dma_addr_t tlr_barmem_base(struct tlr_pcie_dev* tlr);
int tlr_barmem_open(struct inode *inode, struct file *filp);

// Functions exported by boards.c
int tlr_map_device_to_board(struct tlr_pcie_dev* tlr);
void tlr_unmap_device_from_board(struct tlr_pcie_dev* tlr);

extern struct file_operations tlr_proc_boards_ops;

#endif
