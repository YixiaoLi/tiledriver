/*
 * Tilera Gx host-side driver
 *
 * This source code is derived for code provided in "Linux Device
 * Drivers" by Alessandro Rubini and Jonathan Corbet, published by
 * O'Reilly & Associates.
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/version.h>



#include <asm/pgtable.h> 		/* io_remap_page_range */
#include <asm/uaccess.h> 

#include "tilegxpci.h"
#include "tilegxpci_host.h"
#include "tilegxpci_version.h"

#include "tilegxpci_host_common.h"

/*
 * The host driver must use MSI, not MSI-X, if the VFs need to generate
 * interrupts to the host.
 */
#if 0
#define USE_MSI_ONLY
#endif

/* 
 * Extended tag support should be enabled on hosts that can support 8-bit 
 * tags since this will improve host-to-tile transfer performance.
 */
#define EXTENDED_TAG_ENABLE 1

/*
 * This is a flag that tells the driver if the Tilera PCIe components (TRIO
 * and PCIe MACs) should be included in the chip reset as part of the chip
 * boot process. On hosts that cannot handle the PCIe "Surprise Down" event,
 * this flag should be 0.
 */
int gxpci_reset_all = 1;
module_param(gxpci_reset_all, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(gxpci_reset_all,
	"Flag indicating if the Tilera PCIe should be included in chip reset. "
	"Set to 0 on hosts that cannot handle the PCIe Surprise Down event.");


/* 
 * This mutex must be held whenever any process tries to start a
 * reboot attempt or tries to reserve permission to reboot (i.e. grab
 * the lock file).  Serializing reboots allows the chip-to-chip
 * communication code to know that no cards will be reset while in the
 * midst of connecting or disconnecting.
 */
DECLARE_MUTEX(reboot_mutex);

#ifdef CONFIG_PCI_IOV

/*
 * This is the max number of VFs that can be supported per PF.
 * This number can be smaller if the VFs enable interfaces other than
 * the Packet Queue interfaces.
 */
static unsigned int max_vfs = GXPCI_MAX_NUM_VFS;
module_param(max_vfs, uint, S_IRUGO);
MODULE_PARM_DESC(max_vfs, "Maximum number of VFs to allocate for each PF");

/*
 * This flag indicates if the VFs enable the tile kernel host-NIC devices.
 */
static int enable_vf_nic = 0;
module_param(enable_vf_nic, int, S_IRUGO);
MODULE_PARM_DESC(enable_vf_nic, "Enable virtual function's support for "
		 "the kernel host NIC interfaces.");

/*
 * This flag indicates if the VFs enable the tile user host-NIC devices.
 */
static int enable_vf_user_nic = 1;
module_param(enable_vf_user_nic, int, S_IRUGO);
MODULE_PARM_DESC(enable_vf_user_nic, "Enable virtual function's support for "
		 "the user-space host NIC interfaces.");

#endif

/*
 * Each PCI domain maintains an array of chip-to-chip queue status
 * structure, which is allocated when the second Gx EP device is
 * probed and freed when the driver is unloaded.
 */
static struct gxpci_c2c_queue_sts_array *c2c_queue_sts_array_addr;

/* The PCI bus address for the gxpci_c2c_queue_sts_array. */
static dma_addr_t c2c_queue_sts_array_bus_addr;

/* This is used to monitor the C2C queue status in each PCI domain. */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
static struct delayed_work c2c_queue_monitor_work;
#else
static struct work_struct c2c_queue_monitor_work;
#endif

/*
 * Array of pointers to all the Gx endpoint devices in the system.
 */
static struct tlr_pcie_dev *gx_ep_ports[MAX_PCIE_PORTS_PER_DOMAIN - 1];

/* Host NIC port number for a single TILE PCIe port. */
static int nic_ports = GXPCI_HOST_NIC_COUNT;
module_param(nic_ports, int, S_IRUGO);
MODULE_PARM_DESC(nic_ports,
	"Number of host virtual NIC interfaces for a single PCIe port.");

/* Per-port host NIC TX queue number. */
static int nic_tx_queues = GXPCI_HOST_NIC_TX_QUEUES;
module_param(nic_tx_queues, int, S_IRUGO);
MODULE_PARM_DESC(nic_tx_queues,
	"Number of TX queues per virtual NIC interface.");

/* Per-port host NIC RX queue number. */
static int nic_rx_queues = GXPCI_HOST_NIC_RX_QUEUES;
module_param(nic_rx_queues, int, S_IRUGO);
MODULE_PARM_DESC(nic_rx_queues,
	"Number of RX queues per virtual NIC interface.");

/* Per-port host Raw DMA H2T queue number. */
static int rd_h2t_queues = GXPCI_RAW_DMA_QUEUE_COUNT;
module_param(rd_h2t_queues, int, S_IRUGO);
MODULE_PARM_DESC(rd_h2t_queues, "Number of PCIe Raw DMA H2T queues.");

/* Per-port host Raw DMA T2H queue number. */
static int rd_t2h_queues = GXPCI_RAW_DMA_QUEUE_COUNT;
module_param(rd_t2h_queues, int, S_IRUGO);
MODULE_PARM_DESC(rd_t2h_queues, "Number of PCIe Raw DMA T2H queues.");

#ifdef RAW_DMA_USE_RESERVED_MEMORY

/*
 * Total PCIe Raw DMA buffer size for H2T queues.
 * The buffer is supposed to be reserved via kernel parameter memmap.
 */
static char *rd_h2t_buf_size;
module_param(rd_h2t_buf_size, charp, S_IRUGO);
MODULE_PARM_DESC(rd_h2t_buf_size,
		 "Total PCIe Raw DMA H2T buffer size in size[KMG].");

/*
 * Total PCIe Raw DMA buffer size for T2H queues.
 * The buffer is supposed to be reserved via kernel parameter memmap.
 */
static char *rd_t2h_buf_size;
module_param(rd_t2h_buf_size, charp, S_IRUGO);
MODULE_PARM_DESC(rd_t2h_buf_size,
		 "Total PCIe Raw DMA T2H buffer size in size[KMG].");

/* PCIe Raw DMA H2T buffer base physical address. */
static char *rd_h2t_buf_base_addr;
module_param(rd_h2t_buf_base_addr, charp, S_IRUGO);
MODULE_PARM_DESC(rd_h2t_buf_base_addr,
		 "Raw DMA H2T buffer physical address in address[KMG].");

/* PCIe Raw DMA T2H buffer base physical address. */
static char *rd_t2h_buf_base_addr;
module_param(rd_t2h_buf_base_addr, charp, S_IRUGO);
MODULE_PARM_DESC(rd_t2h_buf_base_addr,
		 "Raw DMA T2H buffer physical address in address[KMG].");

/*
 * The Raw DMA T2H DMA ring buffers are configured with
 * "rd_t2h_mems=[<size>@<PA>][:[<size>@<PA>]]*",
 * and the Raw DMA H2T DMA ring buffers are configured with
 * "rd_h2t_mems=[<size>@<PA>][:[<size>@<PA>]]*",
 * where <PA> and <size> (in size[KMG]) are the Raw DMA buffer's physical
 * address and size, respectively, with the relative position of the PA/size
 * pair in the parameter string indicating the Raw DMA interface number.
 *
 * Note: if either parameter is specified, the PA/size will be calculated
 * based on the other Raw DMA paramters, evenly dividing the buffer.
 */
static char *rd_t2h_mems;
module_param(rd_t2h_mems, charp, S_IRUGO);
MODULE_PARM_DESC(rd_t2h_mems, "Physical address and size (in size[KMG]) "
	"configurations for the Raw DMA T2H DMA buffer.");

static char *rd_h2t_mems;
module_param(rd_h2t_mems, charp, S_IRUGO);
MODULE_PARM_DESC(rd_h2t_mems, "Physical address and size (in size[KMG]) "
	"configurations for the Raw DMA H2T DMA buffer.");

#else

/*
 * The Raw DMA T2H DMA ring buffers are configured with
 * "rd_t2h_mems=[<size>][,<numa_node>][:[<size>][,<numa_node>]]*",
 * and the Raw DMA H2T DMA ring buffers are configured with
 * "rd_h2t_mems=[<size>][,<numa_node>][:[<size>][,<numa_node>]]*",
 * with the relative position of the 2-tuple in the parameter string indicating
 * the Raw DMA interface number.
 *
 * Note: <size> (in size[KMG]) is always HOST_RD_SEGMENT_MAX_SIZE aligned, and
 * its MAX value is (HOST_RD_SEGMENT_MAX_SIZE * HOST_RD_SEGMENT_MAX_NUM); if
 * not specified, the default size of HOST_RD_SEGMENT_MAX_SIZE is allocated.
 * numa_node is ranged from 0 to (MAX_NUMNODES - 1); if it is not specified, or
 * not online, the DMA ring buffer is allocated from the current NUMA node.
 */
static char *rd_t2h_mems;
module_param(rd_t2h_mems, charp, S_IRUGO);
MODULE_PARM_DESC(rd_t2h_mems, "Size (in size[KMG]) and NUMA node "
	"for the Raw DMA T2H buffer allocation.");

static char *rd_h2t_mems;
module_param(rd_h2t_mems, charp, S_IRUGO);
MODULE_PARM_DESC(rd_h2t_mems, "Size (in size[KMG]) and NUMA node "
	"for the Raw DMA H2T buffer allocation.");

#endif

/**********************************************************************/
/*                   Interrupt Handler and Worker Thread              */
/**********************************************************************/

static void
tlr_handle_completions(struct tlr_stream* stream)
{	
	uint32_t h2t_completions = 
		readl(&stream->h2t_regs->completion_posted_count);
	uint32_t t2h_completions = 
		readl(&stream->t2h_regs->completion_posted_count);
	uint32_t h2t_last = stream->h2t_completions_last;
	uint32_t t2h_last = stream->t2h_completions_last;
	uint32_t index;
	uint32_t completed;
	uint64_t size;
	pcie_host_completion_t* cmp;

	if (h2t_completions - h2t_last)
		TRACE("HOST: Interrupt found %d completion delta of H2T.\n",
		      h2t_completions - h2t_last);

	if (t2h_completions - t2h_last)
		TRACE("HOST: Interrupt found %d completion delta of T2H.\n",
		      t2h_completions - t2h_last);

	/* For each completion, update its associated stream. */
	for (/* above */; (h2t_completions - h2t_last) != 0; h2t_last++) {
		index = h2t_last & (PCIE_CMD_QUEUE_ENTRIES - 1);
		cmp = &stream->h2t_completion_array[index];

		size = cmp->size;

		TRACE("HOST: H2T size = %lld\n", size);

		/* Advance counter. */
		stream->writes_completed++;
	}

	/* Write is complete; advance counter and wake. */
	if (h2t_completions != stream->h2t_completions_last) {
		wake_up_interruptible(&stream->write_queue);
		stream->h2t_completions_last = h2t_completions;
		writel(h2t_completions, 
		       &stream->h2t_regs->completion_consumed_count);
	}

	/* For each completion, update its associated stream. */
	for (/* above */; (t2h_completions - t2h_last) != 0; t2h_last++) {
		index = t2h_last & (PCIE_CMD_QUEUE_ENTRIES - 1);
		cmp = &stream->t2h_completion_array[index];

		size = cmp->size;

		TRACE("HOST: T2H size = %lld\n", size);

		/* Read complete: save size, advance counter. */
		completed = stream->reads_completed;
		index = completed & (BUFFERS_PER_STREAM - 1);
		stream->read_sizes[index] = size;

		wmb();
		stream->reads_completed = completed + 1;
	}

	/* Wake up. */
	if (t2h_completions != stream->t2h_completions_last) {
		wake_up_interruptible(&stream->read_queue);
		stream->t2h_completions_last = t2h_completions;
		writel(t2h_completions, 
		       &stream->t2h_regs->completion_consumed_count);
	}
}

#if GXPCI_HOST_ZC_QUEUE_COUNT
void 
tlr_handle_zc_completions(struct tlr_zc_stream* stream)
{
	uint32_t completions;
	uint32_t last;
	uint32_t index;
	uint64_t size;
	pcie_host_completion_t* cmp;
	struct pcie_host_completion *completion_array;
	struct tlr_zc_cmd_q *cmd_queue = stream->cmd_queue;

	if (stream->cmd_queue->type == TLR_ZC_CMD_H2T) {
		completions = 
			readl(&stream->h2t_regs->completion_posted_count);
		last = stream->h2t_completions_last;
		completion_array = stream->h2t_completion_array;
	}
	else {
		completions = 
			readl(&stream->t2h_regs->completion_posted_count);
		last = stream->t2h_completions_last;
		completion_array = stream->t2h_completion_array;
	}

	if (completions - last)
		TRACE("HOST: Interrupt found %d completion delta.\n",
		      completions - last);

	/* For each completion, update its associated stream. */
	for (/* above */; (completions - last) != 0; last++) {
		index = last & (PCIE_CMD_QUEUE_ENTRIES - 1);
		cmp = &completion_array[index];

		size = cmp->size;

		TRACE("HOST: size = %lld\n", size);

		tlr_zc_cmd_q_comp(cmd_queue, cmp, size);
	}

	if (stream->cmd_queue->type == TLR_ZC_CMD_H2T) {
		if (completions != stream->h2t_completions_last) {
			stream->h2t_completions_last = completions;
			writel(completions, 
			       &stream->h2t_regs->completion_consumed_count);
		}
	}
	else {
		if (completions != stream->t2h_completions_last) {
			stream->t2h_completions_last = completions;
			writel(completions, 
			       &stream->t2h_regs->completion_consumed_count);
		}
	}
}
#endif

static irqreturn_t
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
irq_handler(int irq, void* dev, struct pt_regs* pt_regs)
#else
irq_handler(int irq, void* dev)
#endif
{
	struct tlr_stream *stream = (struct tlr_stream*) dev;

	tlr_handle_completions(stream);

	return IRQ_HANDLED;
}

/*
 * tlr_request_irq - initialize interrupts
 *
 * Attempts to configure interrupts using the best available
 * capabilities of the hardware and kernel, i.e. MSI-X or MSI.
 **/
static int
tlr_request_irq(struct tlr_stream *stream)
{
	struct tlr_pcie_dev *tlr = stream->dev;
	int msix_table_index;
	int err;

	if (tlr->msix_vectors) {
		msix_table_index = tlr->msix_cs_q_intr_vec_base + stream->index;
		err = request_irq(tlr->msix_entries[msix_table_index].vector,
				  irq_handler, 0, driver_name, stream);
		if (err) {
			dev_err(&tlr->pci_dev->dev,
				"MSI-X tlr_request_irq failure: %d\n", err);
		}
	} 
	else {
		err = request_irq(tlr->pci_dev->irq, irq_handler, IRQF_SHARED,
				  driver_name, stream);
		if (err) {
			dev_err(&tlr->pci_dev->dev,
				"MSI tlr_request_irq failure: %d\n", err);
		}
	}

	return err;
}

/* Note that free_irq() leads to a PCIe READ to the Gx PCIe port. */
static void
tlr_free_irq(struct tlr_stream* stream)
{
	struct tlr_pcie_dev *tlr = stream->dev;
	int msix_table_index;

	if (tlr->msix_vectors) {
		msix_table_index = tlr->msix_cs_q_intr_vec_base + stream->index;
		free_irq(tlr->msix_entries[msix_table_index].vector, stream);
	} else {
		free_irq(tlr->pci_dev->irq, stream);
	}
}

/* Post a set of read buffers to the PCIe device. */
static void
post_read_buffers(struct tlr_stream* stream, u32 start, u32 stop)
{
	u32 stream_posted;
	u32 cmds_posted;
	u32 cmd_index;

	/* Build a template command on the local stack. */
	pcie_host_buffer_cmd_t cmd = { 0 };
	cmd.size = BUFFER_SIZE;

	cmds_posted = stream->t2h_commands_posted;

	/* This loop posts buffers from the per-stream ring buffer
	 * into the global command ring buffer.  We keep a separate
	 * 'posted' count for each ring, and calculate the index
	 * within each ring as (posted % ring_entries). */
	for (stream_posted = start; stream_posted != stop; stream_posted++) {
		u32 index = stream_posted & (BUFFERS_PER_STREAM - 1);
		dma_addr_t addr = stream->read_dma_addrs[index];

		cmd.buffer_addr = addr;

		CMD_TRACE("READ size= %d, bus_addr= %#llx\n",
			  cmd.size, addr);

		cmd_index = cmds_posted & (PCIE_CMD_QUEUE_ENTRIES - 1);
		stream->t2h_buffer_cmd_array[cmd_index] = cmd;
		cmds_posted++;
	}

	if (start != stop) {
		stream->t2h_commands_posted = cmds_posted;
		writel(cmds_posted, &stream->t2h_regs->buffer_cmd_posted_count);

		wmb();
		/* Trgger the tile-side's MMI interrupt. */
		writeq(PCIE_HOST_INTR_CPL_AVAIL, stream->intr_regs);
	}
}

/* Post a set of write buffers to the PCIe device. */
static void
post_write_buffers(struct tlr_stream* stream, u32 start, u32 stop)
{
	uint32_t stream_posted;
	uint32_t cmds_posted;
	uint32_t cmd_index;

	/* Build a template command on the local stack. */
	pcie_host_buffer_cmd_t cmd = { 0 };

	cmds_posted = stream->h2t_commands_posted;

	/*
	 * This loop posts buffers from the per-stream ring buffer
	 * into the global command ring buffer.  We keep a separate
	 * 'posted' count for each ring, and calculate the index
	 * within each ring as (posted % ring_entries).
	 */
	for (stream_posted = start; stream_posted != stop; stream_posted++) {
		uint32_t index = stream_posted & (BUFFERS_PER_STREAM - 1);
		dma_addr_t addr = stream->write_dma_addrs[index];

		cmd.buffer_addr = addr;
		cmd.size = stream->write_sizes[index];

		CMD_TRACE("HOST: WRITE size= %d, bus_addr= %#llx\n", 
			  cmd.size, addr);

		cmd_index = cmds_posted & (PCIE_CMD_QUEUE_ENTRIES - 1);
		stream->h2t_buffer_cmd_array[cmd_index] = cmd;

		cmds_posted++;
	}

	if (start != stop) {
		stream->h2t_commands_posted = cmds_posted;
		writeq(cmds_posted, &stream->h2t_regs->buffer_cmd_posted_count);

		wmb();
		/* Trgger the tile-side's MMI interrupt. */
		writeq(PCIE_HOST_INTR_CPL_AVAIL, stream->intr_regs);
	}
}

/* 
 * Check whether the PCIe drivers on the host side and the Tile side are 
 * compatible. 
 *
 * Return 0 if the drivers are compatible, otherwise -1.
 */
static int
tlr_check_version(struct tlr_pcie_dev *tlr)
{
	uint32_t endp_version = readl(&tlr->regs->version);
	uint32_t host_version = 
		PCIE_VERSION_DEF(TILEGXPCI_MAJOR, TILEGXPCI_MINOR);
	
	if (!PCIE_VERSION_MATCH(endp_version, host_version)) {
		tlr->drv_mismatch = 1;

		dev_err(&tlr->pci_dev->dev,
			"Host vs. Tile version mismatch: host driver=%d.%d, "
			"card booted=%d.%d\n",
			PCIE_VERSION_MAJOR(host_version),
			PCIE_VERSION_MINOR(host_version),
			PCIE_VERSION_MAJOR(endp_version),
			PCIE_VERSION_MINOR(endp_version));
		
		return -1;
	}
	
	return 0;
}

/* 
 * Check whether the char stream has been initialized on the tile side 
 * successfully.
 *
 * Return status if the tile's char stream is initialized, which is 
 * GXPCI_TILE_CHAN_READY or GXPCI_TILE_CHAN_READY_ACK.
 */
static uint32_t 
check_char_stream_ready(struct tlr_stream* stream, uint32_t status)
{
	struct tlr_pcie_dev *tlr = stream->dev;
	unsigned long flags;
	uint32_t result = 0;
	uint32_t h2t_status;
	uint32_t t2h_status;

	/* First, do a fast-path check to see if the stream is already
	 * up.  We're not going to transition anything yet, so we only
	 * need the spinlock. */
	spin_lock_irqsave(&tlr->is_ready_lock, flags);
	if (stream->is_ready) {
		spin_unlock_irqrestore(&tlr->is_ready_lock, flags);
		return status;
	}
	spin_unlock_irqrestore(&tlr->is_ready_lock, flags);

	h2t_status = readl(&stream->h2t_regs->queue_status);	
	t2h_status = readl(&stream->t2h_regs->queue_status);	

	if (h2t_status == status && t2h_status == status) 
		result = status;

	return result;
}

/* Free all resources associated with a stream. */
static void
release_stream(struct tlr_stream* stream)
{
	struct tlr_pcie_dev* tlr = stream->dev;
	int i;

	for (i = 0; i < BUFFERS_PER_STREAM; i++) {
		if (stream->read_dma_addrs[i]) {
			dma_unmap_single(&tlr->pci_dev->dev,
					 stream->read_dma_addrs[i],
					 BUFFER_SIZE,
					 DMA_FROM_DEVICE);
			stream->read_dma_addrs[i] = 0;
		}
		if (stream->read_buffers[i]) {
			free_page((unsigned long) stream->read_buffers[i]);
			stream->read_buffers[i] = 0;
		}
	}

	for (i = 0; i < BUFFERS_PER_STREAM; i++) {
		if (stream->write_dma_addrs[i]) {
			dma_unmap_single(&tlr->pci_dev->dev,
					 stream->write_dma_addrs[i],
					 BUFFER_SIZE,
					 DMA_TO_DEVICE);
			stream->write_dma_addrs[i] = 0;
		}
		if (stream->write_buffers[i]) {
			free_page((unsigned long) stream->write_buffers[i]);
			stream->write_buffers[i] = 0;
		}
	}

	stream->writes_posted = 0;
	stream->writes_completed = 0;
	stream->reads_completed = 0;
	stream->reads_consumed = 0;
	stream->partial_read_bytes = 0;
	stream->is_ready = 0;
}

/* Initialize a newly opened stream. */
static int
init_stream(struct tlr_stream* stream)
{
	struct tlr_pcie_dev* tlr = stream->dev;
	uint64_t value;
	int i;
	int result = 0;
	int flag = 0;
	int sleep_msecs = 0;
	const int POLLING_MSECS = 200;
	const int MAX_MSECS = 5000;

	/* We expect ~4k pages. */
	if (PAGE_SIZE < BUFFER_SIZE) {
		dev_err(&tlr->pci_dev->dev,
			"Page size must be at least %d\n", BUFFER_SIZE);
		return -ENOMEM;
	}

	/* Check whether the chip is ready. */
	while (1) {
		spin_lock(&tlr->is_ready_lock);
		if (tlr->is_ready) {
			spin_unlock(&tlr->is_ready_lock);
			break;
		}
		spin_unlock(&tlr->is_ready_lock);

		/* Don't waste more time here with incompatible SW. */
		if (tlr->drv_mismatch)
			return -EPERM;

		/* Avoid being optimized into register by the compiler. */
		barrier();

		if (sleep_msecs >= MAX_MSECS)
			break;
		msleep(POLLING_MSECS);
		sleep_msecs += POLLING_MSECS;
	}

	if (sleep_msecs >= MAX_MSECS)
		return -ENXIO;

	sleep_msecs = 0;

	/* 
	 * Check whether the char stream is ready. The timeout value is set to
	 * MAX_MSECS. 
	 */
	while ((flag = check_char_stream_ready(stream, GXPCI_TILE_CHAN_READY)) 
	       != GXPCI_TILE_CHAN_READY) {

		/* Avoid being optimized into register by the compiler. */
		barrier();

		if (sleep_msecs >= MAX_MSECS)
			break;
		msleep(POLLING_MSECS);
		sleep_msecs += POLLING_MSECS;
	}

	if (flag != GXPCI_TILE_CHAN_READY) {
		result = -ENXIO;
		goto check_ready_fail;
	}

	/* Configure completion array location. */
	TRACE("HOST: Programming H2T completion bus address.\n");
	value = (uint64_t)stream->h2t_completion_handle;
	TRACE("HOST: completion_array=%16llXh\n", value);
	writeq(value, &stream->h2t_regs->completion_array);

	TRACE("HOST: Programming T2H completion bus address.\n");
	value = (uint64_t)stream->t2h_completion_handle;
	TRACE("HOST: completion_array=%16llXh\n", value);
	writeq(value, &stream->t2h_regs->completion_array);

	/* Configure command buffer array. */
	TRACE("HOST: Programming H2T command buffer bus address.\n");
	value = (uint64_t)stream->h2t_buffer_cmd_handle;
	TRACE("HOST: buffer_cmd_array=%16llXh\n", value);
	writeq(value, &stream->h2t_regs->buffer_cmd_array);

	TRACE("HOST: Programming T2H command buffer bus address.\n");
	value = (uint64_t)stream->t2h_buffer_cmd_handle;
	TRACE("HOST: buffer_cmd_array=%16llXh\n", value);
	writeq(value, &stream->t2h_regs->buffer_cmd_array);

	/* 
	 * Get initial values of the command and completion counter MMIO 
	 * registers. 
	 */
	TRACE("HOST: Retreiving H2T posted & comple starting values.\n");
	stream->h2t_commands_posted =
		readl(&stream->h2t_regs->buffer_cmd_posted_count);
	stream->h2t_completions_last =
		readl(&stream->h2t_regs->completion_posted_count);
	TRACE("HOST: CMD=%08Xh COMP=%08Xh\n",
	      stream->h2t_commands_posted,
	      stream->h2t_completions_last);

	TRACE("HOST: Retreiving T2H posted & comple starting values.\n");
	stream->t2h_commands_posted =
		readl(&stream->t2h_regs->buffer_cmd_posted_count);
	stream->t2h_completions_last =
		readl(&stream->t2h_regs->completion_posted_count);
	TRACE("HOST: CMD=%08Xh COMP=%08Xh\n",
	      stream->t2h_commands_posted,
	      stream->t2h_completions_last);

	/* Error code for all the following fail tags. */
	result = -ENOMEM;

	/* Allocate read buffers. */
	for (i = 0; i < BUFFERS_PER_STREAM; i++) {
		stream->read_buffers[i] = (void*) __get_free_page(GFP_KERNEL);
		if (!stream->read_buffers[i])
			goto fail;
		stream->read_dma_addrs[i] =
			dma_map_single(&tlr->pci_dev->dev,
				       stream->read_buffers[i],
				       BUFFER_SIZE, DMA_FROM_DEVICE);
		if (!stream->read_dma_addrs[i])
			goto fail;
	}

	/* Allocate write buffers. */
	for (i = 0; i < BUFFERS_PER_STREAM; i++) {
		stream->write_buffers[i] = (void*) __get_free_page(GFP_KERNEL);
		if (!stream->write_buffers[i])
			goto fail;
		stream->write_dma_addrs[i] =
			dma_map_single(&tlr->pci_dev->dev,
				       stream->write_buffers[i],
				       BUFFER_SIZE, DMA_TO_DEVICE);
		if (!stream->write_dma_addrs[i])
			goto fail;
	}

	for (i = 0; i < BUFFERS_PER_STREAM; i++) {
		TRACE("HOST: write_dma_addrs[%d] is %llx\n", 
		      i, stream->write_dma_addrs[i]);
		TRACE("HOST: read_dma_addrs[%d] is %llx\n", 
		      i, stream->read_dma_addrs[i]);
	}

	result = tlr_request_irq(stream);
	if (result < 0) {
		result = -ENXIO;
		goto irq_failed;
	}

	/* Indicate the gx card that we are ready to go. */
	writel(GXPCI_HOST_CHAN_READY, &stream->h2t_regs->queue_status);
	writel(GXPCI_HOST_CHAN_READY, &stream->t2h_regs->queue_status);

	/* Complete the handshake process with the endpoint. */
	sleep_msecs = 0;
	while ((flag = 
		check_char_stream_ready(stream, GXPCI_TILE_CHAN_READY_ACK)) 
	       != GXPCI_TILE_CHAN_READY_ACK) {

		/* Avoid being optimized into register by the compiler. */
		barrier();

		if (sleep_msecs >= MAX_MSECS)
			break;
		msleep(POLLING_MSECS);
		sleep_msecs += POLLING_MSECS;
	}

	/*
	 * Timeout happens but tile side char stream is still not ready, just
	 * return the open error.
	 */
	if (flag != GXPCI_TILE_CHAN_READY_ACK) {
		result = -ENXIO;
		goto check_ready_ack_fail;
	}

	/* 
	 * Setup the ready flag to avoid this stream being initialized 
	 * more than once. 
	 */
	stream->is_ready = 1;

	/* Post buffers for incoming read data. */
	post_read_buffers(stream, 0, BUFFERS_PER_STREAM);

	return result;

 check_ready_ack_fail:
	writel(0, &stream->h2t_regs->queue_status);
	writel(0, &stream->t2h_regs->queue_status);

	tlr_free_irq(stream);
 irq_failed:
 fail:
	release_stream(stream);
 check_ready_fail:
	return result;
}

/**********************************************************************/
/*                        Character Device Routines                   */
/**********************************************************************/

static ssize_t
tlr_cdev_read(struct file *filp, char __user *buf, size_t count,
	      loff_t *f_pos)
{
	struct tlr_stream *stream = filp->private_data;
	struct tlr_pcie_dev *tlr = stream->dev;
	size_t already_read;
	u32 reads_consumed;
	size_t bytes_read;

	FOP_TRACE("Entered \n");

	if (count == 0)
		return 0;

	/* Grab the stream write lock. */
	if (down_interruptible(&stream->read_mutex)) {
		EX_TRACE("Exit -ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	/* Abandon this operation if the stream is poisoned due to reset. */
	if (stream->chip_reset_poison) {
		up(&stream->read_mutex);
		return -ENXIO;
	}

	/* Wait for data to appear in the read FIFO. */
	while (stream->reads_completed == stream->reads_consumed) {
		up(&stream->read_mutex);
		if (filp->f_flags & O_NONBLOCK) {
			EX_TRACE("Exit  -EAGAIN\n");
			return -EAGAIN;
		}

		/* Wait for the worker loop to put some data into the FIFO. */
		FOP_TRACE("Waiting on read_queue\n");
		if (wait_event_interruptible(stream->read_queue,
					     ((stream->reads_completed !=
					       stream->reads_consumed) ||
					      (stream->chip_reset_poison)))) {
			EX_TRACE("Exit tlr_cdev_read -ERESTARTSYS\n");
			return -ERESTARTSYS;
		}
		FOP_TRACE("Woke from read_queue\n");

		/* Get the read lock again. */
		if (down_interruptible(&stream->read_mutex)) {
			EX_TRACE("Exit tlr_cdev_read -ERESTARTSYS\n");
			return -ERESTARTSYS;
		}

		/* We could have been poisoned while sleeping. */
		if (stream->chip_reset_poison) {
			up(&stream->read_mutex);
			return -ENXIO;
		}
	}

	/* At this point we hold the read mutex and we know that there
	 * is at least one buffer of data available.  Copy as many
	 * buffers as possible to userspace. */
	already_read = stream->partial_read_bytes;
	bytes_read = 0;
	for (reads_consumed = stream->reads_consumed;
	     reads_consumed != stream->reads_completed;
	     reads_consumed++) {
		u32 index = reads_consumed & (BUFFERS_PER_STREAM - 1);
		size_t buf_remaining = stream->read_sizes[index] - already_read;
		size_t size = min(count, buf_remaining);

		if (size == 0)
			break;

		/* We need to sync each new buffer for CPU access. */
		if (already_read == 0)
			dma_sync_single_for_cpu(&tlr->pci_dev->dev,
						stream->read_dma_addrs[index],
						BUFFER_SIZE, DMA_FROM_DEVICE);

		if (copy_to_user(buf + bytes_read,
				 stream->read_buffers[index] + already_read,
				 size)) {
			if (bytes_read > 0)
				break;
			else {
				up(&stream->read_mutex);
				return -EFAULT;
			}
		}
		bytes_read += size;
		count -= size;

		if (size == buf_remaining) {
			/* We've completely consumed that buffer, sync
			 * it back to the device before reposting. */
			already_read = 0;
			dma_sync_single_for_device(&tlr->pci_dev->dev,
						   stream->read_dma_addrs[index],
						   BUFFER_SIZE,
						   DMA_FROM_DEVICE);
		}
		else {
			/* User only asked for part of the buffer. */
			already_read += size;
			break;
		}
	}
	stream->partial_read_bytes = already_read;

	/* Re-post any buffers that we completely consumed. */
	post_read_buffers(stream, stream->reads_consumed, reads_consumed);
	stream->reads_consumed = reads_consumed;

	up(&stream->read_mutex);
	EX_TRACE("Exit tlr_cdev_read %d\n", (int) bytes_read);
	return bytes_read;
}


static ssize_t
tlr_cdev_write(struct file *filp, const char __user *buf,
	       size_t count, loff_t *f_pos)
{
	ssize_t	ret;
	struct tlr_stream *stream = filp->private_data;
	struct tlr_pcie_dev *tlr = stream->dev;
	size_t bytes_written;
	size_t written;
	u32 writes_posted;

	ret = 0;
	FOP_TRACE("Entered \n");

	if (count == 0) {
		ret = 0;
		EX_TRACE("Exit \n");
		goto err_exit;
	}

	/* Grab the stream write lock. */
	if (down_interruptible(&stream->write_mutex)) {
		ret = -ERESTARTSYS;
		EX_TRACE("Exit \n");
		goto err_exit;
	}


	/* Abandon this operation if the stream is poisoned due to reset. */
	if (stream->chip_reset_poison) {
		up(&stream->write_mutex);
		ret = -ENXIO;
		EX_TRACE("Exit \n");
		goto err_exit;
	}

	/* Wait for a NULL write_buffer, indicating we can allocate and fill
	 * a new one.
	 */
	while ((stream->writes_posted - stream->writes_completed)
	       >= BUFFERS_PER_STREAM) {
		up(&stream->write_mutex);
		if (filp->f_flags & O_NONBLOCK) {
			EX_TRACE("Exit tlr_cdev_write -EAGAIN\n");
			return -EAGAIN;
		}

		/* Wait for the worker loop to indicate that we're ready
		 * for a new buffer.
		 */
		FOP_TRACE("Waiting on write_queue\n");
		if (wait_event_interruptible(stream->write_queue,
					     (((stream->writes_posted -
						stream->writes_completed) <
					       BUFFERS_PER_STREAM) ||
					      (stream->chip_reset_poison)))) {
			EX_TRACE("Exit tlr_cdev_write -ERESTARTSYS\n");
			return -ERESTARTSYS;
		}
		FOP_TRACE("Woke from write_queue\n");

		/* Get the write lock again. */
		if (down_interruptible(&stream->write_mutex)) {
			EX_TRACE("Exit tlr_cdev_write -ERESTARTSYS\n");
			return -ERESTARTSYS;
		}

		/* We could have been poisoned while sleeping. */
		if (stream->chip_reset_poison) {
			up(&stream->write_mutex);
			return -ENXIO;
		}
	}

	/* At this point we hold the write mutex and we know that
	 * there is at least one write buffer available.  Copy as much
	 * data as possible into buffers...
	 */
	bytes_written = 0;
	written = 0;
	for (writes_posted = stream->writes_posted;
	     writes_posted - stream->writes_completed < BUFFERS_PER_STREAM;
	     writes_posted++) {
		u32 index = writes_posted & (BUFFERS_PER_STREAM - 1);
		size_t size = min(count, (size_t)BUFFER_SIZE);

		if (size == 0)
			break;

		/* We need to sync each new buffer for CPU access. */
		dma_sync_single_for_cpu(&tlr->pci_dev->dev,
					stream->write_dma_addrs[index],
					BUFFER_SIZE, DMA_TO_DEVICE);

		stream->write_sizes[index] = size;
		if (copy_from_user(stream->write_buffers[index],
				   buf + written, size)) {
			if (written > 0)
				break;
			else {
				up(&stream->write_mutex);
				return -EFAULT;
			}
		}
		written += size;
		count -= size;

		/* The buffer is ready, sync back to the device. */
		dma_sync_single_for_device(&tlr->pci_dev->dev,
					   stream->write_dma_addrs[index],
					   BUFFER_SIZE, DMA_TO_DEVICE);
	}

	/* ...and then post the buffers. */
	post_write_buffers(stream, stream->writes_posted, writes_posted);
	stream->writes_posted = writes_posted;

	up(&stream->write_mutex);
	EX_TRACE("Exit tlr_cdev_write %d\n", (int) written);
	return written;

 err_exit:
	return(ret);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static int
tlr_cdev_ioctl(struct inode *inode, struct file *filp,
	       unsigned int cmd, unsigned long arg)
#else
static long
tlr_cdev_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
	struct tlr_stream *stream = filp->private_data;

	switch (cmd) {
	case TILEPCI_IOC_CHANNEL_RESET:

		stream->need_write_soc = 1;
		stream->need_read_soc = 1;

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static unsigned int
tlr_cdev_poll(struct file *filp, poll_table *table)
{
	struct tlr_stream *stream = filp->private_data;
	unsigned int result = 0;

	FOP_TRACE("Entered tlr_cdev_poll\n");

	/* Add wait queues to the poll table; we don't actually wait here. */
	poll_wait(filp, &stream->read_queue, table);
	poll_wait(filp, &stream->write_queue, table);

	/* Grab both the read and write semaphores so that this operation is
	 * ordered with respect to any other processes that may be reading
	 * or writing.  Are we allowed to return -ERESTARTSYS here?  Can't
	 * seem to find the appropriate documentation...
	 */
	if (down_interruptible(&stream->read_mutex)) {
		EX_TRACE("Exit tlr_cdev_poll\n");		
		return -ERESTARTSYS;
	}
	if (down_interruptible(&stream->write_mutex)) {
		up(&stream->read_mutex);
		EX_TRACE("Exit tlr_cdev_poll\n");		
		return -ERESTARTSYS;
	}

	if (stream->chip_reset_poison)
		result = POLLERR;
	else {
		if (stream->reads_consumed != stream->reads_completed)
			result |= (POLLIN | POLLRDNORM); /* readable */	
		if ((stream->writes_posted - stream->writes_completed) <
		    BUFFERS_PER_STREAM)
			result |= (POLLOUT | POLLWRNORM); /* writable */
	}

	up(&stream->write_mutex);
	up(&stream->read_mutex);

	EX_TRACE("Exit tlr_cdev_poll\n");		
	return result;
}

static int
tlr_cdev_release(struct inode* inode, struct file* filp)
{
	struct tlr_stream *stream = filp->private_data;

	/* Note that the stream is closed; clear the poison bit. */
	down(&stream->read_mutex);
	down(&stream->write_mutex);

	if (--stream->open_count == 0) {
		stream->chip_reset_poison = 0;
	}

	up(&stream->write_mutex);
	up(&stream->read_mutex);

	return 0;
}

static int tlr_cdev_open(struct inode *inode, struct file *filp);

struct file_operations tlr_cdev_ops = {
	.owner = THIS_MODULE,
	.open = tlr_cdev_open,
	.read = tlr_cdev_read,
	.write = tlr_cdev_write,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	.ioctl = tlr_cdev_ioctl,
#else
	.unlocked_ioctl = tlr_cdev_unlocked_ioctl,
#endif
	.poll = tlr_cdev_poll,
	.release = tlr_cdev_release,
};

static int
tlr_cdev_open(struct inode *inode, struct file *filp)
{
	int result = 0;
	struct tlr_pcie_dev* tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);
	struct tlr_stream *stream;
	int stream_index;	

	if (tlr->drv_mismatch)
		return -EPERM;

	/* Set the private data to point at our stream. */
	stream_index = MINOR(inode->i_rdev) - FIRST_PUBLIC_MINOR;
	stream = tlr_get_stream(tlr, stream_index);
	if (stream == NULL) {
		dev_warn(&tlr->pci_dev->dev,
			 "Char stream %d is not initialized\n", stream_index);
		return -ENXIO;
	}
	filp->private_data = stream;

	/* Use the stream read, write, etc. */
	filp->f_op = &tlr_cdev_ops;

	FOP_TRACE("Enter tlr_cdev_open\n");

	/* Initialize the stream if ready. */
	if (down_interruptible(&stream->read_mutex)) 
		return -ERESTARTSYS;
	if (down_interruptible(&stream->write_mutex)) {
		up(&stream->read_mutex);
		return -ERESTARTSYS;
	}

	/* If 'chip_reset_poison', some other file handle is
	 * still open on a dead stream; don't allow any more
	 * opens until that one goes away. */
	if (stream->chip_reset_poison)
		result = -ENXIO;

	/* Make sure the stream is actually initialized; we may have
	 * reset the chip and lost our connection. */
	else if (!stream->is_ready) 
		result = init_stream(stream);

	/* Advance the open counter in case of a successful stream 
	 * initialization, or if the stream is already initialized by 
	 * others. */
	if (!result) 
		++stream->open_count;

	up(&stream->write_mutex);
	up(&stream->read_mutex);

	return result;
}

/*
 * Reset all write (and read?) streams so that they contain no data.
 * Also, poison any open file handles so that the holders must close
 * and open a new session before getting more data.
 *
 * This method must be called while the following are true:
 *  - the caller holds all read and write semaphores
 *  - the worker thread has been descheduled
 *  - is_ready = 0 so that no interrupts will be processed
 *  - the chip has been reset but not booted
 */
static void
tlr_cdev_chip_reset(struct tlr_pcie_dev* tlr)
{
	int i;
	for (i = 0; i < NUM_CHAR_STREAMS; i++) {
		struct tlr_stream* stream = tlr_get_stream(tlr, i);

		/* Free irq. */
		if (stream->is_ready)
			tlr_free_irq(stream);

		release_stream(stream);
		if (stream->open_count > 0) {
			stream->chip_reset_poison = 1;

			wake_up_interruptible(&stream->read_queue);
			wake_up_interruptible(&stream->write_queue);
		}			
		stream->need_write_soc = 1;
		stream->need_read_soc = 1;
	}
}

/**********************************************************************/
/*                          Boot Device Routines                      */
/**********************************************************************/
static ssize_t tlr_boot_write(struct file *filp, const char __user *buf,
			      size_t count, loff_t *f_pos);
static int tlr_boot_release(struct inode* inode, struct file* filp);

struct file_operations tlr_boot_ops = {
	.owner = THIS_MODULE,
	.write = tlr_boot_write,
	.release = tlr_boot_release
};

/* Start of Maximum Link Width mask in Link Capabilities */
#define  PCI_EXP_LNKCAP_MLW_SHIFT	4
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
#define  PCI_EXP_LNKCAP_SLS	0x0000000f /* Supported Link Speeds */
#define  PCI_EXP_LNKCAP_MLW	0x000003f0 /* Maximum Link Width */
#endif

/*
 * Get the max link speed and width from the PCIe device capability registers.
 * The link speed and width parameters are specified according to the PCIe spec.
 */
static void
get_max_link_speed_width(struct pci_dev *dev, int *speed, int *width)
{
	int ppos;
	u32 link_cap;

	ppos = pci_find_capability(dev, PCI_CAP_ID_EXP);

	pci_read_config_dword(dev, ppos + PCI_EXP_LNKCAP, &link_cap);
	*speed = link_cap & PCI_EXP_LNKCAP_SLS;
	*width = (link_cap & PCI_EXP_LNKCAP_MLW) >> PCI_EXP_LNKCAP_MLW_SHIFT;
}

/*
 * This function calculates the trained link width and speed that are
 * expected, based on the switch downstream port's link capability.
 */
static void
tlr_get_expected_link_speed_width(struct tlr_pcie_dev* tlr)
{
	int dev_speed;
	int dev_width;
	int bridge_speed;
	int bridge_width;
	struct pci_dev *pci_dev = tlr->pci_dev;
	struct pci_dev *bridge_port = pci_dev->bus->self;

        get_max_link_speed_width(pci_dev, &dev_speed, &dev_width);
        get_max_link_speed_width(bridge_port, &bridge_speed, &bridge_width);

	tlr->expected_link_speed = min(dev_speed, bridge_speed);
	tlr->expected_link_width = min(dev_width, bridge_width);
}

/*
 * This function saves of PCI BIOS state that we may need to refresh
 * if the device is reset.
 */
static void
tlr_boot_save_link_state(struct tlr_pcie_dev* tlr)
{
	pci_save_state(tlr->pci_dev);
}

/* Save all PCI state on a reset board. */
static void
tlr_boot_save_state(tlr_board_t* board)
{
	int i;

	for (i = 0; i < board->num_ports; i++)
		tlr_boot_save_link_state(board->ports[i]);
}

/*
 * Attempt to restore the PCI BIOS state.
 */
static void
tlr_boot_restore_link_state(struct tlr_pcie_dev* tlr)
{
	pci_restore_state(tlr->pci_dev);
}


/* Restore all PCI state on a reset board. */
static void
tlr_boot_restore_state(tlr_board_t* board)
{
	int i;

	for (i = 0; i < board->num_ports; i++)
		tlr_boot_restore_link_state(board->ports[i]);
}

/* Helper function for releasing some or all the stream semaphores.
 * 'count' specifies the number of streams to be released; this will
 * up() streams 0 through count - 1.
 */
static void
release_stream_semaphores(struct tlr_pcie_dev* tlr, int count)
{
	int i;
	for (i = count - 1; i >= 0; i--) {
		struct tlr_stream* stream = tlr_get_stream(tlr, i);
		up(&stream->read_mutex);
		up(&stream->write_mutex);
	}  
}

/* Helper function for grabbing all the stream semaphores. */
static int
grab_all_stream_semaphores(struct tlr_pcie_dev* tlr)
{
	int i;
	for (i = 0; i < NUM_CHAR_STREAMS; i++) {
		struct tlr_stream* stream = tlr_get_stream(tlr, i);
		if (down_interruptible(&stream->write_mutex)) {
			release_stream_semaphores(tlr, i);
			return -ERESTARTSYS;
		}
		if (down_interruptible(&stream->read_mutex)) {
			up(&stream->write_mutex);
			release_stream_semaphores(tlr, i);
			return -ERESTARTSYS;
		}
	}
	return 0;
}


/* Helper function for grabbing all the stream and zero-copy semaphores. */
static int
grab_all_semaphores(struct tlr_pcie_dev* tlr)
{
	int err;

	err = grab_all_stream_semaphores(tlr);
	if (err)
		return err;

#if GXPCI_HOST_ZC_QUEUE_COUNT
	err = grab_all_zc_semaphores(tlr);
	if (err) {
		release_stream_semaphores(tlr, NUM_CHAR_STREAMS);
		return err;
	}
#endif
	return 0;	
}

/* Helper function for releasing semaphores taken by grab_all_semaphores(). */
static void
release_all_semaphores(struct tlr_pcie_dev* tlr)
{
#if GXPCI_HOST_ZC_QUEUE_COUNT
	release_zc_semaphores(tlr, GXPCI_HOST_ZC_QUEUE_COUNT);
#endif
	release_stream_semaphores(tlr, NUM_CHAR_STREAMS);
}

/* Lock down a board in preparation for boot.  This method will fail
 * if the board is already rebooting or another process already holds
 * the lock file.  On success, it will mark the board as in the
 * process of rebooting. */
static int
tlr_boot_lock_board(tlr_board_t* board)
{
	int i, j;
	int err;

	/* Hold the reboot mutex while we figure out whether or not we
	 * have permission to boot and whether we can reset the
	 * board. */
	if (down_interruptible(&reboot_mutex)) {
		EX_TRACE("Exit -ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	/* Determine whether the board is currently rebooting. */
	if (board->is_rebooting) {
		ERR("Denying boot access - currently rebooting\n");
		up(&reboot_mutex);
		return -EBUSY;
	}
	if (board->locked_pid >= 0 && board->locked_pid != current->pid) {
		ERR("Denying boot access - device is locked\n");
		up(&reboot_mutex);
		return -EBUSY;
	}
	if (board->packet_queue_cnt > 0) {
		ERR("Denying boot access - packet_queue file is open\n");
		up(&reboot_mutex);
		return -EBUSY;		
	}
	if (board->raw_dma_cnt > 0) {
		ERR("Denying boot access - raw_dma file is open\n");
		up(&reboot_mutex);
		return -EBUSY;		
	}
	if (board->barmem_cnt > 0) {
		ERR("Denying boot access - barmem file is open\n");
		up(&reboot_mutex);
		return -EBUSY;		
	}
	if (board->host_nic_cnt > 0) {
		ERR("Denying boot access - host_nic file is open\n");
		up(&reboot_mutex);
		return -EBUSY;		
	}

#ifdef CONFIG_PCI_IOV
	/*
	 * Prevent reboot if any VF is in use.
	 * We determine if a VF is in use by checking if its config command
	 * register has Bus Master Enable bit set, which is done when the
	 * VF driver is loaded on the VM.
	 */
	for (i = 0; i < board->num_ports; i++) {
		struct pci_dev *dev = board->ports[i]->pci_dev;
		struct pci_dev *pdev;
		u16 pci_command;

		list_for_each_entry(pdev, &dev->bus->devices, bus_list) {
			if (pdev->device == TILERA_GX36_DEV_ID)
				continue;
			pci_read_config_word(pdev, PCI_COMMAND, &pci_command);
			if (pci_command & PCI_COMMAND_MASTER) {
				ERR("Denying boot access - VFs in use, "
				    "need to unload the VF driver first\n");
				up(&reboot_mutex);
				return -EBUSY;		
			}
		}
	}
#endif

	/* Prevent any reads or writes from starting by grabbing all
	 * the semaphores. */
	for (i = 0; i < board->num_ports; i++) {
		err = grab_all_semaphores(board->ports[i]);
		if (err)
			goto fail_grab_sem;
	}

	/* Clear the is_ready bits. */
	for (i = 0; i < board->num_ports; i++) {
		spin_lock(&board->ports[i]->is_ready_lock);
		board->ports[i]->is_ready = 0;
		spin_unlock(&board->ports[i]->is_ready_lock);
	}

	/* Okay, everything's ready; commit to doing the reboot. */
	board->is_rebooting = TRUE;

	up(&reboot_mutex);

	return 0;

 fail_grab_sem:
	for (j = 0; j < i; j++)
		release_all_semaphores(board->ports[j]);

	up(&reboot_mutex);

	return err;
}


/* Release a locked board. */
static void
tlr_boot_unlock_board(tlr_board_t* board)
{
	int i;

	for (i = 0; i < board->num_ports; i++)
		release_all_semaphores(board->ports[i]);

	board->is_rebooting = FALSE;
}


/* Clear all communication state on a locked board so that any open
 * connections will be terminated. */
static void
tlr_boot_reset_connections(tlr_board_t* board)
{
	int nic;
	int i;

	for (i = 0; i < board->num_ports; i++) {
		struct tlr_pcie_dev *tlr = board->ports[i];

		/* TBD: Invalidate all C2C queues involving this port. */

		for (nic = 0; nic < GXPCI_HOST_NIC_COUNT; nic++)
			gxpci_net_dev_close(tlr, nic);

		tlr_cdev_chip_reset(tlr);
#if GXPCI_HOST_ZC_QUEUE_COUNT
		tlr_zc_chip_reset(tlr);
#endif
		/*
		 * Release the MSI/MSI-X resources we've allocated.
		 * The intr vectors should have been freed by their
		 * respective owners, i.e. PCIe queues.
		 */
		if (tlr->msix_vectors)
			pci_disable_msix(tlr->pci_dev);
		else
			pci_disable_msi(tlr->pci_dev);
	}
}

/*
 * Reset registers TRIO_PCIE_INTFC_RX_BAR0_ADDR_MASK and TRIO_MAP_RSH_BASE,
 * by triggering a RSHIM SWINT3 interrupt to tile.
 */
static void
tlr_reset_bar_mask(struct tlr_pcie_dev* tlr)
{
	rshim_reg_write(tlr, RSH_SWINT, RSH_INT_VEC0_RTC__SWINT3_MASK);
}

/*
 * Reset the board if necessary, making sure to restore any
 * configuration information that was setup by BIOS.
 */
static int
tlr_boot_reset_board(tlr_board_t* board)
{
	int err;

	/* Reset any connections that are open across chip reset. */
	tlr_boot_reset_connections(board);

	/* Make sure the board's links are up before we attempt to do
	 * anything.  This might be necessary if somebody pushed the
	 * manual reset button, did a manual rshim reset, or otherwise
	 * reset the device without going through proper reset flow
	 * performed by this driver. */
	if (board->is_link_down(board)) {
		ERR("PCI link down; cannot boot.\n");
		return -ENODEV;
	}

	/*
	 * Save the device PCIe state before resetting it. The current
	 * state has some differences with the one saved at the probe()
	 * time, e.g. the Max_Read_Request_Size and the MSI/MSI-X info.
	 */
	if (gxpci_reset_all)
		tlr_boot_save_state(board);

	/* Perform a reset if the board registers indicate it's
	 * already booted or we happen to know that we've already
	 * injected a bootstream. */
	if (board->has_booted(board) ||
	    board->need_reset) {
		err = board->do_reset(board);
		if (err)
			return err;
		else
			board->need_reset = 0;
	}

	/* Restore the PCIe states. */
	if (gxpci_reset_all)
		tlr_boot_restore_state(board);

	return 0;
}


/* Reset the chip and get it ready to boot.  This function also
 * disables reading and writing of the stream devices until booting is
 * complete. */
static int
tlr_boot_open(struct inode *inode, struct file *filp)
{
	struct tlr_pcie_dev* tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);
	tlr_board_t* board = tlr->board;
	int err;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
	cancel_delayed_work_sync(&tlr->ep_queue_monitor_work);
#else
	if (!cancel_delayed_work(&tlr->ep_queue_monitor_work))
		flush_scheduled_work();
#endif

	/*
	 * Clear the driver version mismatch flag, because it could be
	 * checked before it is properly set after the Gx EP driver starts.
	 */
	tlr->drv_mismatch = 0;

	/* Set the private data to point at our state object. */
	filp->private_data = tlr;

	/* Use the stream read, write, etc. */
	filp->f_op = &tlr_boot_ops;

	/* Lock down the board. */
	err = tlr_boot_lock_board(board);
	if (err)
		goto fail_pre_lock;

	/* Reset the board. */
	board->need_reset = 1;
	err = tlr_boot_reset_board(board);
	if (err)
		goto fail_post_lock;

	TRACE("Boot success.\n");
	return 0;

 fail_post_lock:
	tlr_boot_unlock_board(tlr->board);
 fail_pre_lock:
	TRACE("Boot failed  %d.\n", err);

	return err;
}


/*
 * Each word written to the boot device is injected into BAR 0.
 */
#define RSH_BOOT_MAX_LOOPS	200
static ssize_t
tlr_boot_write(struct file *filp, const char __user * buf,
	       size_t count, loff_t * f_pos)
{
	struct tlr_pcie_dev *tlr = filp->private_data;
	size_t size = count & -8;
	size_t bytes_remain;
	size_t write_bytes;
	size_t written = 0;

	uint64_t words_comp;
	uint64_t words_sent;
	uint64_t credits;
	int loops = RSH_BOOT_MAX_LOOPS;

	if (size == 0)
		return 0;

	if (!access_ok(VERIFY_READ, buf, size))
		return -EFAULT;

	bytes_remain = size;

	/*
	 * Get the initial count.  This assumes previous calls waited
	 * for the boot stream to complete.  Since other agents (BTK) can
	 * change the pg_ctl between writes to the boot file, we need
	 * to re-read PG_CTL here and assume that boot fifo is empty.
	 */
	words_comp = ((rshim_reg_read(tlr, RSH_PG_CTL) >>
		       RSH_PG_CTL__SENT_COUNT_SHIFT) &
		      RSH_PG_CTL__SENT_COUNT_RMASK);
	words_sent = words_comp;
	credits = PCIE_BOOT_FIFO_WORDS;

	while (bytes_remain) {
		int i;

		write_bytes = min((size_t) (credits << 3), bytes_remain);

		/* Copy the bootrom chunk from userspace. */
		if (copy_from_user
		    ((char *) tlr->boot_buf, (buf + written), write_bytes))
			return -EFAULT;

		for (i = 0; i < (write_bytes >> 3); i++)
			rshim_reg_write(tlr, RSH_PG_DATA, tlr->boot_buf[i]);

		bytes_remain -= write_bytes;
		written += write_bytes;
		words_sent = ((words_sent + (write_bytes >> 3)) &
			      RSH_PG_CTL__SENT_COUNT_RMASK);

		/* 
		 * We implement flow control to prevent boot transactions
		 * from backing up onto the PCIe link. The FC consists of
		 * polling the RSHIM's Packet Generator data-words-sent
		 * counter to see how much data has been sent.
		 * Rather than waiting for just 1 credit, we send bursts of 
		 * PCIE_BOOT_COPY_CHUNK size to improve bus utilization and
		 * amortize any overhead.
		 */
		do {
			words_comp =
			    ((rshim_reg_read(tlr, RSH_PG_CTL) >>
			      RSH_PG_CTL__SENT_COUNT_SHIFT) &
			     RSH_PG_CTL__SENT_COUNT_RMASK);
			credits =
			    (PCIE_BOOT_FIFO_WORDS -
			     ((words_sent -
			       words_comp) &
			      RSH_PG_CTL__SENT_COUNT_RMASK));
			if (loops-- == 0) {
				if (msleep_interruptible(5))
					return -EINTR;

				loops = RSH_BOOT_MAX_LOOPS;
			}
		} while (credits < (PCIE_BOOT_COPY_CHUNK >> 3));
	}

	/* 
	 * Wait for all remaining data to be sent so subsequent call
	 * properly tracks credits. 
	 */
	while (words_comp != words_sent) {
		words_comp = ((rshim_reg_read(tlr, RSH_PG_CTL) >>
			       RSH_PG_CTL__SENT_COUNT_SHIFT) &
			      RSH_PG_CTL__SENT_COUNT_RMASK);
		if (loops-- == 0) {
			if (msleep_interruptible(5))
				return -EINTR;

			loops = RSH_BOOT_MAX_LOOPS;
		}
	}

	return written;
}

static int
tlr_boot_release(struct inode* inode, struct file* filp)
{
	struct tlr_pcie_dev* tlr = filp->private_data;
	tlr_board_t* board = tlr->board;
	int i;

	tlr_boot_unlock_board(board);

	for (i = 0; i < board->num_ports; i++) {
		/*
		 * Schedule task to set up the EP link after the EP
		 * completes boot.
		 */
		schedule_delayed_work(&board->ports[i]->ep_setup_work,
				      EP_DRV_READY_TIMEOUT);

		/* Schedule work to monitor various queues' status. */
		schedule_delayed_work(&tlr->ep_queue_monitor_work,
				      GXPCI_QUEUE_MONITOR_INTERVAL);
	}

	return 0;
}


/**********************************************************************/
/*                          Device Lock Routines                      */
/**********************************************************************/

static int tlr_lock_release(struct inode* inode, struct file* filp);

struct file_operations tlr_lock_ops = {
	.owner = THIS_MODULE,
	.release = tlr_lock_release
};

/*
 * The lock file allows a particular process to lock access to the
 * boot device, preventing any other process from booting the Tile via
 * PCIe.
 */
static int
tlr_lock_open(struct inode *inode, struct file *filp)
{
	int result = -EBUSY;
	struct tlr_pcie_dev* tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);
	tlr_board_t* board = tlr->board;

	/* Set the private data to point at our state object. */
	filp->private_data = board;

	/* Use the tlr_lock_release() method. */
	filp->f_op = &tlr_lock_ops;

	/* Synchronize access to 'is_rebooting' and 'locked_pid' so
	 * that we don't race with somebody opening the boot file. */
	if (down_interruptible(&reboot_mutex))
		return -ERESTARTSYS;

	if (board->is_rebooting) {
		dev_err(&tlr->pci_dev->dev, "Attempted to lock "
			"board while reboot is in flight.\n");
		result = -EBUSY;
	}
	if (board->locked_pid < 0) {
		board->locked_pid  = current->pid;
		result = 0;
	}
	up(&reboot_mutex);

	return result;
}

static int
tlr_lock_release(struct inode* inode, struct file* filp)
{
	tlr_board_t* board = filp->private_data;

	/* Synchronize access to 'is_rebooting' and 'locked_pid' so
	 * that we don't race with somebody opening the boot file. */
	down(&reboot_mutex);

	board->locked_pid = -1;

	up(&reboot_mutex);

	return 0;
}


/**********************************************************************/
/*                        Chip Information File                       */
/**********************************************************************/

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
#define  PCI_EXP_LNKSTA_CLS     0x000f  /* Current Link Speed */
#define  PCI_EXP_LNKSTA_NLW     0x03f0  /* Nogotiated Link Width */
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
#define  PCI_EXP_LNKSTA_NLW_SHIFT 4     /* start of NLW mask in link status */
#endif

/* Get the link speed and width from the PCI Config header. */
void
get_link_speed_width(struct pci_dev *dev)
{
	struct tlr_pcie_dev *tlr; 
	int pcie_caps_offset;
	int link_status_offset;
	u16 link_status;

	/* Find the link status regs. */
	pcie_caps_offset = pci_find_capability(dev, PCI_CAP_ID_EXP);
	if (pcie_caps_offset == 0) {
		dev_err(&dev->dev,
			"Could not read PCI-Express capability regs.\n");
	}

	tlr = pci_get_drvdata(dev);

	/* Check whether we actually got at least the specified link BW. */
	link_status_offset = pcie_caps_offset + PCI_EXP_LNKSTA;
	pci_read_config_word(dev, link_status_offset, &link_status);
	tlr->link_speed = (link_status & PCI_EXP_LNKSTA_CLS);
	tlr->link_width = (link_status & PCI_EXP_LNKSTA_NLW) >>
			  PCI_EXP_LNKSTA_NLW_SHIFT;
}

static int
tlr_info_seq_show(struct seq_file *s, void *token)
{
	struct tlr_pcie_dev *tlr = s->private;
	int i;

	get_link_speed_width(tlr->pci_dev);

	seq_printf(s, "CHIP_VERSION %d\n", tlr->chip_version);
	seq_printf(s, "CHIP_WIDTH %d\n", tlr->chip_width);
	seq_printf(s, "CHIP_HEIGHT %d\n", tlr->chip_height);
	seq_printf(s, "BOARD_INDEX %d\n", tlr->board->board_index);
	seq_printf(s, "BOARD_LINKS %d\n", tlr->board->num_ports);
	seq_printf(s, "PORT_INDEX %d\n", tlr->port_index);
	seq_printf(s, "HOST_LINK_INDEX %d\n", tlr->link_index);
	seq_printf(s, "LINK_SPEED_GEN %d\n", tlr->link_speed);
	seq_printf(s, "LINK_WIDTH %d\n", tlr->link_width);
	seq_printf(s, "HOST_VNIC_PORTS %d\n", tlr->nic_ports);
	seq_printf(s, "HOST_VNIC_TX_QUEUES %d\n", tlr->nic_tx_queues);
	seq_printf(s, "HOST_VNIC_RX_QUEUES %d\n", tlr->nic_rx_queues);
	seq_printf(s, "HOST_PQ_H2T_QUEUES %d\n", tlr->pq_h2t_queues);
	seq_printf(s, "HOST_PQ_T2H_QUEUES %d\n", tlr->pq_t2h_queues);

	for (i = 0; i < tlr->pq_h2t_queues; i++) {
		seq_printf(s, "HOST_PQ_H2T_QUEUE %d:\n", i);
		seq_printf(s, "  RING_BUF_SIZE %ldB\n",
			   tlr->pq_h2t[i].ring_size);
		seq_printf(s, "  RING_BUF_NUMA_NODE %d\n",
			   tlr->pq_h2t[i].numa_node);
		seq_printf(s, "  RING_BUF_SHARED %s\n",
			   tlr->pq_h2t[i].share_h2t_t2h_ring ? "YES" : "NO");
	}

	for (i = 0; i < tlr->pq_t2h_queues; i++) {
		seq_printf(s, "HOST_PQ_T2H_QUEUE %d:\n", i);
		seq_printf(s, "  RING_BUF_SIZE %ldB\n",
			   tlr->pq_t2h[i].ring_size);
		seq_printf(s, "  RING_BUF_NUMA_NODE %d\n",
			   tlr->pq_t2h[i].numa_node);
		seq_printf(s, "  RING_BUF_SHARED %s\n",
			   tlr->pq_t2h[i].share_h2t_t2h_ring ? "YES" : "NO");
	}

	seq_printf(s, "HOST_RAW_DMA_H2T_QUEUES %d\n", tlr->rd_h2t_queues);
	seq_printf(s, "HOST_RAW_DMA_T2H_QUEUES %d\n", tlr->rd_t2h_queues);

	for (i = 0; i < tlr->rd_h2t_queues; i++) {
		seq_printf(s, "HOST_RAW_DMA_H2T_QUEUE %d:\n", i);
		seq_printf(s, "  RING_BUF_SIZE %ldB\n",
			   tlr->rd_h2t[i].buf_size);
		seq_printf(s, "  RING_BUF_NUMA_NODE %d\n",
			   tlr->rd_h2t[i].numa_node);
	}

	for (i = 0; i < tlr->rd_t2h_queues; i++) {
		seq_printf(s, "HOST_RAW_DMA_T2H_QUEUE %d:\n", i);
		seq_printf(s, "  RING_BUF_SIZE %ldB\n",
			   tlr->rd_t2h[i].buf_size);
		seq_printf(s, "  RING_BUF_NUMA_NODE %d\n",
			   tlr->rd_t2h[i].numa_node);
	}

	return 0;
}

static struct file_operations tlr_info_ops = {
	.owner          = THIS_MODULE,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int
tlr_info_open(struct tlr_pcie_dev *tlr, struct file *filp)
{
	filp->f_op = &tlr_info_ops;
	return single_open(filp, tlr_info_seq_show, tlr);
}


static void
fill_chip_version(struct tlr_pcie_dev* tlr)
{
	switch (tlr->pci_dev->device) {
	case TILERA_GX36_DEV_ID:
		tlr->chip_version = 10;
		break;
	default:
		tlr->chip_version = -1;
	}
}

static void
fill_chip_size(struct tlr_pcie_dev* tlr)
{
	u64 fabric_dim;
	int col;

	tlr->chip_width = 0;
	tlr->chip_height = 0;

	if (!tlr->rshim_regs)
		return;

	//
	// First set the height and width to the fabric height and width.
	//
	fabric_dim = rshim_reg_read(tlr, RSH_FABRIC_DIM);
	tlr->chip_width = (fabric_dim >> RSH_FABRIC_DIM__DIM_X_SHIFT) &
			  RSH_FABRIC_DIM__DIM_X_RMASK;
	tlr->chip_height = (fabric_dim >> RSH_FABRIC_DIM__DIM_Y_SHIFT) &
			   RSH_FABRIC_DIM__DIM_Y_RMASK;

	//
	// Now examine the tile disable bits to see if the actual tile grid
	// is smaller than the fabric.  We know that the usable tiles form
	// a rectangle which contains tile (0,0).
	//
	for (col = 0; col < tlr->chip_width; col++) {
		//
		// Walk through the columns.  If we hit a column that's
		// totally disabled, set our width to that column's index,
		// and stop walking, since we won't find any other tiles.
		// Otherwise, clip our height based on the number of tiles
		// in this column, and keep going.
		//
		u64 thiscol_dis;
		int tiles;

		thiscol_dis = rshim_reg_read(tlr,
					     RSH_TILE_COL_DISABLE + 8 * col);

		thiscol_dis |= ~(((u64) 1 << tlr->chip_height) - 1);
		tiles = __builtin_ctz(thiscol_dis);

		if (tiles == 0) {
			tlr->chip_width = col;
			break;
		}
		tlr->chip_height = min(tlr->chip_height, tiles);
	}

}

/**********************************************************************/
/*                             RShim Access                           */
/**********************************************************************/

/*
 * Access to the RSHIM registers is provided through the RSHIM's byte access
 * interface, for the following reasons:
 * 1. It works on both 64-bit and 32-bit hosts.
 * 2. It doesn't have any read side effect.
 * There is practically no noticeable difference in performance between
 * this implementation and the native 64-bit access method.
 * Note that these functions can't be invoked from the interrupt context due to 
 * the sleep call, and there is no known reason to access the RSHIM registers
 * from the interrupt context.
 */
u64
rshim_reg_read(struct tlr_pcie_dev *tlr, u32 reg)
{
	u64 val;
	spinlock_t *lock;
	static DEFINE_SPINLOCK(rshim_lock);
	tlr_board_t *board = tlr->board;

	/*
	 * This is a hack to solve the chicken-and-egg problem, where we might
	 * need to read the RSHIM Scratch register while holding
	 * board->rshim_reg_lock before the board structure can be allocated.
	 *
	 * No protection is needed for the board == NULL check because board
	 * can only be NULL in the device probe stage which is serialized by
	 * by the PCI core layer.
	 */
	if (board == NULL)
		lock = &rshim_lock;
	else
		lock = &board->rshim_reg_lock;

	while (!spin_trylock(lock)) {
		if (msleep_interruptible(1))
			return -EINTR;
	}

	while (readl(tlr->rshim_regs + RSH_BYTE_ACC_CTL) &
		     RSH_BYTE_ACC_CTL__PENDING_MASK) {
	}

	/* Now we are ready to go. */
	writel(RSH_BYTE_ACC_CTL__SIZE_VAL_4BYTE <<
		RSH_BYTE_ACC_CTL__SIZE_SHIFT,
		tlr->rshim_regs + RSH_BYTE_ACC_CTL);
	writel(reg, tlr->rshim_regs + RSH_BYTE_ACC_ADDR);
	writel((RSH_BYTE_ACC_CTL__SIZE_VAL_4BYTE <<
		RSH_BYTE_ACC_CTL__SIZE_SHIFT) |
		(1 << RSH_BYTE_ACC_CTL__READ_TRIG_SHIFT),
		tlr->rshim_regs + RSH_BYTE_ACC_CTL);

	while (readl(tlr->rshim_regs + RSH_BYTE_ACC_CTL) &
		     RSH_BYTE_ACC_CTL__PENDING_MASK) {
	}

	val = readl(tlr->rshim_regs + RSH_BYTE_ACC_RDAT);

	while (readl(tlr->rshim_regs + RSH_BYTE_ACC_CTL) &
		     RSH_BYTE_ACC_CTL__PENDING_MASK) {
	}

	val += ((u64)readl(tlr->rshim_regs + RSH_BYTE_ACC_RDAT)) << 32;

	spin_unlock(lock);

	return val;
}

void
rshim_reg_write(struct tlr_pcie_dev *tlr, u32 reg, u64 val)
{
	spinlock_t *lock;
	static DEFINE_SPINLOCK(rshim_lock);
	tlr_board_t *board = tlr->board;

	/*
	 * This is a hack to solve the chicken-and-egg problem, where we might
	 * need to write the RSHIM Scratch register while holding
	 * board->rshim_reg_lock before the board structure can be allocated.
	 *
	 * No protection is needed for the board == NULL check because board
	 * can only be NULL in the device probe stage which is serialized by
	 * by the PCI core layer.
	 */
	if (board == NULL)
		lock = &rshim_lock;
	else
		lock = &board->rshim_reg_lock;

	while (!spin_trylock(lock)) {
		if (msleep_interruptible(1))
			return;
	}

	while (readl(tlr->rshim_regs + RSH_BYTE_ACC_CTL) &
		     RSH_BYTE_ACC_CTL__PENDING_MASK) {
	}

	/* Now we are ready to go. */
	writel(RSH_BYTE_ACC_CTL__SIZE_VAL_4BYTE <<
		RSH_BYTE_ACC_CTL__SIZE_SHIFT,
		tlr->rshim_regs + RSH_BYTE_ACC_CTL);
	writel(reg, tlr->rshim_regs + RSH_BYTE_ACC_ADDR);
	writel(RSH_BYTE_ACC_CTL__SIZE_VAL_4BYTE <<
		RSH_BYTE_ACC_CTL__SIZE_SHIFT,
		tlr->rshim_regs + RSH_BYTE_ACC_CTL);

	writel(val, tlr->rshim_regs + RSH_BYTE_ACC_WDAT);

	while (readl(tlr->rshim_regs + RSH_BYTE_ACC_CTL) &
		     RSH_BYTE_ACC_CTL__PENDING_MASK) {
	}

	writel(val >> 32, tlr->rshim_regs + RSH_BYTE_ACC_WDAT);

	spin_unlock(lock);
}

static ssize_t
rshim_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct tlr_pcie_dev *tlr = filp->private_data;
	loff_t pos = *f_pos;
	u64 value;

	/* rshim registers are all 8-byte aligned. */
	if ((pos & 0x7) || (count != sizeof(u64)) ||
	    ((pos + count) > GXPCI_RSHIM_WINDOW_SIZE))
		return -EINVAL;

	value = rshim_reg_read(tlr, pos);
	if (copy_to_user(buf, &value, count))
		return -EFAULT;

	return count;
}

static ssize_t
rshim_write(struct file *filp, const char __user *buf,
	    size_t count, loff_t *f_pos)
{
	struct tlr_pcie_dev *tlr = filp->private_data;
	loff_t pos = *f_pos;
	u64 value;

	/* rshim registers are all 8-byte aligned. */
	if ((pos & 0x7) || (count != sizeof(u64)) ||
	    ((pos + count) > GXPCI_RSHIM_WINDOW_SIZE))
		return -EINVAL;

	if (copy_from_user(&value, buf, count))
		return -EFAULT;

	rshim_reg_write(tlr, pos, value);

	return count;
}

static struct file_operations rshim_file_ops = {
	.owner          = THIS_MODULE,
	.read           = rshim_read,
	.write          = rshim_write,
};

static int
tlr_rshim_open(struct tlr_pcie_dev *tlr, struct file *filp)
{
	filp->f_op = &rshim_file_ops;
	filp->private_data = tlr;

	return 0;
}


/**********************************************************************/
/*                   Module Loading and Device Probe                  */
/**********************************************************************/

static struct pci_device_id ids[] = {
	{ PCI_DEVICE(TILERA_VENDOR_ID, TILERA_GX36_DEV_ID), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, ids);

static int
tlr_generic_open(struct inode *inode, struct file *filp)
{
	unsigned int minor = MINOR(inode->i_rdev);
	int result;
	int channel;
	struct tlr_pcie_dev *tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);

	FOP_TRACE("Enter minor %d\n", minor);

	if ((minor >= FIRST_PUBLIC_MINOR) &&
	    (minor <= LAST_PUBLIC_MINOR)) {
		result = tlr_cdev_open(inode, filp);
	}
#if GXPCI_HOST_ZC_QUEUE_COUNT
	else if ((minor >= FIRST_ZC_H2T_MINOR) &&
		 (minor <= LAST_ZC_H2T_MINOR)) {
		result = tlr_zc_open(inode, filp);
	}
	else if ((minor >= FIRST_ZC_T2H_MINOR) &&
		 (minor <= LAST_ZC_T2H_MINOR)) {
		result = tlr_zc_open(inode, filp);
	}
#endif
	else if (minor == TILEPCI_BOOT_MINOR) {
		result = tlr_boot_open(inode, filp);
	}
	else if (minor == TILEPCI_LOCK_MINOR) {
		result = tlr_lock_open(inode, filp);
	}
	else if (minor == TILEPCI_INFO_MINOR) {
		result = tlr_info_open(tlr, filp);
	}
	else if (minor == TILEPCI_RSHIM_MINOR) {
		result = tlr_rshim_open(tlr, filp);
	}
	else if ((minor >= TILEPCI_FIRST_PQ_H2T_MINOR) &&
		 (minor <= (TILEPCI_FIRST_PQ_H2T_MINOR +
		 tlr->pq_h2t_queues - 1))) {
		channel = (minor - TILEPCI_FIRST_PQ_H2T_MINOR);
		result = tlr_packet_queue_open(tlr, filp, tlr->pq_h2t, channel);
	}
	else if ((minor >= TILEPCI_FIRST_PQ_T2H_MINOR) &&
		 (minor <= (TILEPCI_FIRST_PQ_T2H_MINOR +
		 tlr->pq_t2h_queues - 1))) {
		channel = (minor - TILEPCI_FIRST_PQ_T2H_MINOR);
		result = tlr_packet_queue_open(tlr, filp, tlr->pq_t2h, channel);
	}
	else if ((minor >= TILEPCI_FIRST_RAW_DMA_RX_MINOR) &&
		 (minor <= (TILEPCI_FIRST_RAW_DMA_RX_MINOR +
		 GXPCI_RAW_DMA_QUEUE_COUNT - 1))) {
		channel = (minor - TILEPCI_FIRST_RAW_DMA_RX_MINOR);
		result = tlr_raw_dma_open(tlr, filp, tlr->rd_h2t, channel);
	}
	else if ((minor >= TILEPCI_FIRST_RAW_DMA_TX_MINOR) &&
		 (minor <= (TILEPCI_FIRST_RAW_DMA_TX_MINOR +
		 GXPCI_RAW_DMA_QUEUE_COUNT - 1))) {
		channel = (minor - TILEPCI_FIRST_RAW_DMA_TX_MINOR);
		result = tlr_raw_dma_open(tlr, filp, tlr->rd_t2h, channel);
	}
	else if (minor == TILEPCI_BARMEM_MINOR) {
		result = tlr_barmem_open(inode, filp);
	}
	else {
		result = -ENODEV;
	}

	EX_TRACE("Exit result %d\n", result);

	return result;
}

struct file_operations tlr_generic_ops = {
	.owner = THIS_MODULE,
	.open = tlr_generic_open,
};

static int
tlr_add_device_nodes(struct tlr_pcie_dev* tlr)
{
	struct cdev* cdev = &tlr->cdev;

	/* Allocate some major/minor numbers. */
	dev_t first;
	int num_devs = NUM_MINOR_DEVICES;
	int err = alloc_chrdev_region(&first, 0, num_devs, (char*)driver_name);
	if (err != 0)
		return err;

	TRACE("Added device nodes at %0Xh for driver %s\n",(unsigned int)first,
	      driver_name);

	/* Register the device. */
	cdev_init(cdev, &tlr_generic_ops);
	cdev->owner = THIS_MODULE;
	err = cdev_add(cdev, first, num_devs);
	if (err != 0)
	{
		TRACE("Failed to add %d.\n", err);
		unregister_chrdev_region(first, num_devs);
		return err;
	}
	TRACE("Registered at %0Xh %d devices\n",(unsigned int)first, num_devs);
	tlr->first_dev = first;
	return 0;
}

static void
tlr_remove_device_nodes(struct tlr_pcie_dev* tlr)
{
	cdev_del(&tlr->cdev);
	unregister_chrdev_region(tlr->first_dev, NUM_MINOR_DEVICES);
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
static void ep_manager(struct work_struct *work)
#else
static void ep_manager(void *data)
#endif
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
	struct tlr_pcie_dev *tlr =
		container_of(work, struct tlr_pcie_dev, ep_setup_work.work);
#else
	struct tlr_pcie_dev *tlr = (struct tlr_pcie_dev *)data;
#endif
	unsigned int ep_status;

	/* If the EP is not booted yet, don't issue any MMIO. */
	if (gx_ep_not_booted(tlr)) {
		schedule_delayed_work(&tlr->ep_setup_work, 
				      EP_DRV_READY_TIMEOUT);
		return;
	}

	ep_status = readl(&tlr->regs->port_status);

	/* If this EP is ready, set it up for host communication. */
	if (ep_status == PCIE_STATUS_READY) {
		int link_index = tlr->link_index;
		int i; 

		/* Check the compatibility of PCIe driver. */
		if (tlr_check_version(tlr))
			return;

		/* Set the number of host NIC queues supported. */
		writel(tlr->nic_rx_queues, &tlr->regs->nic_t2h_queues);
		writel(tlr->nic_tx_queues, &tlr->regs->nic_h2t_queues);

		/* Set the number of Raw DMA queues supported. */
		writel(tlr->rd_t2h_queues, &tlr->regs->raw_dma_t2h_queues);
		writel(tlr->rd_h2t_queues, &tlr->regs->raw_dma_h2t_queues);

		/* Inform the EP of the queue status array bus address. */
		writeq(tlr->queue_sts_bus_addr,
		       &tlr->regs->queue_sts_array_bus_addr);

		/* Inform the EP of the C2C queue status array bus address. */
		if (c2c_queue_sts_array_bus_addr)
			writeq(c2c_queue_sts_array_bus_addr,
				&tlr->regs->c2c_queue_sts_array_bus_addr);

		writel(pci_resource_len(tlr->pci_dev, 0),
		       &tlr->regs->bar0_size);
		writel(pci_resource_len(tlr->pci_dev, 2),
		       &tlr->regs->bar2_size);

		/* Save this EP's BAR0 and BAR2 address. */
		c2c_queue_sts_array_addr->bar0_addr[link_index] =
			tlr->bar0_address;
		c2c_queue_sts_array_addr->bar2_addr[link_index] =
			tlr->bar2_address;

		writeq(tlr->bar0_address, &tlr->regs->bar0_addr);
		writeq(tlr->bar2_address, &tlr->regs->bar2_addr);
		writel(tlr->bar0_size, &tlr->regs->bar0_size);
		writel(tlr->bar2_size, &tlr->regs->bar2_size);
#ifdef CONFIG_PCI_IOV
		writel(tlr->vf_bar0_size, &tlr->regs->vf_bar0_size);
		writel(tlr->vf_bar2_size, &tlr->regs->vf_bar2_size);
#endif

		/*
		 * Enable MSI-X or MSI.
		 * Ideally we want to have one interrupt vector per queue.
		 * If that is not possible, we fall back to using MSI with
		 * a single vector, instead of mapping the allocated vectors
		 * to the queues.
		 */
#ifndef USE_MSI_ONLY
		for (i = 0; i < tlr->msix_vectors; i++)
			tlr->msix_entries[i].entry = i;

		i = pci_enable_msix(tlr->pci_dev, tlr->msix_entries,
				    tlr->msix_vectors);
		if (i) {
			/* MSI-X allocation failed. */
			dev_err(&tlr->pci_dev->dev,
				"MSI-X allocation failure: %d, try MSI\n", i);
#endif
			tlr->msix_vectors = 0;
			i = pci_enable_msi(tlr->pci_dev);
			if (i) {
				dev_err(&tlr->pci_dev->dev,
					"MSI allocation failure: %d, "
					"give up\n", i);
				return;
			}
#ifndef USE_MSI_ONLY
		}
#endif

		spin_lock(&tlr->is_ready_lock);
		tlr->is_ready = 1;
		spin_unlock(&tlr->is_ready_lock);

		/* Write the link index to BAR0. */
		writel(link_index, &tlr->regs->link_index);

	} else
		schedule_delayed_work(&tlr->ep_setup_work, 
				      EP_DRV_READY_TIMEOUT);
}

static int
tlr_parse_config(struct tlr_pcie_dev* tlr)
{
	int total_nic_queues, i;
#ifdef RAW_DMA_USE_RESERVED_MEMORY
	unsigned long raw_dma_t2h_buf_addr = 0;
	unsigned long raw_dma_h2t_buf_addr = 0;
	unsigned long raw_dma_t2h_queue_size = 0;
	unsigned long raw_dma_h2t_queue_size = 0;
	unsigned long raw_dma_buf_size = 0;
#endif
	char rd_param_string[1024] = "";
	char *rd_sub_opt_ptr;
	char *rd_opt_ptr;
	char *rd_ptr;
	char *tmp;
	int ret;

	ret = pq_parse_config(tlr);
	if (ret < 0)
		return ret;

	/* Raw DMA H2T/T2H queue number configurations. */
	if (rd_h2t_queues > GXPCI_RAW_DMA_QUEUE_COUNT) {
		ERR("Raw DMA H2T queue number invalid, use default value.\n");
		rd_h2t_queues = GXPCI_RAW_DMA_QUEUE_COUNT;
	}
	if (rd_t2h_queues > GXPCI_RAW_DMA_QUEUE_COUNT) {
		ERR("Raw DMA T2H queue number invalid, use default value.\n");
		rd_t2h_queues = GXPCI_RAW_DMA_QUEUE_COUNT;
	}
	tlr->rd_h2t_queues = rd_h2t_queues;
	tlr->rd_t2h_queues = rd_t2h_queues;

#ifdef RAW_DMA_USE_RESERVED_MEMORY
	/* Raw DMA buffer size. */
	if (rd_h2t_buf_size) {
		raw_dma_buf_size = memparse(rd_h2t_buf_size, &tmp);
		if (!raw_dma_buf_size) {
			ERR("Raw DMA H2T buffer size invalid, quit.\n");
			return -EINVAL;
		}
		raw_dma_h2t_queue_size = raw_dma_buf_size / tlr->rd_h2t_queues;
	}

	if (rd_t2h_buf_size) {
		raw_dma_buf_size = memparse(rd_t2h_buf_size, &tmp);
		if (!raw_dma_buf_size) {
			ERR("Raw DMA T2H buffer size invalid, quit.\n");
			return -EINVAL;
		}
		raw_dma_t2h_queue_size = raw_dma_buf_size / tlr->rd_t2h_queues;
	}

	/* Raw DMA buffer base address. */
	if (rd_h2t_buf_base_addr) {
		raw_dma_h2t_buf_addr = memparse(rd_h2t_buf_base_addr, &tmp);
		if (!raw_dma_h2t_buf_addr) {
			ERR("Raw DMA H2T buffer address invalid, quit.\n");
			return -EINVAL;
		}
	}

	if (rd_t2h_buf_base_addr) {
		raw_dma_t2h_buf_addr = memparse(rd_t2h_buf_base_addr, &tmp);
		if (!raw_dma_t2h_buf_addr) {
			ERR("Raw DMA T2H buffer address invalid, quit.\n");
			return -EINVAL;
		}
	}

	/* Raw DMA H2T queue buffer configurations. */
	if (rd_h2t_mems)
		strncpy(rd_param_string, rd_h2t_mems, sizeof(rd_param_string));
	rd_ptr = rd_param_string;

	for (i = 0; i < tlr->rd_h2t_queues; i++) {

		/* Initialize default configurations. */
		tlr->rd_h2t[i].buf_size = raw_dma_h2t_queue_size;
		tlr->rd_h2t[i].rd_mem_handle =
			raw_dma_h2t_buf_addr + i * raw_dma_h2t_queue_size;

		if ((rd_opt_ptr = strsep(&rd_ptr, ":")) != NULL) {
			rd_sub_opt_ptr = strsep(&rd_opt_ptr, "@");
			if (rd_sub_opt_ptr)
				tlr->rd_h2t[i].buf_size =
					memparse(rd_sub_opt_ptr, &tmp);
			if (rd_opt_ptr)
				tlr->rd_h2t[i].rd_mem_handle =
					memparse(rd_opt_ptr, &tmp);
		}
	}

	/* Raw DMA T2H queue buffer configurations. */
	if (rd_t2h_mems)
		strncpy(rd_param_string, rd_t2h_mems, sizeof(rd_param_string));
	rd_ptr = rd_param_string;

	for (i = 0; i < tlr->rd_t2h_queues; i++) {

		/* Initialize default configurations. */
		tlr->rd_t2h[i].buf_size = raw_dma_t2h_queue_size;
		tlr->rd_t2h[i].rd_mem_handle =
			raw_dma_t2h_buf_addr + i * raw_dma_t2h_queue_size;

		if ((rd_opt_ptr = strsep(&rd_ptr, ":")) != NULL) {
			rd_sub_opt_ptr = strsep(&rd_opt_ptr, "@");
			if (rd_sub_opt_ptr)
				tlr->rd_t2h[i].buf_size =
					memparse(rd_sub_opt_ptr, &tmp);
			if (rd_opt_ptr)
				tlr->rd_t2h[i].rd_mem_handle =
					memparse(rd_opt_ptr, &tmp);
		}
	}

#else

	/* Raw DMA H2T queue buffer configurations. */
	if (rd_h2t_mems)
		strncpy(rd_param_string, rd_h2t_mems, sizeof(rd_param_string));
	rd_ptr = rd_param_string;

	for (i = 0; i < tlr->rd_h2t_queues; i++) {

		/* Initialize default configurations. */
		tlr->rd_h2t[i].buf_size = HOST_RD_SEGMENT_MAX_SIZE;
		tlr->rd_h2t[i].numa_node = numa_node_id();

		if ((rd_opt_ptr = strsep(&rd_ptr, ":")) != NULL) {
			rd_sub_opt_ptr = strsep(&rd_opt_ptr, ",");
			if (rd_sub_opt_ptr)
				tlr->rd_h2t[i].buf_size =
					memparse(rd_sub_opt_ptr, &tmp);
			rd_sub_opt_ptr = strsep(&rd_opt_ptr, ",");
			if (rd_sub_opt_ptr)
				tlr->rd_h2t[i].numa_node =
					simple_strtol(rd_sub_opt_ptr, NULL, 0);
		}

		/* Error handlings. */
		if (tlr->rd_h2t[i].buf_size == 0)
			tlr->rd_h2t[i].buf_size = HOST_RD_SEGMENT_MAX_SIZE;

		if (tlr->rd_h2t[i].buf_size % HOST_RD_SEGMENT_MAX_SIZE) {
			ERR("RD ring size not "
			    "HOST_RD_SEGMENT_MAX_SIZE-aligned, fixed.\n");
			tlr->rd_h2t[i].buf_size =
				ALIGN(tlr->rd_h2t[i].buf_size,
				      HOST_RD_SEGMENT_MAX_SIZE);
		}

		if (!node_online(tlr->rd_h2t[i].numa_node)) {
			ERR("RD NUMA node invalid, use default value.\n");
			tlr->rd_h2t[i].numa_node = numa_node_id();
		}

		tlr->rd_h2t[i].num_segments =
			tlr->rd_h2t[i].buf_size / HOST_RD_SEGMENT_MAX_SIZE;
	}

	/* Raw DMA T2H queue buffer configurations. */
	if (rd_t2h_mems)
		strncpy(rd_param_string, rd_t2h_mems, sizeof(rd_param_string));
	rd_ptr = rd_param_string;

	for (i = 0; i < tlr->rd_t2h_queues; i++) {

		/* Initialize default configurations. */
		tlr->rd_t2h[i].buf_size = HOST_RD_SEGMENT_MAX_SIZE;
		tlr->rd_t2h[i].numa_node = numa_node_id();

		if ((rd_opt_ptr = strsep(&rd_ptr, ":")) != NULL) {
			rd_sub_opt_ptr = strsep(&rd_opt_ptr, ",");
			if (rd_sub_opt_ptr)
				tlr->rd_t2h[i].buf_size =
					memparse(rd_sub_opt_ptr, &tmp);
			rd_sub_opt_ptr = strsep(&rd_opt_ptr, ",");
			if (rd_sub_opt_ptr)
				tlr->rd_t2h[i].numa_node =
					simple_strtol(rd_sub_opt_ptr, NULL, 0);
		}

		/* Error handlings. */
		if (tlr->rd_t2h[i].buf_size == 0)
			tlr->rd_t2h[i].buf_size = HOST_RD_SEGMENT_MAX_SIZE;

		if (tlr->rd_t2h[i].buf_size % HOST_RD_SEGMENT_MAX_SIZE) {
			ERR("RD ring size not "
			    "HOST_RD_SEGMENT_MAX_SIZE-aligned, fixed.\n");
			tlr->rd_t2h[i].buf_size =
				ALIGN(tlr->rd_t2h[i].buf_size,
				      HOST_RD_SEGMENT_MAX_SIZE);
		}

		if (!node_online(tlr->rd_t2h[i].numa_node)) {
			ERR("RD NUMA node invalid, use default value.\n");
			tlr->rd_t2h[i].numa_node = numa_node_id();
		}

		tlr->rd_t2h[i].num_segments =
			tlr->rd_t2h[i].buf_size / HOST_RD_SEGMENT_MAX_SIZE;
	}

#endif

	/* NIC queue configurations. */
	if (nic_ports <= 0 || nic_ports > GXPCI_HOST_NIC_COUNT) {
		dev_err(&tlr->pci_dev->dev,
			"Illegal host NIC port number, default "
			"configuration will be used instead.\n");
		nic_ports = GXPCI_HOST_NIC_COUNT;
	}

	if (nic_tx_queues != nic_rx_queues) {
		dev_err(&tlr->pci_dev->dev,
			"NIC queue number not equal, use min value.\n");
		nic_tx_queues = MIN(nic_tx_queues, nic_rx_queues);
		nic_rx_queues = nic_tx_queues;
	}

	total_nic_queues = nic_ports * nic_tx_queues;
	if (total_nic_queues <= 0 ||
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
	    nic_tx_queues > GXPCI_HOST_NIC_SIMPLEX_QUEUES_MAX ||
	    nic_rx_queues > GXPCI_HOST_NIC_SIMPLEX_QUEUES_MAX) {
#else /* Older kernel supports single Tx/Rx queue only. */
	    nic_tx_queues > 1 || nic_rx_queues > 1) {
#endif
		dev_err(&tlr->pci_dev->dev,
			"NIC queue number invalid, use default value.\n");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
		nic_tx_queues = GXPCI_HOST_NIC_TX_QUEUES;
		nic_rx_queues = GXPCI_HOST_NIC_RX_QUEUES;
#else
		nic_tx_queues = 1;
		nic_rx_queues = 1;
#endif
	}
	tlr->nic_ports = nic_ports;
	tlr->nic_tx_queues = nic_tx_queues;
	tlr->nic_rx_queues = nic_rx_queues;

	return 0;
}

static int 
tlr_char_stream_init(struct tlr_pcie_dev *tlr)
{
	struct pci_dev *pci_dev = tlr->pci_dev;
	struct tlr_stream *stream;
	int num_devs = GXPCI_HOST_CHAR_STREAMS_COUNT;
	int err = -ENOMEM;
	int i;

	/* Char stream init. */
	for (i = 0; i < num_devs; i++) {
		stream = tlr_get_stream(tlr, i);
		stream = (tlr_stream_t*)kmalloc(sizeof(*stream), 
						GFP_KERNEL);
		if (stream == NULL) 
			goto stream_alloc_failed;

		tlr_set_stream(tlr, i, stream);
		memset(stream, 0, sizeof(*stream));

		stream->dev = tlr;
		stream->pci_dev = pci_dev;
		stream->index = i;

		init_MUTEX(&stream->write_mutex);
		init_waitqueue_head(&stream->write_queue);

		init_MUTEX(&stream->read_mutex);
		init_waitqueue_head(&stream->read_queue);

		/* 
		 * Get some memory for the device's command and completion 
		 * arrays. 
		 */
		stream->h2t_buffer_cmd_array =
			pci_alloc_consistent(pci_dev, 
					     PCIE_HOST_BUFFER_CMD_ARRAY_SIZE,
					     &stream->h2t_buffer_cmd_handle);
		stream->t2h_buffer_cmd_array =
			pci_alloc_consistent(pci_dev, 
					     PCIE_HOST_BUFFER_CMD_ARRAY_SIZE,
					     &stream->t2h_buffer_cmd_handle);
		stream->h2t_completion_array =
			pci_alloc_consistent(pci_dev, 
					     PCIE_HOST_COMPLETION_ARRAY_SIZE,
					     &stream->h2t_completion_handle);
		stream->t2h_completion_array =
			pci_alloc_consistent(pci_dev, 
					     PCIE_HOST_COMPLETION_ARRAY_SIZE,
					     &stream->t2h_completion_handle);

		if ((stream->h2t_buffer_cmd_array == 0) || 
		    (stream->t2h_buffer_cmd_array == 0) || 
		    (stream->h2t_completion_array == 0) ||
		    (stream->t2h_completion_array == 0))
			goto array_alloc_failed;

		memset(stream->h2t_buffer_cmd_array, 0, 
		       PCIE_HOST_BUFFER_CMD_ARRAY_SIZE);
		memset(stream->t2h_buffer_cmd_array, 0, 
		       PCIE_HOST_BUFFER_CMD_ARRAY_SIZE);
		memset(stream->h2t_completion_array, 0, 
		       PCIE_HOST_COMPLETION_ARRAY_SIZE);
		memset(stream->t2h_completion_array, 0, 
		       PCIE_HOST_COMPLETION_ARRAY_SIZE);

		/* Map in the host queue registers. */
		err = -EIO;
		stream->h2t_regs = ioremap(pci_resource_start(pci_dev, 0) +
				       GXPCI_HOST_CHAR_STREAMS_H2T_REGS_OFFSET +
				       GXPCI_HOST_CHAR_REGS_MAP_SIZE * i,
				       sizeof(struct gxpci_host_queue_regs));
		if (stream->h2t_regs == NULL)
			goto ioremap_failed;

		stream->t2h_regs = ioremap(pci_resource_start(pci_dev, 0) +
				       GXPCI_HOST_CHAR_STREAMS_T2H_REGS_OFFSET +
				       GXPCI_HOST_CHAR_REGS_MAP_SIZE * i,
				       sizeof(struct gxpci_host_queue_regs));
		if (stream->t2h_regs == NULL)
			goto ioremap_failed;

		/* Map in the interrupt triggering bus address. */ 
		stream->intr_regs = ioremap(pci_resource_start(pci_dev, 0) +
			GXPCI_HOST_CHAR_STREAMS_MMI_REGS_OFFSET,
			sizeof(uint64_t));
		if (stream->intr_regs == NULL)
			goto ioremap_intr_failed;

		continue;

ioremap_intr_failed:
ioremap_failed:
array_alloc_failed:
	if (stream->h2t_buffer_cmd_array)
		pci_free_consistent(pci_dev, PCIE_HOST_BUFFER_CMD_ARRAY_SIZE,
				    stream->h2t_buffer_cmd_array,
				    stream->h2t_buffer_cmd_handle);

	if (stream->t2h_buffer_cmd_array)
		pci_free_consistent(pci_dev, PCIE_HOST_BUFFER_CMD_ARRAY_SIZE,
				    stream->t2h_buffer_cmd_array,
				    stream->t2h_buffer_cmd_handle);

	if (stream->h2t_completion_array)
		pci_free_consistent(pci_dev, PCIE_HOST_COMPLETION_ARRAY_SIZE,
				    stream->h2t_completion_array,
				    stream->h2t_completion_handle);

	if (stream->t2h_completion_array)
		pci_free_consistent(pci_dev, PCIE_HOST_COMPLETION_ARRAY_SIZE,
				    stream->t2h_completion_array,
				    stream->t2h_completion_handle);

	kfree(stream);
	tlr_set_stream(tlr, i, NULL);
stream_alloc_failed:
		break;
	}

	if (i > 0)
		return i;
	else 
		return err;
}

static void 
tlr_char_stream_remove(struct tlr_pcie_dev *tlr)
{
	struct pci_dev *pci_dev = tlr->pci_dev;
	struct tlr_stream *stream;
	int num_devs = GXPCI_HOST_CHAR_STREAMS_COUNT;
	int i;

	/* Char stream release. */
	for (i = 0; i < num_devs; i++) {
		stream = tlr_get_stream(tlr, i);

		if (stream != NULL) {
			iounmap(stream->h2t_regs);
			iounmap(stream->t2h_regs);
			iounmap(stream->intr_regs);

			pci_free_consistent(pci_dev, 
					    PCIE_HOST_BUFFER_CMD_ARRAY_SIZE,
					    stream->h2t_buffer_cmd_array,
					    stream->h2t_buffer_cmd_handle);

			pci_free_consistent(pci_dev, 
					    PCIE_HOST_BUFFER_CMD_ARRAY_SIZE,
					    stream->t2h_buffer_cmd_array,
					    stream->t2h_buffer_cmd_handle);

			pci_free_consistent(pci_dev, 
					    PCIE_HOST_COMPLETION_ARRAY_SIZE,
					    stream->h2t_completion_array,
					    stream->h2t_completion_handle);

			pci_free_consistent(pci_dev, 
					    PCIE_HOST_COMPLETION_ARRAY_SIZE,
					    stream->t2h_completion_array,
					    stream->t2h_completion_handle);

			kfree(stream);
			tlr_set_stream(tlr, i, NULL);
		}
	}
}

/*
 * Define scheduled work that is called periodically to monitor the
 * data transfer queues' status, e.g. readiness and reset. This task
 * relays queue status changes between the two ends of the queues,
 * because the two ends don't have direct visibility into each other's
 * status due to the fact that it is impractical to set up PIO windows
 * just for queue event monitoring. For each queue, there are two
 * places that maintain its status information: queue-pair status array
 * maintained in RC memory for each Gx EP port, and the queue flag that
 * is accessable to the user-space. The device release functions modify
 * the flgas in the queue-pair status array only and don't touch the
 * user-accessable queue flags which are set by this monitor function
 * based on the flags in the queue-pair status array.
 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
static void
ep_queue_monitor(struct work_struct *work)
#else
static void
ep_queue_monitor(void *data)
#endif
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
	struct tlr_pcie_dev *tlr = container_of(work, struct tlr_pcie_dev,
						ep_queue_monitor_work.work);
#else
	struct tlr_pcie_dev *tlr = (struct tlr_pcie_dev *)data;
#endif
	struct gxpci_queue_pair_status *queue_pair_status;
#ifdef GXPCI_HOST_NIC_P2P
	struct gxpci_nic *nic;
#endif
	unsigned long flags;
	int queue;

	spin_lock_irqsave(&tlr->lock, flags);

	pq_queue_monitor(tlr);

	/* Monitor the Raw DMA queues. */
	for (queue = 0; queue < GXPCI_RAW_DMA_QUEUE_COUNT; queue++) {
		struct gxpci_host_rd_regs_drv *tile_regs;

		/* Look at T2H queues first. */
		queue_pair_status = &tlr->queue_sts_array->rd_sts_t2h[queue];

		/*
		 * If both ends has GXPCI_CHAN_RESET, both are ready for
		 * reset and we set GXPCI_CHAN_RESET_ACK to both. Note that
		 * this is where both queues should end up under normal
		 * conditions.
		 */
		if (queue_pair_status->tile_queue_status ==
			GXPCI_CHAN_RESET &&
			queue_pair_status->host_queue_status ==
			GXPCI_CHAN_RESET) {
			queue_pair_status->tile_queue_status =
				GXPCI_CHAN_RESET_ACK;
			queue_pair_status->host_queue_status =
				GXPCI_CHAN_RESET_ACK;
		} else if (queue_pair_status->tile_queue_status == 
				GXPCI_CHAN_RESET) {
			/*
			 * Tile side initiated the reset. We alert the host
			 * queue by setting GXPCI_CHAN_RESET to the host
			 * user-visible status.
			 */
			struct tlr_rd_status *queue_status;

			queue_status = tlr->rd_t2h[queue].queue_status;

			/*
			 * If the host side app hasn't started, do nothing
			 * here and the tile side's release function would
			 * time out and return.
			 */
			if (queue_pair_status->host_queue_opened &&
				(queue_status->status !=
				GXPCI_CHAN_UNINITIALIZED)) {
				queue_status->status = GXPCI_CHAN_RESET;

				/*
				 * Upon detecting GXPCI_CHAN_RESET, the user app
				 * should call close() to synchronize the reset
				 * sequence between the two ends.
				 */
			}
		} else if (queue_pair_status->host_queue_status == 
				GXPCI_CHAN_RESET) {
			/*
			 * Check for the special case where the tile application
			 * exits due to a signal. The tile app, via the EP
			 * driver, sets host_queue_status to GXPCI_CHAN_RESET
			 * before setting tile_queue_status to
			 * GXPCI_CHAN_UNINITIALIZED. Here we just notify the
			 * host app of the reset event.
			 *
			 * Note that we fall through to the common case.
			 */
			if (queue_pair_status->tile_queue_status ==
				GXPCI_CHAN_UNINITIALIZED) {
				struct tlr_rd_status *queue_status;

				queue_status = tlr->rd_t2h[queue].queue_status;

				/*
				 * If the host app hasn't started yet,
				 * don't set GXPCI_CHAN_RESET to its queue
				 * status.
				 */
				if (queue_pair_status->host_queue_opened)
					queue_status->status = GXPCI_CHAN_RESET;

				/*
				 * Clear the host_queue_status, otherwise it
				 * will cause the next tile app to be reset.
				 */
				queue_pair_status->host_queue_status =
					GXPCI_CHAN_UNINITIALIZED;

				/*
				 * Upon detecting GXPCI_CHAN_RESET, the user app
				 * should call close() to synchronize the reset
				 * sequence between the two ends.
				 */
			}

			/*
			 * Host side initiated the reset. We alert the tile
			 * queue by setting GXPCI_CHAN_RESET to the tile
			 * user-visible status. If the host app hasn't started
			 * yet, do nothing here. For the sake of correctness,
			 * do not write to the tile's status if the tile app
			 * hasn't started yet.
			 */
			if (queue_pair_status->host_queue_opened &&
			    queue_pair_status->tile_queue_opened) {
				tile_regs = tlr->rd_t2h[queue].regs;
				writel(GXPCI_CHAN_RESET,
				       &tile_regs->queue_status);
			}

			/*
			 * Upon detecting GXPCI_CHAN_RESET, the user app
			 * should call close() to synchronize the reset
			 * sequence between the two ends.
			 */
		}

		/* Look at H2T queues. */
		queue_pair_status = &tlr->queue_sts_array->rd_sts_h2t[queue];

		/*
		 * If both ends has GXPCI_CHAN_RESET, both are ready for
		 * reset and we set GXPCI_CHAN_RESET_ACK to both. Note that
		 * this is where both queues should end up under normal
		 * conditions.
		 */
		if (queue_pair_status->tile_queue_status ==
			GXPCI_CHAN_RESET &&
			queue_pair_status->host_queue_status ==
			GXPCI_CHAN_RESET) {
			queue_pair_status->tile_queue_status =
				GXPCI_CHAN_RESET_ACK;
			queue_pair_status->host_queue_status =
				GXPCI_CHAN_RESET_ACK;
		} else if (queue_pair_status->tile_queue_status == 
				GXPCI_CHAN_RESET) {
			/*
			 * Tile side initiated the reset. We alert the host
			 * queue by setting GXPCI_CHAN_RESET to the host
			 * user-visible status.
			 */
			struct tlr_rd_status *queue_status;

			queue_status = tlr->rd_h2t[queue].queue_status;

			/*
			 * If the host side app hasn't started, do nothing
			 * here and the tile side's release function would
			 * time out and return.
			 */
			if (queue_pair_status->host_queue_opened &&
				(queue_status->status !=
				GXPCI_CHAN_UNINITIALIZED)) {
				queue_status->status = GXPCI_CHAN_RESET;

				/*
				 * Upon detecting GXPCI_CHAN_RESET, the user app
				 * should call close() to synchronize the reset
				 * sequence between the two ends.
				 */
			}
		} else if (queue_pair_status->host_queue_status == 
				GXPCI_CHAN_RESET) {
			/*
			 * Check for the special case where the tile application
			 * exits due to a signal. The tile app, via the EP
			 * driver, sets host_queue_status to GXPCI_CHAN_RESET
			 * before setting tile_queue_status to
			 * GXPCI_CHAN_UNINITIALIZED. Here we just notify the
			 * host app of the reset event.
			 *
			 * Note that we fall through to the common case.
			 */
			if (queue_pair_status->tile_queue_status ==
				GXPCI_CHAN_UNINITIALIZED) {
				struct tlr_rd_status *queue_status;

				queue_status = tlr->rd_h2t[queue].queue_status;

				/*
				 * If the host app hasn't started yet,
				 * don't set GXPCI_CHAN_RESET to its queue
				 * status.
				 */
				if (queue_pair_status->host_queue_opened)
					queue_status->status = GXPCI_CHAN_RESET;

				/*
				 * Clear the host_queue_status, otherwise it
				 * will cause the next tile app to be reset.
				 */
				queue_pair_status->host_queue_status =
					GXPCI_CHAN_UNINITIALIZED;

				/*
				 * Upon detecting GXPCI_CHAN_RESET, the user app
				 * should call close() to synchronize the reset
				 * sequence between the two ends.
				 */
			}

			/*
			 * Host side initiated the reset. We alert the tile
			 * queue by setting GXPCI_CHAN_RESET to the tile
			 * user-visible status. If the host app hasn't started
			 * yet, do nothing here. For the sake of correctness,
			 * do not write to the tile's status if the tile app
			 * hasn't started yet.
			 */
			if (queue_pair_status->host_queue_opened &&
			    queue_pair_status->tile_queue_opened) {
				tile_regs = tlr->rd_h2t[queue].regs;
				writel(GXPCI_CHAN_RESET,
 				       &tile_regs->queue_status);
			}

			/*
			 * Upon detecting GXPCI_CHAN_RESET, the user app
			 * should call close() to synchronize the reset
			 * sequence between the two ends.
			 */
		}
	}

#ifndef GXPCI_NETLIB_VNIC 
	/* Monitor the host NIC queues. */
	for (queue = 0; queue < tlr->nic_ports; queue++) {

#ifndef GXPCI_HOST_NIC_P2P
		queue_pair_status = &tlr->queue_sts_array->nic_sts[queue];

		/*
		 * If both ends has GXPCI_CHAN_RESET, both are ready for
		 * reset and we set GXPCI_CHAN_RESET_ACK to both. Note that
		 * this is where both queues should end up under normal
		 * conditions.
		 */
		if (queue_pair_status->tile_queue_status == GXPCI_CHAN_RESET &&
		    queue_pair_status->host_queue_status == GXPCI_CHAN_RESET) {
			queue_pair_status->tile_queue_status =
				GXPCI_CHAN_RESET_ACK;
			queue_pair_status->host_queue_status =
				GXPCI_CHAN_RESET_ACK;
		} else if (queue_pair_status->tile_queue_status ==
				GXPCI_CHAN_RESET) {
			/*
			 * Tile side initiated the reset. We bring down
			 * the NIC interface with dev_close() which sets
			 * GXPCI_CHAN_RESET for the host queue status.
			 */

			spin_unlock_irqrestore(&tlr->lock, flags);
			gxpci_net_dev_close(tlr, queue);
			spin_lock_irqsave(&tlr->lock, flags);

		} else if (queue_pair_status->host_queue_status == 
				GXPCI_CHAN_RESET) {
			/*
			 * Host NIC driver initiated the reset and should have
			 * set GXPCI_CHAN_RESET to the tile queue status
			 * register. Upon detecting GXPCI_CHAN_RESET, the user
			 * app should call close() to synchronize the reset
			 * sequence between the two ends. Therefore there is
			 * nothing to be done here.
			 */
		}
#else
		nic = tlr->net_devs[queue];

		/* Go out if NIC doesn't exist or if its regs are unmapped. */
		if (!nic || !nic->nic_regs)
			continue;

		/* If the EP is not booted yet, don't issue any MMIO. */
		if (gx_ep_not_booted(tlr))
			continue;

		queue_pair_status = &nic->nic_regs->queue_pair_status;

		/* Both host and Tile endpoint are ready. */ 
		if (readl(&queue_pair_status->tile_queue_status) ==
			GXPCI_TILE_CHAN_READY &&
		    readl(&queue_pair_status->host_queue_status) ==
			GXPCI_HOST_CHAN_READY) {

			if (nic->channel_state == __GXPCI_NIC_DOWN) {

				INFO("%s: Link is up\n", nic->netdev->name);

				/* Mark host NIC port as UP. */
				nic->channel_state = __GXPCI_NIC_UP;

				/* Clear carrier/operstate OFF/DOWN states. */
				clear_bit(__GXPCI_DOWN, &nic->state);
				netif_carrier_on(nic->netdev);

				/* Trigger the Tile side MMI interrupt. */
				writeq(1, nic->intr_regs);
			}

		/* Host performs channel reset once Tile is DOWN. */
		} else if (readl(&queue_pair_status->tile_queue_status) ==
				GXPCI_CHAN_RESET) {

			if (nic->channel_state == __GXPCI_NIC_UP) {

				INFO("%s: Link is down\n", nic->netdev->name);

				/* Mark host NIC port as DOWN. */
				nic->channel_state = __GXPCI_NIC_DOWN;

				/* Reset host NIC port. */
				gxpci_reset(nic);
			}

			/* Notify the Tile side of the host reset ack. */
			writel(GXPCI_CHAN_RESET_ACK,
			       &queue_pair_status->tile_queue_status);
		}
#endif
	}
#endif

	spin_unlock_irqrestore(&tlr->lock, flags);

	/* Do it again later. */
	schedule_delayed_work(&tlr->ep_queue_monitor_work,
			      GXPCI_QUEUE_MONITOR_INTERVAL);
}

/*
 * Define scheduled work that is called periodically to monitor the C2C
 * data transfer queues' status, e.g. readiness and reset. This task
 * relays queue status changes between the two ends of the queues,
 * because the two ends don't have direct visibility into each other's
 * status due to the fact that it is impractical to set up PIO windows
 * just for queue event monitoring. For each queue, there are two
 * places that maintain its status information: queue-pair status array
 * maintained in RC memory for each PCI domain, and the queue flag that
 * is accessable to the user-space. The device release functions modify
 * the flgas in the queue-pair status array only and don't touch the
 * user-accessable queue flags which are set by this monitor function
 * based on the flags in the queue-pair status array.
 *
 * Unlike the ep_queue_monitor, which has one instance per Gx EP port
 * in each PCIe domain and is used to monitor queues between the Gx port
 * and the x86 host, there is a single c2c_queue_monitor instance per
 * PCIe domain.
 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
static void
c2c_queue_monitor(struct work_struct *work)
#else
static void
c2c_queue_monitor(void *data)
#endif
{
	struct tlr_c2c_status *send_port_status;
	struct tlr_c2c_status *recv_port_status;
	struct gxpci_c2c_queue_sts *queue_sts;
	struct tlr_pcie_dev *send_port;
	struct tlr_pcie_dev *recv_port;
	uint16_t send_link_index;
	uint16_t recv_link_index;
	int queue;

	if (c2c_queue_sts_array_addr == NULL)
		return;

	for (queue = 0; queue < GXPCI_C2C_QUEUE_COUNT; queue++) {

		queue_sts = &c2c_queue_sts_array_addr->c2c_queue_sts[queue];

		if (!queue_sts->active)
			continue;

		send_link_index = queue_sts->sender_link_index;
		recv_link_index = queue_sts->receiver_link_index;
		send_port = gx_ep_ports[send_link_index - 1];
		recv_port = gx_ep_ports[recv_link_index - 1];
		send_port_status = &send_port->regs->c2c_send_status[queue];
		recv_port_status = &recv_port->regs->c2c_recv_status[queue];

		/*
		 * If both ends has GXPCI_CHAN_RESET, indicating that
		 * both ends are ready for reset, we set GXPCI_CHAN_RESET_ACK
		 * on both sides and invalidate the queue. Note that
		 * this is where both queues should end up under normal
		 * conditions.
		 */
		if ((queue_sts->sender_status == GXPCI_CHAN_RESET) &&
			(queue_sts->receiver_status == GXPCI_CHAN_RESET)) {

			queue_sts->sender_status = GXPCI_CHAN_RESET_ACK;
			queue_sts->receiver_status = GXPCI_CHAN_RESET_ACK;

			queue_sts->active = 0;

		} else if (queue_sts->sender_status == GXPCI_CHAN_RESET) {
			/*
			 * Check for the special case where the receiver
			 * exits due to a signal. The receiver app, via the EP
			 * driver, sets sender_status to GXPCI_CHAN_RESET
			 * before setting receiver_status to
			 * GXPCI_CHAN_UNINITIALIZED. Here we just notify the
			 * sender app of the reset event.
			 */
			if (queue_sts->receiver_status ==
				GXPCI_CHAN_UNINITIALIZED) {

				writel(GXPCI_CHAN_RESET,
				       &send_port_status->status);

				/* Deactivate the queue. */
				queue_sts->active = 0;

				/*
				 * Upon detecting GXPCI_CHAN_RESET, the sender
				 * should call close() to synchronize the reset
				 * sequence between the two ends.
				 */
			} else {
				/*
				 * Sender initiated the reset. Alert receiver
				 * by setting GXPCI_CHAN_RESET to the receiver's
				 * user-visible status.
				 */
				writel(GXPCI_CHAN_RESET,
				       &recv_port_status->status);

				/*
				 * Upon detecting GXPCI_CHAN_RESET, the receiver
				 * should call close() to synchronize the reset
				 * sequence between the two ends.
			 	 */
			}
		} else if (queue_sts->receiver_status == GXPCI_CHAN_RESET) {
			/*
			 * Check for the special case where the sender
			 * exits due to a signal. The sender app, via the EP
			 * driver, sets receiver_status to GXPCI_CHAN_RESET
			 * before setting sender_status to
			 * GXPCI_CHAN_UNINITIALIZED. Here we just notify the
			 * receiver app of the reset event.
			 */
			if (queue_sts->sender_status ==
				GXPCI_CHAN_UNINITIALIZED) {

				writel(GXPCI_CHAN_RESET,
				       &recv_port_status->status);

				/* Deactivate the queue. */
				queue_sts->active = 0;

				/*
				 * Upon detecting GXPCI_CHAN_RESET, the receiver
				 * should call close() to synchronize the reset
				 * sequence between the two ends.
				 */
			} else {
				/*
				 * Receiver initiated the reset. Alert sender
				 * by setting GXPCI_CHAN_RESET to the sender's
				 * user-visible status.
				 */
				writel(GXPCI_CHAN_RESET,
				       &send_port_status->status);

				/*
				 * Upon detecting GXPCI_CHAN_RESET, the sender
				 * should call close() to synchronize the reset
				 * sequence between the two ends.
			 	 */
			}
		}
	}

	/* Do it again later. */
	schedule_delayed_work(&c2c_queue_monitor_work,
			      GXPCI_QUEUE_MONITOR_INTERVAL);
}

#if defined(CONFIG_UIO) || defined(CONFIG_UIO_MODULE)

static void gxpci_uio_devs_init(struct tlr_pcie_dev *tlr)
{
	struct uio_info *info = &tlr->uio;
	struct pci_dev  *pdev = tlr->pci_dev;
	unsigned long addr, len;
	int mems, res, err;
	void *internal_addr;

	memset(info, 0, sizeof(*info));
	info->priv = tlr;
	info->name = "tilegx";
	info->version = kasprintf(GFP_KERNEL, "%d.%d",
				  TILEGXPCI_MAJOR, TILEGXPCI_MINOR);
	info->irq = UIO_IRQ_NONE;

	/* expose all regions */
	for (res = 0, mems = 0; mems < MAX_UIO_MAPS; res++) {
		addr = pci_resource_start(pdev, res);
		len = pci_resource_len(pdev, res);
		if (!addr || !len)
			break;
		internal_addr = ioremap(addr, len);
		if (!internal_addr) {
			dev_err(&pdev->dev, "failed to map resource %d\n",
				res);
			continue;
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
		info->mem[mems].name = kasprintf(GFP_KERNEL, "region-%d", res);
#endif
		info->mem[mems].addr = addr;
		info->mem[mems].internal_addr = internal_addr;
		info->mem[mems].size = len;
		info->mem[mems].memtype = UIO_MEM_PHYS;
		mems++;
	}

	err = uio_register_device(&pdev->dev, info);

	if (err) {
		dev_err(&pdev->dev, "failed to register uio device\n");

		/* unmap regions */
		for (mems--; mems >= 0; mems--)
			iounmap(info->mem[mems].internal_addr);
	} else {
		dev_info(&pdev->dev, "registered uio device with %d maps\n",
			 mems);
	}
}

static void gxpci_uio_devs_remove(struct tlr_pcie_dev *tlr)
{
	struct uio_info *info = &tlr->uio;
	struct pci_dev  *pdev = tlr->pci_dev;
	int mem;

	uio_unregister_device(info);
	for (mem = 0; mem < ARRAY_SIZE(info->mem); mem++) {
		iounmap(info->mem[mem].internal_addr);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
		kfree(info->mem[mem].name);
#endif
	}
	kfree(info->version);
	dev_info(&pdev->dev, "unregistered uio device\n");
}

#endif

static int
probe(struct pci_dev *pci_dev, const struct pci_device_id *id)
{
	int			 err;
	struct tlr_pcie_dev	*tlr;
	int			 host_nic_queue_vectors;
	int			 i;
#if EXTENDED_TAG_ENABLE
	int			 pcie_caps_offset;
#endif

	TRACE("Probing\n");

	/* Our device can use 64-bit DMA addresses. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
	if (!pci_set_dma_mask(pci_dev, DMA_64BIT_MASK) &&
	    !pci_set_consistent_dma_mask(pci_dev, DMA_64BIT_MASK)) {
		TRACE("Using 64-bit PCI DMA addressing\n");
	} else if ((err = pci_set_dma_mask(pci_dev, DMA_32BIT_MASK)) ||
		   (err = pci_set_consistent_dma_mask(pci_dev,
						      DMA_32BIT_MASK))) {
		dev_err(&pci_dev->dev,
			"Failed to find usable DMA configuration\n");
		return err;
	}
#else
	if (!pci_set_dma_mask(pci_dev, DMA_BIT_MASK(64))) {
		err = pci_set_consistent_dma_mask(pci_dev, DMA_BIT_MASK(64));
		if (!err)
			TRACE("Using 64-bit PCI DMA addressing\n");
	} else {
		if (pci_set_dma_mask(pci_dev, DMA_BIT_MASK(32))) {
			if ((err = pci_set_consistent_dma_mask(pci_dev,
				DMA_BIT_MASK(32)))) {
				dev_err(&pci_dev->dev,
					"Failed to find usable DMA "
					"configuration\n");
				return err;
			}
		}
	}
#endif

	/* Get some memory for this device's driver state. */
	tlr = kmalloc(sizeof(*tlr), GFP_KERNEL);
	if (tlr == NULL)
		return -ENOMEM;
	memset(tlr, 0, sizeof(*tlr));

	/* Initialize TLR object */
	tlr->pci_dev = pci_dev;
	dev_set_drvdata(&pci_dev->dev, tlr);

	/* Parse host configurations. */
	err = tlr_parse_config(tlr);
	if (err < 0) {
		kfree(tlr);
		return err;
	}

	spin_lock_init(&tlr->is_ready_lock);
	spin_lock_init(&tlr->lock);

	/*
	 * Get the unique endpoint link index which starts at 1.
	 * Global link index 0 is reserved for the RC port.
	 */
	for (i = 0; i < MAX_PCIE_PORTS_PER_DOMAIN - 1; i++) {
		if (!gx_ep_ports[i]) {
			tlr->link_index = i + 1;
			gx_ep_ports[i] = tlr;
			break;
		}
	}
	if (i == MAX_PCIE_PORTS_PER_DOMAIN - 1) {
		dev_err(&pci_dev->dev, "Too many Gx EP ports\n");
		err = -ENOMEM;
		goto link_index_failed;
	}

	tlr->bar0_address = pci_resource_start(pci_dev, 0);
	tlr->bar2_address = pci_resource_start(pci_dev, 2);
	tlr->bar0_size = pci_resource_len(pci_dev, 0);
	tlr->bar2_size = pci_resource_len(pci_dev, 2);
#ifdef CONFIG_PCI_IOV
	tlr->vf_bar0_size = pci_resource_len(pci_dev, PCI_IOV_RESOURCES);
	tlr->vf_bar2_size = pci_resource_len(pci_dev, PCI_IOV_RESOURCES + 2);
#endif

	/* Enable the device. */
	err = pci_enable_device(pci_dev);
	if (err != 0)
		goto enable_failed;

	/* Map in the memory-mapped IO registers. */
	tlr->regs = ioremap(pci_resource_start(pci_dev, 0) +
			    GXPCI_HOST_REGS_OFFSET,
			    sizeof(struct gxpci_host_regs));

	if (tlr->regs == NULL) {
		dev_err(&pci_dev->dev,
			"Failed to map gxpci_host_regs regs\n");
		err = -ENOMEM;
		goto map_failed;
	}

	/* Figure out what kind of chip architecture it is. */
	fill_chip_version(tlr);

	/* Map in the rshim registers. */
	if (tlr->chip_version == 10) {
		tlr->rshim_regs = ioremap(pci_resource_start(pci_dev, 0) +
					  GXPCI_RSHIM_WINDOW_OFFSET,
					  GXPCI_RSHIM_WINDOW_SIZE);
		if (tlr->rshim_regs == NULL) {
			dev_err(&pci_dev->dev,
				"Failed to map rshim registers\n");
			err = -ENOMEM;
			goto rshim_map_failed;
		}
	}

	fill_chip_size(tlr);

	/* Map this device to a board. */	
	err = tlr_map_device_to_board(tlr);
	if (err != 0)
		goto board_failed;

	/* 
	 * Initialize char streams, and the successfully initialized number of 
	 * streams is returned.
	 */
	err = tlr_char_stream_init(tlr);
	if (err < 0)
		goto char_failed;
	else if (err < GXPCI_HOST_CHAR_STREAMS_COUNT) 
		dev_warn(&tlr->pci_dev->dev,
			 "Only the first %d char streams are created\n", err);

#if GXPCI_HOST_ZC_QUEUE_COUNT
	/* Allocate and initialize the zero-copy command queues. */
	err = tlr_zc_init(tlr);
	if (err != 0)
		goto zc_init_failed;
#endif

	pq_queue_init(tlr);

	for (i = 0; i < GXPCI_RAW_DMA_QUEUE_COUNT; i++) {
		tlr->rd_h2t[i].tlr = tlr;
		tlr->rd_t2h[i].tlr = tlr;
		init_MUTEX(&tlr->rd_h2t[i].mutex);
		init_MUTEX(&tlr->rd_t2h[i].mutex);
	}

	/* Create our character and boot devices. */
	err = tlr_add_device_nodes(tlr);
	if (err != 0)
		goto cdev_failed;

	/* Enable PCI bus mastering. */
	pci_set_master(pci_dev);

#if EXTENDED_TAG_ENABLE
	/* Enable extended-tag support */
	pcie_caps_offset = pci_find_capability(pci_dev, PCI_CAP_ID_EXP);
	if (pcie_caps_offset) {
		int pcie_dctl_reg;
		uint16_t pcie_dctl;

		pcie_dctl_reg = pcie_caps_offset + PCI_EXP_DEVCTL;
		pci_read_config_word(pci_dev, pcie_dctl_reg, &pcie_dctl);
		pcie_dctl |= PCI_EXP_DEVCTL_EXT_TAG;
		pci_write_config_word(pci_dev, pcie_dctl_reg, pcie_dctl);
	}
#endif

	if (gxpci_reset_all)
		tlr_boot_save_link_state(tlr);

	/* 
	 * Allocate and map storage for non-C2C queue status array.
	 */
	tlr->queue_sts_array = pci_alloc_consistent(pci_dev, 
		sizeof(struct gxpci_queue_status_array),
		&tlr->queue_sts_bus_addr);

	/*
	 * Allocate the C2C queue status array.
	 */
	if (c2c_queue_sts_array_addr == NULL) {
		c2c_queue_sts_array_addr = pci_alloc_consistent(pci_dev, 
			sizeof(struct gxpci_c2c_queue_sts_array),
			&c2c_queue_sts_array_bus_addr);
		if (c2c_queue_sts_array_addr == NULL) {
			err = -ENOMEM;
			goto c2c_sts_array_failed;
		}
	}

	/*
	 * Schedule work to wait for the endpoint driver ready status,
	 * after the chip is booted either from the host
	 * or from other boot sources, e.g. the onboard SROM.
	 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
	INIT_DELAYED_WORK(&tlr->ep_setup_work, ep_manager);
#else
	INIT_WORK(&tlr->ep_setup_work, ep_manager, tlr);
#endif

	host_nic_queue_vectors = MAX(tlr->nic_tx_queues, tlr->nic_rx_queues);

	/* Set the number of MSI-X interrupt vectors requested. */
	tlr->msix_vectors = GXPCI_HOST_CHAR_STREAMS_COUNT +
		GXPCI_HOST_ZC_QUEUE_COUNT +
		tlr->nic_ports * host_nic_queue_vectors;
	tlr->msix_cs_q_intr_vec_base = GXPCI_HOST_CS_INTR_VECTOR_BASE;
#ifdef GXPCI_INTR_VECTOR_PER_QUEUE
	tlr->msix_vectors *= 2;
	tlr->msix_zc_q_intr_vec_base = GXPCI_HOST_CS_INTR_VECTOR_BASE +
		GXPCI_HOST_CHAR_STREAMS_COUNT * 2;
	tlr->msix_host_nic_intr_vec_base = GXPCI_HOST_CS_INTR_VECTOR_BASE +
		(GXPCI_HOST_CHAR_STREAMS_COUNT + GXPCI_HOST_ZC_QUEUE_COUNT) * 2;
#else
	tlr->msix_zc_q_intr_vec_base = GXPCI_HOST_CS_INTR_VECTOR_BASE +
		GXPCI_HOST_CHAR_STREAMS_COUNT;
	tlr->msix_host_nic_intr_vec_base = GXPCI_HOST_CS_INTR_VECTOR_BASE +
		GXPCI_HOST_CHAR_STREAMS_COUNT + GXPCI_HOST_ZC_QUEUE_COUNT;
#endif
	tlr->msix_entries = 
		kmalloc(sizeof(struct msix_entry) * tlr->msix_vectors, 
			GFP_KERNEL);
	if (tlr->msix_entries == NULL) {
		err = -ENOMEM;
		goto alloc_msix_table_failed;
	}
	memset(tlr->msix_entries, 0, 
	       sizeof(struct msix_entry) * tlr->msix_vectors);

	/*
	 * If there are more than one Gx port, 
	 * start the C2C queue status monitor.
	 */
	if (tlr->link_index == 2) {
		/*
		 * Schedule work to monitor C2C queues' status.
		 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
		INIT_DELAYED_WORK(&c2c_queue_monitor_work, c2c_queue_monitor);
#else
		INIT_WORK(&c2c_queue_monitor_work, c2c_queue_monitor, NULL);
#endif
		schedule_delayed_work(&c2c_queue_monitor_work,
				      GXPCI_QUEUE_MONITOR_INTERVAL);
	}

#ifdef GXPCI_HOST_SDNIC
	/*
 	 * Intialize the netdev ctrl channel. Successful discovery will bringup
 	 * netdevs.
 	 */
	tlr_netdev_ctrl_init(tlr);
#else
	/* Create the PCIe network devices. */
	gxpci_net_devs_init(tlr);
#endif

#if defined(CONFIG_UIO) || defined(CONFIG_UIO_MODULE)
	/* Create the PCIe UIO devices. */
	gxpci_uio_devs_init(tlr);
#endif

	/* Calculate the expected link speed. */
	tlr_get_expected_link_speed_width(tlr);

	/*
	 * Schedule work to monitor various queues' status.
	 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
	INIT_DELAYED_WORK(&tlr->ep_queue_monitor_work, ep_queue_monitor);
#else
	INIT_WORK(&tlr->ep_queue_monitor_work, ep_queue_monitor, tlr);
#endif

#ifdef CONFIG_PCI_IOV
	if (max_vfs > GXPCI_MAX_NUM_VFS) {
		dev_err(&pci_dev->dev,
			"SR-IOV enable failure: max_vfs exceeding %d\n",
			GXPCI_MAX_NUM_VFS);
	} else if (max_vfs > 0) {
		int err;

		/*
		 * The order of the following two if checks depends on
		 * the two variables' default values. Right now, the support
		 * for user-space host NIC interfaces is the default config.
		 */
		if (enable_vf_user_nic)
			max_vfs = MIN(max_vfs, GXPCI_MAX_NUM_VFS_USER_NIC);

		if (enable_vf_nic)
			max_vfs = MIN(max_vfs, GXPCI_MAX_NUM_VFS_KERNEL_NIC);

		err = pci_enable_sriov(pci_dev, max_vfs);
		if (err) {
			dev_err(&pci_dev->dev,
				"SR-IOV enable failure: %d\n", err);
		} else {
			dev_info(&pci_dev->dev, "Enabled SR-IOV with %u VFs\n",
				 max_vfs);
			tlr->num_vfs = max_vfs;
		}
	}
#endif

#if defined(GXPCI_NETLIB_VNIC)
	INFO("Driver (NETLIB enabled) attached.\n");
#elif defined(GXPCI_HOST_NIC_P2P)
	INFO("Driver (P2P enabled) attached.\n");
#elif defined(GXPCI_HOST_SDNIC)
	INFO("Driver (SDNIC enabled) attached.\n");
#else
	INFO("Driver attached.\n");
#endif

	return 0;

 alloc_msix_table_failed:
 c2c_sts_array_failed:
	pci_free_consistent(pci_dev, sizeof(struct gxpci_queue_status_array),
		tlr->queue_sts_array, tlr->queue_sts_bus_addr);
 cdev_failed:
#if GXPCI_HOST_ZC_QUEUE_COUNT
 zc_init_failed:
#endif
 char_failed:
	tlr_unmap_device_from_board(tlr);
 board_failed:
	if (tlr->rshim_regs)
		iounmap(tlr->rshim_regs);
 rshim_map_failed:
	iounmap(tlr->regs);
 map_failed:
	pci_disable_device(pci_dev);
 enable_failed:
 link_index_failed:
	kfree(tlr);
	TRACE("Error, exiting driver. %d\n", err);
	return err;
}


/* Called via pci_unregister_driver() when the module is removed. */
static void remove(struct pci_dev *pci_dev)
{
	struct tlr_pcie_dev* tlr = dev_get_drvdata(&pci_dev->dev);
	int	i;

#ifdef CONFIG_PCI_IOV
	/*
	 * If any VF is in use, don't unload the PF driver until
	 * the VF driver is loaded on the VM.
	 */
	struct pci_dev *pdev;
	u16 pci_command;

	list_for_each_entry(pdev, &pci_dev->bus->devices, bus_list) {
		/*
		 * Since the VF PCI device ID is different from the PF's,
		 * we simply use the device ID to identify the PF.
		 */
		if (pdev->device == TILERA_GX36_DEV_ID)
			continue;
		while (1) {
			/*
			 * If the link is down, read value from PCI_COMMAND will
			 * confuse the driver. So we check the link status here.
			 */
			u8 hdr_type;
			if (pci_read_config_byte(pdev, PCI_HEADER_TYPE, &hdr_type) ||
			    (hdr_type & 0x7f) == 0x7f) {
				dev_err(&pdev->dev,
					"link down, host power-cycle is needed.\n");

				break;
			}

			pci_read_config_word(pdev, PCI_COMMAND, &pci_command);
			if (pci_command & PCI_COMMAND_MASTER) {
				dev_printk(KERN_EMERG, &pci_dev->dev,
					   "VFs in use, need to unload "
					   "the VF driver first\n");
				msleep(5000);
			} else
				break;
		}
	}
#endif

	/*
	 * Delete scheduled work here, i.e. early in the remove()
	 * before closing the devices, because the scheduled work
	 * could try accessing the devices, e.g. via MMIO.
	 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
	cancel_delayed_work_sync(&tlr->ep_queue_monitor_work);
#else
	if (!cancel_delayed_work(&tlr->ep_queue_monitor_work))
		flush_scheduled_work();
#endif

#ifdef GXPCI_HOST_SDNIC
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
	cancel_delayed_work_sync(&tlr->board->netdev_ctrl.netdev_ctrl_work);
#else
	if (!cancel_delayed_work(&tlr->board->netdev_ctrl.netdev_ctrl_work))
		flush_scheduled_work();
#endif
	tlr_netdev_ctrl_close(tlr);
#else
	gxpci_net_devs_remove(tlr);
#endif

#if defined(CONFIG_UIO) || defined(CONFIG_UIO_MODULE)
	/* Free the PCIe UIO devices. */
	gxpci_uio_devs_remove(tlr);
#endif

	tlr_remove_device_nodes(tlr);

	/*
	 * Free the char stream resources.
	 * Note that tlr_cdev_chip_reset() should be called before the chip
	 * is reset. Otherwise the MMIO READ to the PCIe port that is initiated
	 * by tlr_cdev_chip_reset will result in READ completion timeout error,
	 * which is fatal.
	 */
	tlr_cdev_chip_reset(tlr);

	tlr_char_stream_remove(tlr);

#if GXPCI_HOST_ZC_QUEUE_COUNT
	/* Free the Zero-copy stream resources. */
	tlr_zc_chip_reset(tlr);

	tlr_zc_free(tlr);
#endif

	/* Now, release all the resources we've allocated. */

	tlr_unmap_device_from_board(tlr);

	/*
	 * Reset TRIO_PCIE_INTFC_RX_BAR0_ADDR_MASK and TRIO_MAP_RSH_BASE.
	 * Otherwise, upon host reboot, the two registers will retain previous
	 * values that don't match the new BAR0 address that is assigned to
	 * the Gx PCIe ports, causing host MMIO access to RSHIM to fail.
	 */
	tlr_reset_bar_mask(tlr);

	pq_queue_free(tlr);

	for (i = 0; i < GXPCI_RAW_DMA_QUEUE_COUNT; i++) {
		if (tlr->rd_h2t[i].queue_status)
			tlr_raw_dma_free(&tlr->rd_h2t[i]);
		if (tlr->rd_t2h[i].queue_status)
			tlr_raw_dma_free(&tlr->rd_t2h[i]);
		if (tlr->rd_h2t[i].regs) {
			iounmap(tlr->rd_h2t[i].regs);
			tlr->rd_h2t[i].regs = NULL;
		}
		if (tlr->rd_t2h[i].regs) {
			iounmap(tlr->rd_t2h[i].regs);
			tlr->rd_t2h[i].regs = NULL;
		}
	}

	pci_free_consistent(pci_dev, sizeof(struct gxpci_queue_status_array),
		tlr->queue_sts_array, tlr->queue_sts_bus_addr);

	/*
	 * Release the MSI/MSI-X resources we've allocated. The intr vectors
	 * should have been freed by their respective owners, i.e. PCIe queues.
	 */
	if (tlr->msix_vectors)
		pci_disable_msix(pci_dev);
	else
		pci_disable_msi(pci_dev);

	/* Clear this device from the Gx endpoint device array. */
	gx_ep_ports[tlr->link_index - 1] = NULL;

	if (tlr->link_index == 2) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
		cancel_delayed_work_sync(&c2c_queue_monitor_work);
#else
		if (!cancel_delayed_work(&c2c_queue_monitor_work))
			flush_scheduled_work();
#endif
	}

	iounmap(tlr->regs);
	if (tlr->rshim_regs)
		iounmap(tlr->rshim_regs);

	kfree(tlr->msix_entries);
	kfree(tlr);

	/* Free the C2C queue status array if it hasn't been done. */
	if (c2c_queue_sts_array_addr) {
		pci_free_consistent(pci_dev,
				    sizeof(struct gxpci_c2c_queue_sts_array),
				    c2c_queue_sts_array_addr,
				    c2c_queue_sts_array_bus_addr);
		c2c_queue_sts_array_addr = NULL;
		c2c_queue_sts_array_bus_addr = 0;
	}

#ifdef CONFIG_PCI_IOV
	pci_disable_sriov(pci_dev);
#endif

	pci_disable_device(pci_dev);
	dev_set_drvdata(&pci_dev->dev, NULL);
}

static struct pci_driver pci_driver = {
	.name = "tilegxpci",
	.id_table = ids,
	.probe = probe,
	.remove = remove,
};

extern struct proc_dir_entry * proc_root_driver;

static int __init
tilegxpci_init(void)
{
	struct proc_dir_entry *entry;

	INFO("Loaded driver version %s\n", TILEGXPCI_VERSION_STRING);

	/* /proc file for listing the boards */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
	entry = proc_create("driver/tilegxpci_boards", 0, NULL, &tlr_proc_boards_ops);
#else
	entry = create_proc_entry("driver/tilegxpci_boards", 0, NULL);
#endif
	if (entry) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
		entry->proc_fops = &tlr_proc_boards_ops;
#endif
	}
	else {
		ERR("Failed to create /proc/driver/tilegxpci_boards.\n");
		return -EIO;
	}

#ifdef GXPCI_HOST_SDNIC
	if (tlr_netdev_ctrl_proc_entry_create() != 0)
		return -EIO;
#endif

	return pci_register_driver(&pci_driver);
}

static void __exit
tilegxpci_exit(void)
{
	remove_proc_entry("driver/tilegxpci_boards", NULL);

#ifdef GXPCI_HOST_SDNIC
	remove_proc_entry("driver/tilegxpci_netdev_ctrl", NULL);
#endif /* GXPCI_HOST_SDNIC */

	pci_unregister_driver(&pci_driver);
}

MODULE_LICENSE(LICENSE_STRING);
MODULE_AUTHOR("Tilera Corp.");
MODULE_VERSION(TILEGXPCI_VERSION_STRING);

module_init(tilegxpci_init);

module_exit(tilegxpci_exit);
