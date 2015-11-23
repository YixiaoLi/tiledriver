/*
 * raw_dma.c - Tilera TILEGx host-side driver routines for the
 *             user-space PCIe Raw DMA queue.
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
#include <linux/compat.h>
#include <linux/delay.h>
#include <asm/uaccess.h> 
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <asm/page.h>
#include <asm/pgtable.h> 		/* io_remap_page_range */

#include "tilegxpci.h"
#include "tilegxpci_host.h"
#include "tilegxpci_version.h"

/* Polling period for the tile application to become ready. */
#define HOST_RD_OPEN_POLLING_MSECS	200

/* Total waiting time for the tile application to become ready. */
#define HOST_RD_OPEN_MAX_MSECS		10000

/* Total waiting time for the reset acknowledgement from the other end. */
#define HOST_RD_RELEASE_MAX_MSECS	1000

/**********************************************************************/
/*                   RAW DMA Support                                  */
/**********************************************************************/

#ifndef RAW_DMA_USE_RESERVED_MEMORY

/* Allocate the Raw DMA buffer. */
static int tlr_raw_dma_buf_alloc(struct tlr_raw_dma_state *rd)
{
	int order = get_order(HOST_RD_SEGMENT_MAX_SIZE), err, i;
	struct page *last_page;
	struct page *page;

	for (i = 0; i < rd->num_segments; i++) {
		/*
 		 * Allocate a single PA-contiguous segment from a
 		 * user-specified NUMA node.
 		 */
		page = alloc_pages_node(rd->numa_node, GFP_KERNEL | __GFP_ZERO,
					order);
		if (page == NULL) {
			err = -ENOMEM;
			goto exit;
		} else {
			last_page = page + (1 << order);

			rd->rd_segment[i] = page_address(page);
			rd->rd_segment_handle[i] = page_to_phys(page);

			while (page < last_page) {
				SetPageReserved(page);

				/*
				 * Increment page count so that the kernel won't
				 * try releasing this page when the application
				 * which has mmap'ed the page exits.
				 */
				get_page(page++);
			}
		}
	}

	return 0;

 exit:
	for (i = 0; i < rd->num_segments; i++) {

		if (rd->rd_segment[i] == NULL)
			break;

		page = virt_to_page(rd->rd_segment[i]);
		last_page = page + (1 << order);

		while (page < last_page)
			ClearPageReserved(page++);

		free_pages((unsigned long) rd->rd_segment[i], order);
		rd->rd_segment_handle[i] = 0;
		rd->rd_segment[i] = NULL;
	}

	return err;
}

/* Free the Raw DMA buffer. */
static void tlr_raw_dma_buf_free(struct tlr_raw_dma_state *rd)
{
	int order = get_order(HOST_RD_SEGMENT_MAX_SIZE), i;
	struct page *last_page;
	struct page *page;

	for (i = 0; i < rd->num_segments; i++) {
		page = virt_to_page(rd->rd_segment[i]);
		last_page = page + (1 << order);

		while (page < last_page)
			ClearPageReserved(page++);

		free_pages((unsigned long) rd->rd_segment[i], order);
		rd->rd_segment_handle[i] = 0;
		rd->rd_segment[i] = NULL;
	}
}

#endif

/* Allocate the queue status buf for the PCIe raw DMA queue. */
static int tlr_raw_dma_sts_alloc(struct tlr_raw_dma_state *rd)
{
	struct tlr_pcie_dev *tlr = rd->tlr;
	struct pci_dev *pci_dev = tlr->pci_dev;

	/*
	 * Strictly speaking, there is no need to map the queue status to
	 * the PCI bus. For now, keep it this way just in case it is needed
	 * in the future.
	 */
	rd->queue_status =
		pci_alloc_consistent(pci_dev, sizeof(struct tlr_rd_status),
			&rd->queue_status_bus_addr);

	if (rd->queue_status == NULL) {
		return -ENOMEM;
	} else {
		int order = get_order(sizeof(struct tlr_rd_status));
		struct page *page = virt_to_page(rd->queue_status);
		struct page *last_page = page + (1 << order);

		while (page < last_page) {
			SetPageReserved(page);

			/*
			 * Increment page count so that the kernel won't
			 * try releasing this page when the application which
			 * has mmap'ed the page exits.
			 */
			get_page(page++);
		}
	}

	return 0;
}

/* Free the queue status memory for the Raw DMA queue. */
static void tlr_raw_dma_sts_free(struct tlr_raw_dma_state *rd)
{
	struct tlr_pcie_dev *tlr = rd->tlr;
	struct pci_dev *pci_dev = tlr->pci_dev;
	int order;
	struct page *page;
	struct page *last_page;

	order = get_order(sizeof(struct tlr_rd_status));
	page = virt_to_page(rd->queue_status);
	last_page = page + (1 << order);

	while (page < last_page)
		ClearPageReserved(page++);

	pci_free_consistent(pci_dev, sizeof(struct tlr_rd_status),
		rd->queue_status, rd->queue_status_bus_addr);

	rd->queue_status = NULL;
}

/* Allocate the queue status and DMA buffer for the Raw DMA queue. */
static int tlr_raw_dma_alloc(struct tlr_raw_dma_state *rd)
{
	int err;

	err = tlr_raw_dma_sts_alloc(rd);
	if (err)
		return err;

#ifndef RAW_DMA_USE_RESERVED_MEMORY

	/* Allocate the Raw DMA buffer. */
	err = tlr_raw_dma_buf_alloc(rd);
	if (err) {
		tlr_raw_dma_sts_free(rd);
		return err;
	}

#endif

	return 0;
}

/* Free the queue status and DMA buffer memory for the Raw DMA queue. */
void tlr_raw_dma_free(struct tlr_raw_dma_state *rd)
{
	tlr_raw_dma_sts_free(rd);

#ifndef RAW_DMA_USE_RESERVED_MEMORY
	tlr_raw_dma_buf_free(rd);
#endif
}

#ifndef RAW_DMA_USE_RESERVED_MEMORY
static void
raw_dma_vma_open(struct vm_area_struct *vma)
{
	struct tlr_raw_dma_state *rd = vma->vm_private_data;

	down(&rd->mutex);

	rd->vmas++;

	up(&rd->mutex);
}

static void
raw_dma_vma_close(struct vm_area_struct *vma)
{
	struct tlr_raw_dma_state *rd = vma->vm_private_data;

	down(&rd->mutex);

	rd->vmas--;

	up(&rd->mutex);
}

#ifdef USE_VM_FAULT
static int
raw_dma_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
#else
static struct page*
raw_dma_vma_nopage(struct vm_area_struct *vma, unsigned long vaddr,
		   int *type)
#endif
{
	struct tlr_raw_dma_state *rd = vma->vm_private_data;
	struct page *page;
	void *page_ptr = NULL;
	unsigned long offset;

#ifdef USE_VM_FAULT
	unsigned long vaddr = (unsigned long)vmf->virtual_address;
	int ret;
#else
	struct page *ret;
#endif
	int segment;
	ret = RETURN_SIGBUS;

	down(&rd->mutex);

	if (vaddr > vma->vm_end) {
		goto exit;
	}

	/* Ignore vma->vm_pgoff, which is unrelated here. */
	offset = vaddr - vma->vm_start;

	/* Calculate the index of the segment. */
	segment = offset / HOST_RD_SEGMENT_MAX_SIZE;
	page_ptr = rd->rd_segment[segment] + offset % HOST_RD_SEGMENT_MAX_SIZE;

	page = virt_to_page(page_ptr);
	get_page(page);

#ifdef USE_VM_FAULT
	ret = 0;
	vmf->page = page;
#else
	ret = page;
	if (type)
		*type = VM_FAULT_MINOR;
#endif

 exit:
	up(&rd->mutex);

	return ret;
}

static struct vm_operations_struct raw_dma_vm_ops = {
	.open   = raw_dma_vma_open,
	.close  = raw_dma_vma_close,
#ifdef USE_VM_FAULT
	.fault = raw_dma_vma_fault,
#else
	.nopage = raw_dma_vma_nopage,
#endif
};
#endif

static void
raw_dma_sts_vma_open(struct vm_area_struct *vma)
{
	struct tlr_raw_dma_state *rd = vma->vm_private_data;

	down(&rd->mutex);

	rd->sts_vmas++;

	up(&rd->mutex);
}

static void
raw_dma_sts_vma_close(struct vm_area_struct *vma)
{
	struct tlr_raw_dma_state *rd = vma->vm_private_data;

	down(&rd->mutex);

	rd->sts_vmas--;

	up(&rd->mutex);
}

#ifdef USE_VM_FAULT
static int
raw_dma_sts_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
#else
static struct page*
raw_dma_sts_vma_nopage(struct vm_area_struct *vma, unsigned long vaddr,
			int *type)
#endif
{
	struct tlr_raw_dma_state *rd = vma->vm_private_data;
	struct page *page;
	void *page_ptr = NULL;
	unsigned long offset;
	
#ifdef USE_VM_FAULT
	unsigned long vaddr = (unsigned long)vmf->virtual_address;
	int ret;
#else
	struct page *ret;
#endif
	ret = RETURN_SIGBUS;

	down(&rd->mutex);

	if (vaddr > vma->vm_end) {
		goto exit;
	}

	/* Ignore vma->vm_pgoff, which is unrelated here. */
	offset = vaddr - vma->vm_start;

	page_ptr = rd->queue_status + offset;

	page = virt_to_page(page_ptr);
	get_page(page);

#ifdef USE_VM_FAULT
	ret = 0;
	vmf->page = page;
#else
	ret = page;
	if (type)
		*type = VM_FAULT_MINOR;
#endif

 exit:
	up(&rd->mutex);

	return ret;
}

static struct vm_operations_struct raw_dma_sts_vm_ops = {
	.open	= raw_dma_sts_vma_open,
	.close	= raw_dma_sts_vma_close,
#ifdef USE_VM_FAULT
	.fault = raw_dma_sts_vma_fault,
#else
	.nopage	= raw_dma_sts_vma_nopage,
#endif
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
static int
tlr_raw_dma_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
		  unsigned long arg)
#else
static long
tlr_raw_dma_unlocked_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long arg)
#endif
{
	struct tlr_raw_dma_state *rd = filp->private_data;
	tilepci_raw_dma_get_buf_t buf_info;
#ifndef RAW_DMA_USE_RESERVED_MEMORY
	int i;
#endif

	switch (cmd) {
	case TILEPCI_IOC_GET_RAW_DMA_BUF:

		buf_info.rd_buf_size = rd->buf_size;
#ifdef RAW_DMA_USE_RESERVED_MEMORY
		buf_info.rd_buf_bus_addr = rd->rd_mem_handle;
#endif
		if (copy_to_user((void __user *) arg, &buf_info,
			sizeof(tilepci_raw_dma_get_buf_t)))
			return -EFAULT;

		break;
	case TILEPCI_IOC_ACTIVATE_RAW_DMA:

		writel(rd->buf_size, &rd->regs->rd_buf_size);

		/*
		 * Let tile app know the base PA of Raw DMA buffer.
		 */
#ifdef RAW_DMA_USE_RESERVED_MEMORY
		writeq(rd->rd_mem_handle, &rd->regs->rd_buf_bus_addr);
#else
		for (i = 0;  i < rd->num_segments; i++)
			writeq(rd->rd_segment_handle[i],
				&rd->regs->segment_bus_addr[i]);
#endif

		/*
		 * Inform both ends of our readiness.
		 */
		rd->queue_status->status = GXPCI_HOST_CHAN_READY;
		writel(GXPCI_HOST_CHAN_READY, &rd->regs->queue_status);

		break;
	default:
		return -EINVAL;
	}	
	return 0;
}

#ifdef CONFIG_COMPAT
static long 
tlr_raw_dma_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Sign-extend the argument so it can be used as a pointer. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
	return tlr_raw_dma_ioctl(filp->f_dentry->d_inode, filp, cmd,
				 (unsigned long)compat_ptr(arg));
#else
	return tlr_raw_dma_unlocked_ioctl(filp, cmd, 
					  (unsigned long)compat_ptr(arg));
#endif
}
#endif 
				
static int
tlr_raw_dma_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct tlr_raw_dma_state *rd = filp->private_data;
	struct tlr_pcie_dev* tlr = rd->tlr;
	struct pci_dev *pci_dev = tlr->pci_dev;
	size_t size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

	if (!(vma->vm_flags & VM_SHARED)) {
		printk("Tile Raw DMA mmap flags must include VM_SHARED\n");
		return -EINVAL;
	}

	/* Map the Raw DMA gxpci_host_rd_regs_app and the queue status. */
	if (offset == TILEPCI_RAW_DMA_REG_MMAP_OFFSET) {
		unsigned int rd_regs_offset;
		resource_size_t phys_addr;
		int channel;

		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

		/*
		 * Figure out this Raw DMA queue's direction and index.
		 * Note that this logic has a dependency on the relative
		 * order of the H2T and T2H tlr_raw_dma_state arrays in
		 * struct tlr_pcie_dev.
		 */
		if (rd >= tlr->rd_t2h) {
			rd_regs_offset = GXPCI_HOST_RD_T2H_REGS_OFFSET;
			channel = rd - tlr->rd_t2h;
		} else {
			rd_regs_offset = GXPCI_HOST_RD_H2T_REGS_OFFSET;
			channel = rd - tlr->rd_h2t;
		}

		phys_addr = pci_resource_start(pci_dev, 0) +
			rd_regs_offset + channel *
			(GXPCI_HOST_RD_REGS_MAP_SIZE +
			GXPCI_HOST_RD_REG_APP_MAP_SIZE) +
			GXPCI_HOST_RD_REGS_MAP_SIZE;

		if (remap_pfn_range(vma,
				    vma->vm_start,
				    phys_addr >> PAGE_SHIFT,
				    size,
				    vma->vm_page_prot)) {
			printk("Tile PCIe Raw DMA queue"
				" remap_pfn_range failed.\n");
			return -EAGAIN;
		}

	} else if (offset == TILEPCI_RAW_DMA_STS_MMAP_OFFSET) {

		if (rd->queue_status == NULL)
			return -EFAULT;

		/* Don't try to swap out physical pages. */
		vma->vm_flags |= VM_RESERVED;

		vma->vm_ops = &raw_dma_sts_vm_ops;
		vma->vm_private_data = rd;

		raw_dma_sts_vma_open(vma);
	}
#ifndef RAW_DMA_USE_RESERVED_MEMORY
	else if (offset == TILEPCI_RAW_DMA_BUF_MMAP_OFFSET) {
		int i;

		for (i = 0;  i < rd->num_segments; i++)
			if (rd->rd_segment[i] == NULL)
				return -EFAULT;

		if (size > rd->buf_size)
			return -EINVAL;

		if (!(vma->vm_flags & VM_SHARED)) {
			printk("Tile PCIe Raw DMA buffer mmap flags"
				" must include VM_SHARED\n");
			return -EINVAL;
		}

		/* Don't try to swap out physical pages. */
		vma->vm_flags |= VM_RESERVED;

		vma->vm_ops = &raw_dma_vm_ops;
		vma->vm_private_data = rd;

		raw_dma_vma_open(vma);
	}
#endif
	else {
		printk("tlr_raw_dma_mmap offset invalid\n");
		return -EINVAL;
	}

	return 0;
}

static int
tlr_raw_dma_release(struct inode* inode, struct file* filp)
{
	struct tlr_pcie_dev* tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);
	struct tlr_raw_dma_state *rd = filp->private_data;
	struct gxpci_queue_pair_status *queue_pair_status;
	unsigned int minor = MINOR(inode->i_rdev);
	int sleep_msecs = 0;
	unsigned long flags;
	int queue_index;
	int err = 0;

	if (minor >= TILEPCI_FIRST_RAW_DMA_TX_MINOR) {
		queue_index = minor - TILEPCI_FIRST_RAW_DMA_TX_MINOR;
		queue_pair_status =
			&tlr->queue_sts_array->rd_sts_t2h[queue_index];
	} else {
		queue_index = minor - TILEPCI_FIRST_RAW_DMA_RX_MINOR;
		queue_pair_status =
			&tlr->queue_sts_array->rd_sts_h2t[queue_index];
	}

	spin_lock_irqsave(&tlr->lock, flags);

	/*
	 * We simply set the queue status to GXPCI_CHAN_RESET informing
	 * the queue status monitor running on the RC node of our intention
	 * to reset the queue. The RC queue status monitor, upon detecting
	 * our reset, sets the queue peer's user-visible queue status to
	 * GXPCI_CHAN_RESET. The host user application, upon detecting
	 * GXPCI_CHAN_RESET, will call close() which sets GXPCI_CHAN_RESET
	 * to its queue status in the queue status array. When the queue
	 * status monitor detects GXPCI_CHAN_RESET for both queues' status,
	 * it sets GXPCI_CHAN_RESET_ACK to both queues' status. We'll be
	 * unblocked then and return. If the peer doesn't call close() in
	 * a timely manner or not at all, we just time out and return.
	 */
	queue_pair_status->host_queue_status = GXPCI_CHAN_RESET;

	while (queue_pair_status->host_queue_status != GXPCI_CHAN_RESET_ACK) {
		if (sleep_msecs >= HOST_RD_RELEASE_MAX_MSECS) {
			err = -ERESTARTSYS;
			break;
		}
		spin_unlock_irqrestore(&tlr->lock, flags);
		if (msleep_interruptible(GXPCI_QUEUE_MONITOR_INTERVAL)) {
			/*
			 * This host application is exiting due to a signal.
			 * We simply send the reset event to tile application
			 * directly, instead of relying on the queue status
			 * monitor.
			 */
			spin_lock_irqsave(&tlr->lock, flags);
			writel(GXPCI_CHAN_RESET, &rd->regs->queue_status);
			err = -EINTR;
			break;
		}
		sleep_msecs += GXPCI_QUEUE_MONITOR_INTERVAL;	

		spin_lock_irqsave(&tlr->lock, flags);
	}

	/* Set the queue status to uninitialized state. */
	queue_pair_status->host_queue_status = GXPCI_CHAN_UNINITIALIZED;
	queue_pair_status->host_queue_opened--;

	/* Set the struct tlr_rd_status. */
	rd->queue_status->status = GXPCI_CHAN_UNINITIALIZED;

	spin_unlock_irqrestore(&tlr->lock, flags);

	/* This file blocks any reboot attempts; synchronize against
	 * somebody opening the boot file. */
	down(&reboot_mutex);
	
	tlr->board->raw_dma_cnt--;
	
	up(&reboot_mutex);
	return err;
}


struct file_operations tlr_raw_dma_ops = {
	.owner = THIS_MODULE,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
	.ioctl = tlr_raw_dma_ioctl,
#else
	.unlocked_ioctl = tlr_raw_dma_unlocked_ioctl,
#endif
#ifdef CONFIG_COMPAT
	.compat_ioctl = tlr_raw_dma_compat_ioctl,
#endif
	.mmap = tlr_raw_dma_mmap,
	.release = tlr_raw_dma_release,
};

int
tlr_raw_dma_open(struct tlr_pcie_dev *tlr, struct file *filp,
		 struct tlr_raw_dma_state *rd_state_base, int channel)
{
	struct tlr_raw_dma_state *rd = &rd_state_base[channel];
	struct gxpci_queue_pair_status *queue_pair_status;
	unsigned int rd_regs_offset;
	int sleep_msecs = 0;
	unsigned long flags;
	int result = 0;

	spin_lock(&tlr->is_ready_lock);
	if (!tlr->is_ready) {
		spin_unlock(&tlr->is_ready_lock);
		return -ENXIO;
	}
	spin_unlock(&tlr->is_ready_lock);

	/*
	 * This file blocks any reboot attempts, even though this open call
	 * has not succeeded yet. Otherwise, the chip could be rebooted while
	 * this call is issuing MMIO access to the chip.
	 */
	down(&reboot_mutex);
	
	tlr->board->raw_dma_cnt++;

	up(&reboot_mutex);

	if (rd_state_base == tlr->rd_h2t)
		queue_pair_status =
			&tlr->queue_sts_array->rd_sts_h2t[channel];
	else
		queue_pair_status =
			&tlr->queue_sts_array->rd_sts_t2h[channel];

	filp->private_data = rd;
	filp->f_op = &tlr_raw_dma_ops;

	/*
	 * Map the Raw DMA queue control registers,
	 * when this Raw DMA queue is opened the first time. The unmapping
	 * is done when the driver is unloaded.
	 */
	down(&rd->mutex);
	if (rd->regs == NULL) {
		unsigned long pa;

		if (rd_state_base == tlr->rd_h2t)
			rd_regs_offset = GXPCI_HOST_RD_H2T_REGS_OFFSET;
		else
			rd_regs_offset = GXPCI_HOST_RD_T2H_REGS_OFFSET;

		pa = pci_resource_start(tlr->pci_dev, 0) + rd_regs_offset +
			channel * (GXPCI_HOST_RD_REGS_MAP_SIZE +
			GXPCI_HOST_RD_REG_APP_MAP_SIZE);
		rd->regs = ioremap(pa, sizeof(struct gxpci_host_rd_regs_drv));
		if (rd->regs == NULL) {
			up(&rd->mutex);
			result = -ENOMEM;
			goto exit;
		}
	}
	up(&rd->mutex);

	/*
	 * Poll for the tile application readiness.
	 */
	while (readl(&rd->regs->queue_status) != GXPCI_TILE_CHAN_READY) {
                if (sleep_msecs >= HOST_RD_OPEN_MAX_MSECS) {
			result = -ENXIO;
			goto exit;
		}
                if (msleep_interruptible(HOST_RD_OPEN_POLLING_MSECS)) {
			result = -EINTR;
			goto exit;
		}
                sleep_msecs += HOST_RD_OPEN_POLLING_MSECS;
	}

	down(&rd->mutex);
	if (rd->queue_status == NULL) {
		int err;

		err = tlr_raw_dma_alloc(rd);
		if (err != 0){
			up(&rd->mutex);
			result = err;
			goto exit;
		}
	}
	up(&rd->mutex);

	spin_lock_irqsave(&tlr->lock, flags);
	queue_pair_status->host_queue_opened++;
	spin_unlock_irqrestore(&tlr->lock, flags);

	/*
	 * This file blocks any reboot attempts; synchronize against
	 * somebody opening the boot file.
	 */
 exit:

	/* Decrement the open counter if this open fails. */
	if (result) {
		down(&reboot_mutex);
	
		tlr->board->raw_dma_cnt--;

		up(&reboot_mutex);
	}

	return result;
}
