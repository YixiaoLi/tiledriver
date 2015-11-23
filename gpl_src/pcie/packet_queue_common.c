/*
 * packet_queue_common.c - Tilera TILEGx host-side driver routines for the
 *                  	   user-space PCIe Packet Queue.
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

#include "tilegxpci_host.h"

/* Polling period for the tile application to become ready. */
#define HOST_PQ_OPEN_POLLING_MSECS	200

/* Total waiting time for the reset acknowledgement from the other end. */
#define HOST_PQ_RELEASE_MAX_MSECS	1000

/* Total waiting time for the tile application to become ready. */
static int host_open_wait_secs = 10;
module_param(host_open_wait_secs, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH); 
MODULE_PARM_DESC(host_open_wait_secs, "Time in seconds to wait for the "
		 "tile application to start, 10 seconds by default.");

extern int pq_numa_capable;

/**********************************************************************/
/*                   Packet Queue Support                             */
/**********************************************************************/
static int
tlr_packet_queue_alloc_queue_status(struct tlr_packet_queue_state *pq)
{
	struct tlr_pcie_dev *tlr = pq->tlr;
	struct pci_dev *pci_dev = tlr->pci_dev;
	struct page *last_page;
	struct page *page;
	int order;

	pq->queue_status =
		pci_alloc_consistent(pci_dev, sizeof(struct tlr_pq_status),
				     &pq->queue_status_bus_addr);
	if (pq->queue_status == NULL) {
		return -ENOMEM;
	} else {
		order = get_order(sizeof(struct tlr_pq_status));
		page = virt_to_page(pq->queue_status);
		last_page = page + (1 << order);

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

static void
tlr_packet_queue_free_queue_status(struct tlr_packet_queue_state *pq)
{
	struct tlr_pcie_dev *tlr = pq->tlr;
	struct pci_dev *pci_dev = tlr->pci_dev;
	struct page *last_page;
	struct page *page;
	int order;

	order = get_order(sizeof(struct tlr_pq_status));
	page = virt_to_page(pq->queue_status);
	last_page = page + (1 << order);

	while (page < last_page)
		ClearPageReserved(page++);

	pci_free_consistent(pci_dev, sizeof(struct tlr_pq_status),
			    pq->queue_status, pq->queue_status_bus_addr);

	pq->queue_status = NULL;
}

static void
tlr_pq_non_numa_free(struct tlr_packet_queue_state *pq)
{
        struct pci_dev *pci_dev = pq->tlr->pci_dev;
	int order = get_order(pq->segment_size);
	size_t size = pq->segment_size;
	struct page *last_page;
	struct page *page;
	int i;

	for (i = 0; i < pq->num_segments; i++) {
		if (pq->pq_segment[i] == NULL)
			continue;

		page = virt_to_page(pq->pq_segment[i]);
		last_page = page + (1 << order);

		while (page < last_page)
			ClearPageReserved(page++);

		pci_free_consistent(pci_dev, size,
			pq->pq_segment[i], pq->pq_segment_handle[i]);

		pq->pq_segment_handle[i] = 0;
		pq->pq_segment[i] = NULL;
	}
}

static int
tlr_pq_non_numa_alloc(struct tlr_packet_queue_state *pq)
{
        struct pci_dev *pci_dev = pq->tlr->pci_dev;
	int order = get_order(pq->segment_size);
	size_t size = pq->segment_size;
	struct page *last_page;
	struct page *page;
	int i;

	for (i = 0; i < pq->num_segments; i++) {
		pq->pq_segment[i] = pci_alloc_consistent(pci_dev, size,
			&pq->pq_segment_handle[i]);
		if (pq->pq_segment[i] == NULL) {
			return -ENOMEM;
		} else {
			page = virt_to_page(pq->pq_segment[i]);
			last_page = page + (1 << order);

			while (page < last_page) {
				SetPageReserved(page);
				get_page(page++);
			}
		}
	}

	return 0;
}

void
tlr_pq_numa_free(struct tlr_packet_queue_state *pq)
{
	int order = get_order(pq->segment_size);
	struct page *last_page;
	struct page *page;
	int i;

	for (i = 0; i < pq->num_segments; i++) {
		if (pq->pq_segment[i] == NULL)
			continue;

		page = virt_to_page(pq->pq_segment[i]);
		last_page = page + (1 << order);

		while (page < last_page)
			ClearPageReserved(page++);

		free_pages((unsigned long) pq->pq_segment[i], order);
		pq->pq_segment_handle[i] = 0;
		pq->pq_segment[i] = NULL;
	}
}

static int
tlr_pq_numa_alloc(struct tlr_packet_queue_state *pq)
{
	int order = get_order(pq->segment_size);
	struct page *last_page;
	struct page *page;
	int err = 0;
	int i;

	for (i = 0; i < pq->num_segments; i++) {
		/*
 		 * Allocate a single PA-contiguous segment from a
 		 * user-specified NUMA node.
 		 */
		page = alloc_pages_node(pq->numa_node, GFP_KERNEL | __GFP_ZERO,
					order);
		if (page == NULL) {
			err = -ENOMEM;
			break;
		} else {
			last_page = page + (1 << order);

			pq->pq_segment[i] = page_address(page);
			pq->pq_segment_handle[i] = page_to_phys(page);

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

	return err;
}

/*
 * Allocate one shared DMA ring buffer and two individual queue status buffers
 * for the PCIe Packet Queue H2T and T2H pair.
 */
static int
tlr_packet_queue_alloc_shared(struct tlr_packet_queue_state *h2t_pq,
			      struct tlr_packet_queue_state *t2h_pq)
{
	int err = 0;
	int i;

	if (pq_numa_capable)
		err = tlr_pq_numa_alloc(h2t_pq);
	else
		err = tlr_pq_non_numa_alloc(h2t_pq);
	if (err < 0)
		goto exit;

	for (i = 0; i < t2h_pq->num_segments; i++) {
		t2h_pq->pq_segment[i] = h2t_pq->pq_segment[i];
		t2h_pq->pq_segment_handle[i] = h2t_pq->pq_segment_handle[i];
	}

	err = tlr_packet_queue_alloc_queue_status(h2t_pq);
	if (err < 0)
		goto exit;

	err = tlr_packet_queue_alloc_queue_status(t2h_pq);
	if (err < 0) {
		tlr_packet_queue_free_queue_status(h2t_pq);
		goto exit;
	}

	return 0;

 exit:
	if (pq_numa_capable)
		tlr_pq_numa_free(h2t_pq);
	else
		tlr_pq_non_numa_free(h2t_pq);
	for (i = 0; i < t2h_pq->num_segments; i++) {
		t2h_pq->pq_segment_handle[i] = 0;
		t2h_pq->pq_segment[i] = NULL;
	}

	return err;
}

/*
 * Free the shared DMA ring buffer and two individual queue status buffers
 * for the PCIe Packet Queue H2T and T2H pair.
 */
void
tlr_packet_queue_free_shared(struct tlr_packet_queue_state *h2t_pq,
			     struct tlr_packet_queue_state *t2h_pq)
{
	int i;

	if (pq_numa_capable)
		tlr_pq_numa_free(h2t_pq);
	else
		tlr_pq_non_numa_free(h2t_pq);
	for (i = 0; i < t2h_pq->num_segments; i++) {
		t2h_pq->pq_segment_handle[i] = 0;
		t2h_pq->pq_segment[i] = NULL;
	}

	tlr_packet_queue_free_queue_status(h2t_pq);
	tlr_packet_queue_free_queue_status(t2h_pq);
}

/* Allocate the ring buffer and queue status buf for the PCIe Packet Queue. */
static int
tlr_packet_queue_alloc(struct tlr_packet_queue_state *pq)
{
	int err = 0;

	if (pq_numa_capable)
		err = tlr_pq_numa_alloc(pq);
	else
		err = tlr_pq_non_numa_alloc(pq);
	if (err < 0)
		goto exit;

	err = tlr_packet_queue_alloc_queue_status(pq);
	if (err < 0)
		goto exit;

	return 0;

 exit:
	if (pq_numa_capable)
		tlr_pq_numa_free(pq);
	else
		tlr_pq_non_numa_free(pq);

	return err;
}

/* Free the ring buffer for the PCIe Packet Queue. */
void
tlr_packet_queue_free(struct tlr_packet_queue_state *pq)
{
	if (pq_numa_capable)
		tlr_pq_numa_free(pq);
	else
		tlr_pq_non_numa_free(pq);

	tlr_packet_queue_free_queue_status(pq);
}

static void
packet_queue_vma_open(struct vm_area_struct *vma)
{
	struct tlr_packet_queue_state *pq = vma->vm_private_data;

	down(&pq->mutex);

	pq->vmas++;

	up(&pq->mutex);
}

static void
packet_queue_vma_close(struct vm_area_struct *vma)
{
	struct tlr_packet_queue_state *pq = vma->vm_private_data;

	down(&pq->mutex);

	pq->vmas--;

	up(&pq->mutex);
}

#ifdef USE_VM_FAULT
static int
packet_queue_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
#else
static struct page*
packet_queue_vma_nopage(struct vm_area_struct *vma, unsigned long vaddr,
			int *type)
#endif
{
	struct tlr_packet_queue_state *pq = vma->vm_private_data;
	struct page *page;
	void *page_ptr = NULL;
	unsigned long offset;
	
#ifdef USE_VM_FAULT
	unsigned long vaddr = (unsigned long) vmf->virtual_address;
	int ret;
#else
	struct page *ret;
#endif
	int index;
	ret = RETURN_SIGBUS;

	down(&pq->mutex);

	if (vaddr > vma->vm_end) {
		goto exit;
	}

	/* Ignore vma->vm_pgoff, which is unrelated here. */
	offset = vaddr - vma->vm_start;

	/* Calculate the index of the segment. */
	index = offset >> pq->segment_size_order;
	offset -= index << pq->segment_size_order;
	page_ptr = pq->pq_segment[index] + offset;

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
	up(&pq->mutex);

	return ret;
}

static struct vm_operations_struct packet_queue_vm_ops = {
	.open	= packet_queue_vma_open,
	.close	= packet_queue_vma_close,
#ifdef USE_VM_FAULT
	.fault = packet_queue_vma_fault,
#else
	.nopage	= packet_queue_vma_nopage,
#endif
};

static void
packet_queue_sts_vma_open(struct vm_area_struct *vma)
{
	struct tlr_packet_queue_state *pq = vma->vm_private_data;

	down(&pq->mutex);

	pq->sts_vmas++;

	up(&pq->mutex);
}

static void
packet_queue_sts_vma_close(struct vm_area_struct *vma)
{
	struct tlr_packet_queue_state *pq = vma->vm_private_data;

	down(&pq->mutex);

	pq->sts_vmas--;

	up(&pq->mutex);
}

#ifdef USE_VM_FAULT
static int
packet_queue_sts_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
#else
static struct page*
packet_queue_sts_vma_nopage(struct vm_area_struct *vma, unsigned long vaddr,
			int *type)
#endif
{
	struct tlr_packet_queue_state *pq = vma->vm_private_data;
	struct page *page;
	void *page_ptr = NULL;
	unsigned long offset;
	
#ifdef USE_VM_FAULT
	unsigned long vaddr = (unsigned long) vmf->virtual_address;
	int ret;
#else
	struct page *ret;
#endif
	ret = RETURN_SIGBUS;

	down(&pq->mutex);

	if (vaddr > vma->vm_end) {
		goto exit;
	}

	/* Ignore vma->vm_pgoff, which is unrelated here. */
	offset = vaddr - vma->vm_start;

	page_ptr = pq->queue_status + offset;

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
	up(&pq->mutex);

	return ret;
}

static struct vm_operations_struct packet_queue_sts_vm_ops = {
	.open	= packet_queue_sts_vma_open,
	.close	= packet_queue_sts_vma_close,
#ifdef USE_VM_FAULT
	.fault = packet_queue_sts_vma_fault,
#else
	.nopage	= packet_queue_sts_vma_nopage,
#endif
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
static int
tlr_packet_queue_ioctl(struct inode *inode, struct file *filp,
		unsigned int cmd, unsigned long arg)
#else
static long
tlr_packet_queue_unlocked_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
#endif
{
	struct tlr_packet_queue_state *pq = filp->private_data;
	tilepci_packet_queue_info_t buf_info;
	tilegxpci_bar_info_t bar_info;
	uint32_t value;
	int i;

	switch (cmd) {
	case TILEPCI_IOC_SET_PACKET_QUEUE_BUF:

		if (copy_from_user((void *)&buf_info, (void *)arg,
				   sizeof(tilepci_packet_queue_info_t))) {
			return -EFAULT;
		}

		/* 
		 * Check ring buffer alignment.
		 * The single packet buffer size must be a power of 2 and
		 * at least 4-byte long, ideally at least 128-byte.
		 */
		if (buf_info.buf_size < 4 ||
		    (buf_info.buf_size & (buf_info.buf_size - 1))) {
			printk("PCIe Packet Queue buffer size invalid.\n");
			return -EINVAL;
		}

		value = buf_info.buf_size * GXPCI_HOST_PQ_SEGMENT_ENTRIES *
			pq->num_segments;
		if (value > pq->ring_size) {
			printk("PCIe Packet Queue ring buffer too large.\n");
			return -EINVAL;
		}

		for (i = 0;  i < pq->num_segments; i++)
			if (pq->pq_segment[i] == NULL)
				return -EFAULT;

		pq->num_bufs = GXPCI_HOST_PQ_SEGMENT_ENTRIES * pq->num_segments;
		pq->buf_size = buf_info.buf_size;

		writel(pq->num_segments, &pq->regs->num_segments);
		writel(pq->segment_size, &pq->regs->segment_size);
		writel(pq->num_bufs, &pq->regs->num_bufs);
		writel(pq->buf_size, &pq->regs->buf_size);

		/*
		 * Set the base PA of ring buffer to the Packet Queue register.
		 */
		for (i = 0;  i < pq->num_segments; i++)
			writeq(pq->pq_segment_handle[i],
			       &pq->regs->segment_bus_addr[i]);

		/*
		 * Inform the EP of the queue status array bus address. This is
		 * needed by the VF driver only, but harmless for the PF driver.
		 */
		writeq(pq->tlr->queue_sts_bus_addr,
			&pq->regs->queue_sts_array_bus_addr);

		/*
		 * Inform both ends of our readiness.
		 */
		pq->queue_status->status = GXPCI_HOST_CHAN_READY;
		writel(GXPCI_HOST_CHAN_READY, &pq->regs->queue_status);

		break;
	case TILEPCI_IOC_GET_PACKET_QUEUE_BUF_PA:
		if (!pq->pq_segment_handle[0] ||
		    copy_to_user((void __user *)arg,
		    &pq->pq_segment_handle[0], sizeof(uint64_t)))
			return -EFAULT;

		break;
	case TILEPCI_IOC_GET_PQ_DEVICE_BAR:
		if (copy_from_user((void *)&bar_info, (void *)arg,
		    sizeof(tilegxpci_bar_info_t)))
			return -EFAULT;

		if (bar_info.bar_index != 0 && bar_info.bar_index != 2) {
			printk("PCIe device BAR index invalid, "
				"must be either 0 or 2.\n");
			return -EINVAL;
		}

                bar_info.bar_addr = pci_resource_start(pq->tlr->pci_dev,
						       bar_info.bar_index);
                bar_info.bar_size = pci_resource_len(pq->tlr->pci_dev,
						     bar_info.bar_index);
                bar_info.link_index = pq->tlr->link_index;
		if (copy_to_user((void __user *)arg, &bar_info,
				 sizeof(tilegxpci_bar_info_t)))
			return -EFAULT;

		break;
	default:
		return -EINVAL;
	}	
	return 0;
}

#ifdef CONFIG_COMPAT
static long 
tlr_packet_queue_compat_ioctl(struct file *filp,
                              unsigned int cmd, unsigned long arg)
{
	/* Sign-extend the argument so it can be used as a pointer. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
	return tlr_packet_queue_ioctl(filp->f_dentry->d_inode, filp,
				      cmd, (unsigned long)compat_ptr(arg));
#else
	return tlr_packet_queue_unlocked_ioctl(filp, cmd, 
					       (unsigned long)compat_ptr(arg));
#endif
}
#endif 
				
static int
tlr_packet_queue_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct tlr_packet_queue_state *pq = filp->private_data;
	struct tlr_pcie_dev* tlr = pq->tlr;
	struct pci_dev *pci_dev = tlr->pci_dev;
	size_t size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	resource_size_t phys_addr;
	int i;

	/* Map the Packet Queue registers and host ring buffer, respectively. */
	if (offset == TILEPCI_PACKET_QUEUE_INDICES_MMAP_OFFSET) {
		unsigned int pq_regs_offset;
		int channel;

		if (!(vma->vm_flags & VM_SHARED)) {
			printk("Tile PCIe Packet Queue indices mmap flags"
				" must include VM_SHARED\n");
			return -EINVAL;
		}

		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

		/*
		 * Figure out this Packet Queue's direction and channel index.
		 * Note that this logic has a dependency on the relative order
		 * of the H2T and T2H tlr_packet_queue_state arrays in
		 * struct tlr_pcie_dev.
		 */
		if (pq >= tlr->pq_t2h) {
			pq_regs_offset = tlr->pq_h2t_regs_offset +
				tlr->pq_drv_regs_map_size +
				tlr->pq_app_regs_map_size;
			channel = pq - tlr->pq_t2h;
		} else {
			pq_regs_offset = tlr->pq_h2t_regs_offset;
			channel = pq - tlr->pq_h2t;
		}

		phys_addr = pci_resource_start(pci_dev, 0) + pq_regs_offset +
			channel * (tlr->pq_drv_regs_map_size +
			tlr->pq_app_regs_map_size) * 2 +
			tlr->pq_drv_regs_map_size;

		if (remap_pfn_range(vma,
				    vma->vm_start,
				    phys_addr >> PAGE_SHIFT,
				    size,
				    vma->vm_page_prot)) {
			printk("Tile PCIe Packet Queue"
				" remap_pfn_range failed.\n");
			return -EAGAIN;
		}

	} else if (offset == TILEPCI_PACKET_QUEUE_BUF_MMAP_OFFSET) {

		for (i = 0;  i < pq->num_segments; i++)
			if (pq->pq_segment[i] == NULL)
				return -EFAULT;

		if (size > pq->segment_size * pq->num_segments)
			return -EINVAL;

		if (!(vma->vm_flags & VM_SHARED)) {
			printk("Tile PCIe Packet Queue buffer mmap flags"
				" must include VM_SHARED\n");
			return -EINVAL;
		}

		/* Don't try to swap out physical pages. */
		vma->vm_flags |= VM_RESERVED;

		vma->vm_ops = &packet_queue_vm_ops;
		vma->vm_private_data = pq;

		packet_queue_vma_open(vma);

	} else if (offset == TILEPCI_PACKET_QUEUE_STS_MMAP_OFFSET) {

		if (pq->queue_status == NULL)
			return -EFAULT;

		if (!(vma->vm_flags & VM_SHARED)) {
			printk("Tile PCIe Packet Queue status mmap flags"
				" must include VM_SHARED\n");
			return -EINVAL;
		}

		/* Don't try to swap out physical pages. */
		vma->vm_flags |= VM_RESERVED;

		vma->vm_ops = &packet_queue_sts_vm_ops;
		vma->vm_private_data = pq;

		packet_queue_sts_vma_open(vma);

	} else {
		printk("tlr_packet_queue_mmap offset invalid\n");
		return -EINVAL;
	}

	return 0;
}

static int
tlr_packet_queue_release(struct inode* inode, struct file* filp)
{
	struct tlr_pcie_dev* tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);
	struct tlr_packet_queue_state *pq = filp->private_data;
	struct gxpci_queue_pair_status *queue_pair_status;
	unsigned int minor = MINOR(inode->i_rdev);
	int sleep_msecs = 0;
	unsigned long flags;
	int queue_index;
	int err = 0;

	if (minor >= TILEPCI_FIRST_PQ_T2H_MINOR) {
		queue_index = minor - TILEPCI_FIRST_PQ_T2H_MINOR;
		queue_pair_status =
			&tlr->queue_sts_array->pq_sts_t2h[queue_index];
	} else {
		queue_index = minor - TILEPCI_FIRST_PQ_H2T_MINOR;
		queue_pair_status =
			&tlr->queue_sts_array->pq_sts_h2t[queue_index];
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
		if (sleep_msecs >= HOST_PQ_RELEASE_MAX_MSECS) {
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
			writel(GXPCI_CHAN_RESET, &pq->regs->queue_status);
			err = -EINTR;
			break;
		}
		sleep_msecs += GXPCI_QUEUE_MONITOR_INTERVAL;	

		spin_lock_irqsave(&tlr->lock, flags);
	}

	/* Set the queue status to uninitialized state. */
	queue_pair_status->host_queue_status = GXPCI_CHAN_UNINITIALIZED;
	queue_pair_status->host_queue_opened--;

	/* Set the struct tlr_pq_status. */
	pq->queue_status->status = GXPCI_CHAN_UNINITIALIZED;

	spin_unlock_irqrestore(&tlr->lock, flags);

#ifndef TILEPCI_VF
	/* This file blocks any reboot attempts; synchronize against
	 * somebody opening the boot file. */
	down(&reboot_mutex);
	
	tlr->board->packet_queue_cnt--;
	
	up(&reboot_mutex);
#endif

	return err;
}

struct file_operations tlr_packet_queue_ops = {
	.owner = THIS_MODULE,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
	.ioctl = tlr_packet_queue_ioctl,
#else
	.unlocked_ioctl = tlr_packet_queue_unlocked_ioctl,
#endif
#ifdef CONFIG_COMPAT
	.compat_ioctl = tlr_packet_queue_compat_ioctl,
#endif
	.mmap = tlr_packet_queue_mmap,
	.release = tlr_packet_queue_release,
};

int
tlr_packet_queue_open(struct tlr_pcie_dev *tlr, struct file *filp,
		      struct tlr_packet_queue_state *pq_state_base, int channel)
{
	struct tlr_packet_queue_state *pq = &pq_state_base[channel];
	struct tlr_packet_queue_state *reverse_pq;
	struct gxpci_queue_pair_status *queue_pair_status;
	unsigned int pq_regs_offset;
	int sleep_msecs = 0;
	unsigned long flags;
	int result = 0;

#ifndef TILEPCI_VF
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
	
	tlr->board->packet_queue_cnt++;

	up(&reboot_mutex);
#endif

	if (pq_state_base == tlr->pq_h2t) {
		queue_pair_status =
			&tlr->queue_sts_array->pq_sts_h2t[channel];
		reverse_pq = &tlr->pq_t2h[channel];
	} else {
		queue_pair_status =
			&tlr->queue_sts_array->pq_sts_t2h[channel];
		reverse_pq = &tlr->pq_h2t[channel];
	}

	filp->private_data = pq;
	filp->f_op = &tlr_packet_queue_ops;

	/*
	 * Map the driver-visible part the PQ control registers,
	 * when this Packet Queue is opened the first time. The unmapping
	 * is done when the driver is unloaded.
	 */
	down(&pq->mutex);
	if (pq->regs == NULL) {
		if (pq_state_base == tlr->pq_h2t)
			pq_regs_offset = tlr->pq_h2t_regs_offset;
		else
			pq_regs_offset = tlr->pq_h2t_regs_offset +
				tlr->pq_drv_regs_map_size +
				tlr->pq_app_regs_map_size;

		pq->regs = ioremap(pci_resource_start(tlr->pci_dev, 0) +
				pq_regs_offset + channel *
				(tlr->pq_drv_regs_map_size +
				tlr->pq_app_regs_map_size) * 2,
				sizeof(struct gxpci_host_pq_regs_drv));
		if (pq->regs == NULL) {
			up(&pq->mutex);
			result = -ENOMEM;
			goto exit;
		}
	}
	up(&pq->mutex);

	/*
	 * Poll for the tile application readiness.
	 */
	while (readl(&pq->regs->queue_status) != GXPCI_TILE_CHAN_READY) {
                if (sleep_msecs >= host_open_wait_secs * 1000) {
			result = -ENXIO;
			goto exit;
		}
                if (msleep_interruptible(HOST_PQ_OPEN_POLLING_MSECS)) {
			result = -EINTR;
			goto exit;
		}
                sleep_msecs += HOST_PQ_OPEN_POLLING_MSECS;
	}

	down(&pq->mutex);
	/* Separate H2T and T2H ring buffers. */
	if (pq->share_h2t_t2h_ring == 0) {
		if (pq->pq_segment[0] == NULL) {
			int err = tlr_packet_queue_alloc(pq);
			if (err != 0){
				up(&pq->mutex);
				result = err;
				goto exit;
			}
		}
	} else { /* Shared H2T and H2T ring buffer, protected by lock. */
		down(&tlr->pq_mutex[channel]);
		if (pq->pq_segment[0] == NULL) {
			int err = tlr_packet_queue_alloc_shared(pq, reverse_pq);
			if (err != 0){
				up(&tlr->pq_mutex[channel]);
				up(&pq->mutex);
				result = err;
				goto exit;
			}
		}
		up(&tlr->pq_mutex[channel]);
	}
	up(&pq->mutex);

	spin_lock_irqsave(&tlr->lock, flags);
	queue_pair_status->host_queue_opened++;
	spin_unlock_irqrestore(&tlr->lock, flags);

 exit:

#ifndef TILEPCI_VF
	/* Decrement the open counter if this open fails. */
	if (result) {
		down(&reboot_mutex);
	
		tlr->board->packet_queue_cnt--;

		up(&reboot_mutex);
	}
#endif

	return result;
}
