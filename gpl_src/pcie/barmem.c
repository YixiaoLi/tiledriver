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
 * Routines for mapping BAR1 into application VA space.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <asm/uaccess.h> 
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <asm/page.h>
#include <asm/pgtable.h> 		/* io_remap_page_range */

#include "tilegxpci.h"
#include "tilegxpci_host.h"
#include "tilegxpci_version.h"

#define BAR1_RESOURCE_INDEX 2

static int
tlr_barmem_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct tlr_pcie_dev* tlr = file->private_data;
	struct pci_dev *pci_dev = tlr->pci_dev;
	size_t size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	resource_size_t phys_addr;

	/* MMIO mappings must be shared; copy-on-write would never
	 * work properly because these memory regions have side
	 * effects. */
	if (!(vma->vm_flags & VM_SHARED))
		return -EINVAL;
	
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	
	phys_addr = pci_resource_start(pci_dev, BAR1_RESOURCE_INDEX) + offset;
	
	if (remap_pfn_range(vma,
			    vma->vm_start,
			    phys_addr >> PAGE_SHIFT,
			    size,
			    vma->vm_page_prot)) {
		ERR("BAR1 remap_pfn_range() failed.\n");
		return -EAGAIN;
	}

	return 0;
}


static int
tlr_barmem_release(struct inode* inode, struct file* filp)
{
	struct tlr_pcie_dev* tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);
	
	/* This file blocks any reboot attempts; synchronize against
	 * somebody opening the boot file. */
	down(&reboot_mutex);
	
	tlr->board->barmem_cnt--;
	
	up(&reboot_mutex);
	return 0;
}

struct file_operations tlr_barmem_ops = {
	.owner = THIS_MODULE,
	.mmap = tlr_barmem_mmap,
	.release = tlr_barmem_release,
};


resource_size_t tlr_barmem_size(struct tlr_pcie_dev* tlr)
{
	return pci_resource_len(tlr->pci_dev, BAR1_RESOURCE_INDEX);
}


dma_addr_t tlr_barmem_base(struct tlr_pcie_dev* tlr)
{
	return 	pci_resource_start(tlr->pci_dev, BAR1_RESOURCE_INDEX);
}


int
tlr_barmem_open(struct inode *inode, struct file *filp)
{
	int result = 0;
	struct tlr_pcie_dev* tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);

	filp->private_data = tlr;
	filp->f_op = &tlr_barmem_ops;
	
	/* This file blocks any reboot attempts; synchronize against
	 * somebody opening the boot file. */
	if (down_interruptible(&reboot_mutex))
		return -ERESTARTSYS;
	
	if (tlr_barmem_size(tlr) == 0)
	{
		ERR("BAR1 is not present.\n");
		result = -ENXIO;
		goto exit;
	}

	tlr->board->barmem_cnt++;

 exit:
	up(&reboot_mutex);
	return result;
}
