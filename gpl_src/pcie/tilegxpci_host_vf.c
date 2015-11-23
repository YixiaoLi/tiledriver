/*
 * Tilera Gx PCIe SR-IOV virtual function driver
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
#include <linux/cdev.h>
#include <linux/delay.h>
#include <asm/uaccess.h> 
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/version.h>

#include "tilegxpci.h"
#include "tilegxpci_host_vf.h"
#include "tilegxpci_version.h"

#include "tilegxpci_host_common.h"

/* For now, no MSI-X support for VFs. */
#define USE_MSI_ONLY

/**********************************************************************/
/*                   Module Loading and Device Probe                  */
/**********************************************************************/

static struct pci_device_id vf_ids[] = {
	{ PCI_DEVICE(TILERA_VENDOR_ID, TILERA_GX36_DEV_VF_ID), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, vf_ids);

static int
tlr_vf_generic_open(struct inode *inode, struct file *filp)
{
	unsigned int minor = MINOR(inode->i_rdev);
	int result;
	int channel;
	struct tlr_pcie_dev *tlr =
		container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);

	if ((minor >= TILEPCI_FIRST_PQ_H2T_MINOR) &&
		(minor < (TILEPCI_FIRST_PQ_H2T_MINOR + tlr->pq_h2t_queues))) {
		channel = (minor - TILEPCI_FIRST_PQ_H2T_MINOR);
		result = tlr_packet_queue_open(tlr, filp, tlr->pq_h2t, channel);
	}
	else if ((minor >= TILEPCI_FIRST_PQ_T2H_MINOR) &&
		 (minor < (TILEPCI_FIRST_PQ_T2H_MINOR + tlr->pq_t2h_queues))) {
		channel = (minor - TILEPCI_FIRST_PQ_T2H_MINOR);
		result = tlr_packet_queue_open(tlr, filp, tlr->pq_t2h, channel);
	}
	else {
		result = -ENODEV;
	}

	return result;
}

static struct file_operations tlr_vf_generic_ops = {
	.owner = THIS_MODULE,
	.open = tlr_vf_generic_open,
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
	cdev_init(cdev, &tlr_vf_generic_ops);
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

static void host_nic_queue_check(struct tlr_pcie_dev *tlr, int port)
{
	struct gxpci_queue_pair_status *queue_pair_status;

#ifndef GXPCI_HOST_NIC_P2P
	queue_pair_status = &tlr->queue_sts_array->nic_sts[port];

	/*
	 * If both ends has GXPCI_CHAN_RESET, both are ready for
	 * reset and we set GXPCI_CHAN_RESET_ACK to both. Note that
	 * this is where both queues should end up under normal
	 * conditions.
	 */
	if (queue_pair_status->tile_queue_status == GXPCI_CHAN_RESET &&
	    queue_pair_status->host_queue_status == GXPCI_CHAN_RESET) {
		queue_pair_status->tile_queue_status = GXPCI_CHAN_RESET_ACK;
		queue_pair_status->host_queue_status = GXPCI_CHAN_RESET_ACK;
	} else if (queue_pair_status->tile_queue_status == GXPCI_CHAN_RESET) {
		/*
		 * Tile side initiated the reset. We bring down
		 * the NIC interface with dev_close() which sets
		 * GXPCI_CHAN_RESET for the host queue status.
		 */
		gxpci_net_dev_close(tlr, port);
	} else if (queue_pair_status->host_queue_status == GXPCI_CHAN_RESET) {
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
	struct gxpci_nic *nic;

	nic = tlr->net_devs[port];
	queue_pair_status = &nic->nic_regs_vf->queue_pair_status;

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
	} else if (queue_pair_status->tile_queue_status == GXPCI_CHAN_RESET) {

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

static void host_nic_queue_monitor(struct tlr_pcie_dev *tlr)
{
	int port;

	/* Monitor all the host NIC ports. */
	for (port = 0; port < tlr->nic_ports; port++)
		host_nic_queue_check(tlr, port);
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
static void ep_queue_monitor(struct work_struct *work)
#else
static void ep_queue_monitor(void *data)
#endif
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
	struct tlr_pcie_dev *tlr = container_of(work,
		struct tlr_pcie_dev, ep_queue_monitor_work.work);
#else
	struct tlr_pcie_dev *tlr = (struct tlr_pcie_dev *)data;
#endif
	unsigned long flags;

	spin_lock_irqsave(&tlr->lock, flags);

	pq_queue_monitor(tlr);

	spin_unlock_irqrestore(&tlr->lock, flags);

	host_nic_queue_monitor(tlr);

	/* Do it again later. */
	schedule_delayed_work(&tlr->ep_queue_monitor_work,
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
vf_probe(struct pci_dev *pci_dev, const struct pci_device_id *id)
{
	static int		link_index = 1;
	int			i;
	int			err;
	struct tlr_pcie_dev	*tlr;
#ifndef USE_MSI_ONLY
	int			host_nic_queue_vectors;
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
		ERR("Failed to find usable DMA configuration\n");
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
				ERR("Failed to find usable DMA "
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
	spin_lock_init(&tlr->lock);

	tlr->link_index = link_index++;

	tlr->bar0_address = pci_resource_start(pci_dev, 0);
	tlr->bar2_address = pci_resource_start(pci_dev, 2);

	/* Enable the device. */
	err = pci_enable_device(pci_dev);
	if (err != 0)
		goto enable_failed;

	/* Check for VM PQ configurations. */
	err = pq_parse_config(tlr);
	if (err < 0)
		goto pq_parse_config_failed;

	pq_queue_init(tlr);

	/* Create our character and boot devices. */
	err = tlr_add_device_nodes(tlr);
	if (err != 0)
		goto cdev_failed;

	/* Enable PCI bus mastering. */
	pci_set_master(pci_dev);

	/* 
	 * Allocate and map storage for queue status array.
	 */
	tlr->queue_sts_array = pci_alloc_consistent(pci_dev, 
		sizeof(struct gxpci_queue_status_array),
		&tlr->queue_sts_bus_addr);
	if (tlr->queue_sts_array == NULL) {
		ERR("Failed to alloc queue_sts_array\n");
		err = -ENOMEM;
		goto pci_alloc_failed;
	}

#ifndef USE_MSI_ONLY
	host_nic_queue_vectors =
		MAX(GXPCI_HOST_NIC_TX_QUEUES_VF, GXPCI_HOST_NIC_RX_QUEUES_VF);

	/* Set the number of MSI-X interrupt vectors requested. */
	tlr->msix_vectors = host_nic_queue_vectors;
	tlr->msix_host_nic_intr_vec_base = 0;
        tlr->msix_entries =
		kmalloc(sizeof(struct msix_entry) * tlr->msix_vectors,
			GFP_KERNEL);
	if (tlr->msix_entries == NULL) {
		err = -ENOMEM;
		goto alloc_msix_table_failed;
	}
	memset(tlr->msix_entries, 0,
		sizeof(struct msix_entry) * tlr->msix_vectors);

	for (i = 0; i < tlr->msix_vectors; i++)
		tlr->msix_entries[i].entry = i;

	i = pci_enable_msix(tlr->pci_dev, tlr->msix_entries, tlr->msix_vectors);
	if (i) {
		/* MSI-X allocation failed. */
		dev_err(&tlr->pci_dev->dev, "MSI-X enable failure: %d\n", i);
		tlr->msix_vectors = 0;
		goto alloc_msix_enable_failed;
	}
#else
	tlr->msix_vectors = 0;
	i = pci_enable_msi(tlr->pci_dev);
	if (i) {
		/* MSI-X allocation failed. */
		dev_err(&tlr->pci_dev->dev, "MSI enable failure: %d\n", i);
		goto msi_enable_failed;
	}
#endif

	/* Create the PCIe network devices. */
	gxpci_net_devs_init(tlr);

#if defined(CONFIG_UIO) || defined(CONFIG_UIO_MODULE)
	/* Create the PCIe UIO devices. */
	gxpci_uio_devs_init(tlr);
#endif

	/*
	 * Schedule work to monitor various queues' status.
	 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
	INIT_DELAYED_WORK(&tlr->ep_queue_monitor_work, ep_queue_monitor);
#else
	INIT_WORK(&tlr->ep_queue_monitor_work, ep_queue_monitor, tlr);
#endif
	schedule_delayed_work(&tlr->ep_queue_monitor_work,
			      GXPCI_QUEUE_MONITOR_INTERVAL);

	TRACE("VF Driver attached.\n");

	return 0;

#ifndef USE_MSI_ONLY
 alloc_msix_enable_failed:
	kfree(tlr->msix_entries);
 alloc_msix_table_failed:
#else
 msi_enable_failed:
#endif
 pci_alloc_failed:
	tlr_remove_device_nodes(tlr);
 cdev_failed:
 pq_parse_config_failed:
	pci_disable_device(pci_dev);
 enable_failed:
	kfree(tlr);
	TRACE("Error, exiting driver. %d\n", err);
	return err;
}


/* Called via pci_unregister_driver() when the module is removed. */
static void vf_remove(struct pci_dev *pci_dev)
{
	struct tlr_pcie_dev* tlr = dev_get_drvdata(&pci_dev->dev);

	gxpci_net_devs_remove(tlr);

#if defined(CONFIG_UIO) || defined(CONFIG_UIO_MODULE)
	/* Free the PCIe UIO devices. */
	gxpci_uio_devs_remove(tlr);
#endif

	tlr_remove_device_nodes(tlr);

	pq_queue_free(tlr);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
	cancel_delayed_work_sync(&tlr->ep_queue_monitor_work);
#else
	if (!cancel_delayed_work(&tlr->ep_queue_monitor_work))
		flush_scheduled_work();
#endif

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

	kfree(tlr->msix_entries);
	kfree(tlr);

	pci_disable_device(pci_dev);
	dev_set_drvdata(&pci_dev->dev, NULL);
}

static struct pci_driver pci_vf_driver = {
	.name = "tilegxpci_vf",
	.id_table = vf_ids,
	.probe = vf_probe,
	.remove = vf_remove,
};

static int __init tilegxpci_vf_init(void)
{
	INFO("Loaded VF driver version %s\n", TILEGXPCI_VERSION_STRING);

	return pci_register_driver(&pci_vf_driver);
}

static void __exit tilegxpci_vf_exit(void)
{
	pci_unregister_driver(&pci_vf_driver);
}

MODULE_LICENSE(LICENSE_STRING);
MODULE_AUTHOR("Tilera Corp.");
MODULE_VERSION(TILEGXPCI_VERSION_STRING);

module_init(tilegxpci_vf_init);
module_exit(tilegxpci_vf_exit);
