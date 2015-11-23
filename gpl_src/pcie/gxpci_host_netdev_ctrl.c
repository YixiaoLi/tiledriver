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
 * Tilera GX Command and Control channel.
 * This code enables communication with the applications running on TileGX
 * card, from the host kernel.
 * Implements a generic communication layer over PCI. 
 * Every board has one channel allocated for control communication.
 * 
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
#include <linux/namei.h>
#include <linux/if_arp.h>

#include <asm/uaccess.h>

#include "tilegxpci.h"
#include "tilegxpci_host.h"
#include "gxpci_host_netdev_ctrl.h"

/* Following code is compiled with GXPCI_HOST_SDNIC definition. */
#ifdef GXPCI_HOST_SDNIC

#ifdef NETDEV_CTRL_DEBUG
#define NETDEV_CTRL_TRACE(...)  printk(KERN_INFO __VA_ARGS__)
#define NETDEV_CTRL_ERR(...)  printk(KERN_ERR __VA_ARGS__)
#else
#define NETDEV_CTRL_TRACE(...)
#define NETDEV_CTRL_ERR(...)
#endif
 

static char proc_value = 'e';

/*
 * netdev_ctrl_async_msg - Handle asynchronous messages from the tile side like
 *			   LINK UP and LINK DOWN.
 */
int
netdev_ctrl_async_msg(tlr_netdev_ctrl_t *netdev_ctrl,
                      tlr_netdev_ctrl_frame_t *frame)
{
	struct tlr_pcie_dev *tlr = netdev_ctrl->tlr;
	if_tag_t if_tag;

	if (!(frame->hdr.flags & TLR_NETDEV_CTRL_FL_NOTIF))
		return 0;

	if (!netdev_ctrl->discovered) {
		NETDEV_CTRL_TRACE("Waiting for discovery, ignore async msgs\n");
		return 0;
	}
  
	if_tag = frame->hdr.if_tag;

	switch (frame->hdr.op_code) {
	case TLR_NETDEV_CTRL_LINK_UP:
		INFO("Link up for gxpci%d-eth%d\n",
		     tlr->link_index, (if_tag.inst + 1));
		netif_carrier_on(tlr->net_devs[if_tag.inst]->netdev);
		break;

	case TLR_NETDEV_CTRL_LINK_DOWN:
		INFO("Link down for gxpci%d-eth%d\n",
		     tlr->link_index, (if_tag.inst + 1));
		netif_carrier_off(tlr->net_devs[if_tag.inst]->netdev);
		break;
	default:
		return -1;
	}
	/* consumed */
	return 0;
}

/*
 * netdev_ctrl_read - Read the header and payload from a control channel to find
 * 		      the length of the payload. Note a buffer will be allocated
 * 		      here, and the user of this function is expected to free it
 * 		      after usage.
 * @netdev_ctrl: netdevice control structure
 * @buf: kernel buffer to be copied
 * @size: size of the buffer allocated
 * @timeout: timeout in milliseconds 
 */
static size_t
netdev_ctrl_read(tlr_netdev_ctrl_t *netdev_ctrl,
                 char *buf, size_t size, int timeout)
{
	unsigned long end = jiffies + jiffies_to_msecs(timeout);
	mm_segment_t old_fs;
	int rc = 0;

	/* Channel is not established return -EPIPE. */
	if (!netdev_ctrl || !netdev_ctrl->up)
		return -EPIPE;

	/* Set the FS to kernel. */
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	do {
		rc = vfs_read(netdev_ctrl->filp, buf, size,
				&netdev_ctrl->filp->f_pos);
		if (rc >= 0)
			break;

		if (rc != -EAGAIN)
			goto ret;

	} while (jiffies < end);

	/* Timeout occured. */
	if (rc == -EAGAIN)
		rc = -ETIME;

ret:
	/* Reset the kernel FS to old_fs. */
	set_fs(old_fs);

	return rc;
}
  
/*
 * netdev_ctrl_write - Write to the control channel. Note that this function
 * 		       does not check channel up, its responsibility of the
 * 		       caller to check whether the channel is up.
 * @netdev_ctrl: netdevice control structure
 * @buf: buffer to be written
 * @size: size of the buffer to be writen with header included
 */
static int
netdev_ctrl_write(tlr_netdev_ctrl_t *netdev_ctrl,
                  char * buf, size_t size, int timeout)
{
	unsigned long end = jiffies + jiffies_to_msecs(timeout);
	mm_segment_t old_fs;
	int rc = 0;

	/* Set the FS to kernel. */
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	do {
		rc = vfs_write(netdev_ctrl->filp, buf, size,
			       &netdev_ctrl->filp->f_pos);
		if (rc >= 0)
			break;
		if (rc != -EAGAIN)
			goto ret;

	} while (jiffies < end);

	/* Timeout occured. */
	if (rc == -EAGAIN)
		rc = -ETIME;

ret:
	/* Reset the FS. */
	set_fs(old_fs);

	return rc;
}

/*
 * tlr_netdev_ctrl_req_rsp - Control channel request and response.
 */
static int
tlr_netdev_ctrl_req_rsp(tlr_netdev_ctrl_t *netdev_ctrl, if_tag_t if_tag, 
			uint8_t app_code, uint32_t op_code, char *req_data,
			size_t req_size,  char *rsp_buf, size_t rsp_size,
			int timeout)
{
	tlr_netdev_ctrl_frame_t *req_frame = NULL;
	tlr_netdev_ctrl_frame_t *rsp_frame = NULL;
	int rc = 0;
	bool locked = FALSE;
  
	if (!netdev_ctrl || !netdev_ctrl->up)
		return -EPIPE;

	req_frame =
		(tlr_netdev_ctrl_frame_t *)kmalloc((TLR_HDR_SIZE + req_size),
						   GFP_ATOMIC);

	rsp_frame =
		(tlr_netdev_ctrl_frame_t *)kmalloc(BUFFER_SIZE, GFP_ATOMIC);

	if (req_frame == NULL || rsp_frame == NULL)
		return -ENOMEM;

	if (down_interruptible(&netdev_ctrl->rsem))
		goto free_and_return;

	/* Lock aquired. */
	locked = TRUE;

	/* Fill the request header. */
	req_frame->hdr.app_code = app_code;
	req_frame->hdr.op_code = op_code;
	req_frame->hdr.payload_len = req_size;
	req_frame->hdr.flags |= TLR_NETDEV_CTRL_FL_REQ;
	req_frame->hdr.if_tag = if_tag;
  
	if (req_size > 0)
		memcpy(req_frame->payload, req_data, req_size);

	rc = netdev_ctrl_write(netdev_ctrl, (char *)req_frame, 
			       TLR_HDR_SIZE + req_size, timeout / 2);

	if (rc < 0) {
		NETDEV_CTRL_TRACE("Write failed.\n");
		goto free_and_return;
	}

  
read_again:
	rc = netdev_ctrl_read(netdev_ctrl, (char *)rsp_frame, 
			      BUFFER_SIZE, timeout / 2);
  
	if (rc <= 0 || rc < TLR_HDR_SIZE) {
		NETDEV_CTRL_ERR("Read returned failure: %d.\n", rc);
		goto free_and_return;
	}

	/* Check for any error-status returned by tile. */
	if (rsp_frame->hdr.flags & TLR_NETDEV_CTRL_FL_ERR) {
		rc = rsp_frame->hdr.status;
		goto free_and_return;
	}
  
	if (rsp_frame->hdr.flags & TLR_NETDEV_CTRL_FL_NOTIF) {
		/* We received a async msg while waiting for rsp. */
		if (netdev_ctrl_async_msg(netdev_ctrl, rsp_frame) == 0) {
			/* Processed succesfully, wait for the response now. */
			goto read_again;
		}
		rc = -1;
		goto free_and_return;
	}
  
	if (!(rsp_frame->hdr.op_code == op_code)) {
		NETDEV_CTRL_ERR("Received unmatced op_code: %d "
				"rsp_op_code: %d.\n",
				op_code, rsp_frame->hdr.op_code);
		rc = -1;
		goto free_and_return;
	}
  
  
	/* Copy response if any exist & the app requests for it. */
	if (rsp_frame->hdr.payload_len > 0 && rsp_size > 0 && rsp_buf != NULL)
		memcpy(rsp_buf, rsp_frame->payload, rsp_size);
  
free_and_return:
	if (locked)
		up(&netdev_ctrl->rsem);

	if (req_frame)
		kfree(req_frame);

	if (rsp_frame)
		kfree(rsp_frame);
  
	return rc;
}

/*
 * tlr_netdev_ctrl_populate - Initalize the control channel pointer int the
 * 			      gxpci_nic structrure. All the NICs on the board
 * 			      have the pointers to a same control channel. Note
 *			      this should be called after all gxpci_nic
 *			      structures are initialized successfully.
 */
int 
tlr_netdev_ctrl_populate(struct net_device *netdev)
{
	struct gxpci_nic *nic = (struct gxpci_nic *)netdev_priv(netdev);
	struct tlr_pcie_dev *tlr = nic->tlr;
  
	/* Initialize the control channel. */
	if (likely(tlr->board->netdev_ctrl.up))
		nic->netdev_ctrl = &(tlr->board->netdev_ctrl);
	else
		nic->netdev_ctrl = NULL;

	return 0;
}

/*
 * tlr_netdev_ctrl_discover - Send a discover message to the TileGX card.
 */
int
tlr_netdev_ctrl_discover(tlr_netdev_ctrl_t *netdev_ctrl)
{
	tlr_board_t *board = container_of(netdev_ctrl, tlr_board_t,
					  netdev_ctrl);
	if_tag_t if_tag;
	int rc = 0;

	if_tag.u_tag = 0xFFFF;
	rc = tlr_netdev_ctrl_req_rsp(netdev_ctrl, if_tag, TLR_NETDEV_CTRL,
				     TLR_NETDEV_CTRL_DISCOVER, NULL, 0,
				     (char *)&board->tile_capability,
				     sizeof(struct tlr_tile_capability),
				     TLR_NETDEV_CTRL_TIMEOUT);
	if (rc > 0) {
		/* Set the number of interfaces. Min(Userconfig, tile links). */
		netdev_ctrl->tlr->nic_ports = 
			(netdev_ctrl->tlr->nic_ports < 
			 board->tile_capability.num_links) ?
			 netdev_ctrl->tlr->nic_ports       : 
			 board->tile_capability.num_links;

		netdev_ctrl->discovered = 1;

		return 0;
	}

	return rc;
}

int
tlr_netdev_ctrl_get_mac_address(tlr_netdev_ctrl_t *netdev_ctrl,
				int if_num, uint8_t *dev_addr)
{

	tlr_board_t *board = container_of(netdev_ctrl, tlr_board_t,
					  netdev_ctrl);

	memcpy(dev_addr, board->tile_capability.link_params[if_num].mac_addr,
	       ETH_ALEN);

	return 0;
}

int
tlr_netdev_ctrl_open(tlr_netdev_ctrl_t *netdev_ctrl)
{
	tlr_board_t *board = container_of(netdev_ctrl, tlr_board_t,
					  netdev_ctrl);
	int major = MAJOR(board->ports[0]->first_dev);
	char device[1024];

	snprintf(device, 1023, "/dev/tilegxpci_%d/1",major);
  
	if (proc_value == 'd')
		return -1;

	netdev_ctrl->filp = filp_open(device, O_RDWR | O_NONBLOCK, 0);
  
	if (IS_ERR(netdev_ctrl->filp)) {
		NETDEV_CTRL_ERR("Unable to open file.\n");
		return -1;
	}
  
	return 0;
}

int
tlr_netdev_ctrl_close(struct tlr_pcie_dev *tlr)
{
	tlr_netdev_ctrl_t *netdev_ctrl = &tlr->board->netdev_ctrl;
	int rc;

	/* Nothing to do. */
	if (!netdev_ctrl->up)
		return 0;

	/* Acquire rsem to avoid closing in middle of transaction. */
	if (down_interruptible(&netdev_ctrl->rsem))
		return -1;

	rc = filp_close(netdev_ctrl->filp, NULL);
  
	NETDEV_CTRL_TRACE("File close returned : %d.\n", rc);

	netdev_ctrl->filp = NULL;
	netdev_ctrl->up = 0;
  
	if (netdev_ctrl->discovered) {
		gxpci_net_devs_remove(tlr);

		/* Re-discover on bringup. */
		netdev_ctrl->discovered = 0;
	}

	INFO("Control channel disabled for board: %d.\n",
	     tlr->board->board_index);

	up(&netdev_ctrl->rsem);

	return rc;
}
  
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
void tlr_netdev_ctrl_work(struct work_struct *work)
#else
void tlr_netdev_ctrl_work(void *data)
#endif
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
	tlr_netdev_ctrl_t *netdev_ctrl = 
		container_of(work, struct tlr_netdev_ctrl_s,
			     netdev_ctrl_work.work);
#else
	tlr_netdev_ctrl_t *netdev_ctrl = (tlr_netdev_ctrl_t *)data;
#endif
	struct tlr_pcie_dev *tlr = netdev_ctrl->tlr;
	tlr_netdev_ctrl_frame_t *frame = NULL;
	int rc;
	bool booting = FALSE;

	if (gx_ep_not_booted(tlr)) {
		booting = TRUE;
		/*
 		 * Tile went down for a reboot from tile-monitor.
		 * close the channel.
		 */
		if (netdev_ctrl->up)
			tlr_netdev_ctrl_close(tlr);

    		goto schd_and_ret;
  	}
  
	if (!netdev_ctrl->up) {
		rc = tlr_netdev_ctrl_open(netdev_ctrl);
		if (rc == 0) {
			NETDEV_CTRL_TRACE("Opened the control channel.\n");
			/* Mark netdev ctrl channel as up. */
			netdev_ctrl->up = 1;
		}
		/* Schedule next iteration for discovery. */
		goto schd_and_ret;
	}
  
	if (!netdev_ctrl->discovered) {
		rc = tlr_netdev_ctrl_discover(netdev_ctrl);
		if (rc == 0) {
			NETDEV_CTRL_TRACE("Got a successful discovery "
					  "response.\n");
			/* Initialize the netdevices. */
			gxpci_net_devs_init(tlr);
			netdev_ctrl->discovered = 1;
		} else {
			tlr_netdev_ctrl_close(tlr);
		}

		goto schd_and_ret;
	}
  
	if (down_trylock(&netdev_ctrl->rsem)) {
		NETDEV_CTRL_TRACE("Another read in progress.\n");
		goto schd_and_ret;
	}

	frame = (tlr_netdev_ctrl_frame_t *)kmalloc(BUFFER_SIZE, GFP_ATOMIC);
	if (frame == NULL)
		goto free_and_ret;
  
	/* Read for any async msg. */
	rc = netdev_ctrl_read(netdev_ctrl, (char *)frame, BUFFER_SIZE, 0);
	if (rc <= 0 || rc < TLR_HDR_SIZE) {
		NETDEV_CTRL_ERR("Read returned failure: %d.\n", rc);
		goto free_and_ret;
	}
  
	if (frame->hdr.flags & TLR_NETDEV_CTRL_FL_NOTIF) {
		/* notification */
		netdev_ctrl_async_msg(netdev_ctrl, frame);
	}

free_and_ret:
	up(&netdev_ctrl->rsem);

	if (frame)
		kfree(frame);
schd_and_ret:
	if (booting)
		schedule_delayed_work(&netdev_ctrl->netdev_ctrl_work,
				      TLR_NETDEV_CTRL_BOOT_TIMEOUT);
	else
		schedule_delayed_work(&netdev_ctrl->netdev_ctrl_work,
				      TLR_NETDEV_CTRL_POLL_TIMEOUT);
	return;
}

/* Application function calls. */

/*
 * tlr_ifconfig_req_rsp - if_config request response.
 */
int
tlr_ifconfig_req_rsp(struct net_device *netdev, uint32_t op_code, char *req_buf,
		     size_t req_size, char *rsp_buf, size_t rsp_size)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	if_tag_t if_tag;

	if (!nic->netdev_ctrl || !nic->netdev_ctrl->up)
		return -EPIPE;

	/* Only Ethernet interfaces supported as of now. */
	if_tag.type = IF_TYPE_ETH;
	if_tag.inst = nic->if_num;

	NETDEV_CTRL_TRACE("tlr_ifconfig_req_rsp: op_code: %d inst: %d.\n",
			  op_code, if_tag.inst);

	return tlr_netdev_ctrl_req_rsp(nic->netdev_ctrl, if_tag, TLR_IFCONFIG,
				       op_code, req_buf, req_size, rsp_buf,
				       rsp_size, TLR_NETDEV_CTRL_TIMEOUT);
}

int
tlr_ethtool_req_rsp(struct net_device *netdev, uint32_t op_code, char *req_buf,
		    size_t req_size, char *rsp_buf, size_t rsp_size)
{
	struct gxpci_nic *nic = netdev_priv(netdev);
	if_tag_t if_tag;
	int rc;
  
	if (!nic->netdev_ctrl || !nic->netdev_ctrl->up)
		return -EPIPE;
  
	/* Only Ethernet interfaces are supported as of now. */
	if_tag.type = IF_TYPE_ETH;
	if_tag.inst = nic->if_num;

	NETDEV_CTRL_TRACE("tlr_ethtool_req_rsp: op_code: %d inst: %d.\n",
			  op_code, if_tag.inst);

	rc = tlr_netdev_ctrl_req_rsp(nic->netdev_ctrl, if_tag, TLR_ETHTOOL,
				     op_code, req_buf, req_size, rsp_buf,
				     rsp_size, TLR_NETDEV_CTRL_TIMEOUT);

	return rc;
} 

/* PROC read and write routines. */

static int
tlr_netdev_ctrl_proc_write(struct file *filp, const char __user *buff,
                           size_t count, void *data)
{
	char val;

	if (copy_from_user(&val, buff, sizeof(char)))
		return -EFAULT;
  
	if (val == 'd' || val == 'D') {
		tlr_netdev_ctrl_exit();
		INFO("The TileGx driver can now be uninstalled.\n");
	}
  
	proc_value = val;

	return count;
}

static int 
tlr_netdev_ctrl_proc_read(char *page, char **start, off_t off, int count,
                          int *eof, void *data)
{
	int len;

	NETDEV_CTRL_TRACE("In netdev_ctrl read.\n");

	len = sprintf(page, "%c\n", proc_value);

	return len;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
static const struct file_operations tilegxpci_netdev_ctrl_proc_fops = {
    .owner = THIS_MODULE,
    .read = tlr_netdev_ctrl_proc_read,
    .write = tlr_netdev_ctrl_proc_write,
};
#endif

int
tlr_netdev_ctrl_proc_entry_create(void)
{
	struct proc_dir_entry *entry = NULL;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
	entry = proc_create("driver/tilegxpci_netdev_ctrl", 0644, NULL, &tilegxpci_netdev_ctrl_proc_fops);
#else
	entry = create_proc_entry("driver/tilegxpci_netdev_ctrl", 0644, NULL);
#endif
	if (entry) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
		entry->data = NULL;
		entry->read_proc = tlr_netdev_ctrl_proc_read;
		entry->write_proc = tlr_netdev_ctrl_proc_write;
#endif
	} else {
		ERR("Failed to create /proc/driver/tilegxpci_netdev_ctrl.\n");
		return -EIO;
	}

	return 0;
}

/*
 * tlr_netdev_ctrl_init - Initalize a control channel between host and TileGx
 * 			  card, find the device's inode and open it, and
 * 			  schedule a delayed work routine to mornitor async
 * 			  notifications.
 */
int
tlr_netdev_ctrl_init(struct tlr_pcie_dev *tlr)
{
	tlr_board_t *board = tlr->board;
	tlr_netdev_ctrl_t *netdev_ctrl = &board->netdev_ctrl;

	board->netdev_ctrl.tlr = tlr;
	sema_init(&netdev_ctrl->rsem, 1);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
	INIT_DELAYED_WORK(&netdev_ctrl->netdev_ctrl_work, tlr_netdev_ctrl_work);
#else
	INIT_WORK(&netdev_ctrl->netdev_ctrl_work,
		  tlr_netdev_ctrl_work, netdev_ctrl);
#endif
	schedule_delayed_work(&netdev_ctrl->netdev_ctrl_work,
			      TLR_NETDEV_CTRL_POLL_TIMEOUT);

	return 0;
}
#endif /* GXPCI_HOST_SDNIC */
