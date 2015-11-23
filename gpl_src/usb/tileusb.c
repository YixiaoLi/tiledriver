/*
 * tileusb.c - Tilera TILE-GX host-side driver
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

/*
 * This source code was originally derived from:
 *
 *   USB Skeleton driver - 2.0
 *
 *   Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * Some code was also lifted from the example drivers in "Linux Device
 * Drivers" by Alessandro Rubini and Jonathan Corbet, published by
 * O'Reilly & Associates.
 */

/* #define TILEUSB_DEBUG */
/* #define TILEUSB_NOISE */

/*
 * This forces only one Gx reset to occur at a time.  Once we've gotten
 * more experience with this mode we'll probably remove the #define.
 */
#define TILEUSB_RESET_MUTEX

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/termios.h>
#include <linux/workqueue.h>
#include <asm/termbits.h>

#include "tileusb.h"
#include "tileusb_version.h"


int tileusb_major = TILEUSB_MAJOR;
int tileusb_nr_devs = TILEUSB_NR_DEVS;

module_param(tileusb_major, int, S_IRUGO);
module_param(tileusb_nr_devs, int, S_IRUGO);

static char *tileusb_minor_names[TILEUSB_MINORS] = {
	[TILEUSB_MINOR_RSHIM] = "rshim",
	[TILEUSB_MINOR_BOOT] = "boot",
	[TILEUSB_MINOR_LOCK] = "lock",
	[TILEUSB_MINOR_INFO] = "info",
	[TILEUSB_MINOR_CONSOLE] = "console",
	[TILEUSB_MINOR_TMFIFO] = "0",
};

/* Table of devices that work with this driver */
static struct usb_device_id tileusb_table[] = {
	{ USB_DEVICE(USB_TILERA_VENDOR_ID, USB_TILEGX_PRODUCT_ID) },
	{ }					/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, tileusb_table);

dev_t tileusb_dev_t;

/*
 * Array of all of the tileusb devices.  The high bits of our minor number
 * index into this table to find the relevant device.
 */
static struct tileusb **tileusb_devs;

/*
 * Array of pointers to kmalloc'ed strings, holding the USB path name for
 * all of the devices we've seen.  If tileusb_devs[i] is non-NULL, then
 * tileusb_dev_names[i] is its path name.  If tileusb_devs[i] is NULL, then
 * tileusb_dev_names[i] is the name that was last used for that device.
 * When we see a new device, we look it up in this table; this allows us to
 * use the same device index we did last time we saw the device.  The
 * strings within the array persist until the driver is unloaded.
 */
static char **tileusb_dev_names;

/*
 * Work queue.  Right now we have one for the whole driver; we might
 * eventually decide that we need one per device, but we'll see.
 */
struct workqueue_struct *tileusb_wq;

/*
 * Mutex protecting tileusb_devs[] and tileusb_dev_names[]; must be held
 * whenever reading or writing those tables.
 */
static DEFINE_MUTEX(tileusb_mutex);

/* Class structure for our device class. */
static struct class *tileusb_class;

/* Random compatibility hacks. */

/* Arguments to an fsync entry point. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
#define FSYNC_ARGS struct file *file, struct dentry *dentry, int datasync
#define FSYNC_CALL file, dentry, datasync
#else
#define FSYNC_ARGS struct file *file, int datasync
#define FSYNC_CALL file, datasync
#endif

/* Arguments to an urb completion handler. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
#define URB_COMP_ARGS struct urb *urb, struct pt_regs *regs
#else
#define URB_COMP_ARGS struct urb *urb
#endif

/* Buffer alloc/free routines. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
#define usb_alloc_coherent usb_buffer_alloc
#define usb_free_coherent usb_buffer_free
#endif

static void tileusb_delete(struct kref *kref)
{
	struct tileusb *dev = container_of(kref, struct tileusb, kref);

	int i;

	for (i = 0; i < TILEUSB_MINORS; i++) {
#ifdef TILEUSB_DEBUG
		char devbuf[32];
#endif
		cdev_del(&dev->cdevs[i]);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
		class_device_destroy(
#else
		device_destroy(
#endif
			       tileusb_class,
			       tileusb_dev_t + dev->dev_index *
			       TILEUSB_MINORS + i);
		TRACE("destroyed dev %s",
		      format_dev_t(devbuf, tileusb_dev_t +
				   dev->dev_index * TILEUSB_MINORS + i));
	}

	tileusb_devs[dev->dev_index] = NULL;

	kfree(dev);
}

/*
 * Wait for boot to complete, if necessary.  Return 0 if the boot is done
 * and it's safe to continue, an error code if something went wrong.  Note
 * that this routine must be called with the device mutex held.  If it
 * returns successfully, the mutex will still be held (although it may have
 * been dropped and reacquired); if it returns unsuccessfully the mutex
 * will have been dropped.
 */
static int wait_for_boot_done(struct tileusb *dev)
{
	int retval;

	if (!dev->rshim_interface || (dev->flags & TILEUSB_FLG_BOOTING)) {
		while (dev->flags & TILEUSB_FLG_BOOTING) {
			TRACE("boot write, waiting for re-probe");
			/* We're booting, and the device isn't ready yet. */
			mutex_unlock(&dev->mutex);
			/*
			 * FIXME: might we want a timeout here, too?  If
			 * the reprobe takes a very long time, something's
			 * probably wrong.  Maybe a couple of minutes?
			 */
			retval = wait_for_completion_interruptible(
				&dev->booting_complete);
			if (retval)
				return retval;
			mutex_lock(&dev->mutex);
		}
		if (!dev->rshim_interface) {
			mutex_unlock(&dev->mutex);
			return -ENODEV;
		}
	}

	return 0;
}

/* Rshim read/write routines */

static int tileusb_read_rshim(struct tileusb *dev, int chan, int addr,
			      u64 *result)
{
	int retval;

	if (!dev->rshim_interface)
		return -ENODEV;

	/* Do a blocking control read to get data from the device */
	retval = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),
			         0,  /* request */
			         USB_RECIP_ENDPOINT | USB_TYPE_VENDOR |
			         USB_DIR_IN,  /* request type */
			         chan, /* value */
			         addr, /* index */
			         result, 8, 2000);

	if (retval == 8)
		return 0;

	/*
	 * These are weird error codes, but we want to use something
	 * the USB stack doesn't use so that we can identify short/long
	 * reads.
	 */
	return retval >= 0 ? (retval > 8 ? -EBADE : -EBADR) : retval;
}

static int tileusb_write_rshim(struct tileusb *dev, int chan, int addr,
			       u64 value)
{
	int retval;

	if (!dev->rshim_interface)
		return -ENODEV;

	/* Do a blocking control write to get data to the device */
	retval = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
			         0,  /* request */
			         USB_RECIP_ENDPOINT | USB_TYPE_VENDOR |
			         USB_DIR_OUT,  /* request type */
			         chan, /* value */
			         addr, /* index */
			         &value, 8, 2000);

	if (retval == 8)
		return 0;

	/*
	 * These are weird error codes, but we want to use something
	 * the USB stack doesn't use so that we can identify short/long
	 * writes.
	 */
	return retval >= 0 ? (retval > 8 ? -EBADE : -EBADR) : retval;
}

/* Rshim file operations routines */

static ssize_t tileusb_rshim_read(struct file *file, char *user_buffer,
				  size_t count, loff_t *ppos)
{
	struct tileusb *dev;
	int retval = 0;
	u64 buf;


	/* rshim registers are all 8-byte aligned. */
	if (count != 8 || (*ppos & 7) != 0)
		return -EINVAL;

	dev = file->private_data;

	mutex_lock(&dev->mutex);
	retval = wait_for_boot_done(dev);
	if (retval) {
		TRACE("rshim read: can't read, no device");
		/* wait_for_boot_done already dropped mutex */
		return retval;
	}

	retval = tileusb_read_rshim(dev,
				    (*ppos >> 16) & 0xF, /* channel # */
				    *ppos & 0xFFFF,	 /* addr */
				    &buf);

	mutex_unlock(&dev->mutex);

	/* If the read was successful, copy the data to userspace */
	if (!retval && copy_to_user(user_buffer, &buf, count))
		return -EFAULT;

	return retval ? retval : count;
}

static ssize_t tileusb_rshim_write(struct file *file, const char *user_buffer,
				   size_t count, loff_t *ppos)
{
	struct tileusb *dev;
	int retval = 0;
	u64 buf;


	/* rshim registers are all 8-byte aligned. */
	if (count != 8 || (*ppos & 7) != 0)
		return -EINVAL;

	/* Copy the data from userspace */
	if (copy_from_user(&buf, user_buffer, count))
		return -EFAULT;

	dev = file->private_data;

	mutex_lock(&dev->mutex);
	retval = wait_for_boot_done(dev);
	if (retval) {
		TRACE("rshim write: can't write, no device");
		/* wait_for_boot_done already dropped mutex */
		return retval;
	}

	retval = tileusb_write_rshim(dev,
				     (*ppos >> 16) & 0xF, /* channel # */
				     *ppos & 0xFFFF,	  /* addr */
				     buf);

	mutex_unlock(&dev->mutex);

	return retval ? retval : count;
}

static int tileusb_rshim_release(struct inode *inode, struct file *file)
{
	struct tileusb *dev = file->private_data;

	mutex_lock(&tileusb_mutex);
	kref_put(&dev->kref, tileusb_delete);
	mutex_unlock(&tileusb_mutex);

	return 0;
}

static const struct file_operations tileusb_rshim_fops = {
	.owner =	THIS_MODULE,
	.read =		tileusb_rshim_read,
	.write =	tileusb_rshim_write,
	.release =	tileusb_rshim_release,
};

static int tileusb_rshim_open(struct file *file)
{
	file->f_op = &tileusb_rshim_fops;
	return 0;
}

/* Boot file operations routines */

static void tileusb_boot_write_callback(URB_COMP_ARGS)
{
	struct tileusb *dev;

	dev = urb->context;

	if (urb->status == -ENOENT)
		TRACE("boot urb canceled, actual length %d",
		      urb->actual_length);
	else if (urb->status)
		TRACE("boot urb failed, status %d, actual length %d",
		      urb->status, urb->actual_length);
	else
		NOISE("boot urb succeeded, actual length %d",
		      urb->actual_length);

#ifdef TILEUSB_DEBUG
	if (urb->status)
		dev->boot_write_badstat++;
	dev->boot_write_actual += urb->actual_length;
	dev->boot_write_requested += urb->transfer_buffer_length;
#endif

	complete_all(&dev->boot_write_complete);
}

static ssize_t tileusb_boot_write(struct file *file, const char *user_buffer,
				  size_t count, loff_t *ppos)
{
	struct tileusb *dev = file->private_data;
	int retval = 0;
	struct urb *urb = NULL;
	int whichbuf = 0;
	size_t bytes_written = 0;
	size_t bytes_left;

	/*
	 * Hardware requires that we send multiples of 8 bytes.  Ideally
	 * we'd handle the case where we got unaligned writes by
	 * accumulating the residue somehow, but none of our clients
	 * typically do this, so we just clip the size to prevent any
	 * inadvertent errors from causing hardware problems.
	 */
	bytes_left = count & - (size_t) 8;

	mutex_lock(&dev->mutex);
	if (dev->flags & TILEUSB_FLG_BOOT_WRITE) {
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	retval = wait_for_boot_done(dev);
	if (retval) {
		TRACE("boot_write: wait for boot failed, err %d", retval);
		/* wait_for_boot_done already dropped mutex */
		return retval;
	}

	/*
	 * We're going to drop the mutex while we wait for any outstanding
	 * urb to complete; this keeps another thread from getting in here
	 * while we do that.
	 */
	dev->flags |= TILEUSB_FLG_BOOT_WRITE;

	/*
	 * FIXME: we should look at the first bit of data to make sure
	 * it's in the right format, for the right chip, etc.
	 */
	while (bytes_left || urb || dev->boot_urb) {
		/*
		 * Phase 1: if we have data, but don't have an urb ready to
		 * go, load one up and get it ready.
		 */
		if (!urb && bytes_left) {
			size_t bytes_this_urb = min((size_t) BOOT_BUF_SIZE,
						    bytes_left);
			char *buf = dev->boot_buf[whichbuf];
			whichbuf ^= 1;

			NOISE("boot_write: copying %zd bytes from user",
			      bytes_this_urb);
			if (copy_from_user(buf, user_buffer, bytes_this_urb)) {
				retval = -EFAULT;
				TRACE("boot_write: copy from user failed");
			} else {
				/* Create an urb */
				urb = usb_alloc_urb(0, GFP_KERNEL);
				if (!urb) {
					retval = -ENOMEM;
					TRACE("boot_write: couldn't allocate "
					      "urb");
				} else {
					/* Initialize the urb properly. */
					usb_fill_bulk_urb(urb, dev->udev,
						  usb_sndbulkpipe(dev->udev,
							  dev->boot_fifo_ep),
						  buf, bytes_this_urb,
						  tileusb_boot_write_callback,
						  dev);
					bytes_left -= bytes_this_urb;
					user_buffer += bytes_this_urb;
				}
			}
		}

		/*
		 * Phase 2: if there's an outstanding urb, wait for it to
		 * finish.  If it got canceled, adjust it to remove any
		 * data that did get sent so we can resubmit it.
		 */
		if (dev->boot_urb) {
			int complete_retval;

			NOISE("boot_write: waiting for urb completion");
			mutex_unlock(&dev->mutex);
			complete_retval = wait_for_completion_interruptible(
				&dev->boot_write_complete);
			mutex_lock(&dev->mutex);

			if (complete_retval) {
				TRACE("boot_write: urb completion interrupted");
				if (!retval)
					retval = complete_retval;
				usb_kill_urb(dev->boot_urb);
				bytes_written += dev->boot_urb->actual_length;
				usb_free_urb(dev->boot_urb);
				dev->boot_urb = NULL;
				usb_free_urb(urb);
				break;
			}

			if (dev->boot_urb->actual_length !=
			    dev->boot_urb->transfer_buffer_length)
				TRACE("length mismatch, exp %d act %d stat %d "
		                      "tot req %lld tot act %lld",
			    		dev->boot_urb->transfer_buffer_length,
					dev->boot_urb->actual_length,
			    		dev->boot_urb->status,
		     			dev->boot_write_requested,
		     			dev->boot_write_actual);

#ifdef TILEUSB_BMC
			/*
			 * The UHCI host controller on the BMC seems to
			 * overestimate the amount of data it's
			 * successfully sent when it sees a babble error.
			 */
			if (dev->boot_urb->status == -EOVERFLOW &&
			    dev->boot_urb->actual_length >= 64) {
				dev->boot_urb->actual_length -= 64;
				TRACE("saw babble, new length %d",
				      dev->boot_urb->actual_length);
			}
#endif

			bytes_written += dev->boot_urb->actual_length;

			if (dev->boot_urb->status == -ENOENT &&
			    dev->boot_urb->transfer_buffer_length !=
			    dev->boot_urb->actual_length) {
				dev->boot_urb->transfer_buffer +=
					dev->boot_urb->actual_length;
				dev->boot_urb->transfer_buffer_length -=
					dev->boot_urb->actual_length;
				TRACE("boot_write: urb canceled, %d bytes "
				      "left, restarting",
				      dev->boot_urb->transfer_buffer_length);
			} else {
				if (dev->boot_urb->status)
					TRACE("boot_write: urb failed, "
					      "status %d",
					      dev->boot_urb->status);
				if (dev->boot_urb->status != -ENOENT && !retval)
					retval = dev->boot_urb->status;
				usb_free_urb(dev->boot_urb);
				dev->boot_urb = NULL;
				if (retval)
					break;
			}
		}

		/*
		 * If we've already gotten an error, we don't want to
		 * submit a new urb.  Note that we don't want to leave
		 * before we've waited for any outstanding urb, which is
		 * why we didn't check for this earlier.
		 */
		if (retval)
			break;

		/*
		 * Phase 3: if we have another urb to submit, do it.  This
		 * might be a new one we generated in phase 1, or one that
		 * was only partially completed from phase 2.
		 */
		if (urb || dev->boot_urb) {
			int urb_retval;

			if (!dev->boot_urb) {
				NOISE("boot_write: issuing new urb");
				dev->boot_urb = urb;
				urb = NULL;
			}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
			INIT_COMPLETION(dev->boot_write_complete);
#else
			reinit_completion(&(dev->boot_write_complete));
#endif
			urb_retval = usb_submit_urb(dev->boot_urb, GFP_KERNEL);

			if (urb_retval) {
				TRACE("boot_write: urb submission failed, "
				      "err %d", urb_retval);
				retval = urb_retval;
				break;
			}
		}
	}

	/*
	 * Not needed in the common case, but if we left the loop above
	 * due to a copy_from_user fault or an urb submission failure,
	 * dev->boot_urb may still be valid.
	 */
	usb_free_urb(dev->boot_urb);
	dev->boot_urb = NULL;

	dev->flags &= ~TILEUSB_FLG_BOOT_WRITE;
	mutex_unlock(&dev->mutex);
	return bytes_written ? bytes_written : retval;
}

static int tileusb_boot_fsync(FSYNC_ARGS)
{
	/*
	 * We don't really need this anymore, since we no longer launch
	 * more than one USB urb at a time, and don't return from write()
	 * until it's completed.  However, previous versions of the driver
	 * supported this entry point, and if we don't have it,
	 * applications which were written using that driver will get
	 * EINVAL from their fsync() calls and fail.
	 */
	return 0;
}

static int tileusb_boot_release(struct inode *inode, struct file *file)
{
	struct tileusb *dev = file->private_data;

	mutex_lock(&dev->mutex);
	dev->flags &= ~TILEUSB_FLG_BOOT_OPEN;

#if 0
	/*
	 * This dumps the rshim scratchpad after each boot; it's useful for
	 * certain sorts of debugging when coombined with special bootroms,
	 * but not so useful that we turn it on whenever debug is enabled.
	 */
	{
		u64 rshim_sp;
		int rv = tileusb_read_rshim(dev, 0, 0x20, &rshim_sp);
		INFO("boot_release, sp read got 0x%llx, rv %d",
		     rshim_sp, rv);
	}
#endif

	TRACE("bad %d tot req %lld tot act %lld",
	      dev->boot_write_badstat,
	      dev->boot_write_requested,
	      dev->boot_write_actual);

	mutex_unlock(&dev->mutex);

	mutex_lock(&tileusb_mutex);
	kref_put(&dev->kref, tileusb_delete);
	mutex_unlock(&tileusb_mutex);

	return 0;
}

static const struct file_operations tileusb_boot_fops = {
	.owner =	THIS_MODULE,
	.write =	tileusb_boot_write,
	.fsync =	tileusb_boot_fsync,
	.release =	tileusb_boot_release,
};

static int tileusb_boot_open(struct file *file)
{
	int retval;
	int i;
	struct tileusb *dev = file->private_data;
#ifdef TILEUSB_RESET_MUTEX
	unsigned long devs_locked = 0;
#endif

	file->f_op = &tileusb_boot_fops;

#ifdef TILEUSB_RESET_MUTEX
	/*
	 * We're going to prevent resets and USB operations from running in
	 * parallel with other resets.  Our method for this is to grab
	 * every device's mutex before doing the reset, and then holding
	 * onto them until the device we reset is reprobed, or a timeout
	 * expires; the latter is mostly paranoia.  Anyway, in order to
	 * find all of the other devices, we're going to need to walk the
	 * device table, so we need to grab its mutex.  We have to do it
	 * before we get our own device's mutex for lock ordering reasons.
	 */
	mutex_lock(&tileusb_mutex);
#endif

	mutex_lock(&dev->mutex);

	if (dev->flags & TILEUSB_FLG_BOOT_OPEN) {
		INFO("can't boot, boot file already open");
		mutex_unlock(&dev->mutex);
#ifdef TILEUSB_RESET_MUTEX
		mutex_unlock(&tileusb_mutex);
#endif
		return -EBUSY;
	}

	if (dev->flags & TILEUSB_FLG_TM_OPEN) {
		INFO("can't boot, tile-monitor file already open");
		mutex_unlock(&dev->mutex);
#ifdef TILEUSB_RESET_MUTEX
		mutex_unlock(&tileusb_mutex);
#endif
		return -EBUSY;
	}

#ifndef TILEUSB_BMC
	/*
	 * Currently tilemon-proxy holds the lock file and opens the boot
	 * file in different processes, so we can't enforce the lock as we
	 * would like.  We need to fix this; it might be sufficient to have
	 * the top-level daemon do setpgrp() and then just save the pgrp of
	 * the locking process.
	*/
	if (dev->lockfile_pid >= 0 && dev->lockfile_pid != current->pid) {
		INFO("can't boot, lock file already open by pid %d",
		     dev->lockfile_pid);
		mutex_unlock(&dev->mutex);
#ifdef TILEUSB_RESET_MUTEX
		mutex_unlock(&tileusb_mutex);
#endif
		return -EBUSY;
	}
#endif

	if (!dev->rshim_interface) {
		mutex_unlock(&dev->mutex);
#ifdef TILEUSB_RESET_MUTEX
		mutex_unlock(&tileusb_mutex);
#endif
		return -ENODEV;
	}

#ifdef TILEUSB_DEBUG
	dev->boot_write_badstat = 0;
	dev->boot_write_actual = 0;
	dev->boot_write_requested = 0;
#endif

	TRACE("begin booting");
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
	INIT_COMPLETION(dev->booting_complete);
#else
	reinit_completion(&(dev->booting_complete));
#endif
	dev->flags |= TILEUSB_FLG_BOOTING;

	/*
	 * Before we reset the chip, make sure we don't have any
	 * outstanding writes, and flush the write and read FIFOs.  (Note
	 * that we can't have any outstanding reads, since we kill those
	 * upon release of the TM FIFO file.)
	 */
	usb_kill_urb(dev->write_urb);
	dev->read_buf_bytes = 0;
	dev->read_buf_pkt_rem = 0;
	spin_lock_irq(&dev->spinlock);
	/* FIXME: should we be waiting for WRITING to go off, instead? */
	dev->spin_flags &= ~TILEUSB_SFLG_WRITING;
	for (i = 0; i < TM_CHANNELS; i++) {
		read_reset(dev, i);
		write_reset(dev, i);
	}
	spin_unlock_irq(&dev->spinlock);

	/*
	 * Soft reset the chip.  First we clear the reset mask; we expect
	 * the USB shim to get reset, so we want to make sure it isn't
	 * in the mask.  We also save and then clear the boot mode, so that
	 * we don't end up rebooting from SROM or I2C.  (That interacts
	 * very badly with our simultaneously trying to boot over USB.)
	 */
	retval = tileusb_write_rshim(dev, 0, RSH_RESET_MASK, 0);
	if (retval) {
		ERROR("boot_open: error %d writing reset mask", retval);
		dev->flags &= ~TILEUSB_FLG_BOOTING;
		mutex_unlock(&dev->mutex);
#ifdef TILEUSB_RESET_MUTEX
		mutex_unlock(&tileusb_mutex);
#endif
		return retval;
	}

	retval = tileusb_read_rshim(dev, 0, RSH_BOOT_CONTROL,
				    &dev->boot_control);
	if (retval) {
		ERROR("boot_open: error %d reading boot control", retval);
		dev->flags &= ~TILEUSB_FLG_BOOTING;
		mutex_unlock(&dev->mutex);
#ifdef TILEUSB_RESET_MUTEX
		mutex_unlock(&tileusb_mutex);
#endif
		return retval;
	}

	retval = tileusb_write_rshim(dev, 0, RSH_BOOT_CONTROL, 0);
	if (retval) {
		ERROR("boot_open: error %d writing boot control", retval);
		dev->flags &= ~TILEUSB_FLG_BOOTING;
		mutex_unlock(&dev->mutex);
#ifdef TILEUSB_RESET_MUTEX
		mutex_unlock(&tileusb_mutex);
#endif
		return retval;
	}

#ifdef TILEUSB_RESET_MUTEX
	/*
	 * Acquire all of the other devices' mutexes, to keep them from
	 * doing anything while we're performing the reset.  Also kill
	 * any outstanding boot urbs; that way we'll restart them, after
	 * the reset is done, and not report errors to the writers.
	 */
        for (i = 0; i < tileusb_nr_devs; i++) {
                if (tileusb_devs[i] && tileusb_devs[i] != dev) {
			mutex_lock(&tileusb_devs[i]->mutex);
			devs_locked |= 1UL << i;
			usb_kill_urb(tileusb_devs[i]->boot_urb);
		}
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
	INIT_COMPLETION(dev->reset_complete);
#else
	reinit_completion(&(dev->reset_complete));
#endif
#endif

	retval = tileusb_write_rshim(dev, 0, RSH_RESET_CONTROL,
				     RSH_RESET_CONTROL__RESET_CHIP_VAL_KEY);
	/*
	 * Note that occasionally, we get various errors on writing to
	 * the reset register.  This appears to be caused by the chip
	 * actually resetting before the response goes out, or perhaps by
	 * our noticing the device unplug before we've seen the response.
	 * Either way, the chip _does_ actually reset, so we just ignore
	 * the error.  Should we ever start getting these errors without
	 * the chip being reset, we'll have to figure out how to handle
	 * this more intelligently.  (One potential option is to not reset
	 * directly, but to set up a down counter to do the reset, but that
	 * seems kind of kludgy, especially since Tile software might also
	 * be trying to use the down counter.)
	 */
	if (retval && retval != -EPROTO && retval != -ESHUTDOWN &&
#ifdef TILEUSB_BMC
	    /*
	     * The host driver on the BMC sometimes produces EOVERFLOW on
	     * reset.  It also seems to have seems to have some sort of bug
	     * which makes it return more bytes than we actually wrote!  In
	     * that case we're returning EBADE.
	     */
	    retval != -EOVERFLOW && retval != -EBADE &&
#endif
	    retval != -ETIMEDOUT && retval != -EPIPE) {
		ERROR("boot_open: error %d writing reset control", retval);
		mutex_unlock(&dev->mutex);
#ifdef TILEUSB_RESET_MUTEX
		while (devs_locked) {
			int i = __builtin_ctzl(devs_locked);
			mutex_unlock(&tileusb_devs[i]->mutex);
			devs_locked &= ~(1UL << i);
		}
		mutex_unlock(&tileusb_mutex);
#endif
		return retval;
	}

	if (retval)
		TRACE("boot_open: got error %d on reset write", retval);

	dev->flags |= TILEUSB_FLG_BOOT_OPEN;

	mutex_unlock(&dev->mutex);

#ifdef TILEUSB_RESET_MUTEX
	mutex_unlock(&tileusb_mutex);
	/*
	 * We wait for reset_complete (signaled by probe), or for an
	 * interrupt, or a timeout.  Note that we dropped dev->mutex above
	 * so that probe can run; the BOOT_OPEN flag should keep our device
	 * from trying to do anything before the device is reprobed.
	 */
	retval = wait_for_completion_interruptible_timeout(&dev->reset_complete,
							   10 * HZ);
	if (retval == 0)
		ERROR("timed out waiting for device reprobe after reset");

        while (devs_locked) {
		int i = __builtin_ctz(devs_locked);
		mutex_unlock(&tileusb_devs[i]->mutex);
		devs_locked &= ~(1UL << i);
	}
#endif

	return 0;
}

/* Lock file operations routines */

static int tileusb_lock_release(struct inode *inode, struct file *file)
{
	struct tileusb *dev = file->private_data;

	mutex_lock(&dev->mutex);
	dev->lockfile_pid = -1;
	mutex_unlock(&dev->mutex);

	mutex_lock(&tileusb_mutex);
	kref_put(&dev->kref, tileusb_delete);
	mutex_unlock(&tileusb_mutex);

	return 0;
}

static const struct file_operations tileusb_lock_fops = {
	.owner =	THIS_MODULE,
	.release =	tileusb_lock_release,
};

static int tileusb_lock_open(struct file *file)
{
	struct tileusb *dev = file->private_data;

	file->f_op = &tileusb_lock_fops;

	mutex_lock(&dev->mutex);

	if (dev->flags & TILEUSB_FLG_BOOT_OPEN) {
		INFO("can't lock, boot file already open");
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	if (dev->lockfile_pid >= 0 && dev->lockfile_pid != current->pid) {
		INFO("can't lock, lock file already open by pid %d",
		     dev->lockfile_pid);
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	dev->lockfile_pid = current->pid;

	mutex_unlock(&dev->mutex);

	return 0;
}

/* Info file operations routines */

static int
tileusb_info_seq_show(struct seq_file *s, void *token)
{
	struct tileusb *dev = s->private;

	seq_printf(s, "CHIP_VERSION %d\n", dev->chip_version);
	seq_printf(s, "CHIP_WIDTH %d\n", dev->chip_width);
	seq_printf(s, "CHIP_HEIGHT %d\n", dev->chip_height);

	return 0;
}

static void
fill_chip_info(struct tileusb *dev)
{
	u64 fabric_dim;
	int col;
	int err;

	dev->chip_version = 10;
	dev->chip_width = 0;
	dev->chip_height = 0;

	err = tileusb_read_rshim(dev, 0, RSH_FABRIC_DIM, &fabric_dim);
	if (err) {
		ERROR("fill_chip_info: error %d reading fabric dim", err);
		return;
	}

	/* First set the height and width to the fabric height and width. */

	dev->chip_width = (fabric_dim >> RSH_FABRIC_DIM__DIM_X_SHIFT) &
		RSH_FABRIC_DIM__DIM_X_RMASK;
	dev->chip_height = (fabric_dim >> RSH_FABRIC_DIM__DIM_Y_SHIFT) &
		RSH_FABRIC_DIM__DIM_Y_RMASK;

	/*
	 * Now examine the tile disable bits to see if the actual tile grid
	 * is smaller than the fabric.  We know that the usable tiles form
	 * a rectangle which contains tile (0,0).
	 */
	for (col = 0; col < dev->chip_width; col++) {
		/*
		 * Walk through the columns.  If we hit a column that's
		 * totally disabled, set our width to that column's index,
		 * and stop walking, since we won't find any other tiles.
		 * Otherwise, clip our height based on the number of tiles
		 * in this column, and keep going.
		 */
		u64 thiscol_dis;
		int tiles;

		err = tileusb_read_rshim(dev, 0, RSH_TILE_COL_DISABLE +
					            8 * col, &thiscol_dis);
		if (err) {
			ERROR("fill_chip_info: error %d reading col disable",
			      err);
			return;
		}

		thiscol_dis |= ~(((u64) 1 << dev->chip_height) - 1);
		tiles = __builtin_ctz(thiscol_dis);

		if (tiles == 0) {
			dev->chip_width = col;
			break;
		}
		dev->chip_height = min(dev->chip_height, tiles);
	}
}

static int tileusb_info_release(struct inode *inode, struct file *file)
{
	/*
	 * Note that since this got turned into a seq file by
	 * tileusb_info_open(), our device pointer isn't in the usual spot
	 * (the file's private data); that's used by the seq file
	 * subsystem.
	 */
	struct tileusb *dev = ((struct seq_file *) file->private_data)->private;
	int retval;

	retval = single_release(inode, file);
	if (retval)
		return retval;

	mutex_lock(&tileusb_mutex);
	kref_put(&dev->kref, tileusb_delete);
	mutex_unlock(&tileusb_mutex);

	return 0;
}

static const struct file_operations tileusb_info_fops = {
	.owner		= THIS_MODULE,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= tileusb_info_release,
};

static int tileusb_info_open(struct file *file)
{
	struct tileusb *dev = file->private_data;
	int retval;

	/*
	 * If file->private_data is non-NULL, seq_open (called by
	 * single_open) thinks it's already a seq_file struct, and
	 * scribbles over it!  Very bad.
	 */
	file->private_data = NULL;

	file->f_op = &tileusb_info_fops;
	retval = single_open(file, tileusb_info_seq_show, dev);

	if (retval >= 0 && dev->chip_version < 0) {
		mutex_lock(&dev->mutex);
		fill_chip_info(dev);
		mutex_unlock(&dev->mutex);
	}

	return retval;
}

/* common tile-monitor FIFO/console FIFO file operations routines */

/*
 * Various forward declarations.
 */
static void tileusb_fifo_outproc(struct tileusb *dev);
static void tileusb_fifo_write_callback(URB_COMP_ARGS);
static void tileusb_fifo_read_callback(URB_COMP_ARGS);

/*
 * Process a control message.  Returns 1 if we did anything which should
 * cause output processing to run.
 */
static int tileusb_fifo_proc_ctl(struct tileusb *dev)
{
	int ctl_type = dev->read_buf[dev->read_buf_next];
	int retval = 0;

	TRACE("proc_ctl: got ctl type %d", ctl_type);
	switch (ctl_type) {
	case TMFIFO_CTL_INIT1:
		dev->proto_init1 = dev->read_buf[dev->read_buf_next + 1];

		if (dev->proto_init1 > TMFIFO_PROTO_VERS)
			ERROR("bad init1 version %d", dev->proto_init1);
		else
			dev->proto_in = dev->proto_init1;

		/* XXX Need to set up to send credit here */
		retval = 1;
		break;

	default:
		ERROR("bogus control packet %d", ctl_type);
		break;
	}

	dev->read_buf_next += TMFIFO_CTL_LEN - TMFIFO_PKT_HDR_LEN;

	return retval;
}

/*
 * Signal an error on the FIFO, and wake up anyone who might need to know
 * about it.
 */
static void tileusb_fifo_err(struct tileusb *dev, int err)
{
	int i;

	dev->tmfifo_error = err;
	wake_up_interruptible_all(&dev->write_completed);
	for (i = 0; i < TM_CHANNELS; i++) {
		wake_up_interruptible_all(&dev->read_fifo[i].operable);
		wake_up_interruptible_all(&dev->write_fifo[i].operable);
	}
}

/* Drain the read buffer, and start another read/interrupt if needed. */
static void tileusb_fifo_drain(struct tileusb *dev)
{
	int do_outproc = 0;

	while (dev->read_buf_next < dev->read_buf_bytes) {
		int copysize;

		/*
		 * If we're at the start of a packet, then extract the
		 * header, and update our count of bytes remaining in the
		 * packet.
		 */
		if (dev->read_buf_pkt_rem == 0) {
			TRACE("next hdr %d", dev->read_buf_next);

			dev->read_buf_pkt_rem =
				(dev->read_buf[dev->read_buf_next] |
				 (dev->read_buf[dev->read_buf_next + 1] <<
				  8)) & 0xFFF;
			/* XXX symbols */

			dev->read_buf_pkt_chan =
				(dev->read_buf[dev->read_buf_next + 1] >> 4) &
				3;

			if (dev->read_buf_pkt_chan < 0 ||
			   dev->read_buf_pkt_chan >= TM_CHANNELS) {
				ERROR("bad channel %d",
				      dev->read_buf_pkt_chan);
				dev->read_buf_pkt_chan = 0;
			}

			dev->read_buf_next += TMFIFO_PKT_HDR_LEN;
			TRACE("next body %d", dev->read_buf_next);

			TRACE("drain: hdr, nxt %d rem %d chn %d",
			      dev->read_buf_next, dev->read_buf_pkt_rem,
			      dev->read_buf_pkt_chan);

			if (dev->read_buf_pkt_rem == 0) {
				do_outproc |= tileusb_fifo_proc_ctl(dev);
				continue;
			}
		}

		if (dev->read_buf_pkt_chan == TM_CONS_CHAN &&
		    !(dev->spin_flags & TILEUSB_SFLG_CONS_OPEN)) {
			/*
			 * If data is coming in for a closed console
			 * channel, we want to just throw it away.
			 * Resetting the channel every time through this
			 * loop is a relatively cheap way to do that.  Note
			 * that this works because the read buffer is no
			 * larger than the read FIFO; thus, we know that if
			 * we reset it here, we will always be able to
			 * drain the read buffer of any console data, and
			 * will then launch another read.
			 */
			read_reset(dev, TM_CONS_CHAN);
		}

		copysize = min(dev->read_buf_pkt_rem,
			       dev->read_buf_bytes - dev->read_buf_next);
		copysize = min(copysize,
			       read_space_to_end(dev, dev->read_buf_pkt_chan));

		TRACE("drain: copysize %d, head %d, tail %d, "
		      "remaining %d", copysize,
		      dev->read_fifo[dev->read_buf_pkt_chan].head,
		      dev->read_fifo[dev->read_buf_pkt_chan].tail,
		      dev->read_buf_pkt_rem);

		if (copysize == 0) {
			/*
			 * We have data, but no space to put it in, so
			 * we're done.
			 */
			TRACE("drain: no more space in channel %d",
			      dev->read_buf_pkt_chan);
			break;
		}

		memcpy(read_space_ptr(dev, dev->read_buf_pkt_chan),
		       &dev->read_buf[dev->read_buf_next],
		       copysize);

		read_add_bytes(dev, dev->read_buf_pkt_chan, copysize);
		dev->read_buf_next += copysize;
		dev->read_buf_pkt_rem -= copysize;

		wake_up_interruptible_all(&dev->read_fifo[
				      dev->read_buf_pkt_chan].operable);
		TRACE("woke up readable chan %d", dev->read_buf_pkt_chan);

		if (dev->read_buf_pkt_rem <= 0)
			dev->read_buf_next = (dev->read_buf_next + 7) & -8;
	}

	/*
	 * We've processed all of the data we can, so now we decide if we
	 * need to launch another I/O.  If there's still data in the read
	 * buffer, or if we're already reading, don't launch any new
	 * operations.  If an interrupt just completed, and said there was
	 * data, or the last time we did a read we got some data, then do
	 * another read.  Otherwise, do an interrupt.
	 */
	if (dev->read_buf_next < dev->read_buf_bytes ||
	    (dev->spin_flags & TILEUSB_SFLG_READING)) {
		/* We're doing nothing. */
		TRACE("fifo_drain: no new read: %s",
		      (dev->read_buf_next < dev->read_buf_bytes) ?
		      "have data" : "already reading");
	} else if ((int) *dev->intr_buf || dev->read_buf_bytes) {
		/* We're doing a read. */

		int retval;
		struct urb *urb = dev->read_or_intr_urb;

		usb_fill_bulk_urb(urb, dev->udev,
				  usb_rcvbulkpipe(dev->udev,
						  dev->tm_fifo_in_ep),
				  dev->read_buf, READ_BUF_SIZE,
				  tileusb_fifo_read_callback,
				  dev);
		urb->transfer_dma = dev->read_buf_dma;
		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

		dev->spin_flags |= TILEUSB_SFLG_READING;
		dev->read_urb_is_intr = 0;
		dev->read_or_intr_retries = 0;

		/* Submit the urb. */
		retval = usb_submit_urb(urb, GFP_ATOMIC);
		if (retval) {
			dev->spin_flags &= ~TILEUSB_SFLG_READING;
			ERROR("fifo_drain: failed submitting read "
			      "urb, error %d", retval);
		}
		TRACE("fifo_read_callback: resubmitted read urb");
	} else {
		/* We're doing an interrupt. */

		int retval;
		struct urb *urb = dev->read_or_intr_urb;

		usb_fill_int_urb(urb, dev->udev,
				 usb_rcvintpipe(dev->udev, dev->tm_fifo_int_ep),
				 dev->intr_buf, sizeof(*dev->intr_buf),
				 tileusb_fifo_read_callback,
				 /* FIXME: is 6 a good interval value?  That's
				  * polling at 8000/(1 << 6) == 125 Hz. */
				 dev, 6);
		urb->transfer_dma = dev->intr_buf_dma;
		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

		dev->spin_flags |= TILEUSB_SFLG_READING;
		dev->read_urb_is_intr = 1;
		dev->read_or_intr_retries = 0;

		/* Submit the urb */
		retval = usb_submit_urb(urb, GFP_ATOMIC);
		if (retval) {
			dev->spin_flags &= ~TILEUSB_SFLG_READING;
			ERROR("fifo_read_callback: failed submitting "
			      "interrupt urb, error %d", retval);
		}
		TRACE("fifo_read_callback: resubmitted interrupt urb");
	}

	if (do_outproc)
		tileusb_fifo_outproc(dev);
}

static void tileusb_fifo_read_callback(URB_COMP_ARGS)
{
	struct tileusb *dev = urb->context;

	spin_lock(&dev->spinlock);

	TRACE("fifo_read_callback: %s urb completed, status %d, "
	      "actual length %d, intr buf %d",
	      dev->read_urb_is_intr ? "interrupt" : "read",
	      urb->status, urb->actual_length, (int) *dev->intr_buf);

	dev->spin_flags &= ~TILEUSB_SFLG_READING;

	if (urb->status == 0) {
		/*
		 * If a read completed, clear the number of bytes available
		 * from the last interrupt, and set up the new buffer for
		 * processing.  (If an interrupt completed, there's nothing
		 * to do, since the number of bytes available was already
		 * set by the I/O itself.)
		 */
		if (!dev->read_urb_is_intr) {
			*dev->intr_buf = 0;
			dev->read_buf_bytes = urb->actual_length;
			dev->read_buf_next = 0;
		}

		/* Process any data we got, and launch another I/O if needed. */
		tileusb_fifo_drain(dev);
	} else if (urb->status == -ENOENT) {
		/*
		 * The urb was explicitly cancelled.  The only time we
		 * currently do this is when we close the stream.  If we
		 * mark this as an error, tile-monitor --resume won't work,
		 * so we just want to do nothing.
		 */
	} else if (urb->status == -ECONNRESET ||
		   urb->status == -ESHUTDOWN) {
		/*
		 * The device went away.  We don't want to retry this, and
		 * we expect things to get better, probably after a device
		 * reset, but in the meantime, we should let upper layers
		 * know there was a problem.
		 */
		tileusb_fifo_err(dev, urb->status);
	} else if (dev->read_or_intr_retries < READ_RETRIES &&
		   urb->actual_length == 0 &&
		   (urb->status == -EPROTO || urb->status == -EILSEQ ||
		    urb->status == -EOVERFLOW)) {
		/*
		 * We got an error which could benefit from being retried.
		 * Just submit the same urb again.  Note that we don't
		 * handle partial reads; it's hard, and we haven't really
		 * seen them.
		 */
		int retval;
		dev->read_or_intr_retries++;
		retval = usb_submit_urb(urb, GFP_ATOMIC);
		if (retval) {
			ERROR("fifo_read_callback: resubmitted urb but "
			      "got error %d", retval);
			/*
			 * In this case, we won't try again; signal the
			 * error to upper layers.
			 */
			tileusb_fifo_err(dev, retval);
		} else {
			dev->spin_flags |= TILEUSB_SFLG_READING;
		}
	} else {
		/*
		 * We got some error we don't know how to handle, or we got
		 * too many errors.  Either way we don't retry any more,
		 * but we signal the error to upper layers.
		 */
		ERROR("fifo_read_callback: %s urb completed abnormally, "
		      "error %d", dev->read_urb_is_intr ? "interrupt" : "read",
		      urb->status);
		tileusb_fifo_err(dev, urb->status);
	}

	spin_unlock(&dev->spinlock);
}

static ssize_t tileusb_fifo_read(struct file *file, char *user_buffer,
				 size_t count, loff_t *ppos, int chan)
{
	struct tileusb *dev = file->private_data;
	size_t rd_cnt = 0;

	mutex_lock(&dev->mutex);

	while (count) {
		size_t readsize;
		int pass1_bytes;
		int pass2_bytes;
		TRACE("fifo_read, top of loop, remaining count %zd", count);

		/*
		 * We check this each time through the loop since the
		 * device could get disconnected while we're waiting for
		 * more data in the read FIFO.
		 */
		if (!dev->tm_interface) {
			mutex_unlock(&dev->mutex);
			TRACE("fifo_read: returning %zd/ENODEV", rd_cnt);
			return rd_cnt ? rd_cnt : -ENODEV;
		}

		if (dev->tmfifo_error) {
			mutex_unlock(&dev->mutex);
			TRACE("fifo_read: returning %zd/%d", rd_cnt,
			      dev->tmfifo_error);
			return rd_cnt ? rd_cnt : dev->tmfifo_error;
		}

		if (read_empty(dev, chan)) {
			TRACE("fifo_read: fifo empty");
			if (rd_cnt || (file->f_flags & O_NONBLOCK)) {
				mutex_unlock(&dev->mutex);
				TRACE("fifo_read: returning %zd/EAGAIN",
				      rd_cnt);
				return rd_cnt ? rd_cnt : -EAGAIN;
			} else {
				mutex_unlock(&dev->mutex);
				TRACE("fifo_read: waiting "
				      "for readable chan %d", chan);
				if (wait_event_interruptible(
						dev->read_fifo[chan].operable,
						    !read_empty(dev, chan))) {
					TRACE("fifo_read: returning "
					      "%zd/ERESTARTSYS", rd_cnt);
					return rd_cnt ? rd_cnt : -ERESTARTSYS;
				}
				mutex_lock(&dev->mutex);
				/*
				 * Since we dropped the mutex, we must make
				 * sure our interface is still there before
				 * we do anything else.
				 */
				continue;
			}
		}

		/*
		 * Figure out how many bytes we will transfer on this pass.
		 */
		spin_lock_irq(&dev->spinlock);

		readsize = min(count, (size_t) read_cnt(dev, chan));

		pass1_bytes = min(readsize,
				  (size_t) read_cnt_to_end(dev, chan));
		pass2_bytes = readsize - pass1_bytes;

		spin_unlock_irq(&dev->spinlock);

		TRACE("fifo_read: readsize %zd, head %d, tail %d", readsize,
		      dev->read_fifo[chan].head, dev->read_fifo[chan].tail);

		if (copy_to_user(user_buffer, read_data_ptr(dev, chan),
				 pass1_bytes) ||
			(pass2_bytes && copy_to_user(user_buffer + pass1_bytes,
						     dev->read_fifo[chan].data,
						     pass2_bytes))) {
			mutex_unlock(&dev->mutex);
			TRACE("fifo_read: returning %zd/EFAULT", rd_cnt);
			return rd_cnt ? rd_cnt : -EFAULT;
		}

		spin_lock_irq(&dev->spinlock);

		read_consume_bytes(dev, chan, readsize);

		/*
		 * We consumed some bytes, so let's see if we can process
		 * any more incoming data.  We don't do this if this is the
		 * console stream and the worker is still running.
		 */
		if (chan != TM_CONS_CHAN || !(dev->flags &
					      TILEUSB_FLG_CONS_WORK))
			tileusb_fifo_drain(dev);

		spin_unlock_irq(&dev->spinlock);

		count -= readsize;
		user_buffer += readsize;
		rd_cnt += readsize;
		TRACE("fifo_read: transferred %zd bytes this pass", readsize);
	}

	mutex_unlock(&dev->mutex);

	TRACE("fifo_read: returning %zd", rd_cnt);
	return rd_cnt;
}

static void tileusb_fifo_outproc(struct tileusb *dev)
{
	/* XXX should do double buffering eventually; when we do, may
	 * need to maintain # bytes written in write_buf between calls */
	int write_buf_next = 0;
	int write_avail = WRITE_BUF_SIZE - write_buf_next;
	int numchan;
	int chan_offset;

	/* If we're already writing, we have nowhere to put data. */
	if (dev->spin_flags & TILEUSB_SFLG_WRITING)
		return;

	/*
	 * If we have a pending version discovery protocol operation, do
	 * it.
	 */
	if (dev->proto_init0 && write_avail >= 8) {
		char *p = dev->write_buf + write_buf_next;
		memset(p, 0, TMFIFO_CTL_LEN);
		p[2] = TMFIFO_CTL_INIT0;
		p[3] = dev->proto_init0;
		dev->proto_init0 = 0;
		write_buf_next += TMFIFO_CTL_LEN;
		write_avail -= TMFIFO_CTL_LEN;
	} else if (dev->proto_init1 && write_avail >= 8) {
		char *p = dev->write_buf + write_buf_next;
		dev->proto_out = min(dev->proto_init1,
				     (u8) TMFIFO_PROTO_VERS);
		dev->proto_init1 = 0;
		memset(p, 0, TMFIFO_CTL_LEN);

		p[2] = TMFIFO_CTL_INIT2;
		p[3] = dev->proto_out;
		write_buf_next += TMFIFO_CTL_LEN;
		write_avail -= TMFIFO_CTL_LEN;
	}

	/*
	 * We can't send data on non-zero channels until our output protocol
	 * version is at least 1.
	 */
	numchan = (dev->proto_out > 0) ? TM_CHANNELS : 1;

	/* Walk through all the channels, sending as much data as possible. */
	for (chan_offset = 0; chan_offset < numchan; chan_offset++) {
		int chan = ((dev->first_outproc_chan) + chan_offset) % numchan;
		int writesize = min(write_avail - TMFIFO_PKT_HDR_LEN,
				(int) write_cnt(dev, chan));

		if (writesize > 0) {
			int pass1_bytes;
			int pass2_bytes;

			dev->write_buf[write_buf_next] = writesize & 0xFF;
			dev->write_buf[write_buf_next + 1] =
				(writesize >> 8) | (chan << 4);
			write_buf_next += TMFIFO_PKT_HDR_LEN;

			pass1_bytes = min(writesize, (int)
					  write_cnt_to_end(dev, chan));
			pass2_bytes = writesize - pass1_bytes;

			TRACE("fifo_outproc: chan %d, writesize %d, next %d, "
			      "head %d, tail %d",
			      chan, writesize, write_buf_next,
			      dev->write_fifo[chan].head,
			      dev->write_fifo[chan].tail);

			memcpy(&dev->write_buf[write_buf_next],
			       write_data_ptr(dev, chan), pass1_bytes);
			memcpy(&dev->write_buf[write_buf_next + pass1_bytes],
			       dev->write_fifo[chan].data, pass2_bytes);

			write_consume_bytes(dev, chan, writesize);
			write_buf_next += writesize;
			write_buf_next = (write_buf_next + 7) & -8;
			write_avail = WRITE_BUF_SIZE - write_buf_next;

			wake_up_interruptible_all(
				&dev->write_fifo[chan].operable);
			TRACE("woke up writable chan %d", chan);
		}
	}

	/*
	 * To prevent starvation, we start at a new channel each time this
	 * routine is called.
	 */
	dev->first_outproc_chan = (dev->first_outproc_chan + 1) % numchan;

	/* If we actually put anything in the buffer, send it. */

	if (write_buf_next) {
		int retval;

		/* Initialize the urb properly. */
		usb_fill_bulk_urb(dev->write_urb, dev->udev,
				  usb_sndbulkpipe(dev->udev,
						  dev->tm_fifo_out_ep),
				  dev->write_buf,
				  write_buf_next,
				  tileusb_fifo_write_callback,
				  dev);
		dev->write_urb->transfer_dma = dev->write_buf_dma;
		dev->write_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		dev->write_retries = 0;

		/* Send the data out the bulk port. */
		retval = usb_submit_urb(dev->write_urb, GFP_ATOMIC);
		if (retval) {
			dev->spin_flags &= ~TILEUSB_SFLG_WRITING;
			ERROR("fifo_write: failed submitting write "
			      "urb, error %d", retval);
			return;
			/* XXX will we recover from this?  depends upon
			 * whether data really got sent, or not.  can we do
			 * any better? */
		}
		TRACE("fifo_outproc: submitted urb, writing %d bytes",
		      write_buf_next);

		dev->spin_flags |= TILEUSB_SFLG_WRITING;
	}
}

static void tileusb_fifo_write_callback(URB_COMP_ARGS)
{
	struct tileusb *dev = urb->context;

	spin_lock(&dev->spinlock);

	TRACE("fifo_write_callback: urb completed, status %d, "
	      "actual length %d, intr buf %d",
	      urb->status, urb->actual_length, (int) *dev->intr_buf);

	dev->spin_flags &= ~TILEUSB_SFLG_WRITING;

	if (urb->status == 0) {
		/* A write completed. */
		wake_up_interruptible_all(&dev->write_completed);
		tileusb_fifo_outproc(dev);
	} else if (urb->status == -ENOENT) {
		/*
		 * The urb was explicitly cancelled.  The only time we
		 * currently do this is when we close the stream.  If we
		 * mark this as an error, tile-monitor --resume won't work,
		 * so we just want to do nothing.
		 */
	} else if (urb->status == -ECONNRESET ||
		   urb->status == -ESHUTDOWN) {
		/*
		 * The device went away.  We don't want to retry this, and
		 * we expect things to get better, probably after a device
		 * reset, but in the meantime, we should let upper layers
		 * know there was a problem.
		 */
		tileusb_fifo_err(dev, urb->status);
	} else if (dev->write_retries < WRITE_RETRIES &&
		   urb->actual_length == 0 &&
		   (urb->status == -EPROTO || urb->status == -EILSEQ ||
		    urb->status == -EOVERFLOW)) {
		/*
		 * We got an error which could benefit from being retried.
		 * Just submit the same urb again.  Note that we don't
		 * handle partial writes; it's hard, and we haven't really
		 * seen them.
		 */
		int retval;
		dev->write_retries++;
		retval = usb_submit_urb(urb, GFP_ATOMIC);
		if (retval) {
			ERROR("fifo_write_callback: resubmitted urb but "
			      "got error %d", retval);
			/*
			 * In this case, we won't try again; signal the
			 * error to upper layers.
			 */
			tileusb_fifo_err(dev, retval);
		} else {
			dev->spin_flags |= TILEUSB_SFLG_WRITING;
		}
	} else {
		/*
		 * We got some error we don't know how to handle, or we got
		 * too many errors.  Either way we don't retry any more,
		 * but we signal the error to upper layers.
		 */
		ERROR("fifo_write_callback: urb completed abnormally, "
		      "error %d", urb->status);
		tileusb_fifo_err(dev, urb->status);
	}

	spin_unlock(&dev->spinlock);
}

static ssize_t tileusb_fifo_write(struct file *file, const char *user_buffer,
				  size_t count, loff_t *ppos, int chan)
{
	struct tileusb *dev = file->private_data;
	size_t wr_cnt = 0;

	mutex_lock(&dev->mutex);

	while (count) {
		size_t writesize;
		int pass1_bytes;
		int pass2_bytes;
		TRACE("fifo_write, top of loop, remaining count %zd", count);

		/*
		 * We check this each time through the loop since the
		 * device could get disconnected while we're waiting for
		 * more space in the write buffer.
		 */
		if (!dev->tm_interface) {
			mutex_unlock(&dev->mutex);
			TRACE("fifo_write: returning %zd/ENODEV", wr_cnt);
			return wr_cnt ? wr_cnt : -ENODEV;
		}

		if (dev->tmfifo_error) {
			mutex_unlock(&dev->mutex);
			TRACE("fifo_write: returning %zd/%d", wr_cnt,
			      dev->tmfifo_error);
			return wr_cnt ? wr_cnt : dev->tmfifo_error;
		}

		if (write_full(dev, chan)) {
			TRACE("fifo_write: fifo full");
			if (file->f_flags & O_NONBLOCK) {
				mutex_unlock(&dev->mutex);
				TRACE("fifo_write: returning %zd/EAGAIN",
				      wr_cnt);
				return wr_cnt ? wr_cnt : -EAGAIN;
			} else {
				mutex_unlock(&dev->mutex);
				TRACE("fifo_write: waiting "
				      "for writable chan %d", chan);
				if (wait_event_interruptible(
					     dev->write_fifo[chan].operable,
						     !write_full(dev, chan))) {
					TRACE("fifo_write: returning "
					      "%zd/ERESTARTSYS", wr_cnt);
					return wr_cnt ? wr_cnt : -ERESTARTSYS;
				}
				mutex_lock(&dev->mutex);
				/*
				 * Since we dropped the mutex, we must make
				 * sure our interface is still there before
				 * we do anything else.
				 */
				continue;
			}
		}

		spin_lock_irq(&dev->spinlock);

		writesize = min(count, (size_t) write_space(dev, chan));
		pass1_bytes = min(writesize,
				  (size_t) write_space_to_end(dev, chan));
		pass2_bytes = writesize - pass1_bytes;

		spin_unlock_irq(&dev->spinlock);

		TRACE("fifo_write: writesize %zd, head %d, tail %d",
		      writesize, dev->write_fifo[chan].head,
		      dev->write_fifo[chan].tail);

		if (copy_from_user(write_space_ptr(dev, chan), user_buffer,
				   pass1_bytes) ||
		    (pass2_bytes && copy_from_user(dev->write_fifo[chan].data,
						   user_buffer + pass1_bytes,
						   pass2_bytes))) {
			mutex_unlock(&dev->mutex);
			TRACE("fifo_write: returning %zd/EFAULT", wr_cnt);
			return wr_cnt ? wr_cnt : -EFAULT;
		}

		spin_lock_irq(&dev->spinlock);

		write_add_bytes(dev, chan, writesize);

		/*
		 * We have some new bytes, let's see if we can write any.
		 * We don't do this if this is the console stream and the
		 * worker is still running.
		 */
		if (chan != TM_CONS_CHAN || !(dev->flags &
					      TILEUSB_FLG_CONS_WORK))
			tileusb_fifo_outproc(dev);

		spin_unlock_irq(&dev->spinlock);

		count -= writesize;
		user_buffer += writesize;
		wr_cnt += writesize;
		TRACE("fifo_write: transferred %zd bytes this pass",
		      writesize);
	}

	mutex_unlock(&dev->mutex);

	TRACE("fifo_write: returning %zd", wr_cnt);
	return wr_cnt;
}

static int tileusb_fifo_fsync(FSYNC_ARGS, int chan)
{
	struct tileusb *dev = file->private_data;

	mutex_lock(&dev->mutex);

	/*
	 * To ensure that all of our data has actually made it to the
	 * device, we first wait until the channel is empty, then we wait
	 * until there is no outstanding write urb.
	 */
	while (!write_empty(dev, chan))
		if (wait_event_interruptible(dev->write_fifo[chan].operable,
					     write_empty(dev, chan))) {
			mutex_unlock(&dev->mutex);
			return -ERESTARTSYS;
		}

	while (dev->spin_flags & TILEUSB_SFLG_WRITING)
		if (wait_event_interruptible(dev->write_completed,
					     !(dev->spin_flags &
					       TILEUSB_SFLG_WRITING))) {
			mutex_unlock(&dev->mutex);
			return -ERESTARTSYS;
		}

	mutex_unlock(&dev->mutex);

	return 0;
}

static unsigned int tileusb_fifo_poll(struct file *file, poll_table *wait,
				      int chan)
{
	struct tileusb *dev = file->private_data;
	unsigned int retval = 0;

	mutex_lock(&dev->mutex);

	poll_wait(file, &dev->read_fifo[chan].operable, wait);
	poll_wait(file, &dev->write_fifo[chan].operable, wait);

	spin_lock_irq(&dev->spinlock);

	if (!read_empty(dev, chan))
		retval |= POLLIN | POLLRDNORM;
	if (!write_full(dev, chan))
		retval |= POLLOUT | POLLWRNORM;
	/*
	 * We don't report POLLERR on the console so that it doesn't get
	 * automatically disconnected when it fails, and so that you can
	 * connect to it in the error state before rebooting the target.
	 * This is inconsistent, but being consistent turns out to be very
	 * annoying.  If someone tries to actually type on it, they'll
	 * get an error.
	 */
	if (dev->tmfifo_error && chan != TM_CONS_CHAN)
		retval |= POLLERR;
	spin_unlock_irq(&dev->spinlock);

	mutex_unlock(&dev->mutex);

        TRACE("poll chan %d file %p returns 0x%x", chan, file, retval);

	return retval;
}


static int tileusb_fifo_release(struct inode *inode, struct file *file,
				int my_open_flag)
{
	struct tileusb *dev = file->private_data;

	mutex_lock(&dev->mutex);

	if (my_open_flag & TILEUSB_FLG_CONS_OPEN) {
		u64 tmp;
		int retval;

		/*
		 * If we aren't the last console file, nothing to do but
		 * fix the reference count.
		 */
		dev->console_opens--;
		if (dev->console_opens) {
			mutex_unlock(&dev->mutex);
			return 0;
		}

		if (dev->rshim_interface) {
			/*
			 * Clear the startup wait-for-rshim-console delay,
			 * in case this machine will be rebooting from
			 * SROM.
			 */
			retval = tileusb_read_rshim(dev, 0,
						    RSH_BREADCRUMB1, &tmp);
			if (!retval) {
				tmp &= ~(u64) CONS_RSHIM_BC1_DELAY_MASK;
				retval = tileusb_write_rshim(dev, 0,
							     RSH_BREADCRUMB1,
							     tmp);
			}
			if (retval)
				ERROR("couldn't modify rshim breadcrumb "
				      "1, err %d", retval);

			/*
			 * Turn off the TM FIFO console.
			 */
			dev->console_flags &= ~CONS_RSHIM_H2T_USE_TMF_CON_MASK;
			retval = tileusb_read_rshim(dev, UART_CHANNEL_PORT1,
						    UART_SCRATCHPAD, &tmp);
			if (!retval) {
				tmp &= ~(u64) CONS_RSHIM_H2T_USE_TMF_CON_MASK;
				retval = tileusb_write_rshim(dev,
							     UART_CHANNEL_PORT1,
							     UART_SCRATCHPAD,
							     tmp);
			}
			if (retval)
				ERROR("couldn't modify uart 1 "
				      "scratchpad, err %d", retval);
		}

		/*
		 * We've told the host to stop using the TM FIFO console,
		 * but there may be a lag before it does.  Unless we
		 * continue to read data from the console stream, the host
		 * may spin forever waiting for the console to be drained
		 * and not realize that it's time to stop using it.
		 * Clearing the CONS_OPEN spin flag will discard any future
		 * incoming console data, but if our input buffers are full
		 * now, we might not be even reading from the hardware
		 * FIFO.  To avoid problems, clear the buffers and call the
		 * drainer so that it knows there's space.
		 */
		spin_lock_irq(&dev->spinlock);

		dev->spin_flags &= ~TILEUSB_SFLG_CONS_OPEN;

		read_reset(dev, TM_CONS_CHAN);
		write_reset(dev, TM_CONS_CHAN);

		if (dev->tm_interface)
			tileusb_fifo_drain(dev);

		spin_unlock_irq(&dev->spinlock);
	}

	dev->flags &= ~my_open_flag;

	if (!(dev->flags & (TILEUSB_FLG_TM_OPEN | TILEUSB_FLG_CONS_OPEN))) {
		usb_kill_urb(dev->read_or_intr_urb);

		spin_lock_irq(&dev->spinlock);
		dev->spin_flags &= ~TILEUSB_SFLG_READING;
		spin_unlock_irq(&dev->spinlock);
	}

	mutex_unlock(&dev->mutex);

	mutex_lock(&tileusb_mutex);
	kref_put(&dev->kref, tileusb_delete);
	mutex_unlock(&tileusb_mutex);

	return 0;
}

/* tile-monitor FIFO file operations routines */

static ssize_t tileusb_tmfifo_read(struct file *file, char *user_buffer,
				   size_t count, loff_t *ppos)
{
	return tileusb_fifo_read(file, user_buffer, count, ppos, TM_FIFO_CHAN);
}

static ssize_t tileusb_tmfifo_write(struct file *file, const char *user_buffer,
				    size_t count, loff_t *ppos)
{
	return tileusb_fifo_write(file, user_buffer, count, ppos, TM_FIFO_CHAN);
}

static int tileusb_tmfifo_fsync(FSYNC_ARGS)
{
	return tileusb_fifo_fsync(FSYNC_CALL, TM_FIFO_CHAN);
}

static unsigned int tileusb_tmfifo_poll(struct file *file, poll_table *wait)
{
	return tileusb_fifo_poll(file, wait, TM_FIFO_CHAN);
}

static int tileusb_tmfifo_release(struct inode *inode, struct file *file)
{
	return tileusb_fifo_release(inode, file, TILEUSB_FLG_TM_OPEN);
}

static const struct file_operations tileusb_tmfifo_fops = {
	.owner =	THIS_MODULE,
	.read =		tileusb_tmfifo_read,
	.write =	tileusb_tmfifo_write,
	.fsync =	tileusb_tmfifo_fsync,
	.poll =		tileusb_tmfifo_poll,
	.release =	tileusb_tmfifo_release,
};

static int tileusb_tmfifo_open(struct file *file)
{
	struct tileusb *dev = file->private_data;
	u64 pg_ctl;
	int retval;

	file->f_op = &tileusb_tmfifo_fops;

	mutex_lock(&dev->mutex);

	if (dev->flags & TILEUSB_FLG_TM_OPEN) {
		TRACE("tmfifo_open: file already open");
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	/*
	 * We can't write to the TM FIFO until the boot FIFO is no
	 * longer in use.  This is signified by automatic send mode
	 * being turned off in the packet generator control
	 * register.
	 */
	retval = tileusb_read_rshim(dev, 0, RSH_PG_CTL, &pg_ctl);
	if (retval) {
		ERROR("tmfifo_open: error %d reading pg_ctl", retval);
		mutex_unlock(&dev->mutex);
		return retval;
	}
	if (pg_ctl & RSH_PG_CTL__AUTO_SEND_MASK) {
		INFO("tmfifo_open: can't open, tile side not ready");
		mutex_unlock(&dev->mutex);
		return -ENOSPC;
	}

	dev->flags |= TILEUSB_FLG_TM_OPEN;

	spin_lock_irq(&dev->spinlock);

	/* Call the drainer to do an initial read, if needed. */
	tileusb_fifo_drain(dev);

	spin_unlock_irq(&dev->spinlock);

	mutex_unlock(&dev->mutex);

	return 0;
}

/* Console file operations routines */

/*
 * See if we can handle any more console input.  Returns the number of
 * jiffies to delay the worker thread.
 */
static int tileusb_console_check_input(struct tileusb *dev)
{
	int retval;
	u64 uart0_sp;
	u64 rptr;
	int more_data = 0;
	int nbytes;
	int space;
	int pass1_bytes;
	int pass2_bytes;
	u64 semval;
	unsigned char buf[1 + (dev->console_in_mask >>
			       CONS_RSHIM_T2H_IN_RPTR_SHIFT)], *p;

	/*
	 * See whether we actually have any data to send, and if so, copy
	 * it out of the write FIFO.  Note that we do the copy before we're
	 * sure that there is actually space in the rshim scratch buffer;
	 * this is almost always true, and assuming it is saves us from
	 * grabbing the spinlock three times.
	 */
	spin_lock_irq(&dev->spinlock);

	nbytes = write_cnt(dev, TM_CONS_CHAN);

	if (!nbytes) {
		spin_unlock_irq(&dev->spinlock);
		return HZ / 2;
	}

	TRACE("have %d bytes for input", nbytes);

	pass1_bytes = min(nbytes, (int) write_cnt_to_end(dev, TM_CONS_CHAN));
	pass2_bytes = nbytes - pass1_bytes;

	memcpy(buf, write_data_ptr(dev, TM_CONS_CHAN), pass1_bytes);
	memcpy(&buf[pass1_bytes], dev->write_fifo[TM_CONS_CHAN].data,
	       pass2_bytes);

	spin_unlock_irq(&dev->spinlock);

	/*
	 * See how much space we have for data, and clip the number of
	 * bytes to transfer.
	 */
	retval = tileusb_read_rshim(dev, UART_CHANNEL, UART_SCRATCHPAD,
				    &uart0_sp);
	if (retval) {
		ERROR("couldn't read UART scratchpad, err %d", retval);
		return HZ;
	}

	rptr = uart0_sp & dev->console_in_mask;

	space = (dev->console_in_mask >> CONS_RSHIM_T2H_IN_RPTR_SHIFT) -
		(((dev->console_in_wptr - rptr) & dev->console_in_mask) >>
		 CONS_RSHIM_T2H_IN_RPTR_SHIFT);

	if (space < nbytes) {
		nbytes = space;
		more_data = 1;
	}

	if (!nbytes)
		return HZ / 10;

	TRACE("have %d bytes for input, with space", nbytes);

	/*
	 * Get the semaphore so it's safe to write the scratch
	 * buffer index register.  If we can't get it, we come back
	 * on the next tick and try again.
	 */
	retval = tileusb_read_rshim(dev, 0, RSH_SEMAPHORE0, &semval);
	if (retval) {
		ERROR("couldn't read rshim semaphore, err %d", retval);
		return HZ;
	}

	if (semval != 0) {
		/* Retry in 1 clock tick */
		return 1;
	}

	/*
	 * Copy the data into the rshim scratch buffer.  Each
	 * pass through this loop we read/process/write one word.
	 */
	p = buf;
	while (nbytes > 0) {
		u64 word;
		int i, sbase, slen;

		retval = tileusb_write_rshim(dev, 0,
					     RSH_SCRATCH_BUF_CTL,
					     dev->console_in_buf_base +
					     (dev->console_in_wptr >>
					      (CONS_RSHIM_T2H_IN_RPTR_SHIFT +
					       3)));
		if (retval) {
			ERROR("couldn't write rshim scratch "
			      "buf ctl, err %d", retval);
			return HZ;
		}

		retval = tileusb_read_rshim(dev, 0,
					    RSH_SCRATCH_BUF_DAT,
					    &word);
		if (retval) {
			ERROR("couldn't read rshim scratch "
			      "buf, err %d", retval);
			return HZ;
		}

		sbase = (dev->console_in_wptr >>
			 CONS_RSHIM_T2H_IN_RPTR_SHIFT) & 7;
		slen = min(8 - sbase, nbytes);

		for (i = sbase; i < sbase + slen; i++)
			word = (word & ~((u64)0xFF << (8 * i))) |
				((u64)*p++ << (8 * i));

		retval = tileusb_write_rshim(dev, 0,
					     RSH_SCRATCH_BUF_CTL,
					     dev->console_in_buf_base +
					     (dev->console_in_wptr >>
					      (CONS_RSHIM_T2H_IN_RPTR_SHIFT +
					       3)));
		if (retval) {
			ERROR("couldn't write rshim scratch "
			      "buf ctl, err %d", retval);
			return HZ;
		}

		retval = tileusb_write_rshim(dev, 0,
					    RSH_SCRATCH_BUF_DAT,
					    word);
		if (retval) {
			ERROR("couldn't write rshim scratch "
			      "buf, err %d", retval);
			return HZ;
		}

		nbytes -= slen;
		dev->console_in_wptr += slen << CONS_RSHIM_T2H_IN_RPTR_SHIFT;
		dev->console_in_wptr &= dev->console_in_mask;
	}

	/* Consume the data from the console write FIFO. */

	spin_lock_irq(&dev->spinlock);

	write_consume_bytes(dev, TM_CONS_CHAN, nbytes);

	spin_unlock_irq(&dev->spinlock);

	wake_up_interruptible_all(&dev->write_fifo[TM_CONS_CHAN].operable);
	TRACE("woke up writable chan %d", TM_CONS_CHAN);

	/* Update the read pointer and release the semaphore. */
	retval = tileusb_write_rshim(dev,
				     UART_CHANNEL_PORT1,
				     UART_SCRATCHPAD,
				     dev->console_in_wptr |
				     dev->console_out_rptr |
				     dev->console_flags);
	if (retval) {
		ERROR("couldn't write UART 1 scratchpad, err %d",
		      retval);
		return HZ;
	}

	retval = tileusb_write_rshim(dev, 0, RSH_SEMAPHORE0, 0);
	if (retval) {
		ERROR("couldn't write rshim semaphore, err %d",
		      retval);
		return HZ;
	}

	return (more_data) ? HZ / 10 : HZ / 2;
}

/*
 * See if we can handle any more console output.  Returns the number of
 * jiffies to delay the worker thread.
 */
static int tileusb_console_check_output(struct tileusb *dev)
{
	int retval;
	u64 uart0_sp;
	u64 wptr;
	int read_data = 0;

	/*
	 * We're using the early console.
	 */
	retval = tileusb_read_rshim(dev, UART_CHANNEL, UART_SCRATCHPAD,
				    &uart0_sp);
	if (retval) {
		ERROR("couldn't read UART scratchpad, err %d", retval);
		return HZ;
	}

	wptr = uart0_sp & dev->console_out_mask;

	/*
	 * If the read & write pointers are different, there's data to be
	 * read.
	 */
	if (dev->console_out_rptr != wptr) {
		u64 semval;
		int nbytes;
		int pass1_bytes;
		int pass2_bytes;
		unsigned char buf[1 + dev->console_out_mask], *p;

		/* Figure out how many bytes are available. */
		nbytes = (wptr - dev->console_out_rptr) &
			 dev->console_out_mask;

		spin_lock_irq(&dev->spinlock);

		nbytes = min(nbytes, (int) read_space(dev, TM_CONS_CHAN));
		pass1_bytes = min(nbytes,
				  (int) read_space_to_end(dev, TM_CONS_CHAN));

		spin_unlock_irq(&dev->spinlock);

		pass2_bytes = nbytes - pass1_bytes;

		TRACE("cons_check_output: nbytes %d pass1 %d pass2 %d",
		      nbytes, pass1_bytes, pass2_bytes);

		if (!nbytes)
			return HZ / 2;

		read_data = 1;

		/*
		 * Get the semaphore so it's safe to write the scratch
		 * buffer index register.  If we can't get it, we come back
		 * on the next tick and try again.
		 */
		retval = tileusb_read_rshim(dev, 0, RSH_SEMAPHORE0, &semval);
		if (retval) {
			ERROR("couldn't read rshim semaphore, err %d", retval);
			return HZ;
		}

		if (semval != 0) {
			/* Retry in 1 clock tick */
			return 1;
		}

		/*
		 * Copy the data out of the rshim scratch buffer.  Each
		 * pass through this loop we read and process one word.
		 */
		p = buf;
		while (nbytes > 0) {
			u64 word;
			int i, sbase, slen;

			retval = tileusb_write_rshim(dev, 0,
						     RSH_SCRATCH_BUF_CTL,
						     dev->console_out_rptr >>
						     3);
			if (retval) {
				ERROR("couldn't write rshim scratch "
				      "buf ctl, err %d", retval);
				return HZ;
			}

			retval = tileusb_read_rshim(dev, 0,
						    RSH_SCRATCH_BUF_DAT,
						    &word);
			if (retval) {
				ERROR("couldn't read rshim scratch "
				      "buf, err %d", retval);
				return HZ;
			}

			sbase = dev->console_out_rptr & 7;
			slen = min(8 - sbase, nbytes);

			word >>= (8 * sbase);

			for (i = 0; i < slen; i++) {
				*p++ = word & 0xFF;
				word >>= 8;
			}

			nbytes -= slen;
			dev->console_out_rptr += slen;
			dev->console_out_rptr &= dev->console_out_mask;
		}

		/* Copy the data into the console read FIFO. */

		spin_lock_irq(&dev->spinlock);

		if (!(dev->spin_flags & TILEUSB_SFLG_CONS_OPEN)) {
			/*
			 * If console data is coming in but the console
			 * file is closed, we want to just throw it away.
			 * Resetting the channel here is a relatively cheap
			 * way to do that.
			 */
			read_reset(dev, TM_CONS_CHAN);
		}

		memcpy(read_space_ptr(dev, TM_CONS_CHAN), buf, pass1_bytes);
		memcpy(dev->read_fifo[TM_CONS_CHAN].data,
		       &buf[pass1_bytes], pass2_bytes);

		read_add_bytes(dev, TM_CONS_CHAN, pass1_bytes + pass2_bytes);

		spin_unlock_irq(&dev->spinlock);

		wake_up_interruptible_all(
			&dev->read_fifo[TM_CONS_CHAN].operable);
		TRACE("woke up readable chan %d", TM_CONS_CHAN);

		/* Update the read pointer and release the semaphore. */
		retval = tileusb_write_rshim(dev,
					     UART_CHANNEL_PORT1,
					     UART_SCRATCHPAD,
					     dev->console_in_wptr |
					     dev->console_out_rptr |
					     dev->console_flags);
		if (retval) {
			ERROR("couldn't write UART 1 scratchpad, err %d",
			      retval);
			return HZ;
		}

		retval = tileusb_write_rshim(dev, 0, RSH_SEMAPHORE0, 0);
		if (retval) {
			ERROR("couldn't write rshim semaphore, err %d",
			      retval);
			return HZ;
		}
	}

	/* If the tile side is done, say we are too. */
	if (uart0_sp & CONS_RSHIM_T2H_DONE_MASK) {
		retval = tileusb_write_rshim(dev,
					     UART_CHANNEL_PORT1,
					     UART_SCRATCHPAD,
					     dev->console_in_wptr |
					     dev->console_out_rptr |
					     dev->console_flags |
					     CONS_RSHIM_H2T_DONE_MASK);
		if (retval) {
			ERROR("couldn't write UART 1 scratchpad, err %d",
			      retval);
			return HZ;
		}

		dev->flags &= ~TILEUSB_FLG_CONS_EARLY;
	}

	return (read_data) ? HZ / 10 : HZ / 2;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
static void tileusb_console_work(void *arg)
{
	struct tileusb *dev = arg;
#else
void tileusb_console_work(struct work_struct *work)
{
	struct tileusb *dev = container_of((struct delayed_work *) work,
					   struct tileusb, work);
#endif
	int retry_interval = 5 * HZ;

	mutex_lock(&dev->mutex);

	/* If we're not supposed to be running, die and don't come back. */
	if (!(dev->flags & TILEUSB_FLG_CONS_WORK)) {
		mutex_unlock(&dev->mutex);
		TRACE("console_work exiting (no flag)");
		return;
	}

	/*
	 * If we're done with (or never started) the early console, see if
	 * we're done with the boot FIFO; if so, do a drain and exit
	 * forever, and if not, come back a bit later to check again.
	 */
	if (!(dev->flags & TILEUSB_FLG_CONS_EARLY)) {
		int boot_fifo_done;
		int err;
		u64 pg_ctl;

		err = tileusb_read_rshim(dev, 0, RSH_PG_CTL, &pg_ctl);
		if (err)
			ERROR("couldn't read pg_ctl, err %d", err);
		boot_fifo_done =
			(!err && (pg_ctl & RSH_PG_CTL__AUTO_SEND_MASK) == 0);

		if (boot_fifo_done) {
			spin_lock_irq(&dev->spinlock);

			/*
			 * Call the output processor to start the protocol
			 * negotiation.
			 */
			tileusb_fifo_outproc(dev);

			/* Call the drainer to do an initial read. */
			tileusb_fifo_drain(dev);

			spin_unlock_irq(&dev->spinlock);

			dev->flags &= ~TILEUSB_FLG_CONS_WORK;

			TRACE("console_work exiting (fifo done)");

			retry_interval = 0;
		}
	} else {
		retry_interval = min(tileusb_console_check_input(dev),
				     tileusb_console_check_output(dev));
	}

	if (retry_interval) {
		queue_delayed_work(tileusb_wq, &dev->work, retry_interval);

		TRACE("console_work resubmitted in %d jiffies, jif %lld",
		      retry_interval, get_jiffies_64());
	}

	mutex_unlock(&dev->mutex);
}

static ssize_t tileusb_console_read(struct file *file, char *user_buffer,
				    size_t count, loff_t *ppos)
{
	return tileusb_fifo_read(file, user_buffer, count, ppos, TM_CONS_CHAN);
}

static ssize_t tileusb_console_write(struct file *file, const char *user_buffer,
				     size_t count, loff_t *ppos)
{
	return tileusb_fifo_write(file, user_buffer, count, ppos, TM_CONS_CHAN);
}

static int tileusb_console_fsync(FSYNC_ARGS)
{
	return tileusb_fifo_fsync(FSYNC_CALL, TM_CONS_CHAN);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
static int tileusb_console_ioctl(struct inode *inode, struct file *file,
				 unsigned int cmd, unsigned long arg)
#else
static long tileusb_console_unlocked_ioctl(struct file *file, unsigned int
					   cmd, unsigned long arg)
#endif
{
	struct tileusb *dev = file->private_data;
	int retval = 0;

	mutex_lock(&dev->mutex);

	switch (cmd) {
	case TCGETS: {
#ifdef TCGETS2
		if (kernel_termios_to_user_termios_1(
#else
		if (kernel_termios_to_user_termios(
#endif
			(struct termios __user *)arg, &dev->cons_termios))
			retval = -EFAULT;
		break;
	}

	case TCSETS:
	case TCSETSW:
	case TCSETSF: {
#ifdef TCGETS2
		if (user_termios_to_kernel_termios_1(
#else
		if (user_termios_to_kernel_termios(
#endif
			&dev->cons_termios, (struct termios __user *)arg))
			retval = -EFAULT;
		break;
	}

	default:
		retval = -EINVAL;
		break;
	}

	mutex_unlock(&dev->mutex);

	return retval;
}

static unsigned int tileusb_console_poll(struct file *file, poll_table *wait)
{
	return tileusb_fifo_poll(file, wait, TM_CONS_CHAN);
}

static int tileusb_console_release(struct inode *inode, struct file *file)
{
	return tileusb_fifo_release(inode, file, TILEUSB_FLG_CONS_OPEN);
}

static const struct file_operations tileusb_console_fops = {
	.owner =	THIS_MODULE,
	.read =		tileusb_console_read,
	.write =	tileusb_console_write,
	.fsync =	tileusb_console_fsync,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
	.ioctl =	tileusb_console_ioctl,
#else
	.unlocked_ioctl = tileusb_console_unlocked_ioctl,
#endif
	.poll =		tileusb_console_poll,
	.release =	tileusb_console_release,
};

static int tileusb_console_open(struct file *file)
{
	struct tileusb *dev = file->private_data;
	u64 pg_ctl;
	u64 tmp;
	int retval;

	file->f_op = &tileusb_console_fops;

	mutex_lock(&dev->mutex);

	if (dev->flags & TILEUSB_FLG_CONS_OPEN) {
		/*
		 * The console is already open.  This is OK, but it means
		 * there's no work to do other than updating the reference
		 * count.
		 */
		dev->console_opens++;
		mutex_unlock(&dev->mutex);
		return 0;
	}

	dev->flags |= TILEUSB_FLG_CONS_OPEN;

	spin_lock_irq(&dev->spinlock);

	dev->spin_flags |= TILEUSB_SFLG_CONS_OPEN;

	spin_unlock_irq(&dev->spinlock);

	/*
	 * Set the startup wait-for-rshim-console delay, in case this
	 * machine will be rebooting from SROM.
	 */
	retval = tileusb_read_rshim(dev, 0, RSH_BREADCRUMB1, &tmp);
	if (!retval) {
		tmp &= ~(u64) CONS_RSHIM_BC1_DELAY_MASK;
		tmp |= CONS_BC1_DELAY;
		retval = tileusb_write_rshim(dev, 0, RSH_BREADCRUMB1, tmp);
	}
	if (retval)
		ERROR("couldn't modify rshim breadcrumb 1, err %d", retval);

	/*
	 * Turn on the TM FIFO console.
	 */
	dev->console_flags |= CONS_RSHIM_H2T_USE_TMF_CON_MASK;
	retval = tileusb_read_rshim(dev, UART_CHANNEL_PORT1,
				    UART_SCRATCHPAD, &tmp);
	if (!retval) {
		tmp |= CONS_RSHIM_H2T_USE_TMF_CON_MASK;
		retval = tileusb_write_rshim(dev, UART_CHANNEL_PORT1,
					     UART_SCRATCHPAD, tmp);
	}
	if (retval)
		ERROR("couldn't modify uart 1 scratchpad, err %d",
		      retval);

	/*
	 * If the console worker is already running, then it will take
	 * care of any transition between the early console and the
	 * regular one.  If not, though, we either start the worker,
	 * or we kick off the regular console, depending upon whether
	 * the boot FIFO is still busy or not.
	 */
	if (!(dev->flags & TILEUSB_FLG_CONS_WORK)) {
		int boot_fifo_done;
		int err;

		err = tileusb_read_rshim(dev, 0, RSH_PG_CTL, &pg_ctl);
		if (err)
			ERROR("couldn't read pg_ctl, err %d", err);
		boot_fifo_done =
			(!err && (pg_ctl & RSH_PG_CTL__AUTO_SEND_MASK) == 0);

		if (boot_fifo_done) {
			spin_lock_irq(&dev->spinlock);

			/*
			 * Call the output processor to start the protocol
			 * negotiation.
			 */
			tileusb_fifo_outproc(dev);

			/* Call the drainer to do an initial read. */
			tileusb_fifo_drain(dev);

			spin_unlock_irq(&dev->spinlock);
		} else {
			dev->flags |= TILEUSB_FLG_CONS_WORK;
			queue_delayed_work(tileusb_wq, &dev->work, 0);
			TRACE("cons_open: console_work submitted");
		}
	}

	dev->console_opens++;
	mutex_unlock(&dev->mutex);

	return 0;
}

/* Common file operations routines */

static int tileusb_open(struct inode *inode, struct file *file)
{
	struct tileusb *dev;
	int subminor = iminor(inode);
	int retval;

	mutex_lock(&tileusb_mutex);

	dev = tileusb_devs[subminor / TILEUSB_MINORS];

	if (!dev) {
		mutex_unlock(&tileusb_mutex);
		return -ENODEV;
	}

	/* Increment our usage count for the device. */
	kref_get(&dev->kref);

	mutex_unlock(&tileusb_mutex);

	/* Save our object in the file's private structure. */
	file->private_data = dev;

	/* Call the minor-specific open routine. */
	switch (subminor % TILEUSB_MINORS) {
	case TILEUSB_MINOR_RSHIM:
		retval = tileusb_rshim_open(file);
		break;
	case TILEUSB_MINOR_BOOT:
		retval = tileusb_boot_open(file);
		break;
	case TILEUSB_MINOR_LOCK:
		retval = tileusb_lock_open(file);
		break;
	case TILEUSB_MINOR_INFO:
		retval = tileusb_info_open(file);
		break;
	case TILEUSB_MINOR_CONSOLE:
		retval = tileusb_console_open(file);
		break;
	case TILEUSB_MINOR_TMFIFO:
		retval = tileusb_tmfifo_open(file);
		break;
	default:
		retval = -ENODEV;
	}

	/* If the minor open failed, drop the usage count. */
	if (retval < 0) {
		mutex_lock(&tileusb_mutex);
		kref_put(&dev->kref, tileusb_delete);
		mutex_unlock(&tileusb_mutex);
	}

	return retval;
}

static const struct file_operations tileusb_fops = {
	.owner =	THIS_MODULE,
	.open =		tileusb_open,
};

/* Probe routines */

/* These make the endpoint test code in tileusb_probe() a lot cleaner. */
#define is_in_ep(ep)   (((ep)->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == \
			USB_DIR_IN)
#define is_bulk_ep(ep) (((ep)->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == \
			USB_ENDPOINT_XFER_BULK)
#define is_int_ep(ep)  (((ep)->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == \
			USB_ENDPOINT_XFER_INT)
#define max_pkt(ep)    le16_to_cpu(ep->wMaxPacketSize)
#define ep_addr(ep)    (ep->bEndpointAddress)


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)

static ssize_t show_usb_path(struct class_device *cdev, char *buf)
{
	struct tileusb *dev = class_get_devdata(cdev);
	return snprintf(buf, PAGE_SIZE, "%s\n",
			tileusb_dev_names[dev->dev_index]);
}

static CLASS_DEVICE_ATTR(usb_path, S_IRUGO, show_usb_path, NULL);

#else

static ssize_t show_usb_path(struct device *cdev,
			     struct device_attribute *attr, char *buf)
{
	struct tileusb *dev = dev_get_drvdata(cdev);
	return snprintf(buf, PAGE_SIZE, "%s\n",
			tileusb_dev_names[dev->dev_index]);
}

static DEVICE_ATTR(usb_path, S_IRUGO, show_usb_path, NULL);

#endif


/* Terminal characteristics for newly created consoles. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
static struct termios init_console_termios = {
#else
static struct ktermios init_console_termios = {
#endif
	.c_iflag = INLCR | ICRNL,
	.c_oflag = OPOST | ONLCR,
	.c_cflag = B115200 | HUPCL | CLOCAL | CREAD | CS8,
	.c_lflag = ISIG | ICANON | ECHOE | ECHOK | ECHOCTL | ECHOKE | IEXTEN,
	.c_line = 0,
	.c_cc = INIT_C_CC,
};

static int tileusb_probe(struct usb_interface *interface,
			 const struct usb_device_id *id)
{
	char *usb_dev_name;
	int dev_name_len = 32;
	int dev_index = -1;
	struct tileusb *dev = NULL;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *ep;
	int i;
	int allocfail = 0;
	int retval = -ENOMEM;

	/*
	 * Get our device pathname.  The usb_make_path interface uselessly
	 * returns -1 if the output buffer is too small, instead of telling
	 * us how big it needs to be, so we just start with a reasonable
	 * size and double it until the name fits.
	 */
	while (1) {
		usb_dev_name = kmalloc(dev_name_len, GFP_KERNEL);
		if (!usb_dev_name)
			goto error;
		if (usb_make_path(interface_to_usbdev(interface), usb_dev_name,
				  dev_name_len) >= 0)
			break;
		kfree(usb_dev_name);
		dev_name_len *= 2;
	}

	TRACE_NODEV("probing %s", usb_dev_name);

	/*
	 * Now see if we've previously seen this device.  If so, we use the
	 * same device number, otherwise we pick the first available one.
	 */
	mutex_lock(&tileusb_mutex);

	/* First look for a match with a previous device name. */
	for (i = 0; i < tileusb_nr_devs; i++)
		if (tileusb_dev_names[i] &&
		    !strcmp(usb_dev_name, tileusb_dev_names[i])) {
			TRACE_NODEV("found match with previous at index "
				    "%d", i);
			dev_index = i;
			break;
		}

	/* Then look for a never-used slot. */
	if (dev_index < 0)
		for (i = 0; i < tileusb_nr_devs; i++)
			if (!tileusb_dev_names[i]) {
				TRACE_NODEV("found never-used slot at index "
					    "%d", i);
				dev_index = i;
				break;
			}

	/* Finally look for a currently-unused slot. */
	if (dev_index < 0)
		for (i = 0; i < tileusb_nr_devs; i++)
			if (!tileusb_devs[i]) {
				TRACE_NODEV("found unused slot at index %d", i);
				dev_index = i;
				break;
			}

	/* If none of that worked, we fail. */
	if (dev_index < 0) {
		ERROR_NODEV("couldn't find slot for new device %s",
			    usb_dev_name);
		retval = -ENODEV;
		mutex_unlock(&tileusb_mutex);
		goto error;
	}

	/*
	 * If we didn't find an existing device structure, then allocate a
	 * new one and initialize it; if we did, just bump its reference
	 * count.
	 */
	if (!tileusb_devs[dev_index]) {
		TRACE_NODEV("creating new tileusb structure");
		dev = kzalloc(sizeof(*dev), GFP_KERNEL);
		if (dev == NULL) {
			ERROR_NODEV("couldn't get memory for new device");
			mutex_unlock(&tileusb_mutex);
			goto error;
		}
		dev->dev_index = dev_index;
		dev->lockfile_pid = -1;
		kref_init(&dev->kref);
		mutex_init(&dev->mutex);
		spin_lock_init(&dev->spinlock);
		for (i = 0; i < TM_CHANNELS; i++) {
			init_waitqueue_head(&dev->read_fifo[i].operable);
			init_waitqueue_head(&dev->write_fifo[i].operable);
		}
		init_waitqueue_head(&dev->write_completed);
		init_completion(&dev->boot_write_complete);
		init_completion(&dev->booting_complete);
#ifdef TILEUSB_RESET_MUTEX
		init_completion(&dev->reset_complete);
#endif
		tileusb_dev_names[dev_index] = usb_dev_name;
		tileusb_devs[dev_index] = dev;
		memcpy(&dev->cons_termios, &init_console_termios,
		       sizeof(init_console_termios));
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
		INIT_WORK(&dev->work, tileusb_console_work, dev);
#else
		INIT_DELAYED_WORK(&dev->work, tileusb_console_work);
#endif

		for (i = 0; i < TILEUSB_MINORS; i++) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
			struct class_device *cl_dev;
#else
			struct device *cl_dev;
#endif
			int err;
			char devbuf[32];

			cdev_init(&dev->cdevs[i], &tileusb_fops);
			dev->cdevs[i].owner = THIS_MODULE;
			/*
			 * FIXME: is this addition really legal, or should
			 * we be using MKDEV?
			 */
			err = cdev_add(&dev->cdevs[i],
				       tileusb_dev_t +
				       dev->dev_index * TILEUSB_MINORS + i,
				       1);
			/*
			 * We complain if this fails, but we don't return
			 * an error; it really shouldn't happen, and it's
			 * hard to go un-do the rest of the adds.
			 */
			if (err)
				dev_err(&dev->udev->dev,
					"tileusb%d: couldn't add minor %d",
					dev_index, i);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
			cl_dev = class_device_create(
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
			cl_dev = device_create_drvdata(
#else
			cl_dev = device_create(
#endif
					       tileusb_class, NULL,
					       tileusb_dev_t +
					       dev->dev_index *
					       TILEUSB_MINORS + i, NULL,
					       "tileusb%d-%s",
					       dev->dev_index,
					       tileusb_minor_names[i]);
			if (IS_ERR(cl_dev))
				dev_err(&dev->udev->dev,
					"tileusb%d: couldn't add dev %s, "
					"err %ld", dev_index,
					format_dev_t(devbuf,
						tileusb_dev_t + dev_index *
							TILEUSB_MINORS + i),
					PTR_ERR(cl_dev));
			else
				TRACE("added class dev %s",
				      format_dev_t(devbuf, tileusb_dev_t +
						   dev->dev_index *
						   TILEUSB_MINORS + i));

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
			class_set_devdata(cl_dev, dev);
			if (class_device_create_file(cl_dev,
					     &class_device_attr_usb_path))
#else
			dev_set_drvdata(cl_dev, dev);
			if (device_create_file(cl_dev, &dev_attr_usb_path))
#endif
				ERROR("could not create usb_path file "
				      "in sysfs");
		}
	} else {
		dev = tileusb_devs[dev_index];
		TRACE("found previously allocated tileusb structure");
		kref_get(&dev->kref);
		kfree(usb_dev_name);
	}

	/*
	 * This has to be done on the first probe, whether or not we
	 * allocated a new tileusb structure, since it's always dropped
	 * on the second disconnect.
	 */
	if (!dev->rshim_interface && !dev->tm_interface)
		dev->udev = usb_get_dev(interface_to_usbdev(interface));

	/*
	 * It would seem more logical to allocate these above when we create
	 * a new tileusb structure, but we don't want to do it until we've
	 * upped the usb device reference count.
	 */
	for (i = 0; i < TM_CHANNELS; i++) {
		if (!dev->read_fifo[i].data)
			dev->read_fifo[i].data =
				kmalloc(READ_FIFO_SIZE, GFP_KERNEL);
		allocfail |= dev->read_fifo[i].data == 0;

		if (!dev->write_fifo[i].data)
			dev->write_fifo[i].data =
				kmalloc(WRITE_FIFO_SIZE, GFP_KERNEL);
		allocfail |= dev->write_fifo[i].data == 0;
	}

	if (!dev->read_buf)
		dev->read_buf = usb_alloc_coherent(dev->udev, READ_BUF_SIZE,
						   GFP_KERNEL,
						   &dev->read_buf_dma);
	allocfail |= dev->read_buf == 0;

	if (!dev->intr_buf)
		dev->intr_buf = usb_alloc_coherent(dev->udev,
						   sizeof(*dev->intr_buf),
						   GFP_KERNEL,
						   &dev->intr_buf_dma);
	allocfail |= dev->intr_buf == 0;

	if (!dev->write_buf)
		dev->write_buf = usb_alloc_coherent(dev->udev, WRITE_BUF_SIZE,
						    GFP_KERNEL,
						    &dev->write_buf_dma);
	allocfail |= dev->write_buf == 0;

	if (!dev->read_or_intr_urb)
		dev->read_or_intr_urb = usb_alloc_urb(0, GFP_KERNEL);
	allocfail |= dev->read_or_intr_urb == 0;

	if (!dev->write_urb)
		dev->write_urb = usb_alloc_urb(0, GFP_KERNEL);
	allocfail |= dev->write_urb == 0;

	for (i = 0; i < 2; i++) {
		if (!dev->boot_buf[i])
			dev->boot_buf[i] = kmalloc(BOOT_BUF_SIZE, GFP_KERNEL);
		allocfail |= dev->boot_buf[i] == 0;
	}

	if (allocfail) {
		ERROR("can't allocate buffers or urbs");
		mutex_unlock(&tileusb_mutex);
		goto error;
	}

	mutex_unlock(&tileusb_mutex);

	/*
	 * The device name is now in the table or already freed, so don't
	 * free it on error.
	 */
	usb_dev_name = NULL;

	iface_desc = interface->cur_altsetting;

	/* Make sure this is a vendor-specific interface class. */
	if (iface_desc->desc.bInterfaceClass != 0xFF)
		goto error;

	/* See which interface this is, then save the correct data. */

	mutex_lock(&dev->mutex);
	if (iface_desc->desc.bInterfaceSubClass == 0) {
		TRACE("found rshim interface");
		/*
		 * We only expect one endpoint here, just make sure its
		 * attributes match.
		 */
		if (iface_desc->desc.bNumEndpoints != 1) {
			ERROR("wrong number of endpoints for rshim interface");
			mutex_unlock(&dev->mutex);
			goto error;
		}
		ep = &iface_desc->endpoint[0].desc;

		/* We expect a bulk out endpoint. */
		if (!is_bulk_ep(ep) || is_in_ep(ep)) {
			mutex_unlock(&dev->mutex);
			goto error;
		}

		dev->rshim_interface = interface;
		dev->boot_fifo_ep = ep_addr(ep);

	} else if (iface_desc->desc.bInterfaceSubClass == 1) {
		TRACE("found tmfifo interface");
		/*
		 * We expect 3 endpoints here.  Since they're listed in
		 * random order we have to use their attributes to figure
		 * out which is which.
		 */
		if (iface_desc->desc.bNumEndpoints != 3) {
			ERROR("wrong number of endpoints for tm interface");
			mutex_unlock(&dev->mutex);
			goto error;
		}
		dev->tm_fifo_in_ep = 0;
		dev->tm_fifo_int_ep = 0;
		dev->tm_fifo_out_ep = 0;

		for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
			ep = &iface_desc->endpoint[i].desc;

			if (is_in_ep(ep)) {
				if (is_bulk_ep(ep)) {
					/* Bulk in endpoint. */
					dev->tm_fifo_in_ep = ep_addr(ep);
				} else if (is_int_ep(ep)) {
					/* Interrupt in endpoint. */
					dev->tm_fifo_int_ep = ep_addr(ep);
				}
			} else {
				if (is_bulk_ep(ep)) {
					/* Bulk out endpoint. */
					dev->tm_fifo_out_ep = ep_addr(ep);
				}
			}
		}

		if (!dev->tm_fifo_in_ep || !dev->tm_fifo_int_ep ||
		    !dev->tm_fifo_out_ep) {
			ERROR("could not find all required endpoints for "
			      "tm interface");
			mutex_unlock(&dev->mutex);
			goto error;
		}
		dev->tm_interface = interface;
	} else {
		mutex_unlock(&dev->mutex);
		goto error;
	}

	/* Save our data pointer in this interface device. */
	usb_set_intfdata(interface, dev);

	if (dev->rshim_interface && dev->tm_interface) {
		u64 bc1;

		/* Clear any previous errors. */
		dev->tmfifo_error = 0;

		/* Initialize the protocol state machine for the TM FIFO. */
		dev->proto_init0 = TMFIFO_PROTO_VERS;

		/*
		 * If the console device is open, enable the
		 * rshim early console.
		 */
		if (dev->flags & TILEUSB_FLG_CONS_OPEN) {
			int retval;

			dev->console_out_mask = 0x1FF;
			dev->console_in_mask = 0x1FF <<
				CONS_RSHIM_T2H_IN_RPTR_SHIFT;
			dev->console_in_buf_base =
				(dev->console_out_mask >> 3) + 1;
			dev->console_out_rptr = 0;
			dev->console_in_wptr = 0;

			dev->console_flags =
				CONS_RSHIM_H2T_OUT_ENABLE_MASK |
				(6 << CONS_RSHIM_H2T_OUT_BUFSIZ_SHIFT) |
				CONS_RSHIM_H2T_IN_ENABLE_MASK |
				(6 << CONS_RSHIM_H2T_IN_BUFSIZ_SHIFT) |
				CONS_RSHIM_H2T_USE_TMF_CON_MASK;

			retval = tileusb_write_rshim(dev,
						UART_CHANNEL_PORT1,
						UART_SCRATCHPAD,
						dev->console_flags);
			if (retval) {
				ERROR("error %d writing "
				      "UART1 scratchpad", retval);
			} else
				dev->flags |= TILEUSB_FLG_CONS_EARLY;
		}

		/*
		 * Set or clear the startup wait-for-rshim-console delay,
		 * depending on whether the console is open, in case this
		 * machine will be rebooting from SROM.  We also do this on
		 * open/close of the console device; you'd think that would
		 * be enough, but we do it here to handle the case where
		 * the console gets opened/closed while the chip is
		 * resetting.  Note that in the open case, we're probably
		 * going to be too late to cause the wait to happen on this
		 * first reset, but subsequent resets will see it.
		 */
		retval = tileusb_read_rshim(dev, 0, RSH_BREADCRUMB1, &bc1);
		if (!retval) {
			bc1 &= ~(u64) CONS_RSHIM_BC1_DELAY_MASK;
			if (dev->flags & TILEUSB_FLG_CONS_OPEN)
				bc1 |= CONS_BC1_DELAY;
			retval = tileusb_write_rshim(dev, 0,
						     RSH_BREADCRUMB1, bc1);
		}
		if (retval)
			ERROR("couldn't modify rshim breadcrumb 1, err %d",
			      retval);


		/*
		 * If someone might be waiting for the device to come up,
		 * tell them it's ready.
		 */
		if (dev->flags & TILEUSB_FLG_BOOTING) {
			dev->flags &= ~TILEUSB_FLG_BOOTING;
			/*
			 * Restore the boot mode register that we saved
			 * at boot time.
			 */
			retval = tileusb_write_rshim(dev, 0, RSH_BOOT_CONTROL,
						     dev->boot_control);

			if (retval)
				ERROR("couldn't restore boot_control, err %d",
				      retval);

			TRACE("signaling booting complete");
			complete_all(&dev->booting_complete);
#ifdef TILEUSB_RESET_MUTEX 
			complete_all(&dev->reset_complete);
#endif
		};

		/* If the console device is open, start the worker. */
		if (dev->flags & TILEUSB_FLG_CONS_OPEN) {
			dev->flags |= TILEUSB_FLG_CONS_WORK;
			TRACE("probe: console_work submitted");
			queue_delayed_work(tileusb_wq, &dev->work, 0);
		}

		/* Tell the user this device is now attached. */
		INFO("%s now attached", tileusb_dev_names[dev_index]);
	}

	/* Note that we need to sense the chip attributes. */
	dev->chip_version = -1;

	mutex_unlock(&dev->mutex);

	return 0;

error:
	if (dev) {
		int i;

		usb_free_urb(dev->read_or_intr_urb);
		dev->read_or_intr_urb = NULL;
		usb_free_urb(dev->write_urb);
		dev->write_urb = NULL;

		usb_free_coherent(dev->udev, READ_BUF_SIZE,
				  dev->read_buf, dev->read_buf_dma);
		dev->read_buf = NULL;

		usb_free_coherent(dev->udev, WRITE_BUF_SIZE,
				  dev->write_buf, dev->write_buf_dma);
		dev->write_buf = NULL;

		for (i = 0; i < TM_CHANNELS; i++) {
			kfree(dev->read_fifo[i].data);
			dev->read_fifo[i].data = NULL;
			kfree(dev->write_fifo[i].data);
			dev->write_fifo[i].data = NULL;
		}

		usb_free_coherent(dev->udev, sizeof(*dev->intr_buf),
				  dev->intr_buf, dev->intr_buf_dma);
		dev->intr_buf = NULL;

		for (i = 0; i < 2; i++) {
			kfree(dev->boot_buf[i]);
			dev->boot_buf[i] = NULL;
		}

		mutex_lock(&tileusb_mutex);
		kref_put(&dev->kref, tileusb_delete);
		mutex_unlock(&tileusb_mutex);
	}
	kfree(usb_dev_name);
	return retval;
}

static void tileusb_disconnect(struct usb_interface *interface)
{
	struct tileusb *dev;
	int flush_wq = 0;

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	/*
	 * Clear this interface so we don't unregister our devices next
	 * time.
	 */
	mutex_lock(&dev->mutex);

	if (dev->rshim_interface == interface) {
		dev->rshim_interface = NULL;
	} else {
		/*
		 * We have to get rid of any USB state, since it may be
		 * tied to the USB device which is going to vanish as soon
		 * as we get both disconnects.  We'll reallocate these
		 * on the next probe.
		 *
		 * Supposedly the code which called us already killed any
		 * outstanding URBs, but it doesn't hurt to be sure.
		 */
		int i;

		/*
		 * We must make sure the console worker isn't running
		 * before we free all these resources, and particularly
		 * before we decrement our usage count, below.  Most of the
		 * time, if it's even enabled, it'll be scheduled to run at
		 * some point in the future, and we can take care of that
		 * by asking that it be canceled.
		 *
		 * However, it's possible that it's already started
		 * running, but can't make progress because it's waiting
		 * for the device mutex, which we currently have.  We
		 * handle this case by clearing the bit that says it's
		 * enabled.  The worker tests this bit as soon as it gets
		 * the mutex, and if it's clear, it just returns without
		 * rescheduling itself.  Note that if we didn't
		 * successfully cancel it, we flush the work entry below,
		 * after we drop the mutex, to be sure it's done before we
		 * decrement the device usage count.
		 *
		 * XXX This might be racy; what if something else which
		 * would enable the worker runs after we drop the mutex
		 * but before the worker itself runs?
		 */
		flush_wq = !cancel_delayed_work(&dev->work);
		dev->flags &= ~TILEUSB_FLG_CONS_WORK;

		usb_kill_urb(dev->read_or_intr_urb);
		usb_free_urb(dev->read_or_intr_urb);
		dev->read_or_intr_urb = NULL;
		usb_kill_urb(dev->write_urb);
		usb_free_urb(dev->write_urb);
		dev->write_urb = NULL;

		usb_free_coherent(dev->udev, READ_BUF_SIZE,
				  dev->read_buf, dev->read_buf_dma);
		dev->read_buf = NULL;

		usb_free_coherent(dev->udev, sizeof(*dev->intr_buf),
				  dev->intr_buf, dev->intr_buf_dma);
		dev->intr_buf = NULL;

		usb_free_coherent(dev->udev, WRITE_BUF_SIZE,
				  dev->write_buf, dev->write_buf_dma);
		dev->write_buf = NULL;

		for (i = 0; i < 2; i++) {
			kfree(dev->boot_buf[i]);
			dev->boot_buf[i] = NULL;
		}

		for (i = 0; i < TM_CHANNELS; i++) {
			kfree(dev->read_fifo[i].data);
			dev->read_fifo[i].data = NULL;
			kfree(dev->write_fifo[i].data);
			dev->write_fifo[i].data = NULL;
		}

		dev->read_buf_bytes = 0;
		dev->read_buf_pkt_rem = 0;
		spin_lock_irq(&dev->spinlock);

		dev->spin_flags &= ~(TILEUSB_SFLG_WRITING |
				     TILEUSB_SFLG_READING);
		for (i = 0; i < TM_CHANNELS; i++) {
			read_reset(dev, i);
			write_reset(dev, i);
		}
		spin_unlock_irq(&dev->spinlock);

		dev->tm_interface = NULL;
	}

	if (!dev->rshim_interface && !dev->tm_interface) {
		usb_put_dev(dev->udev);
		dev->udev = NULL;
		INFO("now disconnected");
	} else {
		TRACE("partially disconnected");
	}

	mutex_unlock(&dev->mutex);

	/* This can't be done while we hold the mutex; see comments above. */
	if (flush_wq)
		flush_workqueue(tileusb_wq);

	/* decrement our usage count */
	mutex_lock(&tileusb_mutex);
	kref_put(&dev->kref, tileusb_delete);
	mutex_unlock(&tileusb_mutex);
}

static struct usb_driver tileusb_driver = {
	.name =		"tileusb",
	.probe =	tileusb_probe,
	.disconnect =	tileusb_disconnect,
	.id_table =	tileusb_table,
};

static int __init usb_tileusb_init(void)
{
	int result;
	int class_registered = 0;

#ifdef TILEUSB_RESET_MUTEX
	/*
	 * We use a long to track the devices whose mutexes we've taken, so
	 * we can't suport any more device than we have bits in a long.  If
	 * this ever becomes an issue we'll have to go to a more general
	 * bitmap.
	 */
	tileusb_nr_devs = max((size_t) tileusb_nr_devs,
			      (size_t) 8 * sizeof (unsigned long));
#endif

	tileusb_dev_names = kzalloc(tileusb_nr_devs *
				    sizeof(tileusb_dev_names[0]), GFP_KERNEL);
	tileusb_devs = kzalloc(tileusb_nr_devs * sizeof(tileusb_devs[0]),
			       GFP_KERNEL);

	if (!tileusb_dev_names || !tileusb_devs) {
		result = -ENOMEM;
		goto error;
	}

	TRACE_NODEV("allocated %d device/name pointers", tileusb_nr_devs);

	tileusb_wq = create_workqueue("tileusb");
	if (!tileusb_wq) {
		result = -ENOMEM;
		goto error;
	}

	TRACE_NODEV("allocated work queue");

	/* Register our device class. */
	tileusb_class = class_create(THIS_MODULE, "tileusb");
	if (IS_ERR(tileusb_class)) {
		result = PTR_ERR(tileusb_class);
		goto error;
	}
	class_registered = 1;

	TRACE_NODEV("registered class tileusb");

	/* Allocate major/minor numbers for our devices. */
	if (tileusb_major) {
		tileusb_dev_t = MKDEV(tileusb_major, 0);
		result = register_chrdev_region(tileusb_dev_t,
						tileusb_nr_devs *
						TILEUSB_MINORS,
						"tileusb");
	} else {
		result = alloc_chrdev_region(&tileusb_dev_t, 0,
					     tileusb_nr_devs * TILEUSB_MINORS,
					     "tileusb");
	}
	if (result < 0) {
		ERROR_NODEV("can't get major %d", MAJOR(tileusb_dev_t));
		goto error;
	}

	TRACE_NODEV("got major %d", MAJOR(tileusb_dev_t));

	/* Register this driver with the USB subsystem. */
	result = usb_register(&tileusb_driver);
	if (result) {
		ERROR_NODEV("usb_register failed, error number %d", result);
		goto error;
	}

	return 0;

error:
	if (tileusb_dev_t)
		unregister_chrdev_region(tileusb_dev_t,
					 tileusb_nr_devs * TILEUSB_MINORS);
	if (class_registered)
		class_destroy(tileusb_class);
	destroy_workqueue(tileusb_wq);
	kfree(tileusb_devs);
	kfree(tileusb_dev_names);

	return result;
}

static void __exit usb_tileusb_exit(void)
{
	int i;

	/* Free our major/minor numbers. */
	unregister_chrdev_region(tileusb_dev_t,
				 tileusb_nr_devs * TILEUSB_MINORS);

	/* Deregister this driver with the USB subsystem. */
	usb_deregister(&tileusb_driver);

	/* Destroy our device class. */
	class_destroy(tileusb_class);

	/* Destroy our work queue. */
	destroy_workqueue(tileusb_wq);

	/* Free any leftover device paths. */
	for (i = 0; i < tileusb_nr_devs; i++)
		kfree(tileusb_dev_names[i]);
	kfree(tileusb_dev_names);
	kfree(tileusb_devs);
}

module_init(usb_tileusb_init);
module_exit(usb_tileusb_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tilera Corporation");
MODULE_VERSION(TILEUSB_VERSION_STRING);
