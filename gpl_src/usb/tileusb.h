/*
 * tileusb.h - Tilera TILE-Gx host-side driver
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
 * USB Skeleton driver - 2.0
 *
 * Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * Some code was also lifted from the example drivers in "Linux Device
 * Drivers" by Alessandro Rubini and Jonathan Corbet, published by
 * O'Reilly & Associates.
 */

#include <linux/circ_buf.h>
#include <linux/usb.h>

#include "drv_tmfifo_proto.h"


/* Our USB vendor/product IDs. */
#define USB_TILERA_VENDOR_ID	0x22dc	 /* Tilera Corporation */
#define USB_TILEGX_PRODUCT_ID	0x0001	 /* Tilera TILE-Gx */

/* Our major number; 0 means to dynamically allocate it. */
#ifndef TILEUSB_MAJOR
#define TILEUSB_MAJOR		0
#endif

/* Default maximum number of devices controlled by this driver. */
#ifndef TILEUSB_NR_DEVS
#define TILEUSB_NR_DEVS		16
#endif

/* Number of minor devices per device (rshim, boot, etc.); must be at least
 * large enough to include all the devices in the next list. */
#define TILEUSB_MINORS		6

#define TILEUSB_MINOR_RSHIM	0    /* rshim control */
#define TILEUSB_MINOR_BOOT	1    /* Boot */
#define TILEUSB_MINOR_LOCK	2    /* Lock */
#define TILEUSB_MINOR_INFO	3    /* Info */
#define TILEUSB_MINOR_CONSOLE	4    /* Console */
#define TILEUSB_MINOR_TMFIFO	5    /* Tile-monitor FIFO */

/* Flag values. */
#define TILEUSB_FLG_BOOTING	0x01  /* Waiting for device to come back. */
#define TILEUSB_FLG_BOOT_OPEN	0x02  /* Boot device is open. */
#define TILEUSB_FLG_TM_OPEN	0x04  /* TM FIFO device is open. */
#define TILEUSB_FLG_CONS_OPEN	0x08  /* Console device is open. */
#define TILEUSB_FLG_CONS_WORK	0x10  /* Console worker thread running. */
#define TILEUSB_FLG_CONS_EARLY	0x20  /* Early console enabled. */
#define TILEUSB_FLG_BOOT_WRITE	0x40  /* A thread is in boot_write(). */

/* Spin flag values. */
#define TILEUSB_SFLG_READING	0x1  /* read_intr_urb is active. */
#define TILEUSB_SFLG_WRITING	0x2  /* write_urb is active. */
#define TILEUSB_SFLG_CONS_OPEN	0x4  /* console stream is open. */

/*
 * Buffer/FIFO sizes.  Note that the FIFO sizes must be powers of 2; also,
 * the read and write buffers must be no larger than the corresponding
 * FIFOs.
 */
#define READ_BUF_SIZE		512
#define WRITE_BUF_SIZE		512
#define READ_FIFO_SIZE		(4 * 1024)
#define WRITE_FIFO_SIZE		(4 * 1024)
#define BOOT_BUF_SIZE		(16 * 1024)

/* Number of retries for the tmfifo read/write path. */
#define READ_RETRIES		5
#define WRITE_RETRIES		5

/* Circular buffer macros. */

#define read_empty(dev, chan) \
	(CIRC_CNT((dev)->read_fifo[chan].head, \
		  (dev)->read_fifo[chan].tail, READ_FIFO_SIZE) == 0)
#define read_full(dev, chan) \
	(CIRC_SPACE((dev)->read_fifo[chan].head, \
		    (dev)->read_fifo[chan].tail, READ_FIFO_SIZE) == 0)
#define read_space(dev, chan) \
	CIRC_SPACE((dev)->read_fifo[chan].head, \
		   (dev)->read_fifo[chan].tail, READ_FIFO_SIZE)
#define read_cnt(dev, chan) \
	CIRC_CNT((dev)->read_fifo[chan].head, \
		 (dev)->read_fifo[chan].tail, READ_FIFO_SIZE)
#define read_cnt_to_end(dev, chan) \
	CIRC_CNT_TO_END((dev)->read_fifo[chan].head, \
			(dev)->read_fifo[chan].tail, READ_FIFO_SIZE)
#define read_data_ptr(dev, chan) \
	((dev)->read_fifo[chan].data + \
	 ((dev)->read_fifo[chan].tail & (READ_FIFO_SIZE - 1)))
#define read_consume_bytes(dev, chan, nbytes) \
	((dev)->read_fifo[chan].tail = \
		((dev)->read_fifo[chan].tail + (nbytes)) & \
		 (READ_FIFO_SIZE - 1))
#define read_space_to_end(dev, chan) \
	CIRC_SPACE_TO_END((dev)->read_fifo[chan].head, \
			  (dev)->read_fifo[chan].tail, READ_FIFO_SIZE)
#define read_space_offset(dev, chan) \
	((dev)->read_fifo[chan].head & (READ_FIFO_SIZE - 1))
#define read_space_ptr(dev, chan) \
	((dev)->read_fifo[chan].data + read_space_offset(dev, (chan)))
#define read_add_bytes(dev, chan, nbytes) \
	((dev)->read_fifo[chan].head = \
		((dev)->read_fifo[chan].head + (nbytes)) & \
		 (READ_FIFO_SIZE - 1))
#define read_reset(dev, chan) \
	((dev)->read_fifo[chan].head = (dev)->read_fifo[chan].tail = 0)

#define write_empty(dev, chan) \
	(CIRC_CNT((dev)->write_fifo[chan].head, \
		  (dev)->write_fifo[chan].tail, WRITE_FIFO_SIZE) == 0)
#define write_full(dev, chan) \
	(CIRC_SPACE((dev)->write_fifo[chan].head, \
		    (dev)->write_fifo[chan].tail, WRITE_FIFO_SIZE) == 0)
#define write_space(dev, chan) \
	CIRC_SPACE((dev)->write_fifo[chan].head, \
		   (dev)->write_fifo[chan].tail, WRITE_FIFO_SIZE)
#define write_cnt(dev, chan) \
	CIRC_CNT((dev)->write_fifo[chan].head, \
		 (dev)->write_fifo[chan].tail, WRITE_FIFO_SIZE)
#define write_cnt_to_end(dev, chan) \
	CIRC_CNT_TO_END((dev)->write_fifo[chan].head, \
			(dev)->write_fifo[chan].tail, WRITE_FIFO_SIZE)
#define write_data_offset(dev, chan) \
	((dev)->write_fifo[chan].tail & (WRITE_FIFO_SIZE - 1))
#define write_data_ptr(dev, chan) \
	((dev)->write_fifo[chan].data + write_data_offset(dev, (chan)))
#define write_consume_bytes(dev, chan, nbytes) \
	((dev)->write_fifo[chan].tail = \
		 ((dev)->write_fifo[chan].tail + (nbytes)) & \
		  (WRITE_FIFO_SIZE - 1))
#define write_space_to_end(dev, chan) \
	CIRC_SPACE_TO_END((dev)->write_fifo[chan].head, \
			  (dev)->write_fifo[chan].tail, WRITE_FIFO_SIZE)
#define write_space_ptr(dev, chan) \
	((dev)->write_fifo[chan].data + \
	 ((dev)->write_fifo[chan].head & (WRITE_FIFO_SIZE - 1)))
#define write_add_bytes(dev, chan, nbytes) \
	((dev)->write_fifo[chan].head = \
	 ((dev)->write_fifo[chan].head + (nbytes)) & \
	  (WRITE_FIFO_SIZE - 1))
#define write_reset(dev, chan) \
	((dev)->write_fifo[chan].head = (dev)->write_fifo[chan].tail = 0)

struct tileusb_fifo {
	unsigned char		*data;
	unsigned int		head;
	unsigned int		tail;
	wait_queue_head_t	operable;
};

/* Number of channels the driver supports. */
#define TM_CHANNELS 2

/* Tile-monitor FIFO channel. */
#define TM_FIFO_CHAN 0

/* Console channel. */
#define TM_CONS_CHAN 1

/* Structure to hold all of our device specific stuff. */
struct tileusb {
	/*
	 * The USB device for this device.  We bump its reference count
	 * when the first interface is probed, and drop the ref when the
	 * last interface is disconnected.
	 */
	struct usb_device	*udev;

	/* The USB interfaces for this device. */
	struct usb_interface	*rshim_interface;
	struct usb_interface	*tm_interface;

	/*
	 * This mutex is used to prevent the interface pointers and the
	 * usb_device pointer from disappearing while a driver entry point
	 * is using them.  It's held throughout a read or write operation
	 * (at least the parts of those operations which depend upon those
	 * pointers) and is also held whenever those pointers are modified.
	 * It also protects flags, lockfile_pid, and booting_complete.
	 */
	struct mutex		mutex;

	/* State flags. */
	int			flags;

	/* pid of the process which has the lock file open, or -1 if none. */
	pid_t			lockfile_pid;

	/* We'll signal completion on this when FLG_BOOTING is turned off. */
	struct completion	booting_complete;

	/*
	 * This spinlock is used to protect items which must be updated by
	 * URB completion handlers, since those can't sleep.  This includes
	 * the read and write buffer pointers, as well as spin_flags.
	 */
	spinlock_t		spinlock;

	/* State for our outstanding boot write. */
	struct urb *boot_urb;
	struct completion	boot_write_complete;

#ifdef TILEUSB_RESET_MUTEX
	/* Signaled when a device is disconnected. */
	struct completion	reset_complete;
#endif

	/* Buffers used for boot writes.  Allocated at startup. */
	char                    *boot_buf[2];

#ifdef TILEUSB_DEBUG
	/* Statistics on progress of boot stream. */

	/* Number of errors encountered. */
	int boot_write_badstat;
	/* Number of bytes reported sent by urb->actual_length. */
	u64 boot_write_actual;
	/* Number of bytes requested to be sent. */
	u64 boot_write_requested;
#endif

	/* More state flags. */
	int			spin_flags;

	/* Read buffer.  This is a USB DMA'able buffer. */
	unsigned char		*read_buf;
	dma_addr_t		read_buf_dma;

	/* Total bytes in the read buffer. */
	int			read_buf_bytes;
	/* Offset of next unread byte in the read buffer. */
	int			read_buf_next;
	/* Bytes left in the current packet, or 0 if no current packet. */
	int			read_buf_pkt_rem;
	/* Channel that the current packet is going to. */
	int			read_buf_pkt_chan;

	/* Read FIFOs. */
	struct tileusb_fifo	read_fifo[TM_CHANNELS];

	/* Interrupt data buffer.  This is a USB DMA'able buffer. */
	u64			*intr_buf;
	dma_addr_t		intr_buf_dma;

	/* Read/interrupt urb, retries, and mode. */
	struct urb		*read_or_intr_urb;
	int			read_or_intr_retries;
	int			read_urb_is_intr;

	/* Write FIFOs. */
	struct tileusb_fifo	write_fifo[TM_CHANNELS];

	/* Write buffer.  This is a USB DMA'able buffer. */
	unsigned char		*write_buf;
	dma_addr_t		write_buf_dma;

	/* Channel we should start at next time we do output. */
	int			first_outproc_chan;

	/* Write urb and retries. */
	struct urb		*write_urb;
	int			write_retries;

	/* First error encountered during read or write. */
	int			tmfifo_error;

	/*
	 * This wait queue supports fsync; it's woken up whenever an
	 * outstanding USB write URB is done.  This will need to be more
	 * complex if we start doing write double-buffering.
	 */
	wait_queue_head_t	write_completed;

	/* Current termios settings for the console. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
	struct termios		cons_termios;
#else
	struct ktermios		cons_termios;
#endif

	/* Work queue entry for the early console handler. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
	struct work_struct	work;
#else
	struct delayed_work	work;
#endif

	/* Various bits of state for the early console protocol. */
	u64			console_out_mask;
	u64			console_in_mask;
	u64			console_flags;
	u64			console_out_rptr;
	u64			console_in_wptr;
	u64			console_in_buf_base;

	/* Number of open console files. */
	long			console_opens;

	/*
	 * Our index in tileusb_devs, which is also the high bits of our
	 * minor number.
	 */
	int			dev_index;

	/* Character device structure for each device. */
	struct cdev		cdevs[TILEUSB_MINORS];

	/* The address of the boot FIFO endpoint. */
	u8			boot_fifo_ep;
	/* The address of the tile-monitor FIFO interrupt endpoint. */
	u8			tm_fifo_int_ep;
	/* The address of the tile-monitor FIFO input endpoint. */
	u8			tm_fifo_in_ep;
	/* The address of the tile-monitor FIFO output endpoint. */
	u8			tm_fifo_out_ep;

	/*
	 * Chip version.  If < 0, we haven't queried the chip yet for
	 * its version, width, or height.
	 */
	int			chip_version;
	/* Chip grid width. */
	int			chip_width;
	/* Chip grid height. */
	int			chip_height;

	/* Saved contents of the boot control register. */
	u64			boot_control;

	/* Protocol version with which to interpret incoming data. */
	u8			proto_in;

	/* Protocol version with which to format outgoing data. */
	u8			proto_out;

	/*
	 * Protocol version we want to use.  This is used as a flag to tell
	 * us that we need to send an INIT0, and is cleared after we've
	 * done so.
	 */
	u8			proto_init0;

	/*
	 * Protocol version received from tile in an INIT1 request.  Note
	 * that this is used as a flag to tell us that we need to send an
	 * INIT2, and is cleared after we've done so.
	 */
	u8			proto_init1;

	/*
	 * The reference count for this structure.  This is incremented by
	 * each open, and by the probe routine (thus, one reference for
	 * each of the two interfaces).  It's decremented on each release,
	 * and on each disconnect.
	 */
	struct kref		kref;
};

/*
 * Output macros.  ERROR, INFO, TRACE, and NOISE assume that the variable
 * dev points to a valid struct tileusb, which is used to supply the unit
 * number as part of the message ("tileusb0: message"), while ERROR_NODEV
 * and TRACE_NODEV make no such assumption ("tileusb: message").
 */

#define ERROR_NODEV(fmt, ...) \
	printk(KERN_ERR "tileusb: " fmt "\n", ## __VA_ARGS__)

#define ERROR(fmt, ...) \
	printk(KERN_ERR "tileusb%d: " fmt "\n", \
	       dev->dev_index , ## __VA_ARGS__)

#define INFO(fmt, ...) \
	printk(KERN_INFO "tileusb%d: " fmt "\n", \
	       dev->dev_index , ## __VA_ARGS__)

#ifdef TILEUSB_DEBUG

#define TRACE_NODEV(fmt, ...) \
	printk(KERN_DEBUG "tileusb: " fmt "\n" , ## __VA_ARGS__)

#define TRACE(fmt, ...) \
	printk(KERN_DEBUG "tileusb%d: " fmt "\n", \
	       dev->dev_index , ## __VA_ARGS__)
#else

#define TRACE_NODEV(fmt, ...) \
	do { } while (0)

#define TRACE(fmt, ...) \
	do { } while (0)

#endif

/*
 * NOISE is for things that are interesting in certain circumstances, but
 * which most of the time impede debugging by just filling up the kernel
 * log.  (e.g., status of each and every boot write call.)
 */
#ifdef TILEUSB_NOISE

#define NOISE(fmt, ...) \
	printk(KERN_DEBUG "tileusb%d: " fmt "\n", \
	       dev->dev_index , ## __VA_ARGS__)

#else

#define NOISE(fmt, ...) \
	do { } while (0)

#endif

/*
 * Various rshim registers.  Defining them here is not exactly clean, but
 * we don't have a good solution for incorporating the <arch/> headers in
 * host software.
 */

#define RSH_BOOT_CONTROL     0x0528
#define RSH_FABRIC_DIM       0x0110
#define RSH_FABRIC_DIM__DIM_X_RMASK 0xf
#define RSH_FABRIC_DIM__DIM_X_SHIFT 4
#define RSH_FABRIC_DIM__DIM_Y_RMASK 0xf
#define RSH_FABRIC_DIM__DIM_Y_SHIFT 0
#define RSH_PG_CTL 0x0400
#define RSH_PG_CTL__AUTO_SEND_MASK  0x1000
#define RSH_RESET_CONTROL    0x0500
#define RSH_RESET_CONTROL__RESET_CHIP_VAL_KEY 0xca710001
#define RSH_RESET_MASK       0x0508
#define RSH_BREADCRUMB1      0x0518
#define RSH_SCRATCH_BUF_CTL  0x0600
#define RSH_SCRATCH_BUF_DAT  0x0610
#define RSH_SEMAPHORE0       0x0028
#define RSH_TILE_COL_DISABLE 0x0b00
#define UART_CHANNEL              1
#define UART_CHANNEL_PORT1        2
#define UART_SCRATCHPAD      0x0020

/*
 * Rshim early console bit definitions.
 */

/*
 * Tile-to-host bits (UART 0 scratchpad).
 */
/** Output write pointer mask.  Note that this is the maximum size; the
 *  write pointer may be smaller if requested by the host. */
#define CONS_RSHIM_T2H_OUT_WPTR_MASK     0x3FF

/** Tile is done mask. */
#define CONS_RSHIM_T2H_DONE_MASK         0x400

/** Input read pointer mask.  Note that this is the maximum size; the read
 *  pointer may be smaller if requested by the host. */
#define CONS_RSHIM_T2H_IN_RPTR_MASK      0x1FF800

/** Input read pointer shift. */
#define CONS_RSHIM_T2H_IN_RPTR_SHIFT     11

/** Tile is done mask. */
#define CONS_RSHIM_T2H_DONE_MASK         0x400

/*
 * Host-to-tile bits (UART 1 scratchpad).
 */
/** Output enable mask. */
#define CONS_RSHIM_H2T_OUT_ENABLE_MASK   0x80000000

/** Output buffer size mask.  This is the log base 2 of the buffer
 *  size in words. */
#define CONS_RSHIM_H2T_OUT_BUFSIZ_MASK   0x70000000

/** Output buffer size shift. */
#define CONS_RSHIM_H2T_OUT_BUFSIZ_SHIFT  28

/** Input enable mask. */
#define CONS_RSHIM_H2T_IN_ENABLE_MASK    0x8000000

/** Input buffer size mask.  This is the log base 2 of the buffer
 *  size in words. */
#define CONS_RSHIM_H2T_IN_BUFSIZ_MASK    0x7000000

/** Input buffer size shift. */
#define CONS_RSHIM_H2T_IN_BUFSIZ_SHIFT   24

/** Tile should use TMFIFO console once early console is closed. */
#define CONS_RSHIM_H2T_USE_TMF_CON_MASK  0x800000

/** Output buffer read pointer mask.  Note that this is the maximum size;
 *  the read pointer may be smaller if requested by the host. */
#define CONS_RSHIM_H2T_OUT_RPTR_MASK     0x3FF

/** Input write pointer mask.  Note that this is the maximum size; the read
 *  pointer may be smaller if requested by the host. */
#define CONS_RSHIM_T2H_IN_WPTR_MASK      0x1FF800

/** Input write pointer shift. */
#define CONS_RSHIM_T2H_IN_WPTR_SHIFT     11

/** Host is done mask. */
#define CONS_RSHIM_H2T_DONE_MASK         0x400

/*
 * Host to tile bits (rshim breadcrumb 1).
 */
/** Seconds to wait for rshim early console before giving up and using the
 *  UART. */
#define CONS_RSHIM_BC1_DELAY_MASK         0x1F

/** Default delay. */
#define CONS_BC1_DELAY                   5
