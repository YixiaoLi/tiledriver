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
 */

/*
 * ** NOTE ** this file is commented using Doxygen standards rather than
 * kernel-doc standards since it is used to build Tilera documentation.
 */

/**
 * @file
 *
 * Defines the kernel-to-userspace API for performing PCI
 * communication between a host machine and a TILEGx processor and
 * between TILEGx processors.
 */
#ifndef _ASM_TILEGXPCI_H
#define _ASM_TILEGXPCI_H

#include <linux/ioctl.h>
#include <linux/types.h>

#if defined(TILEPCI_HOST)
/** Hard-code TRIO_DMA_DESC_WORD0__XSIZE_WIDTH for the host driver. */
#define TRIO_DMA_DESC_WORD0__XSIZE_WIDTH	14
#else
#include <arch/trio_shm.h>	/** Get TRIO_DMA_DESC_WORD0__XSIZE_WIDTH. */
#endif

/** A macro for defining the PCIe subsystem revision.
 *
 * @param Major Major revision number, new values signal incompatible changes.
 * @param Minor Minor revision number, new values should be compatible.
 */
#define PCIE_VERSION_DEF(Major, Minor)		((Major << 8) | (Minor))

/** Extract 'major' from a version number. */
#define PCIE_VERSION_MAJOR(x)			(((x) >> 8) & 0xFF)

/** Extract 'minor' from a version number. */
#define PCIE_VERSION_MINOR(x)			((x) & 0xFF)

/**
 * A macro for checking whether the host driver and PCIe subsystem
 * revision are compatible.  The macro guarantees that the major
 * numbers match.
 */
#define PCIE_VERSION_MATCH(x, y) \
	(PCIE_VERSION_MAJOR(x) == PCIE_VERSION_MAJOR(y))

/** The maximum size of packet represented by a single TRIO DMA command.
    16KB for Gx36 */
#define GXPCI_MAX_CMD_SIZE			(1 << \
	TRIO_DMA_DESC_WORD0__XSIZE_WIDTH)

/** Number of bits to represent the maximum size of a data command. */
#define GXPCI_MAX_CMD_SIZE_BITS			(1 + \
	TRIO_DMA_DESC_WORD0__XSIZE_WIDTH)

#ifndef __DOXYGEN__
/** A unique ioctl prefix for the TILEGx PCI subsystem. */
#define TILEPCI_IOC_TYPE			0xC5
#endif

/** Base ioctl number for the RSHIM control. */
#define TILEPCI_IOC_NR_BASE_RSHIM		0x00

/** Base ioctl number for channel control. */
#define TILEPCI_IOC_NR_BASE_CHAN		0x10

/** Base ioctl number for the zero-copy API. */
#define TILEPCI_IOC_NR_BASE_ZC			0x20

/** Base ioctl number for the packet-queue API. */
#define TILEPCI_IOC_NR_BASE_PQ			0x30

/** Base ioctl number for the barmem API. */
#define TILEPCI_IOC_NR_BASE_BARMEM		0x40

/** Base ioctl number for the Raw DMA API. */
#define TILEPCI_IOC_NR_BASE_RD			0x50

/** Base ioctl number for the host NIC API. */
#define TILEPCI_IOC_NR_BASE_NIC			0x60

/** Base ioctl number for the chip-to-chip API. */
#define TILEPCI_IOC_NR_BASE_C2C			0x70

/** Base ioctl number for the StreamIO API. */
#define TILEPCI_IOC_NR_BASE_SIO			0x80

/**
 * @mainpage PCIe User Space Communication API
 *
 * The tilegxpci driver allows user space applications to send and
 * receive data via a PCIe link that connects a host machine to the
 * TILExpress card.  This document describes the kernel APIs that
 * allow user space applications to perform PCIe communication.  For
 * information on configuring the PCIe drivers, see the System
 * Programmer's Guide.
 *
 * All of the PCIe user space APIs are exported via device files in
 * /dev.  The path to these files depends on the type of card being
 * used.  On the host machine, the device files are located in the
 * /dev/tilegxpciN/ directory, where N identifies which of potentially
 * many TILEncore-Gx cards is being used.  The user can get a unique
 * link index (N + 1) of each TILEncore-Gx card via HOST_LINK_INDEX from 
 * /dev/tilegxpciN/info file.  On the Tile side, the PCIe links are
 * accessed via the /dev/trioN-macM/ directory, where N is the TRIO 
 * instance number which is always 0 for Gx36 and M is the PCIe port 
 * number on this TRIO instance.  The user can get the major device 
 * number, TRIO instance number N, PCIe port number M on TRIO instance N
 * and the link index in the PCI domain that this port belongs to from 
 * /proc/driver/gxpci_ep_major_mac_link file. 
 *
 * The PCIe driver stack provides the user with several communication
 * mechanisms, each tuned for different usage models:
 *
 * - @ref char_streams provide a simple socket-like communication model
 * between host and Tile applications.  They provide a familiar API
 * and modest performance.
 *
 * - @ref zero_copy provide high-performance communication via an API
 * that allows the programmer to send commands directly to the PCIe
 * driver's DMA engine via Linux system calls.  This interface achieves
 * high performance at the expense of a more complicated programming
 * interface.  The zero copy API can also be used to send buffers
 * between multiple Tile chips connected to the same host.
 *
 * - @ref barmem allows a Tile program to register huge pages that
 * will be mapped to BAR1 of a specially-configured TILEGx device.
 * This interface is often used to allow other PCIe devices (capture
 * cards, FPGAs, etc.) to DMA data to and from Tile Processor memory.
 *
 * - @ref packet_queue allows data transfers between the user-space
 * buffers of the Gx application and the host application without the
 * expense of linux system calls and host interrupts, achieving both
 * low latency and high throughput.
 *
 * - @ref raw_dma allows data transfers between the user-space
 * buffers of the Gx application and remote PCI devices without the
 * expense of application-level flow control, linux system calls and
 * interrupts, achieving both low latency and high throughput.
 *
 * The final section of this document, @ref other, provides a brief
 * description of the device files used to perform non-communication
 * tasks like booting and device locking.
 */

/**
 * @defgroup char_streams Character Streams
 * @{
 *
 * Character streams provide a simple socket-like communication
 * interface between host and tile side applications.  Each PCIe link
 * provides 4 independent, bidirectional character stream devices.
 * Writing to a stream on the host transmits data that can be read on
 * the tile side, and vice versa.
 *
 * Each character stream is represented by a pair of device files -
 * one on the host machine and one in the Tile Linux filesystem.  The
 * device file is named /dev/tilegxpci0/N on the host and /dev/trio0-mac0/N
 * on Tile Linux.  For example, writing to the special file
 * /dev/tilegxpci0/3 on the host machine transmits data to
 * /dev/trio0-mac0/3 on the Tile Processor, which any Tile Linux process
 * may then read.  Similarly, data written to /dev/trio0-mac0/3 is
 * readable from the host machine's /dev/tilegxpci0/3 special file.
 *
 * The write() system call is used to send data via a character
 * stream.  The read() system call is used to receive data on the
 * other side.  On success, these functions return the number of
 * bytes sent or received.  If an error occurs, the call will return
 * -1 and set errno to one of the following values:
 *
 * - ENXIO: Connection is down.  This can occur if the TILExpress card
 * is reset while the zero-copy command queue file handle is open.
 *
 * - EAGAIN: The operation would block.  This error is only returned
 * if the file handle has the O_NONBLOCK flag set.
 *
 * - EFAULT: The data buffer is not in accessible address space.
 *
 * - EINTR: The request was interrupted by a signal before any
 * commands were posted.
 *
 * @}
 */

/**
 * @defgroup zero_copy Zero-Copy Command Queues
 * @{
 *
 * The zero-copy command queue API allows high-bandwidth PCIe
 * applications to transmit and receive buffers without any
 * memory-copy overhead in the driver stack.  Using zero-copy
 * communication allows these applications to achieve maximum PCIe
 * communication bandwidth while minimizing the amount of DRAM
 * bandwidth consumed by the driver stack.
 *
 * Unlike the character stream interface, which provides socket-like
 * bidirectional communication via a device file, zero-copy device
 * files provide a unidirectional transfer mechanism.  Each zero-copy
 * device file provides a 'command queue' abstraction.  Writing to the
 * device file posts a send or receive command and reading from the
 * device file returns a structure indicating the completion status of
 * a previously written command.
 *
 * To include the API declarations for the zero-copy API, do:
 *
 * @code
 * #include <asm/tilegxpci.h>
 * @endcode
 *
 * The MDE provides separate copies of this header file for compiling
 * Tile-side and host-side applications.  To resolve the include when
 * compiling on the host, add '-I\<mde_path\>/include to your compiler
 * flags.  The Tile-side header (in \<mde_path\>/tile/include) is
 * automatically included by tile-cc.
 *
 * @section open Opening Zero-Copy Device Files
 *
 * Each unidirectional, zero-copy command queue is represented as a
 * device file in the /dev file hierarchy.  On the host (x86) machine,
 * the files are named "/dev/tilegxpci0/h2t/N" for host-to-tile queues
 * and "/dev/tilegxpci0/t2h/N" for tile-to-host queues, where N is the
 * queue number.  Similarly, on the endpoint (Tile) processor, the
 * files are named "/dev/trio0-mac0/h2t/N" and "/dev/trio0-mac0/t2h/N".  
 * If a connection can be established, opening one of these files will
 * succeed and the new file descriptor is returned.  If an error
 * occurs, the call will return -1 and set errno to one of the
 * following values:
 *
 * - ENOENT: No such file or directory.  This can occur if the device
 * file name is wrong.
 *
 * - ENXIO: Connection is down.  This can occur if the TILExpress card
 * is reset while the zero-copy command queue file handle is open.
 *
 * The zero-copy command queue can be opened multiple processes
 * simultaneously.  By allowing the device to be opened several times,
 * it provides better support to a cluster of cooperating processes
 * sharing the device.  On the other hand, the applications need to do
 * careful synchronization among the processes or threads sharing the
 * device.
 *
 * Whether a device file is used to send or receive is determined by
 * the combination of which side opens the file and which direction
 * that file transfers data.  Thus, opening a host-to-tile file on the
 * host machine returns a file handle that can be used to send data.
 * For example, the following host-side code opens the sending side of
 * host-to-tile queue 0.
 *
 * @code
 * int fd = open("/dev/tilegxpci0/h2t/0", O_RDWR);
 * assert(fd >= 0);
 * @endcode
 *
 * Similarly, the following code opens the receiving end of the queue
 * on the Tile-side.
 *
 * @code
 * int fd = open("/dev/trio0-mac0/h2t/0", O_RDWR);
 * assert(fd >= 0);
 * @endcode
 *
 * @section mmap Allocating Packet Memory
 *
 * The zero-copy API provides the user with direct access to the PCI
 * subsystem's buffer transfer mechanism.  This mechanism requires
 * that all buffers passed to the device be in contiguous physical
 * memory.  Consequently, the user application needs to guarantee that
 * its buffers do not cross page boundaries that could cause a
 * contiguous range of virtual addresses to map to multiple,
 * discontiguous physical pages.
 *
 * Managing contiguous physical memory is a complicated task, so the
 * tilegxpci zero-copy API provides several tools to help with the job.
 * First, Tile linux uses a 64kB page size, so the user can use
 * memalign() to guarantee that even large packets do not cross a page
 * boundary.  Second, the user needs to register this new allocated buffer 
 * with TRIO via a TILEPCI_IOC_REG_BUF ioctl().  For example, the following 
 * code allocates a buffer on a page boundary, and then registers it with TRIO,
 * so that packet sizes of up to 64kB can be sent or received without 
 * crossing a page boundary.
 *
 * @code
 * // On the Tile side, packet buffers can come from any memory region,
 * // but they must not cross a page boundary.
 * void *buffer = memalign(getpagesize(), PACKET_SIZE);
 * assert(buffer != NULL);
 *
 * // Register thie new allocated buffer to TRIO. 
 * tilegxpci_buf_info_t buf_info = { 
 *   .va = (uintptr_t) buffer, 
 *   .size = getpagesize(),
 * };
 * int result = ioctl(fd, TILEPCI_IOC_REG_BUF, &buf_info);
 * assert(result == 0);
 * @endcode
 *
 * On the other hand, many host machines only support a 4kB page size.
 * The tilegxpci zero-copy driver enables the use of buffers larger than
 * 4kB by allowing the user to mmap() a region of special memory that
 * is guaranteed to be physically contiguous up to 64kB boundaries.
 * The region must be mapped with the MAP_SHARED flag.  The user can
 * then use this memory just as they would on the Tile side, where any
 * 64kB memory region is contiguous.  Thus, host-side programs can use
 * an mmap() call like the following to allocate PCI packet memory to
 * be used with a particular command queue.
 *
 * @code
 * // On the host side, packet buffers must come from the per-channel
 * // 'fast memory' region.
 * void* buffer = mmap(0, PACKET_SIZE, PROT_READ | PROT_WRITE,
 *                     MAP_SHARED, fd, 0);
 * assert(buffer != MAP_FAILED);
 * @endcode
 *
 * @section transfer Transferring Packets
 *
 * The read() and write() system calls are used to transfer data via a
 * zero-copy command queue.  The write() system call posts a command
 * to the queue and the read() system call retrieves completion
 * information for posted commands.  There is a one-to-one correlation
 * between commands posted via write() and completions returned via
 * read().  In addition, each command queue has a limited number of
 * commands that can be outstanding at any time.  The user must use
 * read() to retrieve completions in order to free up command slots so
 * that more commands can be posted via write().
 *
 * @subsection write Posting Commands with write()
 *
 * Writing a ::tilepci_xfer_req_t structure to a zero-copy command
 * queue posts a send or receive command to that channel.  Writing
 * the request structure to the host end of a host-to-tile queue or
 * the tile end of a tile-to-host queue will cause data to be sent.
 * Similarly, writing this structure to the host end of a tile-to-host
 * queue or the tile end of a host-to-tile queue will cause data to be
 * received.
 *
 * Passing an array of ::tilepci_xfer_req_t structures to write() will
 * attempt to post all of those commands to the command queue.  The
 * device will either post all or none of the requests.  If the
 * command(s) cannot be posted, the write() call will return -1 and
 * errno will be set to indicate the cause of the failure:
 *
 * - EINVAL: The write size was not a multiple of
 * sizeof(tilepci_xfer_req_t), or a buffer specified by one of the
 * requests was not physically contiguous.
 *
 * - ENOBUFS: The number of commands set by the ::TILEPCI_IOC_SET_NCMD
 * ioctl() is too small to ever transmit this many requests.
 *
 * - ENXIO: Connection is down.  This can occur if the TILExpress card
 * is reset while the zero-copy command queue file handle is open.
 *
 * - EAGAIN: The operation would block.  This error is only returned
 * if the file handle has the O_NONBLOCK flag set.
 *
 * - EFAULT: Either the command array passed to write() or the data
 * buffer specified by one of the written commands were not in
 * accessible address space.
 *
 * - EINTR: The request was interrupted by a signal before any
 * commands were posted.
 *
 * The following code demonstrates how to post a send command to a
 * zero-copy command queue.
 *
 * @code
 * // Given that we're the host side of a host-to-tile channel, we can
 * // construct a send command and post it to the PCI subsystem.
 * tilepci_xfer_req_t send_cmd = {
 *   .addr = buffer,
 *   .len = PACKET_SIZE,
 *   .flags = TILEPCI_SEND_EOP,
 *   .cookie = 0x12340000,
 * };
 * int rc;
 * while ((rc = write(fd, &send_cmd, sizeof(send_cmd))) != sizeof(send_cmd))
 *   fprintf(stderr, "write() failed: %s\n", strerror(rc));
 * @endcode
 *
 * The following code posts a receive command that will consume the
 * sent data when it arrives at the other side.  Notice that it is
 * almost identical to the code for posting a send command, except
 * that the file handle is on the other side of the uni-directional
 * communication channel and the 'flags' value is different.
 *
 * @code
 * // Given that we're the tile side of a host-to-tile channel, we can
 * // construct a receive command and post it to the PCI subsystem.
 * tilepci_xfer_req_t recv_cmd = {
 *   .addr = buffer,
 *   .len = PACKET_SIZE,
 *   .flags = TILEPCI_RCV_MUST_EOP,
 *   .cookie = 0,
 * };
 * int rc;
 * while ((rc = write(fd, &recv_cmd, sizeof(recv_cmd))) != sizeof(recv_cmd))
 *   fprintf(stderr, "write() failed: %s\n", strerror(rc));
 * @endcode
 *
 * @subsection read Retrieving Completions with read()
 *
 * Once a send or receive command is posted via the write() system
 * call, the user must retrieve a 'completion structure' to determine
 * that the command has finished.  The ::tilepci_xfer_comp_t structure is
 * filled in by the read() syscall in order to return completion
 * status to the user.  This structure includes information about how
 * much data was transfered and whether a receive operation was the
 * end of a packet or overflowed the buffer.  It also includes a word
 * of metadata that is copied from the sender's
 * ::tilepci_xfer_req_t::cookie field to the receiver's
 * ::tilepci_xfer_comp_t::cookie field.
 *
 * If an array of more than one ::tilepci_xfer_comp_t structures is
 * passed to read(), the driver attempts to fill all of the array
 * entries.  If there are not enough completions to fill the array,
 * the behavior depends on whether the file handle is configured as
 * blocking or non-blocking.  If the file handle is non-blocking, the
 * read() method will return immediately with as many completions as
 * are available, or indicate EAGAIN if none are available.  If the
 * file handle is blocking, the driver will block until it can return
 * at least "high water mark" completions.  By default, this means
 * waiting until at least one completion is available; the "high water
 * mark" can be changed via the ::TILEPCI_IOC_SET_MIN_READ_COMPS ioctl().
 *
 * The read() method may return the following errors:
 *
 * - EINVAL: The read size was not a multiple of sizeof(tilepci_xfer_comp_t).
 *
 * - ENXIO: Connection is down.  This can occur if the TILExpress card
 * is reset while the zero-copy command queue file handle is open.
 *
 * - EAGAIN: The operation would block.  This error is only returned
 * if the file handle has the O_NONBLOCK flag set.
 *
 * - EFAULT: The completion array is not in accessible address space.
 *
 * - EINTR: The request was interrupted by a signal before any
 * commands were posted.
 *
 * The following code shows how to use read() to retrieve a completion
 * from a zero-copy command queue.
 *
 * @code
 * // Read back the completion.
 * tilepci_xfer_comp_t comp;
 * while ((rc = read(fd, &comp, sizeof(comp))) != sizeof(comp))
 *   fprintf(stderr, "read() failed: %s\n", strerror(rc));
 * @endcode
 *
 * Note that the example above assumes that the command queue file
 * descriptor is blocking; a non-blocking file handle would return
 * EAGAIN if the completion was not yet available.
 *
 * @section ncmd Limitations on the Number of Commands
 *
 * The PCIe subsystem provides a limited number of 'command slots'
 * that can be used to post commands to the DMA engine.  Each
 * zero-copy command queue file handle is allocated a fraction of
 * these commands slots.  An application is not allowed to post more
 * than this number of commands to the device without first consuming
 * a completion.
 *
 * By default, the maximum number of commands that can be posted to
 * a queue is 16.
 *
 * The number of commands that can be outstanding on a particular
 * command queue can be read via the ::TILEPCI_IOC_GET_NCMD ioctl() and
 * set via the ::TILEPCI_IOC_SET_NCMD ioctl().  The ioctl() returns
 * 0 on success, or returns -1 on error and sets errno to one of the following:
 *
 * - ENOTTY: Unknown ioctl code; usually results from issuing the code
 * on a file handle that is not a zero-copy command queue.
 *
 * - ENXIO: Connection is down.  This condition can occur if the
 * TILExpress card is reset while the zero-copy command queue file
 * handle is open.
 *
 * - ENOBUFS: Could not allocate enough command slots to complete the
 * request.
 *
 * The following code demonstrates the use of ioctl() to set the
 * number of command slots allocated to a particular command queue:
 *
 * @code
 * int ret = ioctl(zc_fd, TILEPCI_IOC_SET_NCMD, 12);
 * @endcode
 *
 * When opened, each command queue is allocated one command slot.
 * Increasing this number will improve performance; four commands per
 * queue is a good setting for many applications.  Applications that
 * need to transfer small buffers at high line rates may required more
 * than four commands outstanding.  The system as a whole has up to
 * ::TILEPCI_CMD_SLOTS slots available for use by zero-copy command
 * queues.
 *
 * @section zc_reset Command Queue Reset
 *
 * Resetting a command queue causes it to immediately complete any
 * previously posted commands so that the user application can restart
 * from a clean state with no commands outstanding.  One common use of
 * this functionality is in handling critical failures; if an
 * application detects such an error, it can reset its communication
 * channel and start over again from a clean slate.
 *
 * A command queue can be reset in two ways.  First, a command queue
 * is automatically reset whenever the last reference to its file is
 * closed.  As a result, applications should be sure to read()
 * completions for all posted commands before closing, since the
 * automatic reset feature can cause some of those commands to be
 * discarded before data transfer can occur.  Second, an application
 * can force command queue reset by issuing the
 * ::TILEPCI_IOC_START_RESET ioctl() to the command queue's file
 * handle.
 *
 * Once a command queue goes into reset mode, all uncompleted commands
 * (on both sides) will be returned with the ::TILEPCI_CPL_RESET bit
 * set.  Applications should always check completions for the reset
 * bit so that the non-resetting side can detect when a channel is put
 * into reset.  For example, if the process attached to the receiving
 * side of a zero-copy command queue dies due to a segmentation
 * violation, the sender side will discover the dead connection when
 * it posts a send command and has its completion returned immediately
 * with the ::TILEPCI_CPL_RESET bit set.
 *
 * After noticing that a command queue is in reset, an application can
 * emerge from reset in two ways.  First, the application could close
 * all references to the device file (including any mapped memory
 * regions) and then reopen the file.  Second, the application can
 * consume any completions it is interested in (since some outstanding
 * commands may have succeeded before reset took effect) and then
 * issue the ::TILEPCI_IOC_FINISH_RESET ioctl().  The 'finish reset'
 * ioctl() will wait for all outstanding commands to complete and
 * discard their completions.  Once it returns, the application can
 * safely continue with the guarantee that no communication operations
 * are outstanding.
 *
 */

/** Get the number of commands that can be issued to a zero-copy stream. */
#define TILEPCI_IOC_GET_NCMD \
	_IOR(TILEPCI_IOC_TYPE, TILEPCI_IOC_NR_BASE_ZC, __u32)

/** Set the number of commands that can be issued to a zero-copy stream. */
#define TILEPCI_IOC_SET_NCMD \
	_IOW(TILEPCI_IOC_TYPE, (TILEPCI_IOC_NR_BASE_ZC + 1), __u32)

/**
 * Get the command high-watermark.
 * A blocking read request will return when either the requested number
 * of completions are available, or when the number of available
 * completions is at least the high-water-mark.
 */
#define TILEPCI_IOC_GET_MIN_READ_COMPS \
	_IOW(TILEPCI_IOC_TYPE, (TILEPCI_IOC_NR_BASE_ZC + 2), __u32)

/**
 * Set the command high-watermark.
 * A blocking read request will return when either the requested number
 * of completions are available, or when the number of available
 * completions is at least the high-water-mark.
 */
#define TILEPCI_IOC_SET_MIN_READ_COMPS \
	_IOW(TILEPCI_IOC_TYPE, (TILEPCI_IOC_NR_BASE_ZC + 3), __u32)

/**
 * Put the command queue into reset mode.
 * While in reset mode, all commands issued by both the sender and
 * receiver are completed immediately and returned with the
 * ::TILEPCI_CPL_RESET flag.
 */
#define TILEPCI_IOC_START_RESET \
	_IO(TILEPCI_IOC_TYPE, (TILEPCI_IOC_NR_BASE_ZC + 4))

/**
 * Finish resetting a command queue.
 * This ioctl() will wait for any outstanding posted commands to complete,
 * and it will discard their completions.  Thus, when the ioctl() returns,
 * the user is guaranteed that no commands are outstanding and data transfer
 * can resume as if the command queue had just been opened.
 */
#define TILEPCI_IOC_FINISH_RESET \
	_IO(TILEPCI_IOC_TYPE, (TILEPCI_IOC_NR_BASE_ZC + 5))

/**
 * A structure used by the TILEPCI_IOC_REG_BUF ioctl of a zero-copy queue.
 * User should call this ioctl with the user-space data buffer starting
 * address and its size before issue data transfers.
 */
typedef struct tilegxpci_buf_info_s {
	__u64		va;	/**< Address of data buffer. */
	__u64		size;	/**< Size of the buffer at 'addr'. */
} tilegxpci_buf_info_t;

/** Register a user-space buffer to TRIO. */
#define TILEPCI_IOC_REG_BUF 			_IOW(TILEPCI_IOC_TYPE, \
	(TILEPCI_IOC_NR_BASE_ZC + 6), tilegxpci_buf_info_t)

/**
 * For receive completions, this flag is set if the receive command was
 * completed as the end of a packet due to either TILEPCI_SEND_EOP
 * or TILEPCI_RCV_MUST_EOP.
 */
#define TILEPCI_CPL_EOP 			(1 << 16)

/**
 * For receive completions, this flag is set if the transmitted data
 * overflowed the receive command's buffer.  If TILEPCI_CPL_EOP is
 * set, some amount of data has been dropped.  If TILEPCI_CPL_EOP is
 * not set, the data will be continued in this channel's next
 * completion.
 */
#define TILEPCI_CPL_OVERFLOW 			(1 << 17)

/**
 * For send and receive completions, this flag is set if the command
 * was returned because the command queue has been reset.
 */
#define TILEPCI_CPL_RESET 			(1 << 18)


/**
 * A user-controlled word of metadata that is copied from a send
 * command to the receive command that consumes its data.
 */
typedef __u64 tilepci_cookie_t;

/**
 * Writing this structure to a zero-copy file handle issues a command
 * to that PCI channel.  Whether the command is a send or receive
 * command depends on the file handle to which it is written.  For
 * example, a host program writing to /dev/tilegxpci0/t2h/0 would be
 * issuing a receive command.
 */
typedef struct tilepci_xfer_req_s {
	__u64              addr;   /**< Address of data buffer. */
	__u32		   len;    /**< Length of the buffer at 'addr'. */

	/** TBD: For send commands, this flag word can contain
	    TILEPCI_SEND_EOP. For receive commands, it can contain
	    TILEPCI_RCV_MAY_EOP or TILEPCI_RCV_MUST_EOP. */
	__u32       	   flags;

	tilepci_cookie_t   cookie; /**< A metadata value (send cmds only). */
} tilepci_xfer_req_t;

/** Writes to a zero-copy file handle must be a multiple of this size. */
#define TILEPCI_XFER_REQ_SZ			(sizeof(tilepci_xfer_req_t))

/**
 * Reading from a zero-copy file handle returns one or more of
 * these completion data structures.  Each completion structure
 * indicates that a send or receive command issued to that file handle
 * has completed.  In the case of receive command, the completion
 * structure also contains information on the size of the received
 * data, flag bits indicating packet framing, and the metadata
 * 'cookie' from the last send command that put data into the buffer
 * specified by the completing receive command.
 */
typedef struct tilepci_xfer_comp_s {
	__u64		   addr;   /**< Buffer from send or receive request.*/
	__u32		   len;    /**< Length of received data. */

	/** For receive completions, this word may contain any of
	    ::TILEPCI_CPL_EOP and ::TILEPCI_CPL_OVERFLOW. */
	__u32		   flags;

	/** Metadata word transmitted from sender to receiver. */
	tilepci_cookie_t   cookie;
} tilepci_xfer_comp_t;

/** Reads from a zero-copy file handle must be a multiple of this size. */
#define TILEPCI_XFER_COMP_SZ			(sizeof(tilepci_xfer_comp_t))

/** The maximum buffer size that can be passed to a zero-copy command queue. */
#define TILEPCI_MAX_XFER_LEN			GXPCI_MAX_CMD_SIZE

/**
 * Invoking mmap() on a zero-copy file handle maps memory that is
 * guaranteed to be physically contiguous up to this number of bytes.
 * The buffer passed to a transfer request should never cross a file
 * mapping offset that is a multiple of this number of bytes.
 */
#define TILEPCI_MMAP_GRANULARITY 		(64 * 1024)

/**
 * The number of command slots available for use with zero-copy
 * command queues.  These command slots can be assigned to specific
 * queues via the TILEPCI_IOC_SET_NCMD ioctl.
 */
#define TILEPCI_CMD_SLOTS 			1024

/**
 * @}
 */


/**
 * @defgroup packet_queue Packet Queue Support
 * @{
 *
 * The PCIe driver stack provides support for direct data transfers between
 * the Tile userspace and the host user space. The host driver allocates
 * a ring buffer in the host memory per direction for each channel.
 * Each entry in the ring buffer has the same size and each new packet is
 * either deposited into or read from the next available ring buffer entry.
 *
 * The user can gain access to the packet_queue interface by opening
 * a 'packet_queue' device file. For example: 
 *
 * @code
 * const char name[64] = "/dev/tilegxpci0/packet_queue/h2t/0";
 * int pq_fd = open(name, O_RDWR);
 * if (pq_fd < 0)
 * {
 *   fprintf(stderr, "Failed to open '%s': %s\n", name, strerror(errno));
 *   exit(EXIT_FAILURE);
 * }
 * @endcode
 *
 * A ring is a circular buffer for each PQ interface, composed of one or more
 * discreet segments. Each segment is a physically contiguous buffer which is
 * divided into individual packet buffers of equal sizes. The number of entries
 * in a segment is defined by macro GXPCI_HOST_PQ_SEGMENT_ENTRIES.
 * 
 * The user application can configure individual buffer size in the ring buffer
 * via the ::TILEPCI_IOC_SET_PACKET_QUEUE_BUF ioctl().  This ioctl will fail
 * if the buffer size is not powers of two, if memory allocaiton
 * failed, or if the application has already invoked mmap() on the
 * packet_queue device.  For example:
 *
 * @code
 * tilepci_packet_queue_info_t buf_info = {
 *   .buf_size = 4096,
 * };
 *
 * int err = ioctl(pq_fd, TILEPCI_IOC_SET_PACKET_QUEUE_BUF, &buf_info);
 * if (err < 0)
 * {
 *   fprintf(stderr, "Failed TILEPCI_IOC_SET_PACKET_QUEUE_BUF: %s\n",
 *           strerror(errno));
 *   exit(EXIT_FAILURE);
 * }
 * @endcode
 *
 * Once the ring buffer parameters have been set, the host application
 * can use mmap() calls to map the queue index registers, the ring
 * buffer and the queue status structure into the application memory
 * space.  The buffer head and tail pointers are mapped in a
 * ::gxpci_host_pq_regs_app structure at mmap file offset
 * ::TILEPCI_PACKET_QUEUE_INDICES_MMAP_OFFSET. The ring buffer (which
 * is of variable size) is mapped starting at mmap file offset
 * ::TILEPCI_PACKET_QUEUE_BUF_MMAP_OFFSET. The queue status struct
 * of type struct tlr_pq_status is mapped starting at mmap file offset
 * ::TILEPCI_PACKET_QUEUE_STS_MMAP_OFFSET.  For example:
 *
 * @code
 * // mmap the packet queue index register space.
 * struct gxpci_host_pq_regs_app* pq_regs =
 *   (struct gxpci_host_pq_regs_app*)
 *   mmap(0, sizeof(struct gxpci_host_pq_regs_app),
 *        PROT_READ | PROT_WRITE,
 *        MAP_SHARED, pq_fd, TILEPCI_PACKET_QUEUE_INDICES_MMAP_OFFSET);
 *
 * //  mmap the receive queue region.
 * void* buffer = mmap(0, host_ring_buffer_size, PROT_READ | PROT_WRITE,
 *                     MAP_SHARED, pq_fd,
 *                     TILEPCI_PACKET_QUEUE_BUF_MMAP_OFFSET);
 * assert(buffer != MAP_FAILED);
 *
 * // mmap the queue status.
 * struct tlr_pq_status *pq_status=
 *   mmap(0, sizeof(struct tlr_pq_status), PROT_READ | PROT_WRITE,
 *        MAP_SHARED, pq_fd, TILEPCI_PACKET_QUEUE_STS_MMAP_OFFSET);
 * assert(pq_status != MAP_FAILED);
 * @endcode
 *
 * Once the queue index registers and ring buffer are mapped into
 * application address space, the program can watch for changes to
 * gxpci_host_pq_regs_app::producer_index to see when packets
 * are posted.  Once a packet is consumed, the application should
 * notify the driver that the ring slot is free again by writing
 * gxpci_host_pq_regs_app::consumer_index. Similarly, the program
 * can watch for changes to gxpci_host_pq_regs_app::consumer_index
 * to see when packets are consumed by the remote tile application.
 * Once a packet is available for transfer, the application should
 * notify the driver that the ring slot is filled again by writing
 * gxpci_host_pq_regs_app::producer_index.
 *
 * Because the packet_queue device maps the TILE processor's MMIO
 * registers directly into userspace, the host driver will not allow a
 * card to reboot until all packet_queue file handles are unmapped and
 * closed.  Any attempt to open the boot device file will fail and set
 * errno to EBUSY.  This prevents bugs that could result from
 * accessing the MMIO registers while the TILE processor is being
 * rebooted.
 */


/**
 * Structure to hold the PCIe packet queue information, used
 * by application to set the packet queue buffer size via the
 * ::TILEPCI_IOC_SET_PACKET_QUEUE_BUF ioctl().
 */
typedef struct tilepci_packet_queue_info_s {
	__u32 buf_size;  /**< Size of each element, must be power of 2.*/
} tilepci_packet_queue_info_t;

/** 
 * The number of entries in the host packet queue Push DMA descriptor rings.
 * These must be at least as large as the
 * (GXPCI_HOST_PQ_SEGMENT_ENTRIES * num_segments).
 * The hardware supports RING_ORDs of 9 (512 entries), 11 (2048 entries), 
 * 13 (8192 entries), and 16 (65536 entries).
 */
#define GXPCI_HOST_PQ_PUSH_DMA_RING_ORD 	13
#define GXPCI_HOST_PQ_PUSH_DMA_RING_LEN 	(1 << \
	GXPCI_HOST_PQ_PUSH_DMA_RING_ORD)

/** 
 * The number of entries in the host packet queue Pull DMA descriptor rings.
 * These must be at least as large as the
 * (GXPCI_HOST_PQ_SEGMENT_ENTRIES * num_segments).
 * The hardware supports RING_ORDs of 9 (512 entries), 11 (2048 entries), 
 * 13 (8192 entries), and 16 (65536 entries).
 */
#define GXPCI_HOST_PQ_PULL_DMA_RING_ORD 	13
#define GXPCI_HOST_PQ_PULL_DMA_RING_LEN 	(1 << \
	GXPCI_HOST_PQ_PULL_DMA_RING_ORD)

/** Set the number and size of elements in the recieve ring buffer. */
#define TILEPCI_IOC_SET_PACKET_QUEUE_BUF _IOW(TILEPCI_IOC_TYPE, \
	TILEPCI_IOC_NR_BASE_PQ, tilepci_packet_queue_info_t)

/** Activate a packet queue. */
#define TILEPCI_IOC_ACTIVATE_PACKET_QUEUE _IO(TILEPCI_IOC_TYPE, \
	TILEPCI_IOC_NR_BASE_PQ + 1)

/**
 * The structure used to keep the H2T and T2H queue info for a single
 * host packet queue interface.
 */
typedef struct tilegxpci_get_pq_queue_msg_s {

	/**< TRIO index.*/
	__u32 trio_index;

	/**< MAC index. */
	__u32 mac_index;

	/**< Number of H2T queues per packet queue interface. */
	__u32 num_h2t_queues;

	/**< Number of T2H queues per packet queue interface. */
	__u32 num_t2h_queues;

} tilegxpci_get_pq_queue_msg_t;

/** Get the H2T and T2H queue info of a single packet queue interface. */
#define TILEPCI_IOC_GET_PQ_QUEUE_MSG	_IOR(TILEPCI_IOC_TYPE, \
	(TILEPCI_IOC_NR_BASE_PQ + 2), tilegxpci_get_pq_queue_msg_t)

/**
 * Get the physical address of the packet queue DMA ring buffer,
 * i.e. the first segment's PA if the ring has multiple segments.
 */
#define TILEPCI_IOC_GET_PACKET_QUEUE_BUF_PA	_IOR(TILEPCI_IOC_TYPE, \
	(TILEPCI_IOC_NR_BASE_PQ + 3), __u64)

/** Get the H2T and T2H queue info of a single packet queue interface for VF. */
#define TILEPCI_IOC_VF_GET_PQ_QUEUE_MSG	_IOR(TILEPCI_IOC_TYPE, \
	(TILEPCI_IOC_NR_BASE_PQ + 4), tilegxpci_get_pq_queue_msg_t)

/** Get the PCI address of a BAR of the physical function. */
#define TILEPCI_IOC_GET_PQ_DEVICE_BAR		_IOR(TILEPCI_IOC_TYPE, \
	(TILEPCI_IOC_NR_BASE_PQ + 5), tilegxpci_bar_info_t)

/** Offset of the receive ring buffer indices in device mmap space. */
#define TILEPCI_PACKET_QUEUE_INDICES_MMAP_OFFSET 0

/** Offset of the receive ring buffer in device mmap space. */
#define TILEPCI_PACKET_QUEUE_BUF_MMAP_OFFSET	4096

/** Offset of the packet queue status in device mmap space. */
#define TILEPCI_PACKET_QUEUE_STS_MMAP_OFFSET 	8192

/**
 * @}
 */

/**
 * @defgroup raw_dma Raw DMA Queue Support
 * @{
 *
 * The PCIe driver stack provides support for direct data transfers between
 * the Tile userspace and the host user space. The host DMA buffer can be
 * allocated in either of the following methods:
 *
 * 1. Using pre-reserved physically contiguous memory via host kernel boot
 * parameter "memmap=". For example, "memmap=2G$4G" reserves physical memory
 * between the range 0x1_0000_0000 and (0x1_0000_0000 + 0x8000_0000). For
 * the detailed usage of memmap, see Linux Documentation file
 * kernel-parameters.txt. The host user application gains access to the
 * reserved memory through the /dev/mem interface, see example code below.
 *
 * The PCIe driver communicates the reserved buffer's physical address and
 * size to the tile side endpoint port driver. The size information is made
 * available to the tile user application which indicates the host buffer
 * by specifying the buffer's offset within the reserved memory.
 *
 * The user on host can gain access to the Raw DMA interface by opening
 * a 'raw_dma' device file. For example:
 *
 * @code
 * const char name[64] = "/dev/tilegxpci0/raw_dma/t2h/0";
 * int rd_fd = open(name, O_RDWR);
 * if (rd_fd < 0)
 * {
 *   fprintf(stderr, "Failed to open '%s': %s\n", name, strerror(errno));
 *   exit(EXIT_FAILURE);
 * }
 * @endcode
 *
 * The host application can obtain the reserved DMA buffer address and size
 * via the ::TILEPCI_IOC_GET_RAW_DMA_BUF ioctl().
 *
 * @code
 * tilepci_raw_dma_get_buf_t buf_info;
 *
 * int err = ioctl(rd_fd, TILEPCI_IOC_GET_RAW_DMA_BUF, &buf_info);
 * if (err < 0)
 * {
 *   fprintf(stderr, "Host: Failed TILEPCI_IOC_GET_RAW_DMA_BUF: %s\n",
 *           strerror(errno));
 *   abort();
 * }
 *
 * host_buf_size = buf_info.rd_buf_size;
 * host_buf_addr = buf_info.rd_buf_bus_addr;
 * @endcode
 *
 * Map the DMA buffer to host user application.
 *
 * @code
 *
 * // On the host side, mmap the receive DMA buffer.
 * int mem_fd = open("/dev/mem", O_RDWR);
 * void* buffer = mmap(0, host_buf_size, PROT_READ | PROT_WRITE,
 *                     MAP_SHARED | MAP_ANONYMOUS, mem_fd, host_buf_addr);
 * assert(buffer != MAP_FAILED);
 *
 * @endcode
 *
 * 2. Alternatively, the DMA buffers can be dynamically allocated by the
 * Raw DMA driver and mmap'ed into the user space, as shown below:
 *
 * @code
 *
 * // On the host side, mmap the transmit DMA buffer *allocated* by the driver.
 * void* buffer = mmap(0, host_buf_size, PROT_READ | PROT_WRITE,
 *                     MAP_SHARED, rd_fd, TILEPCI_RAW_DMA_BUF_MMAP_OFFSET);
 * assert(buffer != MAP_FAILED);
 *
 * @endcode
 *
 * Note that the above two options are mutually exclusive. A macro
 * RAW_DMA_USE_RESERVED_MEMORY is used to select the option, off by default.
 *
 * Map the memory containing the DMA queue status to host user application.
 *
 * @code
 *
 * // On the host side, mmap the queue status.
 * struct tlr_rd_status *rd_status=
 *   mmap(0, sizeof(struct tlr_rd_status), PROT_READ | PROT_WRITE,
 *        MAP_SHARED, rd_fd, TILEPCI_RAW_DMA_STS_MMAP_OFFSET);
 * assert(rd_status != MAP_FAILED);
 *
 * @endcode
 *
 * The host application can detect the tile application's exit
 * by monitoring the queue status which is updated by the Raw DMA
 * driver.
 *
 * @code
 *
 *  if (rd_status->status == GXPCI_CHAN_RESET)
 *  {
 *    printf("Host: T2H: channel is reset\n");
 *    break;
 *  }
 *
 * @endcode
 *
 * Unlike the Packet Queue API, the Raw DMA API doesn't implement flow-control
 * within itself. Instead, it provides programming support to the user
 * application which can optionally implement application-specific
 * flow-control mechanism. The basic idea is that either side can write
 * the flow-control information, i.e. consumer or producer index, to the other
 * side. For example, in a packet capture application (T2H), flow-control
 * can be implemented in the following manner:
 * 1. The host application designates an area in the reserved DMA buffer, e.g.
 *    the very first 4 bytes, to hold the producer index that is updated by the
 *    tile application via a Raw DMA write command. The host application polls
 *    the producer index to determine if more packets have arrived.
 * 2. The Raw DMA API implements struct gxpci_host_rd_regs_app which contains
 *    the T2H queue consumer index and this structure is allocated in the
 *    Raw DMA queue context structure on the tile side.
 * 3. The host application creates the MMIO mapping to the 
 *    gxpci_host_rd_regs_app struct and updates the consumer index via MMIO
 *    writes to this structure.
 * 4. By polling the gxpci_host_rd_regs_app structure, the tile application
 *    determines if the packets have been consumed by the host application.
 *
 * @code
 *
 * // mmap the FC register, e.g. the consumer index for T2H queue.
 * struct gxpci_host_rd_regs_app *rd_regs =
 *   mmap(0, sizeof(struct gxpci_host_rd_regs_app), PROT_READ | PROT_WRITE,
 *        MAP_SHARED, rd_fd, TILEPCI_RAW_DMA_REG_MMAP_OFFSET);
 * assert(rd_regs != MAP_FAILED);
 *
 * @endcode
 *
 * Finally, activate the Raw DMA queue before data transfer can occur
 * via the ::TILEPCI_IOC_ACTIVATE_RAW_DMA ioctl().
 *
 * @code
 *
 * // Activate this receive queue.
 * err = ioctl(rd_fd, TILEPCI_IOC_ACTIVATE_RAW_DMA, NULL);
 * if (err < 0)
 * {
 *   fprintf(stderr, "Host: Failed TILEPCI_IOC_ACTIVATE_RAW_DMA: %s\n",
 *           strerror(errno));
 *   abort();
 * }
 *
 * @endcode
 */

/**
 * The number of send and receive queue pairs in a Raw DMA channel.
 */
#define GXPCI_RAW_DMA_QUEUE_COUNT		8

/**
 * The number of entries in the raw DMA queue Push DMA descriptor rings.
 * The hardware supports RING_ORDs of 9 (512 entries), 11 (2048 entries),
 * 13 (8192 entries), and 16 (65536 entries).
 */
#define GXPCI_RAW_DMA_PUSH_DMA_RING_ORD 	16
#define GXPCI_RAW_DMA_PUSH_DMA_RING_LEN 	(1 << \
	GXPCI_RAW_DMA_PUSH_DMA_RING_ORD)

/**
 * The number of entries in the raw DMA queue Pull DMA descriptor rings.
 * The hardware supports RING_ORDs of 9 (512 entries), 11 (2048 entries),
 * 13 (8192 entries), and 16 (65536 entries).
 */
#define GXPCI_RAW_DMA_PULL_DMA_RING_ORD 	16
#define GXPCI_RAW_DMA_PULL_DMA_RING_LEN 	(1 << \
	GXPCI_RAW_DMA_PULL_DMA_RING_ORD)

/**
 * The Raw DMA buffer can be allocated in either of the following ways:
 *
 * 1. Reserve one physically contiguous memory, using kernel boot parameter
 *    "memmap".
 * 2. Dynamically allocate one or more segments using the kernel page
 *    allocator.
 *
 * To choose reserved memory, define RAW_DMA_USE_RESERVED_MEMORY.
 *
 */
#if 0
#define RAW_DMA_USE_RESERVED_MEMORY
#endif

/**
 * Max size of a single host Raw DMA segment.
 * This can be increased up to the maximum kernel allocation size limit.
 */
#define HOST_RD_SEGMENT_MAX_SIZE	(1UL << 22)

/**
 * Max number of the host Raw DMA segments that can be chained together.
 */
#define HOST_RD_SEGMENT_MAX_NUM		(1 << 5)

/**
 * This structure is used by host-side applications to retrieve
 * the Raw DMA buffer size and physical address via the
 * ::TILEPCI_IOC_GET_RAW_DMA_BUF ioctl().
 */
typedef struct tilepci_raw_dma_get_buf_s {
	/** Size of the Raw DMA buffer in bytes. */
	__u32 rd_buf_size;

	/** Pad. */
	__u32 pad;

#ifdef RAW_DMA_USE_RESERVED_MEMORY
	/** Bus address of the Raw DMA buffer in host memory. */
	__u64 rd_buf_bus_addr;
#endif

} tilepci_raw_dma_get_buf_t;

/** Get the size and address of the host Raw DMA buffer. */
#define TILEPCI_IOC_GET_RAW_DMA_BUF _IOW(TILEPCI_IOC_TYPE, \
        TILEPCI_IOC_NR_BASE_RD, tilepci_raw_dma_get_buf_t)

/** Activate a Raw DMA queue. */
#define TILEPCI_IOC_ACTIVATE_RAW_DMA _IO(TILEPCI_IOC_TYPE, \
	TILEPCI_IOC_NR_BASE_RD + 1)

/**
 * This structure is used by tile-side applications to retrieve
 * the H2T and T2H queue numbers of a Raw DMA interface.
 */
typedef struct tilegxpci_get_rd_queue_cfg_s {

	/**< TRIO index.*/
	__u32 trio_index;

	/**< MAC index. */
	__u32 mac_index;

	/**< Number of H2T queues per Raw DMA channel. */
	__u32 num_h2t_queues;

	/**< Number of T2H queues per Raw DMA channel. */
	__u32 num_t2h_queues;

} tilegxpci_get_rd_queue_cfg_t;

/** Get the H2T and T2H queue info of a single Raw DMA interface. */
#define TILEPCI_IOC_GET_RD_QUEUE_CFG	_IOR(TILEPCI_IOC_TYPE, \
	(TILEPCI_IOC_NR_BASE_RD + 2), tilegxpci_get_rd_queue_cfg_t)

/** Offset of Raw DMA gxpci_host_rd_regs_app struct in device mmap space. */
#define TILEPCI_RAW_DMA_REG_MMAP_OFFSET		0

/** Offset of the Raw DMA queue status in device mmap space. */
#define TILEPCI_RAW_DMA_STS_MMAP_OFFSET		4096

/** Offset of the Raw DMA buffer in device mmap space. */
#define TILEPCI_RAW_DMA_BUF_MMAP_OFFSET		8192

/**
 * @}
 */

/**
 * @defgroup barmem BAR-Mapped Memory
 * @{
 *
 * Specially-configured TILEGx devices can map huge memory pages into
 * the PCIe address space using the PCIe shim's second "base address
 * register" (BAR).  A PCIe BAR defines a region of PCIe address space
 * to which read and write requests can be routed.  If a Tile
 * application registers memory pages to back BAR1, the PCIe subsystem
 * will automatically apply incoming read and write requests to those
 * pages.
 *
 * @section barmem_tile Mapping Tile-Side Memory to BAR1
 *
 * The Tile-side Linux PCIe driver provides a special "barmem" device
 * file for mapping huge pages into BAR1.  To map huge pages, open the
 * "/dev/trioN-macM/barmem" file, fill in a ::tilepci_barmem_map_t to
 * specify which VAs you wish to map, and issue the
 * ::TILEPCI_IOC_BARMEM_MAP ioctl() to perform the mapping.  For
 * example:
 *
 * Once a program has mapped huge pages to DRAM, it can read and write
 * values as it chooses.  However, be aware that incoming PCIe read
 * and write requests modify values in DRAM, so be sure to flush
 * written values to DRAM and invalidate data pulled into a cache by
 * load instructions.
 *
 * @code
 * // Open the 'barmem' device file.
 * int fd = open("/dev/trio0-mac0/barmem", O_RDWR);
 * assert(fd >= 0);
 *
 * // Allocate some huge pages.
 * size_t size = HPAGE_SIZE * 3;
 * alloc_attr_t attr = ALLOC_INIT;
 * alloc_set_huge(&attr);
 * void* mem = alloc_map(&attr, size);
 * assert(mem != 0);
 *
 * // Fill in the tilepci_barmem_map_t structure and ioctl() it.
 * tilepci_barmem_map_t map = {
 *   .va = (uintptr_t) mem,
 *   .size = size,
 *   .offset = 0,
 * };
 * int err = ioctl(fd, TILEPCI_IOC_BARMEM_MAP, &map);
 * if (err < 0)
 * {
 *   fprintf(stderr, "write() returned %d (%s)\n", errno, strerror(errno));
 *   exit(EXIT_FAILURE);
 * }
 *
 * // Write a string to our BAR-mapped memory, then flush it to DRAM so
 * // that incoming PCIe read requests to BAR1 will find the new value.
 * strcpy(mem, "Hello world\n");
 * tmc_mem_flush(mem, 64);
 * @endcode
 *
 * @section barmem_host Reading and Writing From the Host
 *
 * In general, the BAR1 memory-mapped window is intended for use
 * communicating with other PCIe endpoint devices (video capture
 * cards, FPGAs, etc.).  However, the host-side Linux PCIe driver does
 * provide a mechanism for mapping BAR1 into the VA space of a user
 * program running on the host machine.  Be aware that MMIO load and
 * store operations from the host machine are often very inefficient,
 * so the zero-copy and character stream devices are a
 * higher-performance (and safer!) mechanism for transferring data
 * between the host and TILExpress card.
 *
 * To map BAR1 into host userspace, simply mmap() the
 * "/dev/tilegxpciN/barmem" file.  This file must be mapped with the
 * MAP_SHARED flag.  For example:
 *
 * @code
 * // Open the device file.
 * int memfd = open("/dev/tilegxpci0/barmem", O_RDWR);
 * assert(memfd >= 0);

 * // mmap 4kB of the BAR1 memory into our VA space.
 * void* shared_mem = mmap(0, 4 * 1024, PROT_READ | PROT_WRITE,
 *                         MAP_FILE | MAP_SHARED, memfd, 0);
 * if (shared_mem == MAP_FAILED)
 * {
 *   fprintf(stderr, "Failed to map memory: (%s)\n", strerror(errno));
 *   exit(EXIT_FAILURE);
 * }
 * printf("Mapped memory to %p\n", shared_mem);
 * @endcode
 */

/**
 * A structure used to specify a VA range to be mapped to a
 * particular offset within PCIe BAR1.  The va, size, and offset
 * parameters must be huge-page aligned.
 */
typedef struct tilepci_barmem_map_s {
	__u64  va;      /**< Virtual Address of first huge page. */
	__u64  size;    /**< Size of VA range, in bytes. */
	__u32  offset;  /**< Offset within BAR1, in bytes. */
} tilepci_barmem_map_t;

/**
 * Pass a tilepci_barmem_map_t address to this ioctl on
 * /dev/trioN-macM/barmem in order to map it into the BAR1 memory mapping
 * window.  Returns 0 on success or non-zero on failure, in which case
 * errno is set to EINVAL if any parameters were not properly aligned
 * or the requested VA range was not composed of huge pages, EFAULT if
 * the specified va range is not accessible to the caller, and
 * EADDRINUSE if the requested offset was already mapped.  May also
 * set EIO for unexpected IO driver errors.
 */
#define TILEPCI_IOC_BARMEM_MAP _IOW(TILEPCI_IOC_TYPE, \
	TILEPCI_IOC_NR_BASE_BARMEM, tilepci_barmem_map_t)

/**
 * Pass a tilepci_barmem_map_t address to this ioctl on
 * /dev/trioN-macM/barmem in order to unmap it from the BAR1 memory
 * mapping window.  The 'va' pointer is ignored when making unmap
 * requests.  Returns 0 on success or non-zero on failure, in which
 * case errno is set to EINVAL if any parameters were not properly
 * aligned.
 */
#define TILEPCI_IOC_BARMEM_UNMAP _IOW(TILEPCI_IOC_TYPE, \
	(TILEPCI_IOC_NR_BASE_BARMEM + 1), tilepci_barmem_map_t)


/**
 * @}
 */

/**
 * @defgroup other Other Device Files
 *
 * This section provides brief descriptions of the other files that
 * are found in the tilegxpci /dev hierarchy.
 *
 * The 'boot' file is only available on the host and is located at
 * /dev/tilegxpciN/boot.  Writing a bootrom to this device file causes
 * the driver to reset the TILExpress board and reboot it using the
 * written file.  The preferred mechanism for booting a board is via
 * 'tile-monitor --dev gxpciN', but applications can also boot by simply
 * cat'ing a bootrom to the boot file, for example by running 'cat
 * my_file.bootrom > /dev/tilegxpci0/boot'. Rebooting a card causes all
 * the character stream and zero-copy connections to that card to be
 * reset.
 *
 * The 'lock' file, available on the host at /dev/tilegxpciN/lock, can
 * be used to guarantee that only one process can access the boot file
 * at a time.  When a process opens the 'lock' file, it prevents any
 * other process from opening the 'boot' file.  This locking mechanism
 * is particularly helpful for preventing any other process in the
 * system from opening the boot file and resetting the board while the
 * lock holder is still communicating with a program running on the
 * board.
 *
 */
 /** @} */


/******************************************************************/
/*               Undocumented, Unstable Interfaces                */
/******************************************************************/
#ifndef __DOXYGEN__

/** Channel reset. */
#define TILEPCI_IOC_CHANNEL_RESET \
	_IOW(TILEPCI_IOC_TYPE, TILEPCI_IOC_NR_BASE_CHAN, __u32)

/** An ioctl that is used to inform us of a reset via rshim. */
#define TILEPCI_IOC_RSHIM_RESET \
	_IO(TILEPCI_IOC_TYPE, TILEPCI_IOC_NR_BASE_RSHIM)

/** Define how many of each interface we provide. */

/** The number of character stream devices. */
#define TILEPCI_NUM_CHAR_STREAMS		4

/**
 * The number of H2T and T2H zero-copy queues between the host and 
 * a TILE PCIe port. That is, this is the number of bi-directional
 * channels on this link for generic zero-copy data transfers.
 */
#define GXPCI_HOST_ZC_QUEUE_COUNT		2

/** The number of host-to-tile zero-copy queues. */
#define TILEPCI_NUM_ZC_H2T_CHAN			GXPCI_HOST_ZC_QUEUE_COUNT

/** The number of tile-to-host zero-copy queues. */
#define TILEPCI_NUM_ZC_T2H_CHAN			GXPCI_HOST_ZC_QUEUE_COUNT

/** Define minor numbers for all the linux device files. */

/** Minor number of the boot device. */
#define TILEPCI_BOOT_MINOR			1

/** Minor number of the lock device. */
#define TILEPCI_LOCK_MINOR			2

/** Minor number for the memory-mapped BAR device. */
#define TILEPCI_BARMEM_MINOR    		3

/** Minor number of chip information. */
#define TILEPCI_INFO_MINOR			4

/** Minor number for rshim access on TILEGx. */
#define TILEPCI_RSHIM_MINOR			5

/**
 * Minor number of first host-to-tile packet queue.
 * On tile side, this is for the PF only. On the host side,
 * this applies to both the host and the guests.
 */
#define TILEPCI_FIRST_PQ_H2T_MINOR 		16

/**
 * Minor number of first tile-to-host packet queue.
 * On tile side, this is for the PF only. On the host side,
 * this applies to both the host and the guests.
 */
#define TILEPCI_FIRST_PQ_T2H_MINOR 		32

/**
 * Minor number of first host NIC device.
 * Note that these device nodes are created on the TILE system only,
 * not on the host, for the sole purpose of queue management.
 */
#define TILEPCI_FIRST_NIC_MINOR 		48

/** Minor number of first character stream device. */
#define TILEPCI_FIRST_CHAR_STREAM_MINOR		64

/** Minor number of first host-to-tile ZC command queue. */
#define TILEPCI_FIRST_ZC_H2T_MINOR		68

/** Minor number of first tile-to-host ZC command queue. */
#define TILEPCI_FIRST_ZC_T2H_MINOR		70

/** Minor number of first receive Raw DMA queue. */
#define TILEPCI_FIRST_RAW_DMA_RX_MINOR		72

/** Minor number of first send Raw DMA queue. */
#define TILEPCI_FIRST_RAW_DMA_TX_MINOR		88

/** Minor number of first host-to-tile packet queue for VF on tile. */
#define TILEPCI_FIRST_VF_PQ_H2T_MINOR 		104

/** Minor number of first tile-to-host packet queue for VF on tile. */
#define TILEPCI_FIRST_VF_PQ_T2H_MINOR 		120

/** Minor number of first receive chip-to-chip queue. */
#define TILEPCI_FIRST_C2C_RX_MINOR		136

/** Minor number of first send chip-to-chip queue. */
#define TILEPCI_FIRST_C2C_TX_MINOR		200

/**
 * Minor number of first host NIC device for VF on tile.
 * Note that these device nodes are created on the TILE system only,
 * not on the host, for the sole purpose of queue management.
 */
#define TILEPCI_FIRST_VF_NIC_MINOR 		264

#endif /* __DOXYGEN__ */

/**
 * The maximum number of PCIE ports in one PCI domain,
 * including the root-complex port.
 */
#define MAX_PCIE_PORTS_PER_DOMAIN		64

/** Define minor numbers for all the linux device files. */

/** Total number of minor devices. */
#define GXPCI_NUM_MINOR_DEVICES			512

/** Minor number for the pseudo RC device in support of C2C apps. */
#define GXPCI_C2C_BARMEM_MINOR			0

/** A unique ioctl prefix for the TILEGx PCI subsystem. */
#define TILEGXPCI_LOCAL_LINK_INDEX		-1

/**
 * The number of entries in the host and Tile buffer queues.  Each
 * side can post up to PCIE_CMD_QUEUE_ENTRIES buffers to the PCIe
 * subsystem.  Once that number of buffers have been posted, the
 * driver must wait for some buffer slots to free up before posting
 * more buffers.
 */
#define PCIE_CMD_QUEUE_ENTRIES			1024

/** Number of bits to represent PCIE_CMD_QUEUE_ENTRIES. */
#define PCIE_CMD_QUEUE_ORDER			10

/**
 * Max size of a single host packet queue segment.
 * This can be increased up to the maximum kernel allocation size limit.
 */
#define HOST_PQ_SEGMENT_MAX_SIZE		(1UL << 22)

/**
 * Max number of the host packet queue ring buffers, i.e. segments, that
 * can be chained together. This must be a power of 2.
 */
#define HOST_PQ_SEGMENT_MAX_NUM			(1 << 5)

/**
 * Max number of entries in a host packet queue segment. This can be changed
 * as long as (GXPCI_HOST_PQ_SEGMENT_ENTRIES * buffer_size) is not larger than
 * HOST_PQ_SEGMENT_MAX_SIZE.
 */
#define GXPCI_HOST_PQ_SEGMENT_ENTRIES		1024

/** The offset in BAR0 of the RSHIM region. */
#define GXPCI_RSHIM_WINDOW_OFFSET		0x0

/** The size the RSHIM region. */
#define GXPCI_RSHIM_WINDOW_SIZE			0x100000

/******************************************************************/
/*               Undocumented, RSHIM internal definitions         */
/******************************************************************/
#ifndef __DOXYGEN__

/** RSHIM registers. */
#define RSH_DEV_INFO				0x0000
#define RSH_DEV_INFO__TYPE_SHIFT		0
#define RSH_DEV_INFO__TYPE_RMASK		0xfff
#define RSH_DEV_INFO__TYPE_VAL_RSHIM		0x21

#define RSH_SCRATCHPAD 				0x0020

#define RSH_BOOT_CONTROL			0x0528
#define RSH_BOOT_CONTROL__BOOT_MODE_VAL_SPI	0x1

#define RSH_RESET_CONTROL			0x0500
#define RSH_RESET_CONTROL__RESET_CHIP_VAL_KEY	0xca710001
#define RSH_RESET_MASK				0x0508

#define RSH_FABRIC_DIM 				0x0110
#define RSH_FABRIC_DIM__DIM_Y_SHIFT 		0
#define RSH_FABRIC_DIM__DIM_Y_RMASK 		0xf
#define RSH_FABRIC_DIM__DIM_X_SHIFT 		4
#define RSH_FABRIC_DIM__DIM_X_RMASK 		0xf

#define RSH_TILE_COL_DISABLE			0x0b00

#define RSH_PG_CTL 				0x0400
#define RSH_PG_CTL__SENT_COUNT_SHIFT		48
#define RSH_PG_CTL__SENT_COUNT_RMASK		0xffff

#define RSH_PG_DATA				0x0408

/** RSHIM boot parameters. */
#define RSH_BOOT_FIFO_SIZE 			0x1000
/**
 * This is the max number of bytes that are written to the chip
 * in one shot, before pause for FC reason. Note that it must be
 * smaller than RSH_BOOT_FIFO_SIZE and must be a multiple of 8 bytes.
 */
#define PCIE_BOOT_COPY_CHUNK			(RSH_BOOT_FIFO_SIZE / 2)

#endif /* __DOXYGEN__ */

enum gxpci_chan_status_t {

	/** The channel uninitialized flag. */
	GXPCI_CHAN_UNINITIALIZED = 0,

	/** The TILE ready flag. */
	GXPCI_TILE_CHAN_READY,

	/** The TILE acknowledgement to the host ready. */
	GXPCI_TILE_CHAN_READY_ACK,

	/** The host ready flag. */
	GXPCI_HOST_CHAN_READY,

	/** The channel running flag. */
	GXPCI_CHAN_RUNNING,

	/** The channel reset flag. */
	GXPCI_CHAN_RESET,

	/** The channel reset acknowledgement flag. */
	GXPCI_CHAN_RESET_ACK,

	/** The PCIe VNIC channel ENDP side restart flag. */
	GXPCI_EP_CHAN_RESTART,

	/** The PCIe VNIC channel RC side restart flag. */
	GXPCI_RC_CHAN_RESTART

};

/**
 * This structure is shared by the host driver and the C2C application
 * to facilitate chip-to-chip queue reset management on the RC node.
 * The C2C driver writes GXPCI_CHAN_RESET to this struct when the C2C
 * queue is being reset. By polling for GXPCI_CHAN_RESET, the application
 * can get the reset event.
 *
 * On RC node, this structure is allocated as shared memory between the
 * host driver and the C2C application. On EP node, this strucure is
 * allocated in struct gxpci_host_regs and is accessed by the host driver
 * via MMIO.
 */
struct tlr_c2c_status {
	/** Queue status. */
	enum gxpci_chan_status_t status;
};

/**
 * The Gx endpoint port BAR0 space assignment:
 *
 *	Range		Description
 *	-----		-----------
 *	0 - 1MB		RSHIM access
 *	1 - 2MB		Port gxpci_host_regs struct, MSI-X tables and Raw DMA
 *	2 - 3MB		Packet Queues registers
 *	3 - 4MB		Host NIC interface registers
 *	4 - 5MB		Character Streams and Zero-Copy Queues registers
 *	5 - 6MB		Gx Chip-to-Chip Sender ports registers
 *	6 - 7MB		Gx Chip-to-Chip Receiver ports registers
 *	7 - 8MB		Gx Chip-to-Chip Receiver Data Region access
 */

/**
 * The offset in BAR0 of the struct gxpci_host_regs which is used,
 * among other things, to set up the chip-to-chip connections.
 */
#define GXPCI_HOST_REGS_OFFSET			0x100000

/**
 * The number of chip-to-chip send queues and receive queues
 * present in a PCI domain.
 */
#define GXPCI_C2C_QUEUE_COUNT			64

/**
 * This defines the state of each C2C queue. The RC maintains an array of
 * this structure for queue status mangement purpose.
 */
struct gxpci_c2c_queue_sts
{
	/** Flag indicating if this C2C queue is active, set by the sender
	    and cleared by the C2C queue status monitor running on the RC node.
	    The C2C queue status monitor skips this queue if inactive. */
	__u32 active;

	/** Global link index of the sender Gx port. 0 for the RC port. */
	__u32 sender_link_index;

	/** Global link index of the receiver Gx port. 0 for the RC port. */
	__u32 receiver_link_index;

	/** Sender status. */
	__u32 sender_status;

	/** Receiver status. */
	__u32 receiver_status;
 
	/** Padding. */
	__u32 padding;
};

/**
 * Array of C2C queue status that is polled by the host driver to
 * monitor the C2C queue status changes. There is a single copy of this
 * array per PCIe domain and is mapped by each EP port driver which updates
 * the queue status at C2C queue open and close times.
 */
struct gxpci_c2c_queue_sts_array
{
	/** Queue status array. */
	struct gxpci_c2c_queue_sts c2c_queue_sts[GXPCI_C2C_QUEUE_COUNT];

	/** The PCI address of BAR0 region of all devices in the domain,
	    including both RC and EP ports. For the x86 RC port, this is set
	    to 0. For Gx RC port, it is set to PCIE_HOST_BAR0_START. */
	__u64 bar0_addr[MAX_PCIE_PORTS_PER_DOMAIN];

	/** The PCI address of BAR2 region of all devices in the domain,
	    including both RC and EP ports. For the x86 RC port, this is set
	    to 0. For Gx RC port, it is set to PCIE_HOST_BAR2_START. */
	__u64 bar2_addr[MAX_PCIE_PORTS_PER_DOMAIN];
};

/**
 * The memory-mapped input/output (MMIO) registers mapped at offset
 * GXPCI_HOST_REGS_OFFSET the PCIe interface's BAR0 space.
 */
struct gxpci_host_regs
{
	/** When read, the status register returns a ::pcie_status
	    value indicating the current state of the PCIe subsystem.
	    Used by the VF. */
	__u32 port_status;

	/** When read, returns the Gx PCIe driver version number.
	    After booting, host drivers should make sure that
	    the version number is compatible. */
	__u32 version;

	/** Global link index of this MAC within the domain. 
 	    0 for the RC port. */
	__u32 link_index;

	/** BAR0 size. */
	__u32 bar0_size;

	/** BAR2 size. */
	__u32 bar2_size;

#ifdef CONFIG_X86_32
	/** Padding. */
	__u32 padding;
#endif

	/** The BAR0 address. */
	__u64 bar0_addr;

	/** The BAR2 address. */
	__u64 bar2_addr;

	/** Number of T2H queues in one network interface. */
	__u32 nic_t2h_queues;

	/** Number of H2T queues in one network interface. */
	__u32 nic_h2t_queues;

	/** Number of Raw DMA T2H queues. */
	__u32 raw_dma_t2h_queues;

	/** Number of Raw DMA H2T queues. */
	__u32 raw_dma_h2t_queues;

	/** The PCI address of queue status record array on the RC node. */
	__u64 queue_sts_array_bus_addr;

	/** The PCI address of C2C queue status record array on the RC node.
	    Similar to queue_sts_array_bus_addr above, this is set by the
	    host driver so that each Gx EP port can set up mapping to the
	    status record array for active queue status update. */
	__u64 c2c_queue_sts_array_bus_addr;

	/** Array of C2C send queue status that is written by the host
	    driver for queue reset notification. */
	struct tlr_c2c_status c2c_send_status[GXPCI_C2C_QUEUE_COUNT];

	/** Array of C2C receive queue status that is written by the host
	    driver for queue reset notification. */
	struct tlr_c2c_status c2c_recv_status[GXPCI_C2C_QUEUE_COUNT];

	/** SR-IOV VF BAR0 size. */
	__u32 vf_bar0_size;

	/** SR-IOV VF BAR2 size. */
	__u32 vf_bar2_size;
};

/** Possible values of gxpci_host_regs::status. */
enum pcie_status {
	PCIE_STATUS_NOT_READY,         /**< Chip is not ready */
	PCIE_STATUS_READY,             /**< Chip ready */
	PCIE_STATUS_RUNNING,           /**< EP driver is running */
};

/**
 * The number of network interfaces supported between the host and a TILE
 * PCIe port. This means that the same number of bi-directional channels
 * need to be created on this link. Each bi-directional channel consists
 * of variable number of uni-directional queues: H2T and T2H.
 */
#define GXPCI_HOST_NIC_COUNT 			4

/**
 * The maximum number of unidirectional queues in one network interface.
 */
#define GXPCI_HOST_NIC_SIMPLEX_QUEUES_MAX	16

/**
 * The number of unidirectional queues in one network interface.
 */
#define GXPCI_HOST_NIC_SIMPLEX_QUEUES		2

/**
 * The number of H2T queues in one network interface.
 */
#define GXPCI_HOST_NIC_TX_QUEUES		GXPCI_HOST_NIC_SIMPLEX_QUEUES

/**
 * The number of T2H queues in one network interface.
 */
#define GXPCI_HOST_NIC_RX_QUEUES		GXPCI_HOST_NIC_SIMPLEX_QUEUES

/**
 * The number of network interfaces supported by each Virtual Function.
 * Note that for user-space apps, only one interface is supported.
 */
#define GXPCI_HOST_NIC_COUNT_VF			1

/**
 * The number of unidirectional queues in one network interface on a VF.
 */
#define GXPCI_HOST_NIC_SIMPLEX_QUEUES_VF	1

/**
 * The number of H2T queues in one network interface on a VF.
 */
#define GXPCI_HOST_NIC_TX_QUEUES_VF	GXPCI_HOST_NIC_SIMPLEX_QUEUES_VF

/**
 * The number of T2H queues in one network interface on a VF.
 */
#define GXPCI_HOST_NIC_RX_QUEUES_VF	GXPCI_HOST_NIC_SIMPLEX_QUEUES_VF

/**
 * The number of packet_queue interfaces supported between the host and a TILE
 * PCIe port, tile-to-host direction.
 */
#define GXPCI_HOST_PQ_T2H_COUNT			8

/**
 * The number of packet_queue interfaces supported between the host and a TILE
 * PCIe port, host-to-tile direction.
 */
#define GXPCI_HOST_PQ_H2T_COUNT			8

/**
 * Max number of total T2H and H2T queues in all packet queue interfaces for
 * a single TILE PCIe port.
 */
#define GXPCI_HOST_PQ_MAX_QUEUE_NUM		\
	(GXPCI_HOST_PQ_T2H_COUNT + GXPCI_HOST_PQ_H2T_COUNT)

/**
 * The max number of packet_queue interfaces supported between a guest and
 * a TILE PCIe port, host-to-tile direction.
 */
#define GXPCI_HOST_PQ_VF_H2T_COUNT		1

/**
 * The max number of packet_queue interfaces supported between a guest and
 * a TILE PCIe port, tile-to-host direction.
 */
#define GXPCI_HOST_PQ_VF_T2H_COUNT		1

/** 
 * The number of character streams between the host and 
 * a TILE PCIe port. That is, this is the number of bi-directional
 * channels on this link for character stream data transfers. 
 */
#define GXPCI_HOST_CHAR_STREAMS_COUNT		TILEPCI_NUM_CHAR_STREAMS

/** The MSI-X interrupt vector base number of the character stream device. */
#define GXPCI_HOST_CS_INTR_VECTOR_BASE		0

/**
 * The size of the BAR0 space that is used to map the struct
 * gxpci_host_rd_regs. Use host page size, i.e. 4KB.
 */
#define GXPCI_HOST_RD_REGS_MAP_SIZE		0x1000

/**
 * The size of the BAR0 space that is used to map the struct
 * gxpci_host_rd_regs_app. Use host page size, i.e. 4KB.
 */
#define GXPCI_HOST_RD_REG_APP_MAP_SIZE		0x1000

/**
 * The offset of the region hosting the H2T Raw DMA queue registers for the
 * GXPCI_RAW_DMA_QUEUE_COUNT channels. Each queue includes structures
 * gxpci_host_rd_regs and gxpci_host_rd_regs_app. Right now, there is 0x80000
 * space for hosting the Raw DMA queue registers, which means there could be
 * at most 32 T2H queues and 32 H2T queues.
 */
#define GXPCI_HOST_RD_H2T_REGS_OFFSET		0x180000

/**
 * The offset of the region hosting the T2H Raw DMA queue registers for the
 * GXPCI_RAW_DMA_QUEUE_COUNT channels. Each queue includes structures
 * gxpci_host_rd_regs and gxpci_host_rd_regs_app.
 */
#define GXPCI_HOST_RD_T2H_REGS_OFFSET \
	(GXPCI_HOST_RD_H2T_REGS_OFFSET + GXPCI_RAW_DMA_QUEUE_COUNT * \
	(GXPCI_HOST_RD_REGS_MAP_SIZE + GXPCI_HOST_RD_REG_APP_MAP_SIZE))

/**
 * The size of the BAR0 space that is used to map the struct
 * gxpci_host_pq_regs_drv.
 */
#define GXPCI_HOST_PQ_REGS_DRV_MAP_SIZE 	0x10000

/**
 * The size of the BAR0 space that is used to map the struct
 * gxpci_host_pq_regs_app.
 */
#define GXPCI_HOST_PQ_REGS_APP_MAP_SIZE 	0x10000

/**
 * The offset of the region hosting the H2T packet queue registers for the
 * GXPCI_HOST_PQ_H2T_COUNT queues. Each queue's register starts at a
 * (GXPCI_HOST_PQ_REGS_DRV_MAP_SIZE + GXPCI_HOST_PQ_REGS_APP_MAP_SIZE)-aligned
 * address.
 */
#define GXPCI_HOST_PQ_H2T_REGS_OFFSET		0x200000

/**
 * The size of the BAR0 space that is used to map the struct
 * gxpci_host_pq_regs_drv for a VF.
 */
#define GXPCI_VF_HOST_PQ_REGS_DRV_MAP_SIZE 	0x8000

/**
 * The size of the BAR0 space that is used to map the struct
 * gxpci_host_pq_regs_app for a VF.
 */
#define GXPCI_VF_HOST_PQ_REGS_APP_MAP_SIZE 	0x8000

/**
 * The offset of the region hosting the H2T packet queue registers for the
 * GXPCI_HOST_PQ_VF_H2T_COUNT queues in the VF's BAR0 space.
 */
#define GXPCI_VF_HOST_PQ_H2T_REGS_OFFSET	0x10000

/**
 * The size of the BAR0 space that is used to map the struct
 * gxpci_host_queue_regs.
 */
#define GXPCI_HOST_NIC_REGS_MAP_SIZE 		0x1000

/* The size of the BAR0 space that is used to map the netlib registers. */
#define GXPCI_HOST_NETLIB_REGS_MAP_SIZE 	0x1000

/* Max number of the exact match registers, i.e. MPIPE_XAUI_EXACT_MATCH_xxx. */
#define GXPCI_HOST_NETLIB_MAX_EMR_NUM		8

/**
 * The offset of the region hosting the H2T queue registers for the
 * GXPCI_HOST_NIC_COUNT channels. Each queue's register starts at
 * a GXPCI_HOST_NIC_REGS_MAP_SIZE-aligned address.
 */
#define GXPCI_HOST_NIC_H2T_REGS_OFFSET		0x300000

/**
 * The offset of the region hosting the T2H queue registers for the
 * GXPCI_HOST_NIC_COUNT channels. Each queue's register starts at
 * a GXPCI_HOST_NIC_REGS_MAP_SIZE-aligned address.
 */
#define GXPCI_HOST_NIC_T2H_REGS_OFFSET \
	(GXPCI_HOST_NIC_H2T_REGS_OFFSET + GXPCI_HOST_NIC_STREAMS_MAP_SIZE)

/** The map size for the GXPCI_HOST_NIC_COUNT channels. */
#define GXPCI_HOST_NIC_STREAMS_MAP_SIZE \
	(GXPCI_HOST_NIC_REGS_MAP_SIZE * \
	GXPCI_HOST_NIC_SIMPLEX_QUEUES * GXPCI_HOST_NIC_COUNT)

/**
 * The offset of the region hosting the MMI registers for the
 * GXPCI_HOST_NIC_COUNT channels. Each channel's registers start at a
 * PAGE_SIZE-aligned address.
 */
#define GXPCI_HOST_NIC_STREAMS_MMI_REGS_OFFSET \
	(GXPCI_HOST_NIC_REGS_BAR_OFFSET + \
	GXPCI_HOST_NIC_REGS_SIZE * GXPCI_HOST_NIC_COUNT)

/**
 * The offset of the region hosting the zero-copy queue registers for the
 * GXPCI_HOST_ZC_QUEUE_COUNT channels.
 */
#define GXPCI_HOST_ZC_QUEUE_REGS_OFFSET		0x400000

/**
 * The offset of the region hosting the char stream registers for the
 * GXPCI_HOST_CHAR_STREAMS_COUNT channels.
 */
#define GXPCI_HOST_CHAR_STREAMS_OFFSET		0x480000

/**
 * 16KB size of the BAR0 space that is used to map each queue of the char
 * stream and ZC channels.
 */
#define GXPCI_HOST_CHAR_REGS_MAP_SIZE		0x4000

/** The map size for the GXPCI_HOST_CHAR_STREAMS_COUNT channels. */
#define GXPCI_HOST_CHAR_STREAMS_MAP_SIZE \
	(GXPCI_HOST_CHAR_REGS_MAP_SIZE * GXPCI_HOST_CHAR_STREAMS_COUNT)

/**
 * The offset of the region hosting the H2T registers for the
 * GXPCI_HOST_CHAR_STREAMS_COUNT channels. Each channel's registers start at a
 * PAGE_SIZE-aligned address.
 */
#define GXPCI_HOST_CHAR_STREAMS_H2T_REGS_OFFSET \
	(GXPCI_HOST_CHAR_STREAMS_OFFSET)

/**
 * The offset of the region hosting the T2H registers for the
 * GXPCI_HOST_CHAR_STREAMS_COUNT channels. Each channel's registers start at a
 * PAGE_SIZE-aligned address.
 */
#define GXPCI_HOST_CHAR_STREAMS_T2H_REGS_OFFSET \
	(GXPCI_HOST_CHAR_STREAMS_H2T_REGS_OFFSET + \
	GXPCI_HOST_CHAR_STREAMS_MAP_SIZE)

/**
 * The offset of the region hosting the MMI registers for the
 * GXPCI_HOST_CHAR_STREAMS_COUNT channels. Each channel's registers start at a
 * PAGE_SIZE-aligned address.
 */
#define GXPCI_HOST_CHAR_STREAMS_MMI_REGS_OFFSET \
	(GXPCI_HOST_CHAR_STREAMS_T2H_REGS_OFFSET + \
	GXPCI_HOST_CHAR_STREAMS_MAP_SIZE)

/** The map size for the GXPCI_HOST_ZC_QUEUE_COUNT channels. */
#define GXPCI_HOST_ZC_QUEUE_MAP_SIZE \
	(GXPCI_HOST_CHAR_REGS_MAP_SIZE * GXPCI_HOST_ZC_QUEUE_COUNT)

/**
 * The offset of the region hosting the zero-copy queue H2T registers for the
 * GXPCI_HOST_ZC_QUEUE_COUNT channels. Each queue's register starts at
 * PAGE_SIZE-aligned address.
 */
#define GXPCI_HOST_ZC_QUEUE_H2T_REGS_OFFSET \
	(GXPCI_HOST_ZC_QUEUE_REGS_OFFSET)

/**
 * The offset of the region hosting the zero-copy queue T2H registers for the
 * GXPCI_HOST_ZC_QUEUE_COUNT channels. Each queue's register starts at
 * PAGE_SIZE-aligned address.
 */
#define GXPCI_HOST_ZC_QUEUE_T2H_REGS_OFFSET \
	(GXPCI_HOST_ZC_QUEUE_H2T_REGS_OFFSET + GXPCI_HOST_ZC_QUEUE_MAP_SIZE)

/**
 * The offset of the region hosting the MMI registers for the
 * GXPCI_HOST_ZC_QUEUE_COUNT channels. Each channel's registers start at a
 * PAGE_SIZE-aligned address.
 */
#define GXPCI_HOST_ZC_QUEUE_MMI_REGS_OFFSET \
	(GXPCI_HOST_ZC_QUEUE_T2H_REGS_OFFSET + \
	GXPCI_HOST_ZC_QUEUE_MAP_SIZE)


enum gxpci_netlib_ops_t
{
	/**< NOP ops. */
	GXPCI_NETLIB_VNIC_NOP = 0,

	/**< Register read ops. */
	GXPCI_NETLIB_VNIC_READ,

	/**< Register write ops. */
	GXPCI_NETLIB_VNIC_WRITE
};

enum gxpci_netlib_cmd_t
{
	/**< mPIPE link up command. */
	GXPCI_TILE_MPIPE_LINK_UP = 0,

	/**< mPIPE link down command. */
	GXPCI_TILE_MPIPE_LINK_DOWN,

	/**< mPIPE link status. */
	GXPCI_TILE_MPIPE_LINK_STATUS,

	/**< mPIPE MAC address. */
	GXPCI_TILE_MPIPE_MAC_ADDR,

	/**< mPIPE promiscuous mode. */
	GXPCI_TILE_MPIPE_PROMISCUOUS,

	/**< mPIPE all multicast mode. */
	GXPCI_TILE_MPIPE_MULTICAST,

	/**< mPIPE unicast mode. */
	GXPCI_TILE_MPIPE_UNICAST,

	/**< mPIPE jumbo support. */
	GXPCI_TILE_MPIPE_JUMBO,

	/**< Set VLAN filter table entry index. */
	GXPCI_TILE_VLAN_FILTER_TABLE_INDEX,

	/**< Get VLAN filter table entry. */
	GXPCI_TILE_VLAN_FILTER_TABLE_ENTRY,

	/**< Add a new VLAN ID to the VLAN filter table entry. */
	GXPCI_TILE_VLAN_FILTER_TABLE_ENTRY_ADD_VID,

	/**< Delete a VLAN ID from the VLAN filter table entry. */
	GXPCI_TILE_VLAN_FILTER_TABLE_ENTRY_DEL_VID,

	/**< mPIPE unicast promiscuous enable. */
	GXPCI_TILE_MPIPE_UPE,

	/**< mPIPE unicast promiscuous disable. */
	GXPCI_TILE_MPIPE_UPD,

	/**< mPIPE exact match register set command. */
	GXPCI_TILE_MPIPE_SET_EMR
};

/** Control registers for Netlib. */
struct gxpci_netlib_regs
{
	/** The register operation. */
	__u32 ops;

	/** This indicates which MAC register would be read/written
	    on the Gx side. */
	__u32 cmd;

	/** The MAC register value. 
	    For command GXPCI_TILE_MPIPE_SET_EMR, the higher 48-bit
	    stands for the MAC address, and the lower 16-bit stands
	    for the exact match register entry. */
	__u64 data;
};


/**
 * This is the host buffer descriptor, sans the buffer bus address.
 * As part of struct tile_nic_dma_cmd, i.e. the DMA command block,
 * this descriptor is prepared by the host NIC driver for Tx packets and
 * is deposited into the tile side's DMA command array via MMIO writes.
 */
typedef union tile_nic_host_desc
{
	struct {
		__u16 filled:1;		/**< Host command filled. */
		__u16 vlan_packet:1;	/**< This is a VLAN packet. */
		__u16 csum:1;		/**< Tx checksum to be offloaded. */
		__u16 size;		/**< Host buffer size. */
		union {
			/* This is used by NIC applications. */
			struct {
				__u8 csum_offset;	/**< Checksum offset. */
				__u8 csum_start;	/**< Checksum start. */
				__u16 vlan_tag;		/**< VLAN Tag. */
			};

			/* This is used by special apps such as OVS. */
			struct {
				__u32 hash;		/**< Hash value. */
			};

			/* This is used for easy info retrieval. */
			__u32 csum_vlan_hash;
		};
	};

	__u64 desc;

} tile_nic_host_desc_t;

/**
 * This is the tile offload message structure, which carries different
 * infomation for host-to-tile and tile-to-host channels separately.
 * For a tile-to-host channel, the Ethernet packet type pkt_type is used;
 * for a host-to-tile channel, the checksum offset and start csum_offset
 * and csum_start are used. 
 */
typedef union tile_offload_msg
{
	struct {
		__u8 csum_offset;	/**< Checksum offset. */
		__u8 csum_start;	/**< Checksum start. */
	};

	__u16 pkt_type;			/**< Ethernet packet type. */

} tile_offload_msg_t;

/**
 * This is the tile buffer descriptor, sans the buffer address.
 * This is used in the following two ways:
 *   1. As part of struct tile_nic_dma_cmd, i.e. the DMA command block,
 *	prepared by the tile host-NIC API SW for both Tx and Rx queues.
 *   2. Stored at the start of the T2H or NIC Rx packet, containing
 * 	information such as Rx packet size, VLAN Packet flag, VLAN Tag
 * 	and Ethernet packet type. In the future, it can store other
 * 	information such as End-of-Packet, etc.
 * Note that the "size" field needs to located at 2-byte offset in this
 * struct for performance reason.
 */
typedef union tile_nic_tile_desc
{
	struct {
		__u16 filled:1;		/**< Tile command filled. */
		__u16 vlan_packet:1;	/**< This is a VLAN packet. */
		/** A flag that is written by host and read by tile,
 		    indcating Tx checksum to be offloaded. */
		__u16 csum_h2t:1;
		/** A flag that is written by tile and read by host,
 		    indcating TCP/UDP checksum has been done. */
		__u16 csum_t2h:1;
		__u16 size;		/**< Transfer size. */
		union {
			/* This is used by NIC applications. */
			struct {
				tile_offload_msg_t msg; /**< Offload info. */
				__u16 vlan_tag;		/**< VLAN Tag. */
			};

			/* This is used by special apps such as OVS. */
			struct {
				__u32 hash;		/**< Hash value. */
			};

			/* This is used for easy info retrieval. */
			__u32 csum_vlan_hash;
		};
	};

	__u64 desc;

} tile_nic_tile_desc_t;

/**
 * The DMA command structure, used to post buffers to the DMA rings, i.e.
 * Transmit Descriptor or Receive Descriptor, to generate DMA requests
 * by the tile SW. This is used by the host NIC API.
 */
typedef struct tile_nic_dma_cmd
{
	__u64 host_addr;	/**< Host buffer bus address. */

	tile_nic_host_desc_t host_desc;	/**< Desc info set by the host. */

	__u64 tile_addr;	/**< Tile buffer address. */

	tile_nic_tile_desc_t tile_desc;	/**< Desc info set by the tile. */

} tile_nic_dma_cmd_t;

/** Structure for the queue-pair status info. */
struct gxpci_queue_pair_status
{
	/** This indicates the queue status on the tile side. */
	__u32 tile_queue_status;

	/** This indicates the queue has been opened on the tile side. */
	__u32 tile_queue_opened;

	/** This indicates the queue status on the host side. */
	__u32 host_queue_status;

	/** This indicates the queue has been opened on the host side. */
	__u32 host_queue_opened;
};

/** Control registers for a unidirectional host NIC queue. */
struct gxpci_host_nic_queue_regs
{
	/** This indicates the queue status on the Gx side. This is written
	    by the Gx application to GXPCI_TILE_CHAN_READY and then set to
	    GXPCI_HOST_CHAN_READY by the host driver. */
	__u32 queue_status;

	/** The interrupt control register. */
	__u32 interrupt_enable;

	/** Number of host commands that have been posted. This is
	    updated by the host. The device reads this register to
	    determine how many commands have been posted. */
	__u32 cmds_posted_count;

	/** Number of host commands that have been completed. This is
	    updated by the device. The host reads this register to
	    determine how many commands have been completed. */
	__u32 cmds_consumed_count;
 
	/** This is a shadow register on the Gx side, which indicates the
 	    channel state from the RC side, i.e. GXPCI_CHAN_RUNNING and
	    GXPCI_CHAN_RESET. */
	__u32 channel_state;

	/** The receive buffer length of the host. This is updated by the
	    host. The device reads this register to determin the actual
	    T2H DMA transfer size. */
	__u32 host_rx_buf_len;
};

/** Control registers for a host NIC interface, mapped in BAR0 space. */
struct gxpci_host_nic_regs
{
	/** Array of T2H queue registers. */
	struct gxpci_host_nic_queue_regs
		t2h_regs[GXPCI_HOST_NIC_SIMPLEX_QUEUES_MAX];

	/** Array of H2T queue registers. */
	struct gxpci_host_nic_queue_regs
		h2t_regs[GXPCI_HOST_NIC_SIMPLEX_QUEUES_MAX];

	/**
	 * Shadow copy of the host queue pair status, needed here so that
	 * the host queue pair status can be accessed in the tile intr
	 * handler without using the PCIe MMIOs.
	 */
	struct gxpci_queue_pair_status queue_pair_status;
};

/** Control registers for a host NIC interface on a VF, mapped in BAR0 space. */
struct gxpci_host_nic_regs_vf
{
	/** Array of T2H queue registers. */
	struct gxpci_host_nic_queue_regs t2h_regs[GXPCI_HOST_NIC_RX_QUEUES_VF];

	/** Array of H2T queue registers. */
	struct gxpci_host_nic_queue_regs h2t_regs[GXPCI_HOST_NIC_TX_QUEUES_VF];

	/**
	 * Shadow copy of the host queue pair status, needed here so that
	 * the host queue pair status can be accessed in the tile intr
	 * handler without using the PCIe MMIOs.
	 * This is used when the NIC is opened in tile kernel space.
	 */
	struct gxpci_queue_pair_status queue_pair_status;

	/**
	 * Bus address of the queue status array in host memory.
	 * This is used when the NIC is opened in tile user space.
	 */
	__u64 queue_sts_array_bus_addr;
};

/**
 * The BAR0 offset of the region hosting the host NIC registers for the
 * GXPCI_HOST_NIC_COUNT interfaces. Each queue's register starts at
 * a GXPCI_HOST_NIC_REGS_SIZE-aligned address.
 */
#define GXPCI_HOST_NIC_REGS_BAR_OFFSET		0x300000

/**
 * The size of the BAR0 space that is used to map the host NIC registers
 * of a single NIC interface. We evenly divide the 1MB BAR0 space
 * into GXPCI_HOST_NIC_COUNT (4) chunks, yielding 256KB for each NIC.
 */
#define GXPCI_HOST_NIC_REGS_SIZE 		(1 << 18)

/** DMA descriptor array for a unidirectional host NIC queue. */
struct gxpci_host_nic_queue_desc
{
	/**
	 * The DMA descriptor array that is filled by both the host
	 * and the tile.
	 */
	struct tile_nic_dma_cmd dma_cmds[PCIE_CMD_QUEUE_ENTRIES];
};

/** DMA descriptor arrays for a host NIC interface, mapped in BAR2 space. */
struct gxpci_host_nic_desc
{
	/** Array of T2H DMA descriptorts. */
	struct gxpci_host_nic_queue_desc
		t2h_desc[GXPCI_HOST_NIC_SIMPLEX_QUEUES_MAX];

	/** Array of H2T DMA descriptorts. */
	struct gxpci_host_nic_queue_desc
		h2t_desc[GXPCI_HOST_NIC_SIMPLEX_QUEUES_MAX];
};

/** DMA descriptor arrays for a VM NIC interface, mapped in VF BAR0 space. */
struct gxpci_host_nic_desc_vf
{
	/** Array of T2H DMA descriptorts. */
	struct gxpci_host_nic_queue_desc t2h_desc[GXPCI_HOST_NIC_RX_QUEUES_VF];

	/** Array of H2T DMA descriptorts. */
	struct gxpci_host_nic_queue_desc h2t_desc[GXPCI_HOST_NIC_TX_QUEUES_VF];
};

/**
 * The size of the BAR2 space that is used to map all the DMA rings
 * of a NIC interface. Each NIC's DMA array starts at
 * a GXPCI_HOST_NIC_DMA_RING_SIZE-aligned address.
 *
 * The BAR2 offset of the region hosting the host NIC DMA arrays for the
 * GXPCI_HOST_NIC_COUNT interfaces is at <port link index> * <huge page size>.
 * This is dictated by the fact that the BAR2 space is divided into 16MB
 * chunks for the C2C use and the chunk at  <port link index> * 16MB is unused.
 */
#define GXPCI_HOST_NIC_DMA_RING_SIZE 		(1 << 22)

/**
 * Virtual Function host NIC driver memory and BAR0 space assignment:
 *
 * We allocate a huge page, shared by all the T2H and the H2T queues
 * in a host NIC interface between a Gx PCIe VF and a host VM, to contain
 * the control registers and the DMA descriptors, as shown below.
 *
 *      Range                Description
 *      ----------           -----------
 *      0 - 1MB              Mapping the VF's 1MB BAR0 space.
 *        0     - 4KB          Do not use.
 *        4KB   - 8KB          MAP-MEM interrupt registers.
 *        8KB   - 12KB         MSI-X Table.
 *        12KB  - 16KB         MSI-X PBA.
 *        16KB  - 32KB         struct gxpci_host_nic_regs_vf.
 *        32KB  - 512KB        struct gxpci_host_nic_desc_vf.
 *
 * Rest of the page is not exported to the PCI space and is used to hold
 * TRIO DMA descriptor command rings.
 *
 */

/**
 * The offset in VF BAR0 of the TRIO Map Mem Interrupts registers.
 */
#define GXPCI_VF_NIC_MAP_INTR_REG_OFFSET	0x1000

/**
 * The offset in VF BAR0 of the struct gxpci_host_nic_regs_vf.
 */
#define GXPCI_VF_HOST_NIC_REG_OFFSET		0x4000

/**
 * The offset in VF BAR0 of the struct gxpci_host_nic_desc_vf.
 */
#define GXPCI_VF_HOST_NIC_DESC_OFFSET		0x8000

/**
 * The size of VF BAR0 space that is dedicated to an VM NIC interface.
 */
#define GXPCI_VF_HOST_NIC_MAP_SIZE		0x80000


/** Control registers for a unidirectional legacy queue. */
struct gxpci_host_queue_regs
{

	/** This indicates the queue status on the Gx side. This is written
	    by the Gx application to GXPCI_TILE_CHAN_READY and then set to
	    GXPCI_HOST_CHAN_READY by the host driver. */
	__u32 queue_status;

	/** The interrupt control register. */
	__u32 interrupt_enable;

	/** Bus address of the pcie_host_buffer_cmd array in host memory. */
	__u64 buffer_cmd_array;

	/** The interrupt pending register. Non-zero if an interrupt is
	    pending to the host. Always set by TILE and reset by host. */
	__u32 interrupt_pending;

	/** The number of pcie_host_buffer_cmd structures that have been
	    posted by the host. The host writes this register to notify the
	    device that more buffer commands need to be fetched. */
	__u32 buffer_cmd_posted_count;

	/** Bus address of the pcie_host_buffer_cpl array in host memory. */
	__u64 completion_array;

	/** The number of pcie_host_completion structures that have been
	    written by the device to the completion array.  This is updated
	    by the device.  The host reads this register to determine
	    how many commands have been completed. */
	__u32 completion_posted_count;

	/** The number of pcie_host_completion structures in the completion
	    array that have been consumed. The host writes this register to help
	    the EP device determine how many more completions can be posted. */
	__u32 completion_consumed_count;

	/** This is a shadow register on the Gx side, which indicates the
 	    channel state from the RC side, i.e. GXPCI_CHAN_RUNNING and
	    GXPCI_CHAN_RESET. */
	__u32 channel_state;
 
#ifdef CONFIG_X86_32
	/** Padding for 32-bit hosts. */
	__u32 padding;
#endif

};

/**
 * The queue-pair status array, maintained in RC memory and mapped
 * to EP nodes, is used to manage non-C2C queue status. The queue status
 * flags in this array are set in the device release functions.
 */
struct gxpci_queue_status_array
{
	/** Host packet queues, H2T. */
	struct gxpci_queue_pair_status
		pq_sts_h2t[GXPCI_HOST_PQ_MAX_QUEUE_NUM];

	/** Host packet queues, T2H. */
	struct gxpci_queue_pair_status
		pq_sts_t2h[GXPCI_HOST_PQ_MAX_QUEUE_NUM];

	/** RAW DMA queues, receive (H2T). */
	struct gxpci_queue_pair_status
		rd_sts_h2t[GXPCI_RAW_DMA_QUEUE_COUNT];

	/** RAW DMA queues, send (T2H). */
	struct gxpci_queue_pair_status
		rd_sts_t2h[GXPCI_RAW_DMA_QUEUE_COUNT];

	/** Host NIC interfaces. */
	struct gxpci_queue_pair_status nic_sts[GXPCI_HOST_NIC_COUNT];
};

/**
 * PCIe packet_queue control registers.
 *
 * Because both the host packet_queue driver and the application
 * need to access a subset of the packet_queue control registers
 * in the BAR0 space, we split the packet_queue control registers into
 * two groups: the user-visible group containing the packet queue's
 * producer and consumer indices, i.e. struct gxpci_host_pq_regs_app,
 * and the host driver-visible group containing other registers,
 * i.e. struct gxpci_host_pq_regs_drv.
 *
 * With struct gxpci_host_pq_regs_drv, the host packet queue driver
 * can set packet ring-buffer parameters like entry size, number
 * of entries, and the host-memory physical address of the ring.
 * The host ring buffer size must be a power of two and a multiple
 * of the single buffer size.
 *
 * With struct gxpci_host_pq_regs_app, the host application can
 * manage the ring buffer directly without invoking system calls.
 * The producer_index and consumer_index represent the absolute write
 * and read indices, respectively. The relative indices are obtained by
 * division modulo the number of buffers in the ring. For the T2H queue,
 * the producer or write index points to the next buffer to which the
 * tile can DMA and the consumer or read index points to the buffer
 * beyond the last buffer to which the tile can write. The unsigned
 * difference (write_index - read_index) always yields the number of
 * buffers to which data has been written by tile but not yet retrieved
 * by the host application. For the H2T queue, the producer or write
 * index points to the next buffer to which the application can fill
 * the data in and the consumer or read index points to the buffer beyond
 * the last buffer to which the application can write. The unsigned
 * difference (write_index - read_index) always yields the number of
 * buffers to which data has been written by host application but not
 * yet read by the tile.
 */
struct gxpci_host_pq_regs_drv
{

	/** This indicates the queue status on the Gx side. This is written
	    by the Gx application to GXPCI_TILE_CHAN_READY and then set to
	    GXPCI_HOST_CHAN_READY by the host driver. */
	__u32 queue_status;

	/** Number of the PA-contiguous segments. */
	__u32 num_segments;

	/** Size of a single PA-contiguous segment in bytes. */
	__u32 segment_size;

	/** Number of the packet buffers in the host ring buffer. */
	__u32 num_bufs;

	/** Size of a single packet buffer in bytes. */
	__u32 buf_size;

#ifdef CONFIG_X86_32
	/** Padding for 32-bit hosts. */
	__u32 padding;
#endif

	/** Bus addresses of the ring buffer in host memory. */
	__u64 segment_bus_addr[HOST_PQ_SEGMENT_MAX_NUM];

	/** Bus address of the queue status array in host memory. */
	__u64 queue_sts_array_bus_addr;

};

/**
 * Application-visible set of PCIe packet queue control registers.
 */
struct gxpci_host_pq_regs_app
{
	/**
	 * Ring buffer producer index.
	 * For GXPCI_PQ_T2H, this is updated by tile application.
	 * For GXPCI_PQ_H2T, this is updated by host application.
	 */
	volatile __u32 producer_index __attribute__((__aligned__(64)));

	/**
	 * Ring buffer consumer index.
	 * For GXPCI_PQ_T2H, this is updated by host application.
	 * For GXPCI_PQ_H2T, this is updated by tile application.
	 */
	volatile __u32 consumer_index __attribute__((__aligned__(64)));

};

/**
 * Driver-visible set of PCIe Raw DMA queue control registers.
 */
struct gxpci_host_rd_regs_drv
{

	/** This indicates the queue status on the Gx side. This is written
	    by the Gx application to GXPCI_TILE_CHAN_READY and then set to
	    GXPCI_HOST_CHAN_READY by the host driver. */
	__u32 queue_status;

	/** Size of the Raw DMA buffer in bytes. */
	__u32 rd_buf_size;

#ifdef RAW_DMA_USE_RESERVED_MEMORY
	/** Bus address of the Raw DMA buffer in host memory. */
	__u64 rd_buf_bus_addr;
#else
	/**
	 * Following are used if the host DMA buffer is dynamically allocated
	 * by kernel, as opposed to reserved via kernel boot parameter.
	 */

	/** Bus addresses of the ring buffer in host memory. */
	__u64 segment_bus_addr[HOST_RD_SEGMENT_MAX_NUM];
#endif

};

/**
 * Application-visible set of PCIe Raw DMA control registers, allocated
 * on tile memory and mapped to the host user space.
 */
struct gxpci_host_rd_regs_app
{
	/**
	 * The host application updates this register for FC purpose.
	 * For GXPCI_RAW_DMA_SEND, this is the consumer index.
	 * For GXPCI_RAW_DMA_RECV, this is the producer index.
	 */
	volatile __u32 host_counter __attribute__((__aligned__(64)));
};

/** Polling interval for monitoring queue events. */
#define GXPCI_QUEUE_MONITOR_INTERVAL		(HZ / 4)

/**
 * This structure is shared by the host driver and the user application
 * to facilitate packet queue reset management on the host.
 */
struct tlr_pq_status {
	/** Queue status. */
	volatile enum gxpci_chan_status_t status;
};

/**
 * This structure is shared by the host driver and the user application
 * to facilitate Raw DMA queue reset management on the host.
 */
struct tlr_rd_status {
	/** Queue status. */
	volatile enum gxpci_chan_status_t status;
};

/**
 * The structure used to post host buffers to the PCIe subsystem, i.e.
 * Transmit Descriptor or Receive Descriptor.
 */
typedef struct pcie_host_buffer_cmd
{
	__u64 buffer_addr;	/**< Buffer bus address. */
	__u64 tag;		/**< Copied from sender to receiver. */

	/** Size of the buffer, maximum 16KB for Gx36. Any values larger than
	    16KB will be treated as 16KB. */
	__u64 size:GXPCI_MAX_CMD_SIZE_BITS;

#ifdef CONFIG_X86_32
	/** Padding for 32-bit hosts. */
	__u32 padding;
#endif
} pcie_host_buffer_cmd_t;

/**
 * The structure used to post host buffers to the PCIe subsystem, i.e.
 * Transmit Descriptor or Receive Descriptor.
 * This is used by the host NIC API, without the tag field.
 */
typedef struct pcie_host_nic_buffer_cmd
{
	__u64 buffer_addr;	/**< Buffer bus address. */

	/** Size of the buffer, maximum 16KB for Gx36. Any values larger than
	    16KB will be treated as 16KB. */
	__u64 size:GXPCI_MAX_CMD_SIZE_BITS;

#ifdef CONFIG_X86_32
	/** Padding for 32-bit hosts. */
	__u32 padding;
#endif
} pcie_host_nic_buffer_cmd_t;

/**
 * The structure used to inform the host that a communication
 * operation has completed.
 */
typedef struct pcie_host_completion
{
	__u64 buffer_addr;	/**< Buffer bus address. */
	__u64 tag;		/**< Copied from send command. */

	/** Size of the actual data transfer. */
	__u64 size:GXPCI_MAX_CMD_SIZE_BITS;

	/** Reset bit. If set, the command was completed without data
	    transfer because the channel is in reset mode. */
	__u64 reset:1;
	    
        /** End of packet bit.  For receive completions, this bit is set if
            this buffer is the end-of-packet buffer. The value is undefined
            for send completions. */
        __u64 eop:1;

        /** Overflow bit. For receive completions, this bit is set if the
            sender's data overflows the buffer specified by this completion.
            If the eop bit is also set, some amount of send data has been
            dropped. The value applies to queues of type GXPCI_NIC_H2T and
            GXPCI_NIC_T2H only, and is undefined for send completions. */
        __u64 overflow:1;

#ifdef CONFIG_X86_32
	/** Padding for 32-bit hosts. */
	__u32 padding;
#endif
} pcie_host_completion_t;

/**
 * The structure used to inform the host that a communication
 * operation has completed.
 * This is used by the host NIC API, without the tag field.
 */
typedef struct pcie_host_nic_completion
{
	__u64 buffer_addr;	/**< Buffer bus address. */

	/** Size of the actual data transfer. */
	__u64 size:GXPCI_MAX_CMD_SIZE_BITS;

	/** Reset bit. If set, the command was completed without data
	    transfer because the channel is in reset mode. */
	__u64 reset:1;
	    
        /** End of packet bit.  For receive completions, this bit is set if
            this buffer is the end-of-packet buffer. The value is undefined
            for send completions. */
        __u64 eop:1;

        /** Overflow bit. For receive completions, this bit is set if the
            sender's data overflows the buffer specified by this completion.
            If the eop bit is also set, some amount of send data has been
            dropped. The value applies to queues of type GXPCI_NIC_H2T and
            GXPCI_NIC_T2H only, and is undefined for send completions. */
        __u64 overflow:1;

#ifdef CONFIG_X86_32
	/** Padding for 32-bit hosts. */
	__u32 padding;
#endif
} pcie_host_nic_completion_t;

/**
 * The structure used to post host buffers to the PCIe subsystem, i.e.
 * Transmit Descriptor or Receive Descriptor.
 */
typedef struct pcie_tile_buffer_cmd
{
	__u64 buffer_addr;	/**< Buffer bus address. */
	__u64 tag;		/**< Copied from sender to receiver. */

	/** Size of the buffer, maximum 16KB for Gx36. Any values larger than
	    16KB will be treated as 16KB. */
	__u64 size:GXPCI_MAX_CMD_SIZE_BITS;

} pcie_tile_buffer_cmd_t;

/**
 * The structure used to inform the host that a communication
 * operation has completed.
 */
typedef struct pcie_tile_completion
{
	__u64 buffer_addr;	/**< Buffer bus address. */
	__u64 tag;		/**< Copied from send command. */

	/** Size of the actual data transfer. */
	__u64 size:GXPCI_MAX_CMD_SIZE_BITS;

	/** Reset bit. If set, the command was completed without data
	    transfer because the channel is in reset mode. */
	__u64 reset:1;
	    
        /** End of packet bit.  For receive completions, this bit is set if
            this buffer is the end-of-packet buffer. The value is undefined
            for send completions. */
        __u64 eop:1;

        /** Overflow bit. For receive completions, this bit is set if the
            sender's data overflows the buffer specified by this completion.
            If the eop bit is also set, some amount of send data has been
            dropped. The value applies to queues of type GXPCI_NIC_H2T and
            GXPCI_NIC_T2H only, and is undefined for send completions. */
        __u64 overflow:1;

} pcie_tile_completion_t;

/**
 * The structure used to keep the H2T and T2H queue info for a single
 * host NIC port.
 */
typedef struct tilegxpci_get_nic_queue_cfg_s {

	/**< TRIO index.*/
	__u32 trio_index;

	/**< MAC index. */
	__u32 mac_index;

	/**< Number of host virtual NIC interfaces. */
	__u32 num_ports;

	/**< Number of TX queues per virtual NIC interface. */
	__u32 num_tx_queues;

	/**< Number of RX queues per virtual NIC interface. */
	__u32 num_rx_queues;

} tilegxpci_get_nic_queue_cfg_t;

/** Get the H2T and T2H queue info of a single host NIC port. */
#define TILEPCI_IOC_GET_NIC_QUEUE_CFG	_IOR(TILEPCI_IOC_TYPE, \
	TILEPCI_IOC_NR_BASE_NIC, tilegxpci_get_nic_queue_cfg_t)

/** Get the H2T and T2H queue info of a single VF host NIC port. */
#define TILEPCI_IOC_VF_GET_NIC_QUEUE_CFG	_IOR(TILEPCI_IOC_TYPE, \
	(TILEPCI_IOC_NR_BASE_NIC + 1), tilegxpci_get_nic_queue_cfg_t)

/** Activate a host NIC queue on VF. */
#define TILEPCI_IOC_VF_ACTIVATE_NIC_QUEUE _IO(TILEPCI_IOC_TYPE, \
	TILEPCI_IOC_NR_BASE_NIC + 2)

/**
 * Structure to get the BAR address and size of a Gx PCIe port in a
 * PCI domain, used by the Gx C2C API. A pointer to this struct
 * is passed to the /dev/trioN-macM/barmem TILEPCI_IOC_GET_BAR ioctl
 * and the BAR address is set in bar_addr. In the case of local BAR,
 * the link_index is set to TILEGXPCI_LOCAL_LINK_INDEX on input and
 * set to the local port's global link index on output.
 */
typedef struct tilegxpci_bar_info_s {

	/**< BAR PCI address. */
	__u64 bar_addr;

	/**< BAR size. */
	__u32 bar_size;

	/** PCIe link index, or TILEGXPCI_LOCAL_LINK_INDEX for local port.
	 *  When the target is a VF BAR, this specifies the VF number.
	 */
	__u16 link_index;

	/**< BAR index. */
	__u16 bar_index;

} tilegxpci_bar_info_t;

/** Get the PCI address of the BAR of a physical function. */
#define TILEPCI_IOC_GET_BAR \
        _IOR(TILEPCI_IOC_TYPE, TILEPCI_IOC_NR_BASE_C2C, tilegxpci_bar_info_t)


/**
 * Structure to activate a chip-to-chip queue by setting the active flag
 * in gxpci_c2c_queue_sts_array kept in RC memory. A pointer to this struct
 * is passed to the /dev/trioN-macM/c2c/send/X TILEPCI_IOC_C2C_ACTIVATE_QUEUE
 * ioctl call. This call should be made by the sender application only.
 */
typedef struct tilegxpci_c2c_activate_queue_s {

	/** Global link index of the sender Gx port. */
	__u32 sender_link_index;

	/** Global link index of the receiver Gx port. */
	__u32 receiver_link_index;

} tilegxpci_c2c_activate_queue_t;

/** Activate a C2C queue. */
#define TILEPCI_IOC_C2C_ACTIVATE_QUEUE	_IOW(TILEPCI_IOC_TYPE, \
	(TILEPCI_IOC_NR_BASE_C2C + 1), tilegxpci_c2c_activate_queue_t)

/** Set the C2C sender status to ready, called from a sender. */
#define TILEPCI_IOC_C2C_SET_SENDER_READY _IO(TILEPCI_IOC_TYPE, \
	(TILEPCI_IOC_NR_BASE_C2C + 2))

/** Get the C2C sender status, called from a receiver. */
#define TILEPCI_IOC_C2C_GET_SENDER_READY _IOR(TILEPCI_IOC_TYPE, \
	(TILEPCI_IOC_NR_BASE_C2C + 3), __u32)

/** Get the PCI address of the BAR of a virtual function. */
#define TILEPCI_IOC_GET_VF_BAR 		_IOR(TILEPCI_IOC_TYPE, \
	(TILEPCI_IOC_NR_BASE_C2C + 4), tilegxpci_bar_info_t)
/**
 * Structure to get number of PCIe MSI-X vectors configured by the Gx 
 * EP port. A pointer to this struct is passed to the 
 * /dev/trioN-macM/barmem TILEPCI_IOC_GET_MSIX ioctl and the MSI-X 
 * vectors info is set in msix_vectors. 
 */
typedef struct tilegxpci_msix_info_s {

	/**< Number of MSI-X vectors at this link. */
	__u32 msix_vectors;

	/**< The MSI-X interrupt vector base number of the host NICs. */
	int msix_host_nic_intr_vec_base;

	/**< The MSI-X interrupt vector base number of the character streams. */
	int msix_cs_q_intr_vec_base;

	/**< The MSI-X interrupt vector base number of the zero-copy queues. */
	int msix_zc_q_intr_vec_base;

} tilegxpci_msix_info_t;

/** Get the PCIe MSI-X vectors info. */
#define TILEPCI_IOC_GET_MSIX _IOR(TILEPCI_IOC_TYPE, \
	(TILEPCI_IOC_NR_BASE_C2C + 5), tilegxpci_msix_info_t)

/**
 * Structure to set the StreamIO port width and rate. A pointer to this
 * struct is passed to the /dev/trioN-macM/streamio/linkup device with
 * TILEPCI_IOC_STREAMIO_CONFIG ioctl.
 */
typedef struct tilegxpci_sio_config_s {

	/**< Port width, values 1: x1, 2: x2, 4: x4. */
	__u32 port_width;

	/**< Port rate, values: 0: 2.5Gbps, 1: 3.125Gbps,
	     2: 5Gbps, 3: 6.25Gbps. */
	__u32 port_rate;

} tilegxpci_sio_config_t;

/** Configure the StreamIO port. */
#define TILEPCI_IOC_STREAMIO_CONFIG \
	_IOW(TILEPCI_IOC_TYPE, TILEPCI_IOC_NR_BASE_SIO, tilegxpci_sio_config_t)

/**
 * Structure to keep the StreamIO MAC error status. A pointer to this
 * struct is passed to the /dev/trioN-macM/streamio/error_detect device with
 * TILEPCI_IOC_STREAMIO_GET_MAC_ERR ioctl.
 */
typedef struct tilegxpci_sio_mac_err_s {

	/**< Decoder error bit. */
	__u32 dec_err:1;

	/**< RX FIFO error bit. */
	__u32 fifo_err:1;

	/**< RX packet CRC error bit. */
	__u32 crc_err:1;

	/**< RX packet framing error bit. */
	__u32 framing_err:1;

	/**< Non-error bits. */
	__u32 resv:16;

} tilegxpci_sio_mac_err_t;

/** Get the StreamIO MAC errors. */
#define TILEPCI_IOC_STREAMIO_GET_MAC_ERR 	_IOR(TILEPCI_IOC_TYPE, \
	(TILEPCI_IOC_NR_BASE_SIO + 1), tilegxpci_sio_mac_err_t)

/** Bring up the StreamIO link. */
#define TILEPCI_IOC_STREAMIO_LINKUP \
	_IO(TILEPCI_IOC_TYPE, (TILEPCI_IOC_NR_BASE_SIO + 2))

/* Number of minor devices per StreamIO device. */
#define GX_SIO_MINOR_DEVICES			3

/** Minor number of the StreamIO port info device. */
#define GX_SIO_INFO_MINOR			0

/** Minor number of the StreamIO port config device. */
#define GX_SIO_CONFIG_MINOR			1

/** Minor number of the StreamIO port error detect device. */
#define GX_SIO_ERR_DETECT_MINOR			2

#endif /* _ASM_TILEGXPCI_H */
