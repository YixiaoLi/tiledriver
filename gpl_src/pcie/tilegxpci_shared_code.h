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

#ifndef __TILEGXPCI_SHARED_CODE_H__
#define __TILEGXPCI_SHARED_CODE_H__

#include <linux/version.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <asm/page.h>

#if defined(TILEPCI_ENDP)
#include <gxio/trio.h>
#include <gxio/iorpc_trio.h>
#include <hv/drv_trio_intf.h>
#endif /* defined(TILEPCI_ENDP) */

#if !defined(TILEPCI_HOST) && !defined(TILEPCI_ENDP)
#error Shared code requires define of either TILEPCI_HOST or TILEPCI_ENDP
#endif

#if !defined(FALSE)
#define FALSE	(0)
#endif /* !defined(FALSE) */

#if !defined(TRUE)
#define TRUE	(1)
#endif /* !defined(TRUE) */

/*
 * Max number of SR-IOV virtual functions that can be supported on Gx36
 * by the SW driver, using the host Packet Queue interfaces.
 * This limit is per MAC and therefore is twice as high on Gx72 devices.
 */
#define GXPCI_MAX_NUM_VFS	16

/*
 * Max number of SR-IOV virtual functions that can be supported on Gx36
 * by the SW driver, using the tile kernel NIC interfaces.
 * This limit is per MAC and therefore is twice as high on Gx72 devices.
 */
#define GXPCI_MAX_NUM_VFS_KERNEL_NIC	8

/*
 * Max number of SR-IOV virtual functions that can be supported on Gx36
 * by the SW driver, using the tile user NIC interfaces.
 * This limit is per MAC and therefore is twice as high on Gx72 devices.
 */
#define GXPCI_MAX_NUM_VFS_USER_NIC	15

struct tlr_stream;
struct tlr_zc_stream;
struct tlr_pcie_dev;

/* Timeout value of the timer checking the endpoint driver ready status. */
#define EP_DRV_READY_TIMEOUT		(HZ / 50)

#define atomic_compiler_barrier() __asm__ __volatile__("" ::: "memory")

/* Timeout value of the timer monitoring pending interrupts on the endpoint. */
#define EP_INTR_TIMER_INTERVAL		(HZ / 500) 

/*
 * Minor devices are allocated as follows
 * 0-15		-- read & write old-style streams
 * 16		-- boot device
 * 17		-- lock device
 * 18		-- control device
 * ....
 * 128-191	-- Host to Tile devices
 * 192-255	-- Tile to Host devices
 */

/*
 * T2H -> Tile to Host direction
 * H2T -> Host to Tile direction
 */

/* Number of character stream devices. */
#define NUM_CHAR_STREAMS (TILEPCI_NUM_CHAR_STREAMS)

#define TLR_PUBLIC_STREAM_NCMD	(4 + 4)
#define TLR_PUBLIC_NCMD		(TLR_PUBLIC_STREAM_NCMD * NUM_CHAR_STREAMS)

/* Each channel can post up to this many buffers in each direction. */
#define BUFFERS_PER_STREAM 16

/* The size of each posted buffer. */
#define BUFFER_SIZE 4096

/*
 * We keep an array containing a zero-copy command queue for all
 * available channels.  Only a subset of the channels are used by
 * zero-copy devices; the others may be NULL or be used by the
 * character streams.
 */
#define NUM_ZC_H2T_CHAN	(TILEPCI_NUM_ZC_H2T_CHAN)
#define NUM_ZC_T2H_CHAN	(TILEPCI_NUM_ZC_T2H_CHAN)

/* The number of independent TRIO that used by zero-copy queues. */
#define ZC_QUEUE_TRIO_ORDER  (2)
#define ZC_QUEUE_TRIO_NUM    (1 << (ZC_QUEUE_TRIO_ORDER))

#define NUM_MINOR_DEVICES    (GXPCI_NUM_MINOR_DEVICES)
#define FIRST_PUBLIC_MINOR   (TILEPCI_FIRST_CHAR_STREAM_MINOR)
#define LAST_PUBLIC_MINOR    (FIRST_PUBLIC_MINOR + NUM_CHAR_STREAMS - 1)
#define FIRST_ZC_H2T_MINOR   (TILEPCI_FIRST_ZC_H2T_MINOR)
#define LAST_ZC_H2T_MINOR    (FIRST_ZC_H2T_MINOR + NUM_ZC_H2T_CHAN - 1)
#define FIRST_ZC_T2H_MINOR   (TILEPCI_FIRST_ZC_T2H_MINOR)
#define LAST_ZC_T2H_MINOR    (FIRST_ZC_T2H_MINOR + NUM_ZC_T2H_CHAN - 1)

#define TLR_ZC_CMD_Q_NCMD_INIT	(0)
#define TLR_ZC_CMD_Q_NCMD_OPEN	(16)

#define TLR_COOKIE_UNKNOWN	(0)
#define TLR_COOKIE_INVALID	(-1)

#define tlr_dma_addr_t	dma_addr_t
#define tlr_phys_addr_t	unsigned long

#define MAX_MAPPED_PAGES_PER_XFER \
	((TILEPCI_MAX_XFER_LEN + PAGE_SIZE - 1) / PAGE_SIZE)

/* Compute the minimum of two values. */
#define MIN(A,B) (((A) < (B)) ? (A) : (B))

/* Compute the maximum of two values. */
#define MAX(A,B) (((A) > (B)) ? (A) : (B))

#if defined(TILEPCI_ENDP)
/* Round up to alignment; align must be power of two. */
#define ROUND_UP(val,align) (((val) + (align) - 1) & -((__typeof(val)) (align)))

/* Convert indices to pointers. */
#define I2P(base, type, index) ((type *)base + index)

/* Convert pointers to indices. */
#define P2I(base, type, ptr) ((type *)ptr - (type *)base)

/* 
 * Char stream pull/push DMA ring length, which is limited to 8192 according 
 * to the ring size we set below. 
 */
#define GXPCI_STREAM_PULL_DMA_RING_ORD		 (9)
#define GXPCI_STREAM_PUSH_DMA_RING_ORD		 (9)

#define GXPCI_STREAM_PULL_DMA_RING_LEN           \
	(1 << GXPCI_STREAM_PULL_DMA_RING_ORD)
#define GXPCI_STREAM_PUSH_DMA_RING_LEN           \
	(1 << GXPCI_STREAM_PUSH_DMA_RING_ORD)

/* Each DMA ring memory size is limited to 128K bytes. */
#define GXPCI_STREAM_H2T_PULL_DMA_DESC_OFFSET    (0x0)
#define GXPCI_STREAM_H2T_PULL_DMA_DATA_OFFSET    (0x20000)
#define GXPCI_STREAM_H2T_PUSH_DMA_COMP_OFFSET    (0x40000)
#define GXPCI_STREAM_T2H_PULL_DMA_DESC_OFFSET    (0x60000)
#define GXPCI_STREAM_T2H_PUSH_DMA_DATA_OFFSET    (0x80000)
#define GXPCI_STREAM_T2H_PUSH_DMA_COMP_OFFSET    (0xa0000)

/* The h2t and t2h queue's DMA command ring buffer's size limit. */
#define GXPCI_HOST_DMA_CMDS_BUF_SIZE_LIMIT       (0x100000)

/* The h2t and t2h queue's host command/completion buffer's size limit. */
#define GXPCI_HOST_CMDS_COMP_BUF_SIZE_LIMIT      (0x100000)

/* The h2t and t2h queue's counter array size limit. */
#define GXPCI_HOST_CNT_ARRAY_SIZE_LIMIT   	 (0x8000)

/* 
 * The h2t queue's host command ring buffer's offset
 * in the backing memory. 
 */
#define GXPCI_H2T_HOST_CMD_DMA_BUF_OFFSET        (0x100000)

/* 
 * The h2t queue's commands counter array offset in the 
 * backing memory.
 */
#define GXPCI_H2T_CMDS_CNT_ARRAY_OFFSET          (0x200000)

/* 
 * The h2t queue's host command completion ring buffer's offset
 * in the backing memory. 
 */
#define GXPCI_H2T_HOST_COMPS_DMA_BUF_OFFSET      (0x300000)

/*
 * The h2t queue's tile command array buffer's offset
 * in the backing memory. 
 */
#define GXPCI_H2T_TILE_CMD_ARRAY_BUF_OFFSET      (0x400000)

/*
 * The h2t queue's tile completion array buffer's offset
 * in the backing memory. 
 */
#define GXPCI_H2T_TILE_COMP_ARRAY_BUF_OFFSET     (0x500000)

/* 
 * The h2t queue's tile command array buffer's offset
 * in the backing memory. 
 */
#define GXPCI_H2T_HOST_CMD_ARRAY_BUF_OFFSET      (0x600000)

/*
 * The h2t queue's DMA command array buffer's offset
 * in the backing memory. 
 */
#define GXPCI_H2T_DMA_CMD_ARRAY_BUF_OFFSET       (0x700000)

/*
 * The h2t queue's completions counter array offset in the
 * backing memory.
 */
#define GXPCI_H2T_CPLS_CNT_ARRAY_OFFSET          \
	(GXPCI_H2T_CMDS_CNT_ARRAY_OFFSET + GXPCI_HOST_CNT_ARRAY_SIZE_LIMIT)

/* 
 * The t2h queue's host command ring buffer's offset
 * in the backing memory. 
 */
#define GXPCI_T2H_HOST_CMD_DMA_BUF_OFFSET        (0x800000)

/* 
 * The t2h queue's host command completion ring buffer's offset
 * in the backing memory. 
 */
#define GXPCI_T2H_HOST_COMPS_DMA_BUF_OFFSET      (0x900000)

/*
 * The t2h queue's tile command array buffer's offset
 * in the backing memory. 
 */
#define GXPCI_T2H_TILE_CMD_ARRAY_BUF_OFFSET      (0xa00000)

/*
 * The t2h queue's tile completion array buffer's offset
 * in the backing memory. 
 */
#define GXPCI_T2H_TILE_COMP_ARRAY_BUF_OFFSET     (0xb00000)

/* 
 * The t2h queue's tile command array buffer's offset
 * in the backing memory. 
 */
#define GXPCI_T2H_HOST_CMD_ARRAY_BUF_OFFSET      (0xc00000)

/*
 * The t2h queue's DMA command array buffer's offset
 * in the backing memory. 
 */
#define GXPCI_T2H_DMA_CMD_ARRAY_BUF_OFFSET       (0xd00000)

/*
 * The t2h queue's commands counter array offset in the
 * backing memory.
 */
#define GXPCI_T2H_CMDS_CNT_ARRAY_OFFSET          (0xe00000)

/*
 * The t2h queue's completions counter array offset in the
 * backing memory.
 */
#define GXPCI_T2H_CPLS_CNT_ARRAY_OFFSET          \
        (GXPCI_T2H_CMDS_CNT_ARRAY_OFFSET + GXPCI_HOST_CNT_ARRAY_SIZE_LIMIT)

/* Flags to mark end of command queue. */
/* Marker for tile cmd list tail. */
#define TILE_CMD_NULL 			       0x7ff

/* Marker for host cmd list tail. */  
#define HOST_CMD_NULL 			       0x7ff

/* Marker for dma cmd list tail. */   
#define DMA_CMD_NULL  			       0 

#else

/* Size of host buffer_cmd array in bytes. */
#define PCIE_HOST_BUFFER_CMD_ARRAY_SIZE \
  (sizeof(struct pcie_host_buffer_cmd) * PCIE_CMD_QUEUE_ENTRIES)

/* Size of host completion array in bytes. */
#define PCIE_HOST_COMPLETION_ARRAY_SIZE \
  (sizeof(struct pcie_host_completion) * PCIE_CMD_QUEUE_ENTRIES)

/* Value to trigger the tile-side interrupt. */
#define PCIE_HOST_INTR_CPL_AVAIL	       0x1

#endif /* defined(TILEPCI_ENDP) */

enum tlr_dir_e {
	TLR_DIR_UNKNOWN,
	TLR_DIR_H2T,
	TLR_DIR_T2H,
};

struct tlr_buf_fragment {
	struct page *page;
};

struct tlr_mmap_state {
	/* Array of buffer fragments, TILEGXPCI_MMAP_GRANULARITY bytes each.*/
	struct tlr_buf_fragment *frags;
	int num_frags;              /* Number of entries in 'fragments'. */
	int ref_cnt;                /* # of VMAs referencing us. */

	struct semaphore mutex;     /* Must hold this to modify the above. */
};

typedef enum tlr_zc_cmd_type_e {
	TLR_ZC_CMD_UNKNOWN,
	TLR_ZC_CMD_H2T,
	TLR_ZC_CMD_T2H,
}
tlr_zc_cmd_type_t;

enum tlr_zc_cmd_st_e {
	TLR_ZC_CMD_ST_UNKNOWN,
	TLR_ZC_CMD_ST_FREE,
	TLR_ZC_CMD_ST_PEND,
	TLR_ZC_CMD_ST_POST,
	TLR_ZC_CMD_ST_COMP,
};

struct tlr_zc_cmd {
	struct list_head		 list;
	struct tlr_pcie_dev		*tlr;
	struct tlr_zc_cmd_q		*cmd_q;
	enum tlr_zc_cmd_type_e		 type;
	enum tlr_zc_cmd_st_e		 state;
	enum dma_data_direction		 dma_dir;
	dma_addr_t			 dma_addr;
	uint64_t			 usr_addr;
	uint32_t			 post_len;
	uint32_t			 comp_len;
	tilepci_cookie_t		 cookie;
	uint32_t			 flags;
	uint32_t			 nmapped_pages;
	struct page		      *mapped_pages[MAX_MAPPED_PAGES_PER_XFER];
#if defined(TILEPCI_HOST)
	struct pcie_host_buffer_cmd	 cmd;
#elif defined(TILEPCI_ENDP)
	struct pcie_tile_buffer_cmd	 cmd;
#else
#error Undefined Architecture
#endif
};
#define SZ_TLR_ZC_CMD	(sizeof(struct tlr_zc_cmd))

/*
 * Each command queue can be reset independently.  When the
 * hypervisor first comes up, each queue is in reset and waiting for a
 * command with SoC.  When the user posts a command, we set the SoC
 * bit and give it to the hypervisor, entering the connected state.
 * Once connected, we can be reset either by closing the file or
 * issuing the 'start reset' ioctl.  Once in reset, we may or may not
 * discard completions; if the file handle is still open we need to
 * keep them for return to the user, and if the file handle is closed
 * or the user issues the 'finish reset' ioctl, we need to discard
 * completions until no commands are outstanding.
 */
enum tlr_zc_cmd_q_st_e {
	CMD_Q_ST_NEED_SOC,       /* In reset, ready for first cmd with SoC. */
	CMD_Q_ST_CONNECTED,      /* Connection is up and running. */
	CMD_Q_ST_RESET_STARTED,  /* In reset, but don't discard cpls yet. */
	CMD_Q_ST_RESET_DISCARD,  /* In reset, discard cpls until all return.*/
	CMD_Q_ST_CHIP_DOWN,      /* The whole chip is in reset. */
};

struct tlr_list {
	struct list_head	 q;
	uint32_t		 len;
};

struct tlr_zc_cmd_q {
	enum tlr_zc_cmd_type_e	 type;
	uint32_t		 chan;

	struct tlr_list		 free_q;
	struct tlr_list		 pend_q;
	struct tlr_list		 post_q;
	struct tlr_list		 comp_q;

	enum tlr_zc_cmd_q_st_e	 state;
	uint32_t		 nreq_tot;
	uint32_t		 ncomp_tot;

	/* This spinlock protects everything from the lists to here. */
	spinlock_t		 lock;

	tilepci_xfer_comp_t	*rd_xfer_comps;
	struct semaphore	 rd_xfer_mutex;

	tilepci_xfer_req_t	*wr_xfer_reqs;
	struct semaphore	 wr_xfer_mutex;

	/* The number of commands the user wants allocated to this queue. */
	uint32_t		 ncmd_wanted;

	/* The number we actually have right now.  This could be
	 * greater than ncmd_wanted; it will never be less than
	 * ncmd_wanted. */
	uint32_t		 ncmd_have;

	/* If passed a large number of completions to be filled, read
	 * should return once it reaches this many completed. */
	uint32_t		 ncmd_min_read;

	/* Length of comp_q at which the readers on comp_queue should
	 * be woken up. */
	uint32_t		 ncmd_comp_wakeup;

	struct tlr_pcie_dev	*tlr;
	wait_queue_head_t	 free_queue;
	wait_queue_head_t	 comp_queue;
	wait_queue_head_t	 reset_drain_queue;

	uint32_t		 nreq_cur;
	uint32_t		 ncomp_cur;
	uint64_t		 nreq_bytes_cur;
	uint64_t		 ncomp_bytes_cur;

	uint64_t		 nreq_bytes_tot;
	uint64_t		 ncomp_bytes_tot;

	uint32_t		 ncomp_mismatch;

	int 			 open_count;
	int 			 is_ready;
	int 			 irq_num;
	int 			 chip_reset_poison;

	void			 *irq_data;
	
	struct 			 tlr_mmap_state mmap_state;
};

#if defined(TILEPCI_ENDP)
/* The structure for the device model's internal tile_cmd. */
typedef struct tile_cmd
{
	void* buffer_addr;              /* the buffer address. */
	uint64_t tag;		        /* Sender->receiver tag. */
  	/* 
	 * Size of the buffer, max 64k.  Any values larger than 64k will
         * be treated as 64k. 
	 */
  	uint64_t size:17;

  	uint64_t next_index:11;         /* Index to next command. */
}
tile_cmd_t;

/* The structure used to post buffers to the PCIE subsystem. */
typedef struct host_cmd
{
	uint64_t buffer_addr;  	 	/* buffer bus address. */
	uint64_t tag;		        /* Copied from sender to receiver. */

  	/* 
	 * Size of the buffer, max 64k.  Any values larger than 64k will
     	 * be treated as 64k. 
	 */
  	uint64_t size:17;

  	uint64_t next_index:11;         /* Index to next command. */
} 
host_cmd_t;

/*
 * This union is defined to make it easier to use struct copies to
 * copy from an API cmd to an internal tile/host cmd and from an
 * internal cmd to a corresponding completion.
 */
typedef union cmd_completion
{
  	tile_cmd_t tile_cmd;                     /* Tile command. */
  	host_cmd_t host_cmd;                     /* Host command. */
  	pcie_tile_completion_t tile_completion;  /* Tile completion. */
  	pcie_host_completion_t host_completion;  /* Host completion. */
}
cmd_completion_t;

/*
 * A linked list structure that is used to keep track of pending
 * commands from both sides, match them, and build the DMA list.  We
 * keep an array of these structures large enough to hold every
 * possible issued command.  The structures themselves are manipulated
 * as linked lists.
 */
typedef struct dma_cmd
{
  	uint64_t host_cmd_index:10; /* Host cmd index. */
  	uint64_t tile_cmd_index:10; /* Tile cmd index. */
  	uint64_t next_index:11;     /* Next DMA cmd index. */
	uint64_t overflow:1;        /* Receive overflow. */
	uint64_t size:17;           /* Size of transfer. */
  	uint64_t host_cmd_done:1;   /* Host command completed. */
  	uint64_t tile_cmd_done:1;   /* Tile command completed. */
  	uint64_t host_cmd_eop:1;    /* Host command EoP. */
  	uint64_t tile_cmd_eop:1;    /* Tile command EoP. */

	/* This is host cmd being complete-with-reset. */ 
  	uint64_t host_cmd_reset:1; 
	
	/* This is tile cmd being complete-with-reset. */ 
  	uint64_t tile_cmd_reset:1;  
  	
}
dma_cmd_t;

/* A linked list of dma_cmd_t structures. */
typedef struct
{
  	dma_cmd_t* head;            /* Dequeue here. */
  	dma_cmd_t* tail;            /* Enqueue here. */
  	unsigned int consumed;	    /* Number of completed DMAs. */
}
dma_cmd_list_t;

/*
 * Each channel has a linked list of pending, unmatched commands.
 * These commands may be either host or target commands, which ever
 * has issued more requests.  Whenever a request matches, we remove it
 * from the per-channel list and add it to the dma_cmds list.
 */
typedef struct pending_cmd_list
{
  	uint32_t head:16;          /* Oldest unmatched cmd, NULL if none. */
  	uint32_t tail:16;          /* Youngest unmatched cmd, invalid if none.*/
  	uint32_t is_host_list;     /* Whether valid list items are host cmds. */
}
pending_cmd_list_t;

/* State of the unidirectional queue. */
struct gxpci_queue_state
{
	/* Bus address of the host buffer_cmd queue. */
  	uint64_t host_cmd_bus_addr; 
	
	/* Bus address of the host completion queue. */
  	uint64_t host_cpl_bus_addr; 
	
  	/* Tile command buffer. */
  	struct pcie_tile_buffer_cmd* tile_cmd_buffer;
	
	/* Number of tile-side commands posted. */
  	uint32_t tile_cmds_posted;
	
	/* Number of tile-side commands consumed. */
  	uint32_t tile_cmds_consumed;
	
  	/* Buffer into which we DMA posted host commands. */
  	struct pcie_host_buffer_cmd* host_cmd_dma_buffer;
	
  	/* Number of host commands whose DMA have been completed. */
  	uint32_t host_cmds_posted;
	
  	/* Number of parsed host commands. */
  	uint32_t host_cmds_consumed;
	
  	/* Number of host commands whose DMA have been started. */
  	uint32_t host_cmds_dma_started;
	
	/* Number of pull DMAs to fetch host commands that have been started. */
	uint32_t fetch_dmas_started;
	
	/* 
	 * Number of pull DMAs to fetch host commands that have been completed. 
	 */
	uint32_t fetch_dmas_completed;
	
	/* 
	 * The H2T and T2H queue's host commands counter array.
	 * Each array element records the number of host commands
	 * that are fetched in one Pull DMA transfer. 
	 */
	uint16_t *host_cmds_cnt_array;

  	/* Buffer out of which we DMA host completions. */
  	struct pcie_host_completion* host_cpl_dma_buffer;
	
  	/* 
  	 * Number of host completions that are posted in the completion 
  	 * buffer. 
  	 */
  	uint32_t host_cpl_posted;
	
  	/* Number of host completions whose DMA have been completed. */
  	uint32_t host_cpl_consumed;
	
  	/* Number of host completions whose DMA have been started. */
  	uint32_t host_cpl_dma_started;
	
	/* Number of push DMAs to send completions that have been started. */
	uint32_t comp_dmas_started;
	
        /* Number of push DMAs to send completions that have been completed. */
        uint32_t comp_dmas_completed;
	
	/*
	 * The H2T and T2H queue's host completion counter array.
	 * Each array element records the number of host completions
	 * that are sent in one Push DMA transfer. */  
	uint16_t *host_cpls_cnt_array;

  	/* Tile command completion buffer. */
  	void *tile_cpl_buffer;
	
  	/* Number of tile completions. */
  	uint32_t tile_cpl_posted;
	
  	/* Number of tile completions sent to tile. */
  	uint32_t tile_cpl_consumed;

  	/* 
	 * DMA command lists.  Note that the maximum number of DMA commands
  	 * we could ever have in flight is (2 * PCIE_CMD_QUEUE_ENTRIES) - 1,
  	 * since every DMA command completes at least one host|tile request
  	 * and it takes two requests to generate a DMA command.
	 * 
  	 * extra slot as an end-of-list flag.
	 */ 
  	dma_cmd_list_t dma_list;    /* h2t and t2h DMAs that are ready to go. */
  	uint32_t dma_list_consumed; /* Number of started DMAs. */
	dma_cmd_list_t cpl_list;    /* List of next-to-complete DMAs. */
  	dma_cmd_t *free_dma_cmds;   /* Free list. */
  	dma_cmd_t *dma_cmd_array;   /* Storage. */

  	pending_cmd_list_t unmatched_cmds; /* Unmatched cmds.*/

  	host_cmd_t *free_host_cmds;        /* Free list. */
  	host_cmd_t *host_cmd_array;        /* Storage. */

  	tile_cmd_t *free_tile_cmds;        /* Free list. */
  	tile_cmd_t *tile_cmd_array;        /* Storage. */

	uint32_t data_completions;   	   /* Number of completed data DMA. */
	uint32_t comp_completions;	   /* Number of completed comp DMA. */
};

/* State of the DMA resource. */
struct gxpci_dma_resource
{
      	unsigned int pull_dma_desc;           /* Pull DMA (desc) ring index. */
      	unsigned int pull_dma_data;           /* Pull DMA (data) ring index. */
      	gxio_trio_dma_queue_t pull_dma_queue_desc; /* Pull DMA queue body. */
      	gxio_trio_dma_queue_t pull_dma_queue_data; /* Pull DMA queue body. */
      	unsigned int push_dma_data;           /* Push DMA (data) ring index. */
      	unsigned int push_dma_comp;           /* Push DMA (comp) ring index. */
      	gxio_trio_dma_queue_t push_dma_queue_data; /* Push DMA queue body. */
      	gxio_trio_dma_queue_t push_dma_queue_comp; /* Push DMA queue body. */

	/* Irq number for the pull dma (desc) ring. */
      	unsigned int pull_dma_desc_irq;
	
	/* Irq number for the pull dma (data) ring. */
      	unsigned int pull_dma_data_irq;
	
	/* Irq number for the push dma (data) ring. */
      	unsigned int push_dma_data_irq;
	
	/* Irq number for the push dma (comp) ring. */
      	unsigned int push_dma_comp_irq;
};

/* State of the shared TRIO. */
struct trio_resource
{
	/* TRIO context. */
	gxio_trio_context_t trio;

	/* Locks of all TRIO-shared ZC queues. */
	struct semaphore trio_queue_mutex;
	spinlock_t trio_queue_lock;

	/* Arrays to keep the ZC queue index of each DMA command. */
	char *cmd_dma_chans;
	char *data_dma_chans;
	char *comp_dma_chans;

	/* Counters which maintain the ZC queue index arrays. */
	uint32_t cmd_dma_chan_enqueued;	
	uint32_t cmd_dma_chan_dequeued;	
	uint32_t data_dma_chan_enqueued;	
	uint32_t data_dma_chan_dequeued;	
	uint32_t comp_dma_chan_enqueued;	
	uint32_t comp_dma_chan_dequeued;

	/* Counter which holds the open count by TRIO-shared queues. */
	uint32_t trio_open_count;
};

/**
 * Structure to the MSI-X message address and data for a PCIe
 * communication channel or queue.
 */
typedef struct tilegxpci_get_msix_msg_s {

	/**< MSI-X message address. */
	__u64 msix_addr;

	/**< MSI-X message data. */
	__u32 msix_data;

	/**< MSI-X table entry offset. */
	void *msix_table_entry_offset;

} tilegxpci_get_msix_msg_t;
#endif /* defined(TILEPCI_ENDP) */

/* State object for each PCIe zero-copy stream interface. */
struct tlr_zc_stream {

	int index;
	struct tlr_pcie_dev *tlr;
	struct pci_dev *pci_dev;

#if defined(TILEPCI_ENDP)
	/* Pointer to the shared TRIO resource. */
	struct trio_resource *trio_state;	

	/* Pointer to the backing memory used by TRIO DMA. */
	void *backing_mem;

	/* Mem-map region index to map the bar memory. */
	unsigned int mem_map;

	/* Irq number of the memory map interrupt. */
	unsigned int mmi_irq;

	/* 
	 * VA of memory that backs up the BAR0's host interface for one H2T 
	 * queue. 
	 */
	struct gxpci_host_queue_regs *h2t_regs;

	/* 
	 * VA of memory that backs up the BAR0's host interface for one T2H 
	 * queue. 
	 */
	struct gxpci_host_queue_regs *t2h_regs;

        /* MSI-X info for one H2T queue. */
        tilegxpci_get_msix_msg_t h2t_msix_msg;

        /* MSI-X info for one T2H queue. */
        tilegxpci_get_msix_msg_t t2h_msix_msg;

        struct pcie_tile_buffer_cmd *h2t_buffer_cmd_array;
        struct pcie_tile_buffer_cmd *t2h_buffer_cmd_array;   

	/* Host completion buffer info for one H2T queue. */
       	uint64_t h2t_cmpl_array_addr;
	uint32_t h2t_cmpl_array_size;

	/* Host completion buffer info for one T2H queue. */
	uint64_t t2h_cmpl_array_addr;
	uint32_t t2h_cmpl_array_size;

	/* Host command buffer info for one H2T queue. */
	uint64_t h2t_buf_cmd_array_addr;
	uint32_t h2t_buf_cmd_array_size;
	
	/* Host command buffer info for one T2H queue. */	
	uint64_t t2h_buf_cmd_array_addr;
	uint32_t t2h_buf_cmd_array_size;

	/* Queue state structures. */
	struct gxpci_queue_state h2t_queue_state;
	struct gxpci_queue_state t2h_queue_state;		      
	
	/* Pointers to DMA resources. */
	struct gxpci_dma_resource *h2t_dma_resource;
	struct gxpci_dma_resource *t2h_dma_resource;

	/* Timer to track any pending MSI-X interrupt to the host. */
	struct timer_list intr_timer;
#else
	/* Name exposed to "/proc/interrupts". */
	char name[32];

        /* Address of the card's host H2T queue region. */
        struct gxpci_host_queue_regs __iomem *h2t_regs;
        
        /* Address of the card's host T2H queue region. */
        struct gxpci_host_queue_regs __iomem *t2h_regs;
	
	/* Address to trigger the card's MMI interrupt. */
        uint64_t __iomem *intr_regs;

	/* Buffers to keep host commands. */
        struct pcie_host_buffer_cmd *h2t_buffer_cmd_array;
        struct pcie_host_buffer_cmd *t2h_buffer_cmd_array;

        /* 
	 * H2T completion queue state (no spinlock - used only by IRQ 
	 * handler). 
	 */
        uint32_t h2t_completions_last;
        struct pcie_host_completion *h2t_completion_array;
        dma_addr_t h2t_completion_handle;

        /* 
	 * T2H completion queue state (no spinlock - used only by IRQ 
	 * handler). 
	 */
        uint32_t t2h_completions_last;
        struct pcie_host_completion *t2h_completion_array;
        dma_addr_t t2h_completion_handle;
#endif /* !defined(TILEPCI_ENDP) */

        /* H2T command queue state. */
        uint32_t h2t_commands_posted;
        dma_addr_t h2t_buffer_cmd_handle;

        /* T2H command queue state. */
        uint32_t t2h_commands_posted;
        dma_addr_t t2h_buffer_cmd_handle;

	/* Command structures used to build command queues. */
	uint32_t ncmd;
	size_t cmds_sz;
	struct tlr_zc_cmd *cmds;
	
	/* Lock for free list. */
	spinlock_t cmd_q_lock; 
	struct tlr_list	cmd_q_free_list;

	/* The command queue. */
	struct tlr_zc_cmd_q *cmd_queue;
};

struct tlr_packet_queue_state {

	/* Back pointer to the port structure. */
	struct tlr_pcie_dev *tlr;

	/* Address of the queue status. */
	struct tlr_pq_status *queue_status;

	/* PCI address of the queue status. */
	dma_addr_t queue_status_bus_addr;

	/* Address of the queue's MMIO region. */
	struct gxpci_host_pq_regs_drv __iomem *regs;

	/* Number of packet buffers allocated for the Packet Queue ring. */
	uint32_t num_bufs;

	/* Size of a single packet buffer in bytes. */
	uint32_t buf_size;

	/* Size of the ring buffer per PQ interface in bytes. */
	unsigned long ring_size;

	/* Number of the PA-contiguous segments. */
	uint32_t num_segments;

	/* Size of a single PA-contiguous segment in bytes. */
	uint32_t segment_size;

	/* Size order of a single PA-contiguous segment. */
	uint32_t segment_size_order;

	/* NUMA node that the ring buffer is allocated from. */
	uint32_t numa_node;

	/*
 	 * DMA ring buffer sharing configuration: 0 to allocate two individual
 	 * DMA ring buffers for Packet Queue H2T and T2H directions,
 	 * respectively; otherwise a shared DMA ring buffer is used instead.
 	 */
	uint32_t share_h2t_t2h_ring;

	/* Pointers to the PA-contiguous segments. */
	void *pq_segment[HOST_PQ_SEGMENT_MAX_NUM];

	/* PCIe bus addresses of the PA-contiguous segments. */
	dma_addr_t pq_segment_handle[HOST_PQ_SEGMENT_MAX_NUM];

	/* mmap info. */
	int vmas;		/* # of VMAs referencing us. */
	int sts_vmas;		/* # of VMAs referencing us. */
	struct semaphore mutex;

	int channel;
	struct gxpci_host_pq_regs_app __iomem *regs_app;

	spinlock_t lock;
};

struct tlr_raw_dma_state {
	/* Back pointer to the port structure. */
	struct tlr_pcie_dev *tlr;

	/* Address of the queue status. */
	struct tlr_rd_status *queue_status;

	/* PCI address of the queue status. */
	dma_addr_t queue_status_bus_addr;

	/* Address of the queue's MMIO region. */
	struct gxpci_host_rd_regs_drv __iomem *regs;

	/* Size of the DMA buffer per Raw DMA interface in bytes. */
	unsigned long buf_size;

#ifdef RAW_DMA_USE_RESERVED_MEMORY
	/* PCIe bus addresses of the DMA buffer if using reserved memory. */
	dma_addr_t rd_mem_handle;
#else
	/* Number of the PA-contiguous segments. */
	uint32_t num_segments;

	/* NUMA node that the DMA buffer is allocated from. */
	uint32_t numa_node;

	/* Pointers to the PA-contiguous segments. */
	void *rd_segment[HOST_RD_SEGMENT_MAX_NUM];

	/* PCIe bus addresses of the PA-contiguous segments. */
	dma_addr_t rd_segment_handle[HOST_RD_SEGMENT_MAX_NUM];

	/* mmap info. */
	int vmas;		/* # of VMAs referencing us. */
#endif

	int sts_vmas;		/* # of VMAs referencing us. */
	struct semaphore mutex;
};

/* We maintain separate state objects for each stream. */
struct tlr_stream {
	int index;
	int need_read_soc;
	int need_write_soc;
	struct tlr_pcie_dev *dev;
	struct pci_dev *pci_dev;
	
	int open_count;
	int is_ready;
	
#if defined(TILEPCI_ENDP)
	/* Mem-map region index to map the bar memory. */
	unsigned int mem_map;

	/* Irq number of the memory map interrupt. */
	unsigned int mmi_irq;

	/* 
	 * VA of memory that backs up the BAR0's host interface for one H2T 
	 * queue. 
	 */
	struct gxpci_host_queue_regs *h2t_regs;

	/* 
	 * VA of memory that backs up the BAR0's host interface for one T2H 
	 * queue. 
	 */
	struct gxpci_host_queue_regs *t2h_regs;

	/* MSIX info. */
	tilegxpci_get_msix_msg_t msix_msg;
#else
        /* Address of the card's host H2T queue region. */
        struct gxpci_host_queue_regs __iomem *h2t_regs;
        
        /* Address of the card's host T2H queue region. */
        struct gxpci_host_queue_regs __iomem *t2h_regs;
	
	/* Address to trigger the card's MMI interrupt. */
        uint64_t __iomem *intr_regs;	
#endif /* defined(TILEPCI_ENDP) */	

#if defined(TILEPCI_ENDP)
        /* H2T command queue state. */
        spinlock_t h2t_cmd_queue_lock;
#endif /* defined(TILEPCI_ENDP) */

        uint32_t h2t_commands_posted;
        dma_addr_t h2t_buffer_cmd_handle;
#if defined(TILEPCI_HOST)
        struct pcie_host_buffer_cmd *h2t_buffer_cmd_array;
#else
        struct pcie_tile_buffer_cmd *h2t_buffer_cmd_array;
#endif /* defined(TILEPCI_HOST) */

#if defined(TILEPCI_ENDP)
        /* T2H command queue state. */
        spinlock_t t2h_cmd_queue_lock;
#endif /* defined(TILEPCI_ENDP) */

        uint32_t t2h_commands_posted;
        dma_addr_t t2h_buffer_cmd_handle;
#if defined(TILEPCI_HOST)
        struct pcie_host_buffer_cmd *t2h_buffer_cmd_array;
#else
        struct pcie_tile_buffer_cmd *t2h_buffer_cmd_array;
#endif /* defined(TILEPCI_HOST) */

#if defined(TILEPCI_HOST)
        /* 
	 * H2T completion queue state (no spinlock - used only by IRQ 
	 * handler). 
	 */
        uint32_t h2t_completions_last;
        struct pcie_host_completion *h2t_completion_array;
        dma_addr_t h2t_completion_handle;

        /* 
	 * T2H completion queue state (no spinlock - used only by IRQ 
	 * handler). 
	 */
        uint32_t t2h_completions_last;
        struct pcie_host_completion *t2h_completion_array;
        dma_addr_t t2h_completion_handle;
#else
	uint64_t h2t_cmpl_array_addr;
	uint64_t t2h_cmpl_array_addr;
	uint32_t h2t_cmpl_array_size;
	uint32_t t2h_cmpl_array_size;

	uint64_t h2t_buf_cmd_array_addr;
	uint64_t t2h_buf_cmd_array_addr;
	uint32_t h2t_buf_cmd_array_size;
	uint32_t t2h_buf_cmd_array_size;
#endif /* defined(TILEPCI_HOST) */

#if defined(TILEPCI_ENDP)
	unsigned long buffer_page;  /* Use one page for all the buffers. */
#else
	dma_addr_t write_dma_addrs[BUFFERS_PER_STREAM];
	dma_addr_t read_dma_addrs[BUFFERS_PER_STREAM];
#endif /* defined(TILEPCI_ENDP) */

	/*
	 * Poison bit indicating that the chip has rebooted but this
	 * channel hasn't been closed and reopened.
	 */
	int chip_reset_poison;

	/* Semaphore for guaranteeing only one one writer at a time. */
	struct semaphore write_mutex;

	/* Queue of processes waiting for write() buffers. */
	wait_queue_head_t write_queue;

	/*
	 * Write-side state.  When opened, the stream allocates a ring
	 * of buffers to be used for all writes.  The buffers are
	 * posted to and returned by the device in-order, so we can
	 * always determine the next buffer address by keeping a count
	 * of the number of buffers posted thus far, and using it to
	 * index the ring.
	 */
	char *write_buffers[BUFFERS_PER_STREAM];
	size_t write_sizes[BUFFERS_PER_STREAM];
	u32 writes_posted;
	u32 writes_completed;

	/* Semaphore for guaranteeing only one one reader at a time. */
	struct semaphore read_mutex;

	/* Queue of processes waiting for read() buffers. */
	wait_queue_head_t read_queue;

	/*
	 * Read-side state.  Buffers are managed just like write
	 * buffers (see above), but we also keep a count of the number
	 * of bytes already taken out of the oldest returned buffer.
	 * This is necessary so that an incoming large packet is
	 * properly handled by smaller reads.
	 */
	char *read_buffers[BUFFERS_PER_STREAM];
	size_t read_sizes[BUFFERS_PER_STREAM];
	u32 reads_completed;        /* Completions from iBound. */
	u32 reads_consumed;         /* Completely delivered to user. */
	size_t partial_read_bytes;

#if defined(TILEPCI_ENDP)
	struct gxpci_queue_state h2t_queue_state;
	struct gxpci_queue_state t2h_queue_state;

	struct gxpci_dma_resource h2t_dma_resource;
	struct gxpci_dma_resource t2h_dma_resource;

	struct timer_list intr_timer;
#endif /* defined(TILEPCI_ENDP) */
};

#if defined(TILEPCI_ENDP)

typedef enum gxpci_stream_type_e {
	GXPCI_CHAR_H2T_STREAM,
	GXPCI_CHAR_T2H_STREAM,
} 
gxpci_stream_type_t;

/* This struct defines the C2C queue state per queue. */
struct gxpci_ep_c2c_state {
	/*
	 * Address of the entry in the C2C queue status array
	 * corresponding to this queue.
	 */
	struct gxpci_c2c_queue_sts *queue_pair_status;

	/* Back pointer to the EP device structure.*/
	struct tlr_pcie_dev *ep_dev;

	/* mmap info. */
	int vmas;

	/* # of VMAs referencing us. */
	struct semaphore mutex;
};

/* This struct defines the per VF state. */
struct gxpci_ep_vf_state {
	/* The VF number.*/
	int instance;

	/* Number of network interfaces per VF.*/
	int nic_ports;

	/* Mem-map region index to map the BAR0's host NIC interface part. */
	unsigned int nic_bar0_mem_map[GXPCI_HOST_NIC_COUNT_VF];

	unsigned long vf_bar0_address;
	unsigned long vf_bar2_address;

	/* The kernel network interfaces per VF. */
	struct gxpci_nic *nic[GXPCI_HOST_NIC_COUNT_VF];

	/* VA of memory that backs up the BAR0's host NIC interface part. */
	void *nic_bar0_mem[GXPCI_HOST_NIC_COUNT_VF];

	/** Bus address of the queue status array in host memory. */
	uint64_t queue_sts_bus_addr;

	/* MSI-X interrupt table base address. */
	void *msix_table_base;

	/* The MSI-X interrupt vector base number of the host NICs. */
	int msix_host_nic_intr_vec_base;

	/* Number of MSI-X vectors. */
	unsigned int msix_vectors;
};

/* Our PCIe endipoint device structure, used on EP. */
struct tlr_pcie_dev {
	struct pci_dev *pci_dev;
	struct cdev cdev;
	dev_t first_dev;

	/* Pointer back to the TRIO that this PCIe port is connected to. */
	gxio_trio_context_t *trio;

	/* PCIe mac index on the TRIO shim. */
	int mac;
	
	/* Index of TRIO shim that contains the MAC. */
	int trio_index;

	/* Global link index for C2C communication. */
	int link_index;

	/* Index of TRIO Mem Map region that maps struct gxpci_host_regs. */
	int host_reg_mem_map;

	/* Index of TRIO Mem Map region that maps the MSI-X table. */
	int host_msix_mem_map;

	/* This is used to manage the EP/RC link setup. */
	struct delayed_work ep_link_work;

	/* This is used to initialize TRIO when EP link is up. */
	struct delayed_work ep_trio_init_work;

	/* Some BIOS info. */
	uint64_t bar0_addr;
	size_t max_payload_size;
	size_t max_read_size;
	int link_width;

	/* 1: Gen1, 2: Gen 2, 4: Gen 3. */
	int link_speed;

	/* VA of memory that backs up the BAR0's host interface part. */
	void *bar_mem;

	/* VA of memory that backs up the BAR0's host NIC interface part. */
	void *nic_bar0_mem;

	/* VA of memory that backs up the BAR2's host NIC interface part. */
	void *nic_bar2_mem;

	/* Mem-map region index to map the bar memory. */
	unsigned int mem_map;

	/* Mem-map region index to map the BAR0's host NIC interface part. */
	unsigned int nic_bar0_mem_map;

	/* Mem-map region index to map the BAR2's host NIC interface part. */
	unsigned int nic_bar2_mem_map;

	/* PIO region index for accessing non-C2C queue status array on RC. */
	unsigned int pio;

	/* Mapped address to the non-C2C queue status array resident on RC. */
	struct gxpci_queue_status_array *queue_sts_array;

	/* Mapped address to the C2C queue status array resident on RC. */
	struct gxpci_c2c_queue_sts_array *c2c_queue_sts_array;

	/* Chip-to-chip send queue state information. */
	struct gxpci_ep_c2c_state c2c_send[GXPCI_C2C_QUEUE_COUNT];

	/* Chip-to-chip receive queue state information. */
	struct gxpci_ep_c2c_state c2c_recv[GXPCI_C2C_QUEUE_COUNT];

	/* Pointers to the network interfaces per PCIe link. */
	struct gxpci_nic *net_devs[GXPCI_HOST_NIC_COUNT];

	struct tlr_stream *streams[NUM_CHAR_STREAMS];

        /* Pointers to the zero-copy H2T stream interfaces per PCIe link. */
        struct tlr_zc_stream *zc_h2t_streams[TILEPCI_NUM_ZC_H2T_CHAN];

        /* Pointers to the zero-copy T2H stream interfaces per PCIe link. */
        struct tlr_zc_stream *zc_t2h_streams[TILEPCI_NUM_ZC_T2H_CHAN];

	/* The MSI-X interrupt vector base number of the host NICs. */
	int msix_host_nic_intr_vec_base;

	/* The MSI-X interrupt vector base number of the character streams. */
	int msix_cs_q_intr_vec_base;

	/* The MSI-X interrupt vector base number of the zero-copy queues. */
	int msix_zc_q_intr_vec_base;

	/* MSI-X interrupt table base address. */
	void *msix_table_base;

	/* Number of MSI-X vectors. */
	unsigned int msix_vectors;

	/* mmap info. */
	int vmas;

	/* # of VMAs referencing us. */
	struct semaphore mutex;

	/* Number of VFs. */
	int num_vfs;

	/* BAR0 size per VF. */
	int vf_bar0_size;

	/* BAR2 size per VF. */
	int vf_bar2_size;

	/* Array of VF states. */
	struct gxpci_ep_vf_state vfs[GXPCI_MAX_NUM_VFS];

	/* This is used to protect the PIO region allocation and usage. */
	struct semaphore pio_sem;
};

#else

/* netdev ctrl dependencies exist for gxpci_host_shared.h */
#include "gxpci_host_netdev_ctrl.h"
#include "gxpci_host_shared.h"

#endif

#if defined(TILEPCI_HOST)
#if defined(TILEPCI_VF)
#define DRIVER_NAME_STRING "tilegxpci_vf"
#else
#define DRIVER_NAME_STRING "tilegxpci"
#endif
#else
#define DRIVER_NAME_STRING "tilegxpci_endp"
#endif

#define STRINGIFY(x) #x
#define TOSTRING(x)	 STRINGIFY(x)
#define MSG_LINE     "%s(%s:" TOSTRING(__LINE__) "-%d):"
#define SIMPLE_MSG_LINE    DRIVER_NAME_STRING "(" TOSTRING(__LINE__) "): "

#define TLR_TRACE(TYPE, HDR, FMT, ...)  printk(TYPE HDR MSG_LINE FMT,	\
					"PCI_DRV", __func__,	\
					smp_processor_id(),		\
					##__VA_ARGS__)
#define TLR_INFO(HDR, FMT, ...)  TLR_TRACE(KERN_INFO, HDR, FMT, ##__VA_ARGS__)

#ifdef DRV_TRACE_LEGACY
#define INT_TRACE(FMT, ...) TLR_INFO("*** ", FMT, ##__VA_ARGS__)
#define WRK_TRACE(FMT, ...) TLR_INFO("### ", FMT, ##__VA_ARGS__)
#define CMD_TRACE(FMT, ...) TLR_INFO("^^^ ", FMT, ##__VA_ARGS__)
#define FOP_TRACE(FMT, ...) TLR_INFO("--> ", FMT, ##__VA_ARGS__)
#define EX_TRACE(FMT, ...) TLR_INFO("<-- ", FMT, ##__VA_ARGS__)
#define TRACE(FMT, ...) TLR_INFO("TRC ", FMT, ##__VA_ARGS__)
#else
#define INT_TRACE(FMT, ...)
#define WRK_TRACE(FMT, ...)
#define CMD_TRACE(FMT, ...)
#define FOP_TRACE(FMT, ...)
#define EX_TRACE(FMT, ...)
#define TRACE(FMT, ...)
#endif

#ifdef DRV_TRACE_ZC
#define HID_INT_TRACE(FMT, ...) TLR_INFO("*** ", FMT, ##__VA_ARGS__)
#define HID_WRK_TRACE(FMT, ...) TLR_INFO("### ", FMT, ##__VA_ARGS__)
#define HID_CMD_TRACE(FMT, ...) TLR_INFO("^^^ ", FMT, ##__VA_ARGS__)
#define HID_FOP_TRACE(FMT, ...) TLR_INFO("--> ", FMT, ##__VA_ARGS__)
#define HID_EX_TRACE(FMT, ...)  TLR_INFO("<-- ", FMT, ##__VA_ARGS__)
#define HID_ERR_TRACE(FMT, ...) TLR_INFO("!!! ", FMT, ##__VA_ARGS__)
#else
#define HID_INT_TRACE(FMT, ...)
#define HID_WRK_TRACE(FMT, ...)
#define HID_CMD_TRACE(FMT, ...)
#define HID_FOP_TRACE(FMT, ...)
#define HID_EX_TRACE(FMT, ...)
#define HID_ERR_TRACE(FMT, ...)
#endif


#define INFO(FMT, ...) \
	printk(KERN_INFO SIMPLE_MSG_LINE FMT, ## __VA_ARGS__)
#define WARNING(FMT, ...) \
	printk(KERN_WARNING SIMPLE_MSG_LINE FMT, ## __VA_ARGS__)

#if defined(TILEPCI_ENDP)
#define ERR(FMT, ...)	printk(KERN_ERR SIMPLE_MSG_LINE FMT, ## __VA_ARGS__)
#else

#define ERR(FMT, ...) do {                                   \
  printk(KERN_ERR SIMPLE_MSG_LINE FMT, ## __VA_ARGS__);      \
} while (0)
#endif

static inline int tlr_zc_cmd_free(struct tlr_zc_cmd_q *q,
				     struct tlr_zc_cmd *cmd);

/* Function for GETTING and SETTING a STREAM object */
#define tlr_get_stream(tlr, idx)      ((tlr)->streams[idx])
#define tlr_set_stream(tlr, idx, str) ((tlr)->streams[idx] = (str))

/* Function for GETTING and SETTING a ZC H2T STREAM object */
#define tlr_get_zc_h2t_stream(tlr, idx)      \
	((tlr)->zc_h2t_streams[idx])
#define tlr_set_zc_h2t_stream(tlr, idx, str) \
	((tlr)->zc_h2t_streams[idx] = (str))

/* Function for GETTING and SETTING a ZC T2H STREAM object */
#define tlr_get_zc_t2h_stream(tlr, idx)      \
	((tlr)->zc_t2h_streams[idx])
#define tlr_set_zc_t2h_stream(tlr, idx, str) \
	((tlr)->zc_t2h_streams[idx] = (str))

/*
 * Set cmd->dma_addr, given a user-space buffer.  To do this, we need
 * to pin the user-space buffer into memory, verify that it's
 * contiguous in PA space, and map it to the PCI bus if necessary.
 */
static inline int
tlr_map_cmd(struct tlr_zc_cmd *cmd, unsigned long user_va, size_t size,
	    int writable)
{
	int ret = 0;
	unsigned long offset;
	unsigned long start_page_addr;
	unsigned long start_pfn;
	unsigned long end_pfn;
	unsigned long page_count;
	int nmapped_pages;
	int i;
#if defined(TILEPCI_ENDP)
	tlr_dma_addr_t va;
#endif /* defined(TILEPCI_ENDP)*/
	tlr_dma_addr_t prev_dma_addr;

	/* Determine which pages are being accessed and bump ref counts. */
	offset = user_va & ~PAGE_MASK;
	start_page_addr = user_va & PAGE_MASK;
	start_pfn = start_page_addr >> PAGE_SHIFT;
	end_pfn = (user_va + size - 1) >> PAGE_SHIFT;
	page_count = end_pfn - start_pfn + 1;

	if (page_count > MAX_MAPPED_PAGES_PER_XFER) {
		ret = -EINVAL;
		HID_ERR_TRACE("page_count %d MAX_MAPPED_PAGES_PER_XFER %d\n",
			      (int)page_count, (int)MAX_MAPPED_PAGES_PER_XFER);
		goto fail_params;
	}

	down_read(&current->mm->mmap_sem);
	/* We have to set writable so that all the pages are populated. */
	writable = 1;
	nmapped_pages = get_user_pages(current, current->mm,
				       start_page_addr,
				       page_count,
				       writable, 0,
				       &cmd->mapped_pages[0], NULL);
	up_read(&current->mm->mmap_sem);

	if (nmapped_pages < 0) {
		ret = nmapped_pages;
		HID_ERR_TRACE("get_user_pages %d\n", nmapped_pages);
		goto fail_get_pages;
	} else if (nmapped_pages == 0) {
		ret = -EINVAL;
		HID_ERR_TRACE("get_user_pages %d\n", nmapped_pages);
		goto fail_get_pages;
	}
	cmd->nmapped_pages = nmapped_pages;


#if defined(TILEPCI_HOST)
	/* Map the pages and verify that they're contiguous. */
	cmd->dma_addr = dma_map_page(&cmd->tlr->pci_dev->dev,
				     cmd->mapped_pages[0], 0, PAGE_SIZE,
				     cmd->dma_dir);
	prev_dma_addr = cmd->dma_addr;
	for (i = 1; i < nmapped_pages; i++) {
		tlr_dma_addr_t bus_addr;
		bus_addr = dma_map_page(&cmd->tlr->pci_dev->dev,
					cmd->mapped_pages[i], 0, PAGE_SIZE,
					cmd->dma_dir);
		if (bus_addr != prev_dma_addr + PAGE_SIZE) {
			dma_unmap_page(&cmd->tlr->pci_dev->dev, bus_addr,
				       PAGE_SIZE, cmd->dma_dir);

			for (bus_addr = cmd->dma_addr;
			     bus_addr < cmd->dma_addr + i * PAGE_SIZE;
			     bus_addr += PAGE_SIZE) {
				dma_unmap_page(&cmd->tlr->pci_dev->dev,
					       bus_addr, PAGE_SIZE,
					       cmd->dma_dir);
			}

			ret = -EINVAL;
			HID_ERR_TRACE("dma_map_page() not contiguous\n");
			for (i = 0; i < nmapped_pages; i++) {
				unsigned long long pa =
					page_to_phys(cmd->mapped_pages[i]);
				printk(KERN_INFO "  %llx\n", pa);
			}
			goto fail_contig;
		}
		prev_dma_addr = bus_addr;
	}
#else
	/* Just verify that the PAs are contiguous. */
	cmd->dma_addr = page_to_pa(cmd->mapped_pages[0]);
	prev_dma_addr = cmd->dma_addr;
	for (i = 1; i < nmapped_pages; i++) {
		tlr_dma_addr_t dma_addr = page_to_pa(cmd->mapped_pages[i]);
		if (dma_addr != prev_dma_addr + PAGE_SIZE) {
			ret = -EINVAL;
			HID_ERR_TRACE("non-contiguous PAs\n");
			goto fail_contig;
		}
		prev_dma_addr = dma_addr;
	}

	va = (tlr_dma_addr_t)page_address(cmd->mapped_pages[0]);
	cmd->dma_addr = va;
#endif /* defined(TILEPCI_HOST) */

	/* Finally, apply the offset. */
	cmd->dma_addr += offset;

	return 0;

 fail_contig:
	for (i = 0; i < cmd->nmapped_pages; i++)
		page_cache_release(cmd->mapped_pages[i]);
	cmd->nmapped_pages = 0;
 fail_get_pages:
 fail_params:
	return ret;
}

static inline int tlr_unmap_cmd(struct tlr_zc_cmd *cmd)
{
	int i;
	struct page *page;

	for (i = 0; i < cmd->nmapped_pages; i++) {
		page = cmd->mapped_pages[i];

		if (!PageReserved(page) && (cmd->dma_dir != DMA_TO_DEVICE))
			SetPageDirty(page);

		page_cache_release(page);
	}
	cmd->nmapped_pages = 0;

	return 0;
}

/*****************************************************************************
 * LIST MANUPULATION FUNCTIONS
 *
 *    This driver uses the kernel CIRCULAR LINKED LIST structure.  In this
 *  implementation, an empty list head points to itself, while a single item
 *  list will have its next and prev point to the same other object.
 *
 *    the struct tlr_list object is a standard list_head, with an additional
 *  length field.
 *
 *  tlr_list_splice_tail(*cur_list, *new_list, *loc) - will acquire the spin
 *    lock then move the items from new_list into cur_list, empty new_list,
 *    and unlock.
 *  tlr_list_add_zc_cmd(*list, *cmd, *lock) - will acquire the spin lock,
 *    then move the cmd into the list specified list.
 *  tlr_alloc_blk_mem(tlr, size, *addr) - allocate a block of DMA visible
 *    memory
 *  tlr_free_blk_mem(tlr, size, *kern_addr, dma_addr - free a block of DMA
 *    memory
 ****************************************************************************/

static inline int tlr_list_init(struct tlr_list *l)
{
	INIT_LIST_HEAD(&l->q);
	l->len = 0;

	return 0;
}

static inline int __tlr_list_splice_tail(struct tlr_list *cur_l,
					 struct tlr_list *new_l)
{
	struct list_head	*cur;
	struct list_head	*new;
	struct list_head	*new_first;
	struct list_head	*new_last;
	struct list_head	*cur_last;

	cur = &cur_l->q;
	new = &new_l->q;

	/*
	 * check if we're splicing an empty list to the tail
	 * of the new list
	 */

	new_first = new->next;
	if (new_first != new) {
		new_last = new->prev;
		cur_last = cur->prev;

		/*
		 * First Adjust the tail of the current list
		 */
		new_first->prev = cur_last;
		cur_last->next = new_first;

		/*
		 * Then adjust the head of the current list
		 */
		new_last->next = cur;
		cur->prev = new_last;

		/*
		 * Finally, adjust the length of the list
		 */
		cur_l->len += new_l->len;
	}

	/* Empty the old list */
	tlr_list_init(new_l);

	return 0;
}

static inline int tlr_list_splice_tail(struct tlr_list *cur_l,
				       struct tlr_list *new_l,
				       spinlock_t *lock)
{
	int		 ret;
	unsigned long	 lock_flags;

	spin_lock_irqsave(lock, lock_flags);
	ret = __tlr_list_splice_tail(cur_l, new_l);
	spin_unlock_irqrestore(lock, lock_flags);

	return ret;
}

static inline int __tlr_list_add_zc_cmd(struct tlr_list		*l,
					struct tlr_zc_cmd	*cmd)
{
	int	 ret;

	ret = 0;

	list_add_tail(&cmd->list, &l->q);
	l->len++;

	return ret;
}

static inline int tlr_list_add_zc_cmd(struct tlr_list		*l,
				      struct tlr_zc_cmd		*cmd,
				      spinlock_t		*lock)
{
	int		 ret;
	unsigned long	 lock_flags;

	spin_lock_irqsave(lock, lock_flags);
	ret = __tlr_list_add_zc_cmd(l, cmd);
	spin_unlock_irqrestore(lock, lock_flags);

	return ret;
}

static inline struct tlr_zc_cmd *__tlr_list_rem_zc_cmd(struct tlr_list *l)
{
	struct tlr_zc_cmd	*ret;
	struct list_head	*entry;

	ret = NULL;

	if (list_empty(&l->q))
		goto exit;

	entry = l->q.next;
	list_del(entry);
	ret = list_entry(entry, struct tlr_zc_cmd, list);
	l->len--;

 exit:
	return ret;
}

static inline struct tlr_zc_cmd *tlr_list_rem_zc_cmd(struct tlr_list	*l,
						     spinlock_t		*lock)
{
	struct tlr_zc_cmd	*ret;
	unsigned long		 lock_flags;

	spin_lock_irqsave(lock, lock_flags);
	ret = __tlr_list_rem_zc_cmd(l);
	spin_unlock_irqrestore(lock, lock_flags);

	return ret;
}

static inline int tlr_list_free_zc_cmds(struct tlr_zc_cmd_q *q,
					struct tlr_list	*l,
					spinlock_t	*lock)
{
	int			 ret;
	struct tlr_zc_cmd	*cmd;
	unsigned long		 lock_flags;

	ret = 0;
	spin_lock_irqsave(lock, lock_flags);
	while (l->len > 0) {
		cmd = __tlr_list_rem_zc_cmd(l);
		tlr_zc_cmd_free(q, cmd);
	}
	spin_unlock_irqrestore(lock, lock_flags);

	return ret;
}

static inline int __tlr_list_len(struct tlr_list *l)
{
	int	 ret;

	ret = l->len;

	return ret;
}

static inline int tlr_list_len(struct tlr_list *l, spinlock_t *lock)
{
	int		 ret;
	unsigned long	 lock_flags;

	spin_lock_irqsave(lock, lock_flags);
	ret = __tlr_list_len(l);
	spin_unlock_irqrestore(lock, lock_flags);

	return ret;
}

static inline int __tlr_list_empty(struct tlr_list *l)
{
	int	 ret;
	int	 len;

	len = l->len;
	ret = len == 0;
	return ret;
}

static inline int tlr_list_empty(struct tlr_list *l, spinlock_t *lock)
{
	int	 ret;
	int	 len;

	len = tlr_list_len(l, lock);
	ret = len == 0;
	return ret;
}

static inline int tlr_zc_cmd_init(struct tlr_zc_cmd		*cmd,
				  struct tlr_zc_cmd_q		*q,
				  struct tlr_pcie_dev		*tlr,
				  enum tlr_zc_cmd_type_e	 type,
				  tilepci_cookie_t		 cookie,
				  struct tlr_list		*list,
				  spinlock_t			*lock)
{
	int	 ret;

	ret = 0;

	memset(cmd, 0, sizeof(*cmd));
	cmd->tlr = tlr;
	cmd->cmd_q = q;
	cmd->type = type;
	cmd->state = TLR_ZC_CMD_ST_FREE;
	cmd->cookie = cookie;
	cmd->flags = 0;
	cmd->nmapped_pages = 0;
	cmd->dma_addr = 0;

	switch (cmd->type) {
#if defined(TILEPCI_HOST)
	case TLR_ZC_CMD_H2T:
		cmd->dma_dir = DMA_TO_DEVICE;
		break;
	case TLR_ZC_CMD_T2H:
		cmd->dma_dir = DMA_FROM_DEVICE;
		break;
	default:
		cmd->dma_dir = DMA_BIDIRECTIONAL;
		break;
#elif defined(TILEPCI_ENDP)
	case TLR_ZC_CMD_H2T:
		cmd->dma_dir = DMA_FROM_DEVICE;
		break;
	case TLR_ZC_CMD_T2H:
		cmd->dma_dir = DMA_TO_DEVICE;
		break;
	default:
		cmd->dma_dir = DMA_BIDIRECTIONAL;
		break;
#else
#error Undefined Architecture
#endif
	}

	if (lock == NULL)
		__tlr_list_add_zc_cmd(list, cmd);
	else
		tlr_list_add_zc_cmd(list, cmd, lock);

	return ret;
}

static inline int tlr_zc_cmd_release(struct tlr_zc_cmd *cmd)
{
	return tlr_unmap_cmd(cmd);
}

static inline int tlr_zc_cmd_free(struct tlr_zc_cmd_q *q,
				  struct tlr_zc_cmd   *cmd)
{
	int		 	 ret;
	struct tlr_pcie_dev	*tlr;
	struct tlr_zc_stream 	*stream;
	
	ret = 0;
	tlr = cmd->tlr;

	if (cmd->type == TLR_ZC_CMD_H2T)
		stream = tlr_get_zc_h2t_stream(tlr, q->chan);
	else
		stream = tlr_get_zc_t2h_stream(tlr, q->chan);
	
	tlr_zc_cmd_release(cmd);
	
	tlr_list_add_zc_cmd(&stream->cmd_q_free_list, cmd,
	  		    &stream->cmd_q_lock);

	return ret;
}

#if 0
static inline int tlr_zc_cmd_new(enum tlr_zc_cmd_type_e	 type,
				 tilepci_cookie_t	 cookie,
				 struct tlr_zc_cmd_q	*q)
{
	int			 ret;
	struct tlr_zc_cmd	*cmd;
	struct tlr_pcie_dev		*tlr;

	ret = 0;
	tlr = q->tlr;

	cmd = tlr_list_rem_zc_cmd(&tlr->zc_state.cmd_q_free_list,
				  &tlr->zc_state.cmd_q_lock);
	if (cmd == NULL)
		goto exit;

	ret = tlr_zc_cmd_init(cmd, q, tlr, type, cookie,
			      &q->free_q, &q->lock);

 exit:
	return ret;
}
#endif // 0

static inline int __tlr_zc_cmd_q_free_xfers(struct tlr_zc_cmd_q *q)
{
	int	 ret;

	ret = 0;

	if (q->rd_xfer_comps != NULL) {
		kfree(q->rd_xfer_comps);
		q->rd_xfer_comps = NULL;
	}
	if (q->wr_xfer_reqs != NULL) {
		kfree(q->wr_xfer_reqs);
		q->wr_xfer_reqs = NULL;
	}

	return ret;
}

static inline int __tlr_zc_cmd_q_alloc_xfers(struct tlr_zc_cmd_q *q)
{
	int		 ret;
	uint32_t	 rd_xfer_sz;
	uint32_t	 wr_xfer_sz;

	ret = 0;
	rd_xfer_sz = sizeof(*q->rd_xfer_comps) * q->ncmd_wanted;
	wr_xfer_sz = sizeof(*q->wr_xfer_reqs) * q->ncmd_wanted;

	__tlr_zc_cmd_q_free_xfers(q);

	if (rd_xfer_sz != 0) {
		q->rd_xfer_comps = kmalloc(rd_xfer_sz, GFP_KERNEL);
		if (q->rd_xfer_comps == NULL) {
			ret = -ENOMEM;
			goto err_exit;
		}
	}

	if (wr_xfer_sz != 0) {
		q->wr_xfer_reqs = kmalloc(wr_xfer_sz, GFP_KERNEL);
		if (q->wr_xfer_reqs == NULL) {
			ret = -ENOMEM;
			goto err_exit;
		}
	}

	return ret;

 err_exit:
	__tlr_zc_cmd_q_free_xfers(q);
	return ret;
}

/*
 * Release command queue resources.  This should only be called when
 * the chip resets.
 */
static inline int tlr_zc_cmd_q_free(struct tlr_zc_cmd_q *q)
{
	int ret = 0;

	q->state = CMD_Q_ST_CHIP_DOWN;
	q->is_ready = FALSE;
	q->ncmd_wanted = 0;
	q->ncmd_have = 0;

	tlr_list_free_zc_cmds(q, &q->free_q, &q->lock);
	tlr_list_free_zc_cmds(q, &q->pend_q, &q->lock);
	tlr_list_free_zc_cmds(q, &q->post_q, &q->lock);
	tlr_list_free_zc_cmds(q, &q->comp_q, &q->lock);

	wmb();  /* ensure visibility before waking others */
	wake_up_interruptible(&q->free_queue);
	wake_up_interruptible(&q->reset_drain_queue);
	wake_up_interruptible(&q->comp_queue);

	__tlr_zc_cmd_q_free_xfers(q);

	return ret;
}

/*
 * Initialize a command queue after chip reset.  This should only be
 * called on queues which have either just come up via
 * tlr_zc_cmd_q_startup(), or on queues which were cleaned up after a
 * chip reset via tlr_zc_cmd_q_free().
 */
static inline int tlr_zc_cmd_q_init(struct tlr_zc_cmd_q	*q)
{
	int ret = 0;

	q->ncmd_wanted = 0;
	q->ncmd_have = 0;
	q->ncomp_tot = 0;
	q->nreq_tot = 0;
	q->ncmd_min_read = 1;
	q->ncmd_comp_wakeup = 1;
	q->rd_xfer_comps = NULL;
	q->wr_xfer_reqs = NULL;
	q->ncomp_mismatch = 0;

	tlr_list_init(&q->free_q);
	tlr_list_init(&q->pend_q);
	tlr_list_init(&q->post_q);
	tlr_list_init(&q->comp_q);

	wmb();  /* ensure visibility before starting up */

	q->state = CMD_Q_ST_NEED_SOC;
	q->is_ready = TRUE;

#if defined(TILEPCI_ENDP)
	__insn_mf();
#endif /* defined(TILEPCI_ENDP) */ 
	
	return ret;
}

/*
 * Constructor for command queues.  This should be called once for
 * each queue when the driver initializes.
 */
static inline int tlr_zc_cmd_q_startup(struct tlr_zc_cmd_q *q,
				       enum tlr_zc_cmd_type_e type,
				       struct tlr_pcie_dev *tlr,
				       uint32_t	index)
{
	int	 ret;

	ret = 0;
	memset(q, 0, sizeof(*q));

	sema_init(&q->mmap_state.mutex, 1);

	q->type = type;
	q->chan = index;
	q->tlr = tlr;
	q->state = CMD_Q_ST_CHIP_DOWN;
	q->ncmd_min_read = 1;
	q->ncmd_comp_wakeup = 1;
	q->irq_num = -1;
	wmb();  /* ensure visibility before starting up */

	sema_init(&q->rd_xfer_mutex, 1);
	sema_init(&q->wr_xfer_mutex, 1);
	init_waitqueue_head(&q->free_queue);
	init_waitqueue_head(&q->reset_drain_queue);
	init_waitqueue_head(&q->comp_queue);
	spin_lock_init(&q->lock);

	return ret;
}

static inline int tlr_zc_cmd_q_get_free(struct tlr_zc_cmd_q *q,
					int wait,
					struct tlr_zc_cmd **ret_cmd)
{
	int			 ret;
	struct tlr_zc_cmd	*cmd;

	cmd = NULL;
	ret = 0;

	for (cmd = tlr_list_rem_zc_cmd(&q->free_q, &q->lock);
	     cmd == NULL;
	     cmd = tlr_list_rem_zc_cmd(&q->free_q, &q->lock)) {
		if (!wait) {
			ret = -EAGAIN;
			goto exit;
		}

		ret = wait_event_interruptible(q->free_queue,
				       !tlr_list_empty(&q->free_q, &q->lock) ||
				       q->chip_reset_poison);
		if (ret != 0) {
			ret = -ERESTARTSYS;
			HID_ERR_TRACE("Exit  -ERESTARTSYS\n");
			goto exit;
		}

		HID_FOP_TRACE("Woke from queue\n");

		/* We could have been poisoned while sleeping. */
		if (q->chip_reset_poison) {
			ret = -ENXIO;
			goto exit;
		}
	}

 exit:
	if (cmd != NULL) {
		cmd->nmapped_pages = 0;
		cmd->dma_addr = 0;
	}
	*ret_cmd = cmd;

	return ret;
}

static inline int tlr_zc_cmd_q_get_comp(struct tlr_zc_cmd_q *q,
					int wait,
					uint32_t comp_wakeup,
					struct tlr_zc_cmd **ret_cmd)
{
	int			 ret;
	struct tlr_zc_cmd	*cmd;

	cmd = NULL;
	ret = 0;

	for (cmd = tlr_list_rem_zc_cmd(&q->comp_q, &q->lock);
	     cmd == NULL;
	     cmd = tlr_list_rem_zc_cmd(&q->comp_q, &q->lock)) {
		if (!wait) {
			ret = -EAGAIN;
			goto exit;
		}

		q->ncmd_comp_wakeup = (comp_wakeup > q->ncmd_min_read) ?
			q->ncmd_min_read : comp_wakeup;
		ret = wait_event_interruptible(q->comp_queue,
			       ((tlr_list_len(&q->comp_q, &q->lock) > 0) ||
				(q->chip_reset_poison)));
		if (ret != 0) {
			ret = -ERESTARTSYS;
			HID_ERR_TRACE("Exit  -ERESTARTSYS\n");
			goto exit;
		}

		HID_FOP_TRACE("Woke from queue\n");

		/* We could have been poisoned while sleeping. */
		if (q->chip_reset_poison) {
			ret = -ENXIO;
			HID_ERR_TRACE("Exit -ENXIO\n");
			goto exit;
		}
	}

 exit:
	*ret_cmd = cmd;

	return ret;
}

static inline int __tlr_zc_cmd_q_put_free(struct tlr_zc_cmd_q *q,
					  struct tlr_zc_cmd *cmd)
{
	int		 ret;

	ret = 0;

	if (q->ncmd_wanted < q->ncmd_have) {
		tlr_zc_cmd_free(q, cmd);
		q->ncmd_have--;
	} else {
		tlr_zc_cmd_release(cmd);
		ret = __tlr_list_add_zc_cmd(&q->free_q, cmd);

		wmb();  /* ensure visibility before waking others */
		wake_up_interruptible(&q->free_queue);
	}

	return ret;
}

static inline int tlr_zc_cmd_q_put_free(struct tlr_zc_cmd_q *q,
					struct tlr_zc_cmd *cmd)
{
	int		 ret;
	unsigned long	 lock_flags;

	spin_lock_irqsave(&q->lock, lock_flags);
	ret = __tlr_zc_cmd_q_put_free(q, cmd);
	spin_unlock_irqrestore(&q->lock, lock_flags);
	return ret;
}

static inline int __tlr_zc_cmd_q_free_list(struct tlr_zc_cmd_q *q,
					   struct tlr_list *l)
{
	int			 ret;
	uint32_t		 l_len;
	int			 i;
	struct tlr_zc_cmd	*cmd;

	ret = 0;

	l_len = __tlr_list_len(l);
	for (i = 0; i < l_len; i++) {
		cmd = __tlr_list_rem_zc_cmd(l);
		__tlr_zc_cmd_q_put_free(q, cmd);
	}

	return ret;
}

static inline int tlr_zc_cmd_q_free_list(struct tlr_zc_cmd_q *q,
					 struct tlr_list *l)
{
	int		 ret;
	unsigned long	 lock_flags;

	spin_lock_irqsave(&q->lock, lock_flags);
	ret = __tlr_zc_cmd_q_free_list(q, l);
	spin_unlock_irqrestore(&q->lock, lock_flags);
	return ret;
}

static inline int __tlr_zc_cmd_q_pend_list(struct tlr_zc_cmd_q *q,
					   struct tlr_list *l)
{
	int		 ret;

	ret = 0;

	ret = __tlr_list_splice_tail(&q->pend_q, l);

	return ret;
}

static inline int tlr_zc_cmd_q_pend_list(struct tlr_zc_cmd_q *q,
					 struct tlr_list *l)
{
	int		 ret;

	ret = 0;

	ret = tlr_list_splice_tail(&q->pend_q, l, &q->lock);

	return ret;
}


#if defined(TILEPCI_HOST)
static inline int __tlr_zc_cmd_q_comp(struct tlr_zc_cmd_q *q,
				      pcie_host_completion_t *cmp,
				      uint32_t comp_len)
#elif defined(TILEPCI_ENDP)
static inline int __tlr_zc_cmd_q_comp(struct tlr_zc_cmd_q *q,
				      pcie_tile_completion_t *cmp,
				      uint32_t comp_len)
#else
#error Undefined Architecture
#endif
{
	int			 ret;
	struct tlr_zc_cmd	*cmd;

	ret = 0;

	cmd = __tlr_list_rem_zc_cmd(&q->post_q);
	if (cmd == NULL) {
		ret = -ENOBUFS;
		HID_ERR_TRACE("Exit ENOBUFS\n");
		goto exit;
	}

	tlr_unmap_cmd(cmd);

	if (q->state == CMD_Q_ST_RESET_DISCARD) {
		/*
		 * In discard mode, we simply free commands as they
		 * come back.  This is essentially what would happen
		 * if we ran the non-discard completion code and then
		 * returned the completion via the read() syscall.
		 */
		__tlr_zc_cmd_q_put_free(q, cmd);
		q->ncomp_tot++;
		q->ncomp_bytes_tot += comp_len;

		/*
		 * If we're discarding, we need to wake up any waiters
		 * when the last completion arrives.
		 */
		if (q->ncomp_tot == q->nreq_tot) {
			wmb();  /* ensure visibility before waking others */
			wake_up_interruptible(&q->reset_drain_queue);
		}
		goto exit;
	}

	cmd->comp_len = comp_len;
	cmd->cookie = cmp->tag;
	cmd->flags = 0;
	if (cmp->eop)
		cmd->flags |= TILEPCI_CPL_EOP;
	if (cmp->overflow)
		cmd->flags |= TILEPCI_CPL_OVERFLOW;
	if (cmp->reset) {
		/*
		 * If we thought things were happily connected, note
		 * that reset was initiated by the other side.
		 */
		if (q->state == CMD_Q_ST_CONNECTED)
			q->state = CMD_Q_ST_RESET_STARTED;
		cmd->flags |= TILEPCI_CPL_RESET;
	}

	__tlr_list_add_zc_cmd(&q->comp_q, cmd);
	if (__tlr_list_len(&q->comp_q) >= q->ncmd_comp_wakeup) {
		wmb();  /* ensure visibility before waking others */
		wake_up_interruptible(&q->comp_queue);
	}

	q->ncomp_cur++;
	q->ncomp_tot++;
	q->ncomp_bytes_cur += comp_len;
	q->ncomp_bytes_tot += comp_len;

 exit:
	return ret;
}

#if defined(TILEPCI_HOST)
static inline int tlr_zc_cmd_q_comp(struct tlr_zc_cmd_q *q,
				    pcie_host_completion_t *cmp,
				    uint32_t comp_len)
#elif defined(TILEPCI_ENDP)
static inline int tlr_zc_cmd_q_comp(struct tlr_zc_cmd_q *q,
				    pcie_tile_completion_t *cmp,
				    uint32_t comp_len)
#else
#error Undefined Architecture
#endif
{
	int		 ret;
	unsigned long	 lock_flags;

	if (q == NULL) {
		ERR("NULL ZC queue for stream %d\n", q->chan);
		return -ENXIO;
	}

	spin_lock_irqsave(&q->lock, lock_flags);
	ret = __tlr_zc_cmd_q_comp(q, cmp, comp_len);
	spin_unlock_irqrestore(&q->lock, lock_flags);

	return ret;
}


/* generic file ops */
int tlr_zc_init(struct tlr_pcie_dev *tlr);
void tlr_zc_free(struct tlr_pcie_dev *tlr);
void tlr_zc_chip_reset(struct tlr_pcie_dev *tlr);

int tlr_zc_open(struct inode *inode, struct file *filp);
void release_zc_semaphores(struct tlr_pcie_dev *tlr, int count);
int grab_all_zc_semaphores(struct tlr_pcie_dev *tlr);

#if GXPCI_HOST_ZC_QUEUE_COUNT
#if defined(TILEPCI_ENDP)
extern void gxpci_zc_check_tile_cmds(struct tlr_zc_stream *stream);
extern void gxpci_zc_parse_host_cmds(struct tlr_zc_stream *stream);
extern void gxpci_zc_process_host_cmds(struct tlr_zc_stream *stream);
extern void gxpci_zc_update_host(struct tlr_zc_stream *stream); 
extern void gxpci_zc_intr_timer_handler(unsigned long arg);
#else
extern void tlr_handle_zc_completions(struct tlr_zc_stream* stream);
#endif /* defined(TILEPCI_ENDP) */
#endif

/*
 * Compatibility defines for dealing with the different page faulting
 * paths before and after 2.6.18.
 */
/* Note: avoid KERNEL_VERSION() to avoid warnings from "sunifdef" */
#if LINUX_VERSION_CODE > 0x020612  /* 2.6.18 */
#define USE_VM_FAULT
#define RETURN_SIGBUS VM_FAULT_SIGBUS
#define RETURN_OOM VM_FAULT_OOM
#else
#define RETURN_SIGBUS NOPAGE_SIGBUS
#define RETURN_OOM NOPAGE_OOM
#endif

/* Backwards compatibility with host-side PCI driver. */
typedef struct tlr_zc_cmd tlr_zc_cmd_t;
typedef struct tlr_zc_cmd_q tlr_zc_cmd_q_t;
typedef struct tlr_stream tlr_stream_t;
typedef struct tlr_zc_stream tlr_zc_stream_t;
typedef struct tlr_pcie_dev tlr_pcie_dev_t;

#endif /* !__TILEGXPCI_SHARED_CODE_H__ */
