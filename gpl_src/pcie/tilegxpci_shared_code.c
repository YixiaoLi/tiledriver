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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/pci.h>
#include <linux/gfp.h>
#include <linux/version.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>

#if defined(TILEPCI_HOST)
#include "tilegxpci.h"
#elif defined(TILEPCI_ENDP)
#include <linux/irq.h>
#include <asm/compat.h>
#include <asm/tilegxpci.h>
#include <asm/homecache.h>
#include "gxpci_endp.h"
#else
#error You must define either TILEPCI_HOST or TILEPCI_ENDP
#endif
#include "tilegxpci_shared_code.h"
#include "tilegxpci_host.h"


static const char driver_name[] = DRIVER_NAME_STRING;


#if GXPCI_HOST_ZC_QUEUE_COUNT
#if defined(TILEPCI_HOST)
/*
 * gxpci_irq_enable - Enable interrupt generation
 */
static inline void gxpci_irq_enable(struct tlr_zc_stream* stream)
{
       if (stream->cmd_queue->type == TLR_ZC_CMD_H2T)
               writel(1, &stream->h2t_regs->interrupt_enable);
       else
               writel(1, &stream->t2h_regs->interrupt_enable);
}


/*
 * gxpci_irq_disable - Mask off interrupt generation on the endpoint
 */
static inline void gxpci_irq_disable(struct tlr_zc_stream* stream)
{
       if (stream->cmd_queue->type == TLR_ZC_CMD_H2T)
               writel(0, &stream->h2t_regs->interrupt_enable);
       else
               writel(0, &stream->t2h_regs->interrupt_enable);
}
#endif

#if defined(TILEPCI_HOST)
static irqreturn_t
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
zc_irq_handler(int irq, void* dev, struct pt_regs* pt_regs)
#else
zc_irq_handler(int irq, void* dev)
#endif
{
        struct tlr_zc_stream* stream = (struct tlr_zc_stream*) dev;

        gxpci_irq_disable(stream);

       	tlr_handle_zc_completions(stream);

        gxpci_irq_enable(stream);

	return IRQ_HANDLED;
}

/*
 * gxpci_request_irq - initialize interrupts
 *
 * Attempts to configure interrupts using the best available
 * capabilities of the hardware and kernel, i.e. MSI-X or MSI.
 */
static int gxpci_request_irq(struct tlr_zc_stream *stream)
{
	struct tlr_pcie_dev *tlr = stream->tlr;
	int msix_table_index;
	int err;

	if (tlr->msix_vectors) {
		snprintf(stream->name, sizeof(stream->name), "%s:zc%d", 
			 driver_name, stream->index);

		msix_table_index = tlr->msix_zc_q_intr_vec_base + stream->index;

#ifdef GXPCI_INTR_VECTOR_PER_QUEUE
		if (stream->cmd_queue->type == TLR_ZC_CMD_H2T)
			msix_table_index += GXPCI_HOST_ZC_QUEUE_COUNT;

                err = request_irq(tlr->msix_entries[msix_table_index].vector,
                                  zc_irq_handler, 0, stream->name, stream);
#else
		err = request_irq(tlr->msix_entries[msix_table_index].vector,
				  zc_irq_handler, IRQF_SHARED, stream->name, 
				  stream);
#endif 
		if (err) {
			ERR("%s: MSI-X gxpci_request_irq failure: %d\n",
			    stream->name, err);
		}
		else
			stream->cmd_queue->irq_num = 
				tlr->msix_entries[msix_table_index].vector;
	} 
	else {
		err = request_irq(tlr->pci_dev->irq, zc_irq_handler, 
				  IRQF_SHARED, driver_name, stream);
		if (err) {
			ERR("%s: MSI gxpci_request_irq failure: %d\n",
			    driver_name, err);
		}
		else
			stream->cmd_queue->irq_num = 
				tlr->pci_dev->irq;
	}

	return err;
}


static void gxpci_free_irq(struct tlr_zc_stream* stream)
{
	struct tlr_pcie_dev *tlr = stream->tlr;
	int msix_table_index;

	if (tlr->msix_vectors) {
		msix_table_index =
			tlr->msix_zc_q_intr_vec_base + stream->index;
#ifdef GXPCI_INTR_VECTOR_PER_QUEUE
                if (stream->cmd_queue->type == TLR_ZC_CMD_H2T)
                        msix_table_index += GXPCI_HOST_ZC_QUEUE_COUNT;
#endif
		free_irq(tlr->msix_entries[msix_table_index].vector, stream);
	} else {
		free_irq(tlr->pci_dev->irq, stream);
	}
	
	stream->cmd_queue->irq_num = -1;
}
#endif /* defined(TILEPCI_HOST) */

#if defined(TILEPCI_ENDP) 
/* Initialize the queue state. */
static int gxpci_queue_state_init(struct tlr_zc_stream *stream,
				  void *backing_mem)
{
	struct gxpci_queue_state *queue_state;
	size_t line_size;
	size_t dma_buff_size; 
	int i;

	/* 
	 * Allocate buffer for DMAing commands and completions from the
	 * host; these must be cacheline aligned and padded.
	 */
	line_size = (1 << CHIP_L2_LOG_LINE_SIZE());
	dma_buff_size = sizeof(*queue_state->host_cmd_dma_buffer) * 
		PCIE_CMD_QUEUE_ENTRIES;
  	dma_buff_size = ROUND_UP(dma_buff_size, line_size);

	if (dma_buff_size > GXPCI_HOST_CMDS_COMP_BUF_SIZE_LIMIT) 
		return -ENXIO;
	
	/* H2T queue state init. */
	if (stream->cmd_queue->type == TLR_ZC_CMD_H2T) {
		queue_state = &stream->h2t_queue_state;
		queue_state->host_cmd_dma_buffer = backing_mem + 
			GXPCI_H2T_HOST_CMD_DMA_BUF_OFFSET;

  		queue_state->host_cmds_posted = 0;
		queue_state->host_cmds_dma_started = 0;
  		queue_state->host_cmds_consumed = 0;
		queue_state->fetch_dmas_started = 0;
		queue_state->fetch_dmas_completed = 0;
		queue_state->host_cmds_cnt_array = backing_mem + 
			GXPCI_H2T_CMDS_CNT_ARRAY_OFFSET;
  		queue_state->host_cpl_dma_buffer = backing_mem + 
			GXPCI_H2T_HOST_COMPS_DMA_BUF_OFFSET;
  
		queue_state->host_cpl_posted = 0;
		queue_state->host_cpl_dma_started = 0;
	  	queue_state->host_cpl_consumed = 0;
		queue_state->comp_dmas_started = 0;
		queue_state->comp_dmas_completed = 0;
		queue_state->host_cpls_cnt_array = backing_mem + 
			GXPCI_H2T_CPLS_CNT_ARRAY_OFFSET;

	     	queue_state->host_cmd_bus_addr =
			stream->h2t_buf_cmd_array_addr;
    		queue_state->host_cpl_bus_addr =
      			stream->h2t_cmpl_array_addr;

		queue_state->tile_cmd_buffer = stream->h2t_buffer_cmd_array;
	
		queue_state->tile_cmds_consumed = 0; 
		queue_state->tile_cmds_posted = 0; 

  		/* Build a free list of tile_cmd_t structures. */
	  	queue_state->tile_cmd_array = (tile_cmd_t *)(backing_mem + 
	  		GXPCI_H2T_TILE_CMD_ARRAY_BUF_OFFSET);

		for (i = 0; i < PCIE_CMD_QUEUE_ENTRIES; i++) {
    			tile_cmd_t *cmd = queue_state->tile_cmd_array + i;
    			cmd->next_index = i + 1;
  		}
	  	queue_state->tile_cmd_array[i - 1].next_index = TILE_CMD_NULL;
  		queue_state->free_tile_cmds = queue_state->tile_cmd_array;
	
	  	queue_state->tile_cpl_buffer = backing_mem + 
			GXPCI_H2T_TILE_COMP_ARRAY_BUF_OFFSET;
  		queue_state->tile_cpl_posted = 0;
	  	queue_state->tile_cpl_consumed = 0;

  		/* Build a free list of host_cmd_t structures. */
 		queue_state->host_cmd_array = (host_cmd_t *)(backing_mem +
                	GXPCI_H2T_HOST_CMD_ARRAY_BUF_OFFSET);
	  	for (i = 0; i < PCIE_CMD_QUEUE_ENTRIES; i++) {
    			host_cmd_t *cmd = queue_state->host_cmd_array + i;
    			cmd->next_index = i + 1;
  		}
  		queue_state->host_cmd_array[i - 1].next_index = HOST_CMD_NULL;
  		queue_state->free_host_cmds = queue_state->host_cmd_array;

  		/*
  	 	 * Build a free list of dma_cmd_t structures; note that we skip 
  	 	 * the first entry in the array so that we can use 0 as an 
  	 	 * end-of-list flag.
  	 	 */
	  	queue_state->dma_cmd_array = (dma_cmd_t *)(backing_mem + 
  			GXPCI_H2T_DMA_CMD_ARRAY_BUF_OFFSET);
  		for (i = 1; i < PCIE_CMD_QUEUE_ENTRIES * 2; i++) {
    			dma_cmd_t *cmd = queue_state->dma_cmd_array + i;
	    		cmd->next_index = i + 1;
  		}
	  	queue_state->dma_cmd_array[i - 1].next_index = DMA_CMD_NULL;
  		queue_state->free_dma_cmds = queue_state->dma_cmd_array + 1;
	}
	else { 
		/* 
		 * T2H queue state init, where the memory offsets are identical
		 * to H2T, since H2T and T2H channels use different memory 
		 * pages.
		 */
		queue_state = &stream->t2h_queue_state;
		queue_state->host_cmd_dma_buffer = backing_mem + 
			GXPCI_H2T_HOST_CMD_DMA_BUF_OFFSET;

	  	queue_state->host_cmds_posted = 0;
  		queue_state->host_cmds_consumed = 0;
		queue_state->host_cmds_dma_started = 0;
		queue_state->fetch_dmas_started = 0;
		queue_state->fetch_dmas_completed = 0;
        	queue_state->host_cmds_cnt_array = backing_mem +
                	GXPCI_H2T_CMDS_CNT_ARRAY_OFFSET;
	  	queue_state->host_cpl_dma_buffer = backing_mem + 
			GXPCI_H2T_HOST_COMPS_DMA_BUF_OFFSET;
  	
		queue_state->host_cpl_posted = 0;
  		queue_state->host_cpl_consumed = 0;
		queue_state->host_cpl_dma_started = 0;
		queue_state->comp_dmas_started = 0;
		queue_state->comp_dmas_completed = 0;
        	queue_state->host_cpls_cnt_array = backing_mem +
                	GXPCI_H2T_CPLS_CNT_ARRAY_OFFSET;

	     	queue_state->host_cmd_bus_addr =
			stream->t2h_buf_cmd_array_addr;
	    	queue_state->host_cpl_bus_addr =
      			stream->t2h_cmpl_array_addr;

		queue_state->tile_cmd_buffer = stream->t2h_buffer_cmd_array;

		queue_state->tile_cmds_consumed = 0;
		queue_state->tile_cmds_posted = 0; 

	  	/* Build a free list of tile_cmd_t structures. */
  		queue_state->tile_cmd_array = (tile_cmd_t *)(backing_mem + 
  			GXPCI_H2T_TILE_CMD_ARRAY_BUF_OFFSET);

		for (i = 0; i < PCIE_CMD_QUEUE_ENTRIES; i++) {
    			tile_cmd_t *cmd = queue_state->tile_cmd_array + i;
    			cmd->next_index = i + 1;
	  	}
  		queue_state->tile_cmd_array[i - 1].next_index = TILE_CMD_NULL;
  		queue_state->free_tile_cmds = queue_state->tile_cmd_array;
		
	  	queue_state->tile_cpl_buffer = backing_mem + 
			GXPCI_H2T_TILE_COMP_ARRAY_BUF_OFFSET;
	  	queue_state->tile_cpl_posted = 0;
  		queue_state->tile_cpl_consumed = 0;

	  	/* Build a free list of host_cmd_t structures. */
  		queue_state->host_cmd_array = (host_cmd_t *)(backing_mem +
                	GXPCI_H2T_HOST_CMD_ARRAY_BUF_OFFSET);
	  	for (i = 0; i < PCIE_CMD_QUEUE_ENTRIES; i++) {
    			host_cmd_t *cmd = queue_state->host_cmd_array + i;
    			cmd->next_index = i + 1;
	  	}
  		queue_state->host_cmd_array[i - 1].next_index = HOST_CMD_NULL;
	  	queue_state->free_host_cmds = queue_state->host_cmd_array;

  		/*
  	 	 * Build a free list of dma_cmd_t structures; note that we skip 
  	 	 * the first entry in the array so that we can use 0 as an 
  	 	 * end-of-list flag.
  	 	 */
	  	queue_state->dma_cmd_array = (dma_cmd_t *)(backing_mem + 
  			GXPCI_H2T_DMA_CMD_ARRAY_BUF_OFFSET);
	  	for (i = 1; i < PCIE_CMD_QUEUE_ENTRIES * 2; i++) {
    			dma_cmd_t *cmd = queue_state->dma_cmd_array + i;
    			cmd->next_index = i + 1;
  		}
  		queue_state->dma_cmd_array[i - 1].next_index = DMA_CMD_NULL;
	  	queue_state->free_dma_cmds = queue_state->dma_cmd_array + 1;
	}

  	return 0;
} 
#endif /* defined(TILEPCI_ENDP) */


#if defined(TILEPCI_HOST)
/* 
 * Check whether the zero-copy stream has been initialized on the tile side 
 * successfully.
 *
 * Return status if the tile's zero-copy stream is initialized, which is 
 * GXPCI_TILE_CHAN_READY or GXPCI_TILE_CHAN_READY_ACK.
 */
static uint32_t check_zc_stream_ready(struct tlr_zc_stream* stream,
				      tlr_zc_cmd_type_t cmd_type,
				      uint32_t status)
{
	struct tlr_pcie_dev *tlr = stream->tlr;
	unsigned long flags;
	uint32_t result = 0;
	uint32_t zc_status;

	/* First, do a fast-path check to see if the stream is already
	 * up.  We're not going to transition anything yet, so we only
	 * need the spinlock. */
	spin_lock_irqsave(&tlr->is_ready_lock, flags);
	if (stream->cmd_queue->is_ready) {
		spin_unlock_irqrestore(&tlr->is_ready_lock, flags);
		return status;
	}
	spin_unlock_irqrestore(&tlr->is_ready_lock, flags);

	if (cmd_type == TLR_ZC_CMD_H2T)
		zc_status = readl(&stream->h2t_regs->queue_status);
	else
		zc_status = readl(&stream->t2h_regs->queue_status);	

	if (zc_status == status) 
		result = status;

	return result;
}
#endif /* defined(TILEPCI_HOST) */


#if defined(TILEPCI_HOST)
/* Initialize a newly opened zero-copy stream. */
static int init_zc_stream(struct tlr_zc_stream* stream, 
			  tlr_zc_cmd_type_t cmd_type)
{
	uint64_t value;
	int result = 0;
	int flag = 0;
	int sleep_msecs = 0;
	const int POLLING_MSECS = 200;
	const int MAX_MSECS = 90000;

        /* We expect ~4k pages. */
        if (PAGE_SIZE < BUFFER_SIZE) {
                ERR("Page size must be at least %d\n", BUFFER_SIZE);
                return -ENOMEM;
        }

        /* Check whether the chip is ready. */
        while (1) {
                spin_lock(&stream->tlr->is_ready_lock);
                if (stream->tlr->is_ready) {
                	spin_unlock(&stream->tlr->is_ready_lock);
                        break;
                }
                spin_unlock(&stream->tlr->is_ready_lock);

                /* Avoid being optimized into register by the compiler. */
                atomic_compiler_barrier();

                if (sleep_msecs >= MAX_MSECS)
                        break;
                msleep(POLLING_MSECS);
                sleep_msecs += POLLING_MSECS;
        }

        if (sleep_msecs >= MAX_MSECS) {
		if (stream->tlr->drv_mismatch)
			return -EPERM;
		else
			return -ENXIO;
	}

        sleep_msecs = 0;

	/* 
	 * Check whether the zero-copy stream is ready. The timeout value is 
	 * set to MAX_MSECS. 
	 */
	while ((flag = check_zc_stream_ready(stream, 
					     cmd_type, 
					     GXPCI_TILE_CHAN_READY)) 
	       != GXPCI_TILE_CHAN_READY) {

		/* Avoid being optimized into register by the compiler. */
		atomic_compiler_barrier();

		if (sleep_msecs >= MAX_MSECS)
			break;
		msleep(POLLING_MSECS);
		sleep_msecs += POLLING_MSECS;
	}

	if (flag != GXPCI_TILE_CHAN_READY) {
		result = -ENXIO;
		goto check_ready_fail;
	}

	if (cmd_type == TLR_ZC_CMD_H2T) {
		/* Configure H2T completion array location. */
		TRACE("Programming H2T completion bus address.\n");
		value = (uint64_t)stream->h2t_completion_handle;
		TRACE("completion_array=%16llXh\n", value);
		writeq(value, &stream->h2t_regs->completion_array);

		/* Configure H2T command buffer array. */
		TRACE("Programming H2T command buffer bus address.\n");
		value = (uint64_t)stream->h2t_buffer_cmd_handle;
		TRACE("buffer_cmd_array=%16llXh\n", value);
		writeq(value, &stream->h2t_regs->buffer_cmd_array);

		/* 
	 	 * Get initial values of the H2T command and completion counter
	 	 * MMIO registers. 
	 	 */
		TRACE("Retreiving H2T posted & comple starting values.\n");
		stream->h2t_commands_posted =
			readl(&stream->h2t_regs->buffer_cmd_posted_count);
		stream->h2t_completions_last =
			readl(&stream->h2t_regs->completion_posted_count);
		TRACE("CMD=%08Xh COMP=%08Xh\n",
	      	      stream->h2t_commands_posted,
	      	      stream->h2t_completions_last);
	}
	else {
		/* Configure T2H completion array location. */
	        TRACE("Programming T2H completion bus address.\n");
        	value = (uint64_t)stream->t2h_completion_handle;
	        TRACE("completion_array=%16llXh\n", value);
        	writeq(value, &stream->t2h_regs->completion_array);
		
		/* Configure T2H command buffer array. */
        	TRACE("Programming T2H command buffer bus address.\n");
	        value = (uint64_t)stream->t2h_buffer_cmd_handle;
        	TRACE("buffer_cmd_array=%16llXh\n", value);
	        writeq(value, &stream->t2h_regs->buffer_cmd_array);

		/* 
	 	 * Get initial values of the T2H command and completion counter 
	 	 * MMIO registers. 
	 	 */
		TRACE("Retreiving T2H posted & comple starting values.\n");
		stream->t2h_commands_posted =
			readl(&stream->t2h_regs->buffer_cmd_posted_count);
		stream->t2h_completions_last =
			readl(&stream->t2h_regs->completion_posted_count);
		TRACE("CMD=%08Xh COMP=%08Xh\n",
		      stream->t2h_commands_posted,
	      	      stream->t2h_completions_last);
	}

        /* Register our interrupt handler. */
        result = gxpci_request_irq(stream);
        if (result < 0) {
		result = -ENXIO;
                goto irq_failed;
	}

	/* Indicate the gx card that we are ready to go. */
	if (cmd_type == TLR_ZC_CMD_H2T)
		writel(GXPCI_HOST_CHAN_READY, &stream->h2t_regs->queue_status);
	else
		writel(GXPCI_HOST_CHAN_READY, &stream->t2h_regs->queue_status);

	/* Complete the handshake process with the endpoint. */
	sleep_msecs = 0;
        while ((flag = 
		check_zc_stream_ready(stream,
				      cmd_type,
				      GXPCI_TILE_CHAN_READY_ACK)) 
	       != GXPCI_TILE_CHAN_READY_ACK) {
                
		/* Avoid being optimized into register by the compiler. */
                atomic_compiler_barrier();

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

	/* Enable interrupt generation on the endpoint. */
        gxpci_irq_enable(stream);

	return result;

check_ready_ack_fail:
        if (cmd_type == TLR_ZC_CMD_H2T)
	        writel(0, &stream->h2t_regs->queue_status);
	else
                writel(0, &stream->t2h_regs->queue_status);

	if (stream->cmd_queue->irq_num >= 0)
		gxpci_free_irq(stream);
irq_failed:
check_ready_fail:
	return result;
}
#elif defined(TILEPCI_ENDP)
static int init_zc_stream(struct tlr_zc_stream* stream, 
			  tlr_zc_cmd_type_t cmd_type)
{
 	struct tlr_pcie_dev *ep_dev = stream->tlr;
	struct tlr_zc_stream *shared_stream; 	
	struct page *page;
	tlr_zc_cmd_type_t type = stream->cmd_queue->type;
	gxio_trio_dma_queue_t *dma_queue;
	gxio_trio_context_t *trio_context;
	size_t dma_ring_size;
	uint32_t open_count = stream->trio_state->trio_open_count;
	uint32_t h2t_status = 0;
	uint32_t t2h_status = 0;
	unsigned long size;
        unsigned int pull_dma_rings[3];
        unsigned int push_dma_rings[3];
        unsigned int pull_dma_irqs[3];
        unsigned int push_dma_irqs[3];
        const int POLLING_MSECS = 200;
        const int MAX_MSECS = 90000;
        void *dma_ring_mem;
        void *backing_mem;
        int timeout_flag = 0;
        int sleep_msecs = 0;
        int err = 0;
	int msix_table_index;
	int dma_ring;
	int asid;
	int irq;
	int cpu;
	int i;

	/* Check the correct command type. */
	if (type == TLR_ZC_CMD_UNKNOWN)
		return -EINVAL;

        /* We expect ~4k pages. */
        if (PAGE_SIZE < BUFFER_SIZE) {
                ERR("Page size must be at least %d\n", BUFFER_SIZE);
                return -ENOMEM;
        }

        /* Initialize a shared trio context by the first caller. */
	if (open_count == 0) {
        	err = gxio_trio_init(&stream->trio_state->trio, 
        			     ep_dev->trio_index);
	        if (err < 0) {
        	        ERR("gxio_trio_init() failed\n");
			return -ENXIO;
	        }

        	asid = gxio_trio_alloc_asids(&stream->trio_state->trio, 1, 0, 0);
	        if (asid == GXIO_TRIO_ERR_NO_ASID) {
        	        ERR("gxio_trio_alloc_asids() failed\n");
			return -ENXIO;
        	}
		stream->trio_state->trio.asid = asid;
	}
	trio_context = &stream->trio_state->trio;

	/* Allocate tile command and tile completion buffers. */
	size = sizeof(struct pcie_tile_buffer_cmd) * PCIE_CMD_QUEUE_ENTRIES;
	page = alloc_pages(GFP_KERNEL | __GFP_ZERO, get_order(size));
	if (!page) {
		err = -ENOMEM;
		goto alloc_tile_cmd_buf_failed;
	}

	if (type == TLR_ZC_CMD_H2T)
		stream->h2t_buffer_cmd_array = 
			(struct pcie_tile_buffer_cmd *)page_address(page);
	else
		stream->t2h_buffer_cmd_array = 
			(struct pcie_tile_buffer_cmd *)page_address(page); 

	/* Allocate H2T DMA resources. */
	if (type == TLR_ZC_CMD_H2T) {
		page = alloc_pages(GFP_KERNEL | __GFP_ZERO, 
				   get_order(HPAGE_SIZE));
		if (!page) {
			err = -ENOMEM;
			goto alloc_dma_buf_failed;
		}

		/* Register this DMA memory. */
        	backing_mem = page_address(page);
		err = gxio_trio_register_page(trio_context, trio_context->asid,
					      backing_mem, HPAGE_SIZE, 0);
		if (err < 0) {
			err = -ENOMEM;
			goto reg_dma_buf_failed;
		}
		stream->backing_mem = backing_mem;

                /* Initialize MSI-X info. */
		msix_table_index =
			ep_dev->msix_zc_q_intr_vec_base + stream->index;
#ifdef GXPCI_INTR_VECTOR_PER_QUEUE
                msix_table_index += GXPCI_HOST_ZC_QUEUE_COUNT;
#endif
		stream->h2t_msix_msg.msix_table_entry_offset =
			ep_dev->msix_table_base +
			PCI_MSIX_ENTRY_SIZE * msix_table_index;
		stream->h2t_msix_msg.msix_addr = *(unsigned long *)
			(stream->h2t_msix_msg.msix_table_entry_offset + 
			 PCI_MSIX_ENTRY_LOWER_ADDR);
		stream->h2t_msix_msg.msix_data = *(unsigned int *)
			(stream->h2t_msix_msg.msix_table_entry_offset + 
			 PCI_MSIX_ENTRY_DATA);

		/* The other TRIO-shared queues should go away. */
		if (open_count > 0)
			goto do_handshake;

		/*
	 	 * Allocate and initialize a pull DMA ring for the command 
		 * descriptors.
		 */
		dma_ring = gxio_trio_alloc_pull_dma_ring(trio_context, 1, 0, 0);
		if (dma_ring < 0) {
			err = -ENXIO;
			goto failed;
		}

       		/*
       		 * Bind the DMA ring to our MAC, and use registered memory to 
		 * store the command ring.
       	 	 */
		dma_ring_size = GXPCI_STREAM_PULL_DMA_RING_LEN * 
			sizeof(gxio_trio_dma_desc_t);
		dma_ring_mem = 
			backing_mem + GXPCI_STREAM_H2T_PULL_DMA_DESC_OFFSET;
		dma_queue = &stream->h2t_dma_resource->pull_dma_queue_desc;
		err = gxio_trio_init_pull_dma_queue(dma_queue, trio_context, 
						    dma_ring, ep_dev->mac, 
					    	    trio_context->asid, 0, 
					    	    dma_ring_mem, 
					    	    dma_ring_size, 
					    	    0);
		if (err < 0) {
			err = -ENXIO;
			goto failed;
		}
		stream->h2t_dma_resource->pull_dma_desc = dma_ring;
		pull_dma_rings[0] = dma_ring;

        	/*
       	 	 * Allocate and initialize a pull DMA ring for the data.
       		 */
        	dma_ring = gxio_trio_alloc_pull_dma_ring(trio_context, 1, 0, 0);
       		if (dma_ring < 0) {
               		err = -ENXIO;
        		goto failed;
       		}

        	/*
       	 	 * Bind the DMA ring to our MAC, and use registered memory to 
		 * store the command ring.
         	 */
        	dma_ring_size = GXPCI_STREAM_PULL_DMA_RING_LEN *
                	sizeof(gxio_trio_dma_desc_t);
	        dma_ring_mem = 
			backing_mem + GXPCI_STREAM_H2T_PULL_DMA_DATA_OFFSET;
        	dma_queue = &stream->h2t_dma_resource->pull_dma_queue_data;
        	err = gxio_trio_init_pull_dma_queue(dma_queue, trio_context, 
						    dma_ring, ep_dev->mac, 
						    trio_context->asid, 0,
                                            	    dma_ring_mem, 
       	                                    	    dma_ring_size, 
               	                            	    0);
        	if (err < 0) {
       	        	err = -ENXIO;
               		goto failed;
        	}
       		stream->h2t_dma_resource->pull_dma_data = dma_ring;
		pull_dma_rings[1] = dma_ring;

        	/*
       	 	 * Allocate and initialize a push DMA ring for writing the
		 * command completions back to the host.
         	 */
	        dma_ring = gxio_trio_alloc_push_dma_ring(trio_context, 1, 0, 0);
       		if (dma_ring < 0) {
               		err = -ENXIO;
               		goto failed;
        	}

        	/*
       	 	 * Bind the DMA ring to our MAC, and use registered memory to 
		 * store the command ring.
         	 */
        	dma_ring_size = GXPCI_STREAM_PUSH_DMA_RING_LEN *
       	        	sizeof(gxio_trio_dma_desc_t);
       		dma_ring_mem = 
			backing_mem + GXPCI_STREAM_H2T_PUSH_DMA_COMP_OFFSET;
        	dma_queue = &stream->h2t_dma_resource->push_dma_queue_comp;
        	err = gxio_trio_init_push_dma_queue(dma_queue, trio_context, 
						    dma_ring, ep_dev->mac, 
						    trio_context->asid, 0,
                                            	    dma_ring_mem, 
       	                                    	    dma_ring_size, 
               	                            	    0);
        	if (err < 0) {
       	        	err = -ENXIO;
               		goto failed;
       		}
        	stream->h2t_dma_resource->push_dma_comp = dma_ring;
		push_dma_rings[0] = dma_ring;
	}
	else {
		/* 
		 * Allocate T2H DMA resources, where the memory offsets 
		 * are identical to H2T, since H2T and T2H channels use 
		 * different memory pages. 
		 */
		page = alloc_pages(GFP_KERNEL | __GFP_ZERO, 
				   get_order(HPAGE_SIZE));
		if (!page) {
			err = -ENOMEM;
			goto alloc_dma_buf_failed;
		}
		/* Register this DMA memory. */
        	backing_mem = page_address(page);
		err = gxio_trio_register_page(trio_context, trio_context->asid,
					      backing_mem, HPAGE_SIZE, 0);
		if (err < 0) {
			err = -ENOMEM;
			goto reg_dma_buf_failed;
		}	
		stream->backing_mem = backing_mem;

                /* Initialize MSIX info for a T2H queue. */
		msix_table_index =
			ep_dev->msix_zc_q_intr_vec_base + stream->index;
		stream->t2h_msix_msg.msix_table_entry_offset =
			ep_dev->msix_table_base +
			PCI_MSIX_ENTRY_SIZE * msix_table_index;
		stream->t2h_msix_msg.msix_addr = *(unsigned long *)
			(stream->t2h_msix_msg.msix_table_entry_offset + 
			 PCI_MSIX_ENTRY_LOWER_ADDR);
		stream->t2h_msix_msg.msix_data = *(unsigned int *)
			(stream->t2h_msix_msg.msix_table_entry_offset + 
			 PCI_MSIX_ENTRY_DATA);

		/* The other TRIO-shared queues should go away. */
		if (open_count > 0)
			goto do_handshake;

	       	/*
         	 * Allocate and initialize a pull DMA ring for the command 
		 * descriptors.
	       	 */
	        dma_ring = gxio_trio_alloc_pull_dma_ring(trio_context, 1, 0, 0);
       		if (dma_ring < 0) {
                	err = -ENXIO;
       	        	goto failed;
       		}

        	/*
       	 	 * Bind the DMA ring to our MAC, and use registered memory to 
		 * store the command ring.
         	 */
        	dma_ring_size = GXPCI_STREAM_PULL_DMA_RING_LEN *
       	        	sizeof(gxio_trio_dma_desc_t);
        	dma_ring_mem = 
			backing_mem + GXPCI_STREAM_H2T_PULL_DMA_DESC_OFFSET;
        	dma_queue = &stream->t2h_dma_resource->pull_dma_queue_desc;
        	err = gxio_trio_init_pull_dma_queue(dma_queue, trio_context, 
						    dma_ring, ep_dev->mac, 
						    trio_context->asid, 0,
                                            	    dma_ring_mem, 
                                            	    dma_ring_size, 
       	                                    	    0);
       		if (err < 0) {
                	err = -ENXIO;
       	        	goto failed;
       		}
        	stream->t2h_dma_resource->pull_dma_desc = dma_ring;
		pull_dma_rings[2] = dma_ring;

		/*
         	 * Allocate and initialize a push DMA ring for the data.
       	 	 */
        	dma_ring = gxio_trio_alloc_push_dma_ring(trio_context, 1, 0, 0);
       		if (dma_ring < 0) {
               		err = -ENXIO;
               		goto failed;
        	}

        	/*
       	 	 * Bind the DMA ring to our MAC, and use registered memory to 
		 * store the command ring.
         	 */
        	dma_ring_size = GXPCI_STREAM_PUSH_DMA_RING_LEN *
       	        	sizeof(gxio_trio_dma_desc_t);
        	dma_ring_mem = 
			backing_mem + GXPCI_STREAM_H2T_PULL_DMA_DATA_OFFSET;
        	dma_queue = &stream->t2h_dma_resource->push_dma_queue_data;
       		err = gxio_trio_init_push_dma_queue(dma_queue, trio_context, 
						    dma_ring, ep_dev->mac, 
						    trio_context->asid, 0,
                               	            	    dma_ring_mem, 
                                       	    	    dma_ring_size, 
                                       		    0);
        	if (err < 0) {
       	        	err = -ENXIO;
               		goto failed;
        	}	
       		stream->t2h_dma_resource->push_dma_data = dma_ring;
		push_dma_rings[1] = dma_ring;

       		/*
       		 * Allocate and initialize a push DMA ring for writing the
		 * command completions back to the host.
       	 	 */
        	dma_ring = gxio_trio_alloc_push_dma_ring(trio_context, 1, 0, 0);
       		if (dma_ring < 0) {
               		err = -ENXIO;
               		goto failed;
        	}

        	/*
       	 	 * Bind the DMA ring to our MAC, and use registered memory to 
		 * store the command ring.
         	 */
       		dma_ring_size = GXPCI_STREAM_PUSH_DMA_RING_LEN *
               		sizeof(gxio_trio_dma_desc_t);
        	dma_ring_mem = 
			backing_mem + GXPCI_STREAM_H2T_PUSH_DMA_COMP_OFFSET;
       		dma_queue = &stream->t2h_dma_resource->push_dma_queue_comp;
        	err = gxio_trio_init_push_dma_queue(dma_queue, trio_context, 
						    dma_ring, ep_dev->mac, 
						    trio_context->asid, 0,
                       	                    	    dma_ring_mem, 
                               	            	    dma_ring_size, 
                                       	    	    0);
        	if (err < 0) {
			err = -ENXIO;
               		goto failed;
       		}
        	stream->t2h_dma_resource->push_dma_comp = dma_ring;
		push_dma_rings[2] = dma_ring;
	}

	/* Initialize DMA command array to track queue number. */
        stream->trio_state->cmd_dma_chans = backing_mem +
        	GXPCI_STREAM_T2H_PULL_DMA_DESC_OFFSET;
        stream->trio_state->data_dma_chans = backing_mem +
                GXPCI_STREAM_T2H_PUSH_DMA_DATA_OFFSET;
        stream->trio_state->comp_dma_chans = backing_mem +
                GXPCI_STREAM_T2H_PUSH_DMA_COMP_OFFSET;          

        /* Initialize DMA command array for all TRIO-shared ZC queues. */
        for (i = 0; i < GXPCI_HOST_ZC_QUEUE_COUNT; i++) {
        	if (i % ZC_QUEUE_TRIO_NUM ==
                    stream->index % ZC_QUEUE_TRIO_NUM) {

                	if (type == TLR_ZC_CMD_H2T)
                		shared_stream = 
                			tlr_get_zc_h2t_stream(ep_dev, i);
                	else
	                	shared_stream = 
                			tlr_get_zc_t2h_stream(ep_dev, i);

                        shared_stream->trio_state->cmd_dma_chans = 
                        	stream->trio_state->cmd_dma_chans;
                        shared_stream->trio_state->data_dma_chans = 
                        	stream->trio_state->data_dma_chans;
                        shared_stream->trio_state->comp_dma_chans = 
                        	stream->trio_state->comp_dma_chans;	
		}
        }

	/* 
	 * Allocate and register a common interrupt for push/pull DMA rings to 
	 * kernel with the same zero-copy stream interrupt handle, 
	 * i.e. zc_stream_intr. 
	 */
	irq = create_irq();
        if (irq < 0) {
		err = -ENXIO;
	       	goto irq_failed;
	}

	stream->cmd_queue->irq_num = irq;
	stream->cmd_queue->irq_data = (void*)stream;
	
        /* Initialize irq info for all the TRIO-shared queues. */
        for (i = 0; i < GXPCI_HOST_ZC_QUEUE_COUNT; i++) {
        	if (i % ZC_QUEUE_TRIO_NUM ==
                    stream->index % ZC_QUEUE_TRIO_NUM) {

                	if (type == TLR_ZC_CMD_H2T)
                		shared_stream = 
                			tlr_get_zc_h2t_stream(ep_dev, i);
                	else
	                	shared_stream = 
                			tlr_get_zc_t2h_stream(ep_dev, i);

                        shared_stream->cmd_queue->irq_num = irq;
                        shared_stream->cmd_queue->irq_data = (void*)stream;
		}
        }

        tile_irq_activate(irq, TILE_IRQ_PERCPU);
	if (type == TLR_ZC_CMD_H2T) {
        	if (request_irq(irq, zc_h2t_dma_intr, 0, driver_name, 
			        (void*)stream)) {
			err = -ENXIO;
       			goto irq_failed;
		}
	}
	else {
       	        if (request_irq(irq, zc_t2h_dma_intr, 0, driver_name,
               	                (void*)stream)) {
			err = -ENXIO;	
       	                goto irq_failed;
		}
	}
	
	/* Choose an arbitrary processor to receive interrupts. */
	cpu = raw_smp_processor_id();

	/* Register irq for ZC H2T stream. */
	if (type == TLR_ZC_CMD_H2T) {
		/* Process two pull DMA rings. */
		for (i = 0; i < 2; i++) {
	        	err = gxio_trio_config_char_intr(trio_context,
                                			 cpu_x(cpu), cpu_y(cpu),
		                                         KERNEL_PL, irq,
		 	                                 ep_dev->mac, 0, 0,
			                                 pull_dma_rings[i],
	        		                         PULL_DMA_SEL);
			if (err < 0) {
				err = -ENXIO;
				goto irq_failed;
			}
	
			pull_dma_irqs[i] = irq;
        	}

       		/* Process one push DMA ring. */
	        for (i = 0; i < 1; i++) {
	                err = gxio_trio_config_char_intr(trio_context,
			                        	 cpu_x(cpu), cpu_y(cpu),
	        		                         KERNEL_PL, irq,
	                        		         ep_dev->mac, 0,
			                                 push_dma_rings[i],
	        		                         0,
	                        		         PUSH_DMA_SEL);
			if (err < 0) {
				err = -ENXIO;
               	        	goto irq_failed;
			}

			push_dma_irqs[i] = irq;
       		}
	}
	else {
		/* Process one pull DMA ring. */
		for (i = 2; i < 3; i++) {
                        err = gxio_trio_config_char_intr(trio_context,
        		                	       	 cpu_x(cpu), cpu_y(cpu),
                        		                 KERNEL_PL, irq,
	        	                                 ep_dev->mac, 0, 0,
        	        	                         pull_dma_rings[i],
                                		         PULL_DMA_SEL);
                        if (err < 0) {
				err = -ENXIO;
				goto irq_failed;
			}

			pull_dma_irqs[i] = irq;
       		}

       		/* Process two push DMA rings. */
	        for (i = 1; i < 3; i++) {
       	                err = gxio_trio_config_char_intr(trio_context,
		                                	 cpu_x(cpu), cpu_y(cpu),
                		                         KERNEL_PL, irq,
                                		         ep_dev->mac, 0,
		                                         push_dma_rings[i],
                		                         0,
                                		         PUSH_DMA_SEL);
                        if (err < 0) {
				err = -ENXIO;
               	        	goto irq_failed;
			}

			push_dma_irqs[i] = irq;
       		}
	}

	if (type == TLR_ZC_CMD_H2T) {
		/* Record the allocated irq numbers. */
		stream->h2t_dma_resource->pull_dma_desc_irq = 
			pull_dma_irqs[0];
		stream->h2t_dma_resource->pull_dma_data_irq = 
			pull_dma_irqs[1];
		stream->h2t_dma_resource->push_dma_comp_irq = 
			push_dma_irqs[0];
	}
	else {
		/* Record the allocated irq numbers. */
		stream->t2h_dma_resource->pull_dma_desc_irq = 
			pull_dma_irqs[2];
		stream->t2h_dma_resource->push_dma_data_irq = 
			push_dma_irqs[1];
		stream->t2h_dma_resource->push_dma_comp_irq = 
			push_dma_irqs[2];
	}

do_handshake:
	if (type == TLR_ZC_CMD_H2T) {
		stream->h2t_commands_posted = 0;

		/* Fill in variables for the host and set up ready flag. */
		stream->h2t_regs->buffer_cmd_posted_count = 0;
		stream->h2t_regs->completion_posted_count = 0;
		stream->h2t_regs->completion_consumed_count = 0;

		/* Fence here to avoid write re-ordering.*/
		__insn_mf();

		/* Start the handshake process */
		stream->h2t_regs->queue_status = GXPCI_TILE_CHAN_READY;

		/* Wait until the host is ready. */
		while (1) {
			h2t_status = stream->h2t_regs->queue_status;
			atomic_compiler_barrier();

			if (h2t_status == GXPCI_HOST_CHAN_READY)
				break;
	
			if (sleep_msecs >= MAX_MSECS) {
				timeout_flag = 1;
				break;
			}

			msleep(POLLING_MSECS);
			sleep_msecs += POLLING_MSECS;	
		}

		/* Error when timeout. */
		if (timeout_flag) {
			err = -ENXIO;
			goto handshake_failed; 
		}

		TRACE("INFO: Host is ready now!\n");

		/* After the flag, record host array bus addresses. */
		stream->h2t_cmpl_array_addr = 
			stream->h2t_regs->completion_array;
		stream->h2t_buf_cmd_array_addr = 
			stream->h2t_regs->buffer_cmd_array;
	}
	else {
		stream->t2h_commands_posted = 0;

		/* Fill in variables for the host and set up ready flag. */
		stream->t2h_regs->buffer_cmd_posted_count = 0; 
		stream->t2h_regs->completion_posted_count = 0;
		stream->t2h_regs->completion_consumed_count = 0;

		/* Fence here to avoid write re-ordering.*/
		__insn_mf();
		
		/* Start the handshake process */
		stream->t2h_regs->queue_status = GXPCI_TILE_CHAN_READY;

		/* Wait until the host is ready. */
		while (1) {
			t2h_status = stream->t2h_regs->queue_status;
			atomic_compiler_barrier();

			if (t2h_status == GXPCI_HOST_CHAN_READY)
				break;

                        if (sleep_msecs >= MAX_MSECS) {
                                timeout_flag = 1;
                                break;
                        }

                        msleep(POLLING_MSECS);
                        sleep_msecs += POLLING_MSECS;	
		}

                /* Error when timeout. */
                if (timeout_flag) {
                        err = -ENXIO;
                        goto handshake_failed;
                }

		TRACE("INFO: Host is ready now!\n");		

		/* After the flag, record host array bus addresses. */
		stream->t2h_cmpl_array_addr = 
			stream->t2h_regs->completion_array;
		stream->t2h_buf_cmd_array_addr = 
			stream->t2h_regs->buffer_cmd_array;	

	}
	
	/* Initialize queue_state. */
	err = gxpci_queue_state_init(stream, backing_mem);
	if (err < 0)
		goto queue_state_init_failed;

	/* Initialize a per queue timer to monitor any pending interrupts. */
	init_timer(&stream->intr_timer);
	stream->intr_timer.data = (unsigned long) stream;
	stream->intr_timer.function = gxpci_zc_intr_timer_handler;

	/* Fence here to avoid write re-ordering.*/
	__insn_mf();

	/* Complete the handshake process. */
	if (type == TLR_ZC_CMD_H2T)
		stream->h2t_regs->queue_status = GXPCI_TILE_CHAN_READY_ACK;
	else
		stream->t2h_regs->queue_status = GXPCI_TILE_CHAN_READY_ACK;

	/* Advance the TRIO-shared open count on success. */
	stream->trio_state->trio_open_count++;

	__insn_mf();

	return err;

queue_state_init_failed:
handshake_failed:
        if (type == TLR_ZC_CMD_H2T)
                stream->h2t_regs->queue_status = 0;
        else
                stream->t2h_regs->queue_status = 0;
	if (open_count == 0 && stream->cmd_queue->irq_num >= 0) {
		free_irq(stream->cmd_queue->irq_num, 
			 stream->cmd_queue->irq_data);
	}
irq_failed:
	if (open_count == 0 && stream->cmd_queue->irq_num >= 0) {
		destroy_irq(stream->cmd_queue->irq_num);
		stream->cmd_queue->irq_num = -1;
	}
failed:
reg_dma_buf_failed:
	homecache_free_pages((unsigned long)backing_mem, get_order(HPAGE_SIZE));
        if (type == TLR_ZC_CMD_H2T)
		stream->backing_mem = NULL;
	else
		stream->backing_mem = NULL;
alloc_dma_buf_failed:
        if (type == TLR_ZC_CMD_H2T) { 
                homecache_free_pages(
                        (unsigned long)stream->h2t_buffer_cmd_array,
                        get_order(size));
		stream->h2t_buffer_cmd_array = NULL;
	}
        else {
		homecache_free_pages(
                        (unsigned long)stream->t2h_buffer_cmd_array,
			get_order(size));
		stream->t2h_buffer_cmd_array = NULL;
	}
alloc_tile_cmd_buf_failed:
	if (open_count == 0 && trio_context->fd >= 0)
		hv_dev_close(trio_context->fd);

	return err;
}
#else
#error Undefined Architecture
#endif


#if defined(TILEPCI_HOST)
/* Release an opened zero-copy stream. */ 
static void release_zc_stream(struct tlr_zc_stream* stream)
{
	/* Mask off interrupt generation on the endpoint. */
	gxpci_irq_disable(stream);

	/* Free interrupts. */
	if (stream->cmd_queue->irq_num >= 0) 
		gxpci_free_irq(stream);
}
#elif defined(TILEPCI_ENDP)
static void release_zc_stream(struct tlr_zc_stream* stream)
{
        gxio_trio_context_t *trio_context = &stream->trio_state->trio;
	unsigned long size = 
		sizeof(struct pcie_tile_buffer_cmd) * PCIE_CMD_QUEUE_ENTRIES;
	uint32_t open_count = --stream->trio_state->trio_open_count;
	const int POLLING_MSECS = 100;
	const int MAX_MSECS = 2000;
	int sleep_msecs = 0;

        /* 
	 * Try to update the host to avoid any missing completion to the host.
	 * This could happen if zc_h2t_dma_intr() or zc_t2h_dma_intr() can not 
	 * get the per-queue lock before release_zc_stream(), and therefore
	 * miss the last chance to update the completion posted number to the
	 * host.
	 */
	while (1) {
		gxpci_zc_update_host(stream);

		if (sleep_msecs >= MAX_MSECS) 
			break;

		msleep(POLLING_MSECS);
		sleep_msecs += POLLING_MSECS;	
	}
	 
	/* Remove the per queue timer. */
	if (stream->intr_timer.data)
		del_timer(&stream->intr_timer);

	/* Free trio resources. */
	if (open_count == 0 && trio_context->fd >= 0) {
	        hv_dev_close(trio_context->fd);
	}

        /* Free interrupts. */
        if (open_count == 0 && stream->cmd_queue->irq_num >= 0) {
        	free_irq(stream->cmd_queue->irq_num, 
			 stream->cmd_queue->irq_data);
                destroy_irq(stream->cmd_queue->irq_num);
                
	        /* Reset counters of the DMA command index arrays. */
        	stream->trio_state->cmd_dma_chan_enqueued = 0;
	        stream->trio_state->cmd_dma_chan_dequeued = 0;
        	stream->trio_state->data_dma_chan_enqueued = 0;
	        stream->trio_state->data_dma_chan_dequeued = 0;
        	stream->trio_state->comp_dma_chan_enqueued = 0;
	        stream->trio_state->comp_dma_chan_dequeued = 0;
        }
        stream->cmd_queue->irq_num = -1;

	/* Free the backing memory for DMA. */
	if (stream->cmd_queue->type == TLR_ZC_CMD_H2T)
		homecache_free_pages(
			(unsigned long)stream->backing_mem, 
			get_order(HPAGE_SIZE));
	else 
		homecache_free_pages(
			(unsigned long)stream->backing_mem, 
			get_order(HPAGE_SIZE));

	/* Free the command buffer array. */
	if (stream->cmd_queue->type == TLR_ZC_CMD_H2T)  
                homecache_free_pages(
                        (unsigned long)stream->h2t_buffer_cmd_array,
                        get_order(size));
	else 
                homecache_free_pages(
                        (unsigned long)stream->t2h_buffer_cmd_array,
                        get_order(size));
}
#else
#error Undefined Architecture
#endif


/* Post a set of write buffers to the iBound. */
static void post_cmds(struct tlr_zc_cmd_q *q)
{
	struct tlr_pcie_dev  	*tlr;
	unsigned long		 q_flags;
	u32			 cmds_posted;
	u32			 cmd_index;
	struct tlr_zc_cmd	*cmd;
	struct tlr_zc_stream  	*stream;
	tlr_dma_addr_t		 dma_addr;

	tlr = q->tlr;

	if (q->type == TLR_ZC_CMD_H2T)
		stream = tlr_get_zc_h2t_stream(tlr, q->chan);
	else
		stream = tlr_get_zc_t2h_stream(tlr, q->chan);
	
	spin_lock_irqsave(&q->lock, q_flags);

	/* Make sure we actually have commands. */
	if (__tlr_list_empty(&q->pend_q))
		goto exit;
	
	/* If this channel needs SoC, add that to the first command. */
	if (q->state == CMD_Q_ST_NEED_SOC) {
		q->state = CMD_Q_ST_CONNECTED;
	}

	if (q->type == TLR_ZC_CMD_H2T)
		cmds_posted = stream->h2t_commands_posted;
	else
		cmds_posted = stream->t2h_commands_posted;

	/* 
	 * This loop posts buffers from the per-stream ring buffer
	 * into the global command ring buffer.  We keep a separate
	 * 'posted' count for each ring, and calculate the index
	 * within each ring as (posted % ring_entries). 
	 */
	while (!__tlr_list_empty(&q->pend_q)) {
		cmd = __tlr_list_rem_zc_cmd(&q->pend_q);

		q->nreq_cur++;
		q->nreq_tot++;
		q->nreq_bytes_cur += cmd->post_len;
		q->nreq_bytes_tot += cmd->post_len;

		dma_addr = cmd->cmd.buffer_addr;

		HID_CMD_TRACE("CMD: "
		      	      "size= %d, "
		      	      "CPA= %#llx\n",
		              cmd->cmd.size,
		      	      dma_addr);

		cmd_index = cmds_posted % PCIE_CMD_QUEUE_ENTRIES;
		if (q->type == TLR_ZC_CMD_H2T)
			stream->h2t_buffer_cmd_array[cmd_index] = cmd->cmd;
		else
			stream->t2h_buffer_cmd_array[cmd_index] = cmd->cmd;
		cmds_posted++;

		__tlr_list_add_zc_cmd(&q->post_q, cmd);
	}

	/* Make sure the commands are visible, then update the posted count.*/
	wmb();
	if (q->type == TLR_ZC_CMD_H2T &&
	    cmds_posted != stream->h2t_commands_posted) {
		stream->h2t_commands_posted = cmds_posted;
#if defined(TILEPCI_HOST)
		writel(cmds_posted, &stream->h2t_regs->buffer_cmd_posted_count);
	        wmb();
                writeq(PCIE_HOST_INTR_CPL_AVAIL, stream->intr_regs);
#else
		stream->h2t_queue_state.tile_cmds_posted = cmds_posted;
#endif /* defined(TILEPCI_HOST) */
	}
 	else if (q->type == TLR_ZC_CMD_T2H &&
		 cmds_posted != stream->t2h_commands_posted) {
		stream->t2h_commands_posted = cmds_posted;
#if defined(TILEPCI_HOST)
		writel(cmds_posted, &stream->t2h_regs->buffer_cmd_posted_count);
                wmb();
                writeq(PCIE_HOST_INTR_CPL_AVAIL, stream->intr_regs);
#else
		stream->t2h_queue_state.tile_cmds_posted = cmds_posted;
#endif /* defined(TILEPCI_HOST) */
	}

 exit:
	spin_unlock_irqrestore(&q->lock, q_flags);
}


/****************************************************************/
/*		      Zero-Copy Helper Functions                */
/****************************************************************/

static void *tlr_alloc_mem(size_t size)
{
	int		 order;
	void		*ret;

	ret = NULL;

	/*
	 * This gives us the number of pages necessary to
	 * hold the request
	 */
	order = get_order(size);
	if (order > 9)
		goto exit;

	ret = (void *)__get_free_pages(GFP_KERNEL, order);
	if (ret == NULL)
		goto exit;

	memset(ret, 'T', size);

exit:
	return ret;
}


static int tlr_free_mem(void *kern_addr, size_t size)
{
	int		 ret = 0;
	int		 order;

	/*
	 * This gives us the number of pages necessary to
	 * hold the request
	 */
	order = get_order(size);
	if (order > 9)
		goto exit;

	free_pages((unsigned long)kern_addr, order);
exit:
	return ret;
}


static int tlr_xfer_validate(struct tlr_zc_cmd_q *q,
			     tilepci_xfer_req_t	*from_xfer)
{
	int	 ret;

	ret = FALSE;
	ret = access_ok(VERIFY_WRITE,
			(void __force __user *)(unsigned long)(from_xfer->addr),
			from_xfer->len);

	return ret;
}


static int tlr_xfer_cvt(struct tlr_zc_cmd_q *q,
			tilepci_xfer_req_t *xfer,
			struct tlr_zc_cmd *cmd)
{
	int		 ret;
	int		 writable;

	ret = 0;

	switch (cmd->dma_dir) {
	case DMA_TO_DEVICE:
		writable = FALSE;
		break;
	case DMA_FROM_DEVICE:
		writable = TRUE;
		break;
	default:
		writable = TRUE;
		break;
	}

	ret = tlr_map_cmd(cmd, (unsigned long)xfer->addr, xfer->len, writable);
	if (ret != 0)
		goto exit;

	/* dma_addr and nmapped_pages are set, set the other fields. */
	cmd->post_len = xfer->len;
	cmd->flags = xfer->flags;
	cmd->usr_addr = xfer->addr;
	cmd->cookie = xfer->cookie;

	/* tlr_flush_buffer(cmd->tlr, cmd->dma_addr, cmd->post_len); */

	HID_CMD_TRACE("cmd %p "
		      "chan %d "
		      "size %d "
		      "dma_addr %llx "
		      "usr_addr %llx "
		      "\n",
		      cmd,
		      cmd->cmd_q->chan,
		      cmd->post_len,
		      cmd->dma_addr,
		      cmd->usr_addr
		);

	cmd->cmd.buffer_addr = cmd->dma_addr;
	cmd->cmd.size = cmd->post_len;
	cmd->cmd.tag = cmd->cookie;

exit:
	return ret;
}


static inline int __tlr_zc_cmd_q_ncmd_set(struct tlr_zc_stream *stream,
					  u32 ncmd_wanted)
{
	int			 ret;
	unsigned long		 tlr_lock_flags;
	unsigned long		 q_lock_flags;
	struct tlr_pcie_dev	*tlr;
	int			 ncmd_diff;
	u32		 	 i;
	struct tlr_zc_cmd	*cmd;
	struct tlr_list		*from_list;
	struct tlr_list		*to_list;
	u32		 	 tlr_free_list_len;
	int			 ncmd_have_inc;

	tlr = stream->tlr;
	tlr_free_list_len = 0;

	/* Abandon this operation if the queue is poisoned due to reset. */
	if (stream->cmd_queue->chip_reset_poison) {
		HID_ERR_TRACE("Exit  ENXIO\n");
		return -ENXIO;
	}

	stream->cmd_queue->ncmd_wanted = ncmd_wanted;
	ncmd_diff = ncmd_wanted - stream->cmd_queue->ncmd_have;
	if (ncmd_diff == 0) {
		HID_EX_TRACE("Exit OK\n");
		return 0;
	}

	spin_lock_irqsave(&stream->cmd_queue->lock, q_lock_flags);
	spin_lock_irqsave(&stream->cmd_q_lock, tlr_lock_flags);

	ret = 0;
	if (ncmd_diff < 0) {
		from_list = &stream->cmd_queue->free_q;
		to_list = &stream->cmd_q_free_list;
		ncmd_diff = -ncmd_diff;
		ncmd_have_inc = -1;
	} else {
		from_list = &stream->cmd_q_free_list;
		to_list = &stream->cmd_queue->free_q;
		tlr_free_list_len =
			__tlr_list_len(&stream->cmd_q_free_list);
		if (tlr_free_list_len < ncmd_diff) {
			ret = -ENOBUFS;
			HID_ERR_TRACE("Exit ENOBUFS\n");
			goto exit;
		}
		ncmd_have_inc = 1;
	}

	for (i = 0; i < ncmd_diff; i++) {
		cmd = __tlr_list_rem_zc_cmd(from_list);
		if (cmd == NULL)
			break;
		tlr_zc_cmd_init(cmd, stream->cmd_queue, tlr, 
				stream->cmd_queue->type, 0,
				to_list, NULL);
		stream->cmd_queue->ncmd_have += ncmd_have_inc;
	}

 exit:
	spin_unlock_irqrestore(&stream->cmd_q_lock, tlr_lock_flags);
	spin_unlock_irqrestore(&stream->cmd_queue->lock, q_lock_flags);

	if (ret == 0)
		ret = __tlr_zc_cmd_q_alloc_xfers(stream->cmd_queue);
	return ret;
}


static int tlr_zc_cmd_q_ncmd_set(struct tlr_zc_stream *stream,
				 u32 ncmd)
{
	int			 ret;
	int			 err;
	int			 have_rd_mutex;
	int			 have_wr_mutex;
	struct tlr_zc_cmd_q	*q = stream->cmd_queue;

	ret = 0;
	have_rd_mutex = FALSE;
	have_wr_mutex = FALSE;

	/*
	 * Grab both the read and write semaphores so that this operation is
	 * ordered with respect to any other processes that may be reading
	 * or writing.  Are we allowed to return -ERESTARTSYS here?  Can't
	 * seem to find the appropriate documentation...
	 */
	err = down_interruptible(&q->rd_xfer_mutex);
	if (err != 0) {
		ret = -ERESTARTSYS;
		HID_ERR_TRACE("Exit ERESTARTSYS\n");
		goto exit;
	}
	have_rd_mutex = TRUE;

	err = down_interruptible(&q->wr_xfer_mutex);
	if (err != 0) {
		ret = -ERESTARTSYS;
		HID_ERR_TRACE("Exit ERESTARTSYS\n");
		goto exit;
	}
	have_wr_mutex = TRUE;

	ret = __tlr_zc_cmd_q_ncmd_set(stream, ncmd);

 exit:
	if (have_wr_mutex) {
		up(&q->wr_xfer_mutex);
		have_wr_mutex = FALSE;
	}

	if (have_rd_mutex) {
		up(&q->rd_xfer_mutex);
		have_rd_mutex = FALSE;
	}

	return ret;
}


/*
 * Put a particular command queue into reset.  If the command queue is
 * already in reset, this function has no effect.
 */
static int tlr_zc_cmd_q_start_reset(struct tlr_zc_cmd_q *q)
{
	unsigned long irq_flags;
#if 0
	struct tlr_pcie_dev *tlr = q->tlr;
#endif // 0
	int res = 0;

	spin_lock_irqsave(&q->lock, irq_flags);

	/* We can only start reset if we're currently connected. */
	if (q->state != CMD_Q_ST_CONNECTED) {
		res = -EINVAL;
		goto exit;
	}
#if 0
	/* Issue the channel reset command. */
#if defined(TILEPCI_HOST)
	writel(q->chan, &tlr->regs->channel_reset);
#else
	{
		res = hv_dev_pwrite(tlr->hv_channel_ctl_fd, 0,
				    (HV_VirtAddr)&q->chan,
				    sizeof(q->chan),
				    PCIE_CHANNEL_CTL_CHANNEL_RESET_OFF);
		if (res != sizeof(q->chan))
			ERR("Tile channel reset failed, %d\n", q->chan);
	}
#endif
#endif // 0
	q->state = CMD_Q_ST_RESET_STARTED;

 exit:
	spin_unlock_irqrestore(&q->lock, irq_flags);
	return res;

	return 0;
}


/*
 * If the specified command queue has started reset, inform the
 * completion handler that it should discard all incoming completions.
 * Once this function completes, it is safe to release all the command
 * queue lists.
 */
static int tlr_zc_cmd_q_enable_reset_discard(struct tlr_zc_cmd_q *q)
{
	unsigned long irq_flags;
	int res = 0;

	spin_lock_irqsave(&q->lock, irq_flags);

	/*
	 * Only makes sense to discard if we've started (and not
	 * completed) a channel reset.
	 */
	if (q->state == CMD_Q_ST_RESET_STARTED) {
		q->state = CMD_Q_ST_RESET_DISCARD;
		res = 0;

		/*
		 * We're now discarding completions, so free all our
		 * cmd lists that aren't currently in flight.  The
		 * 'posted queue' cannot be freed at this point
		 * because those commands haven't come back from the
		 * HV yet; some of them will still be doing DMA and
		 * some will come back complete-with-reset.
		 */
		__tlr_zc_cmd_q_free_list(q, &q->pend_q);
		__tlr_zc_cmd_q_free_list(q, &q->comp_q);
	}
	/*
	 * If multiple threads want to wait-for-discard, we shouldn't
	 * return errors to those that don't call in first.
	 */
	else if (q->state != CMD_Q_ST_RESET_DISCARD)
		res = -EINVAL;

	spin_unlock_irqrestore(&q->lock, irq_flags);
	return res;
}


static int tlr_zc_cmd_q_wait_for_need_soc(struct tlr_zc_cmd_q *q)
{
	unsigned long irq_flags;
	int res = 0;

	spin_lock_irqsave(&q->lock, irq_flags);

	/*
	 * If this is called on a command queue that is already
	 * connected or hasn't gotten into discard mode, something's
	 * gone very wrong.
	 */
	if (q->state == CMD_Q_ST_CONNECTED ||
	    q->state == CMD_Q_ST_RESET_STARTED) {
		res = -EIO;
		goto exit;
	}

	/* Check to see if we're already ready to connect. */
	if (q->state == CMD_Q_ST_NEED_SOC) {
		res = 0;
		goto exit;
	}

	/*
	 * At this point, we need to wait for any outstanding
	 * completions to return.  When that's done, we can advance to
	 * the 'need SoC' state.
	 */
	while (q->ncomp_tot != q->nreq_tot) {
		int wret;

		spin_unlock_irqrestore(&q->lock, irq_flags);
		wret = wait_event_interruptible(q->reset_drain_queue,
						(q->ncomp_tot == q->nreq_tot));
		spin_lock_irqsave(&q->lock, irq_flags);

		if (wret != 0) {
			res = -ERESTARTSYS;
			goto exit;
		}
	}

	q->state = CMD_Q_ST_NEED_SOC;

 exit:
	spin_unlock_irqrestore(&q->lock, irq_flags);
	return res;
}


static int tlr_zc_stream_init(struct tlr_zc_stream *stream,
			      tlr_zc_cmd_type_t cmd_type)
{
	struct tlr_zc_cmd *cmds;
	struct tlr_zc_cmd *cmd;
	struct tlr_zc_cmd_q *q;
	int ncmds;
	int cmds_sz;
	int err;
	int i;

	tlr_list_init(&stream->cmd_q_free_list);
	spin_lock_init(&stream->cmd_q_lock);

	/* Allocate zero-copy commands. */
	ncmds = PCIE_CMD_QUEUE_ENTRIES;
	cmds_sz = ncmds * sizeof(*cmds);
	cmds = tlr_alloc_mem(cmds_sz);
	if (cmds == NULL) {
		err = -ENOMEM;
		goto cmds_alloc_failed;
	}	
	stream->ncmd = ncmds;
	stream->cmds_sz = cmds_sz;
	stream->cmds = cmds;
	memset(cmds, 0, cmds_sz);

	/* Initialize the new commands. */
	for (i = 0; i < ncmds; i++) {
		cmd = &cmds[i];

		tlr_zc_cmd_init(cmd, NULL, stream->tlr,
				TLR_ZC_CMD_UNKNOWN, TLR_COOKIE_UNKNOWN,
				&stream->cmd_q_free_list,
				&stream->cmd_q_lock);
	}

	q = kmalloc(sizeof(*q), GFP_KERNEL);
	if (q == NULL) {
		err = -ENOMEM;
		goto q_alloc_failed;
	}
	stream->cmd_queue = q;
	
	err = tlr_zc_cmd_q_startup(q, cmd_type, stream->tlr, stream->index);
	if (err)
		goto q_alloc_failed;

	return 0;

 q_alloc_failed:
	q = stream->cmd_queue;
	if (q != NULL) {
		kfree(q);
		stream->cmd_queue = NULL;
	}

	if (stream->cmds != NULL) {
		tlr_free_mem(stream->cmds, stream->cmds_sz);
		stream->cmds = NULL;
	}
 cmds_alloc_failed:
	return err;
}


/****************************************************************/
/*		       Zero-Copy mmap() Region                  */
/****************************************************************/

static int tlr_zc_mmap_check_num_frags(struct tlr_mmap_state *map, int frag_idx)
{
	int min_frags = frag_idx + 1;
	int new_num_frags;
	struct tlr_buf_fragment *frags;

	if (min_frags <= map->num_frags)
		return 0;

	/* Always grow to some power of two. */
	new_num_frags = 1;
	while (new_num_frags < min_frags)
		new_num_frags *= 2;

	frags = kmalloc(sizeof(*frags) * new_num_frags, GFP_KERNEL);
	if (frags == NULL)
		return -ENOMEM;

	memset(frags, 0, sizeof(*frags) * new_num_frags);
	if (map->num_frags > 0) {
		memcpy(frags, map->frags, sizeof(*frags) * map->num_frags);
		kfree(map->frags);
	}

	map->frags = frags;
	map->num_frags = new_num_frags;

	return 0;
}


static int tlr_zc_mmap_alloc_frag(struct tlr_buf_fragment *frag)
{
	int order;
	int i;

	order = get_order(TILEPCI_MMAP_GRANULARITY);

	/*
	 * Older kernels don't provide __GFP_ZERO, so we zero the
	 * pages manually.
	 */
	frag->page = alloc_pages(GFP_HIGHUSER, order);
	HID_INT_TRACE("alloced page = %p\n", frag->page);
	if (frag->page == NULL) {
		return -ENOMEM;
	} else {
		/*
		 * Zero the pages and bump refcnts for those that
		 * alloc_pages() hasn't already bumped.
		 */
		BUG_ON(in_interrupt());
		clear_highpage(frag->page);
		for (i = 1; i < (1 << order); i++) {
			get_page(frag->page + i);
			clear_highpage(frag->page + i);
		}
		return 0;
	}
}


static void tlr_zc_mmap_release(struct tlr_mmap_state *map)
{
	HID_INT_TRACE("release num_frags = %d\n", map->num_frags);

	if (map->num_frags) {
		int order = get_order(TILEPCI_MMAP_GRANULARITY);
		int i, j;
		for (i = 0; i < map->num_frags; i++) {
			struct page *page = map->frags[i].page;
			if (page != NULL) {
				for (j = 0; j < (1 << order); j++)
					put_page(page + j);
				map->frags[i].page = NULL;
			}
		}

		kfree(map->frags);
		map->frags = NULL;
		map->num_frags = 0;
	}
}


static void tlr_zc_vma_open(struct vm_area_struct *vma)
{
	struct tlr_mmap_state *map = vma->vm_private_data;

	down(&map->mutex);

	map->ref_cnt++;

	up(&map->mutex);
}


static void tlr_zc_vma_close(struct vm_area_struct *vma)
{
	struct tlr_mmap_state *map = vma->vm_private_data;

	down(&map->mutex);

	map->ref_cnt--;
	if (map->ref_cnt == 0)
		tlr_zc_mmap_release(map);

	up(&map->mutex);
}


#ifdef USE_VM_FAULT
static int tlr_zc_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
#else
static struct page *tlr_zc_vma_nopage(struct vm_area_struct *vma,
				      unsigned long address, int *type)
#endif
{
#ifdef USE_VM_FAULT
	unsigned long address = (unsigned long)vmf->virtual_address;
	int ret;
#else
	struct page *ret;
#endif
	unsigned long addr_off;
	unsigned long mmap_off;
	unsigned long offset;
	unsigned long frag_idx;
	unsigned long frag_off;
	struct page *page;
	int err;
	int have_mutex;
	struct tlr_mmap_state *map;

	ret = RETURN_SIGBUS;

	have_mutex = FALSE;
	map = vma->vm_private_data;

	/* Figure out the fragment index and offset within that fragment. */
	mmap_off = vma->vm_pgoff << PAGE_SHIFT;
	addr_off = (address - vma->vm_start);
	offset = mmap_off + addr_off;
	frag_idx = offset / TILEPCI_MMAP_GRANULARITY;
	frag_off = offset % TILEPCI_MMAP_GRANULARITY;

	HID_INT_TRACE("map = %p, frag_idx = %lx, frag_off = %lx\n",
		      map, frag_idx, frag_off);


	err = down_interruptible(&map->mutex);
	if (err != 0) {
		HID_ERR_TRACE("Interrupted\n");
		goto exit;
	}
	have_mutex = TRUE;

	/* Lookup fragment info or allocate a new fragment. */
	if (tlr_zc_mmap_check_num_frags(map, frag_idx) == -ENOMEM) {
		HID_ERR_TRACE("Couldn't resize frag array.\n");
		ret = RETURN_OOM;
		goto exit;
	}
	page = map->frags[frag_idx].page;
	if (page == NULL) {
		if (tlr_zc_mmap_alloc_frag(&map->frags[frag_idx]) == -ENOMEM) {
			HID_ERR_TRACE("Couldn't alloc pages.\n");
			ret = RETURN_OOM;
			goto exit;
		}
		page = map->frags[frag_idx].page;
	}

	/* Return the appropriate page within that fragment. */
	page += (frag_off >> PAGE_SHIFT);
	HID_INT_TRACE("returning PA = %llx\n",
		      (unsigned long long) page_to_phys(page));
	get_page(page);

#ifdef USE_VM_FAULT
	ret = 0;
	vmf->page = page;
#else
	ret = page;
	if (type != NULL)
		*type = VM_FAULT_MINOR;
#endif

 exit:
	if (have_mutex) {
		up(&map->mutex);
		have_mutex = FALSE;
	}

	return ret;
}


static struct vm_operations_struct tlr_zc_vm_ops = {
	.open	= tlr_zc_vma_open,
	.close	= tlr_zc_vma_close,
#ifdef USE_VM_FAULT
	.fault  = tlr_zc_vma_fault,
#else
	.nopage	= tlr_zc_vma_nopage,
#endif
};


static int tlr_zc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct tlr_zc_stream *stream;
	struct tlr_zc_cmd_q *q;

	stream = filp->private_data;
	q = stream->cmd_queue;

	/* All mappings must be performed with MAP_SHARED. */
	if (!(vma->vm_flags & VM_SHARED)) {
		HID_ERR_TRACE("mmap flags must include VM_SHARED\n");
		return -EINVAL;
	}

	vma->vm_ops = &tlr_zc_vm_ops;
	vma->vm_flags |= VM_LOCKED | VM_RESERVED;
	vma->vm_private_data = &q->mmap_state;

	tlr_zc_vma_open(vma);

	return 0;
}


/****************************************************************/
/*		      Zero-Copy Syscall Support                 */
/****************************************************************/

#ifdef HAVE_UNLOCKED_IOCTL
static long tlr_zc_ioctl(struct file *filp,
			 unsigned int cmd, unsigned long arg)
#else
static int tlr_zc_ioctl(struct inode *inode, struct file *filp,
			unsigned int cmd, unsigned long arg)
#endif
{
#ifdef HAVE_UNLOCKED_IOCTL
	long			 ret;
#else
	int			 ret;
#endif
	struct tlr_zc_stream    *stream;
	struct tlr_zc_cmd_q	*q;
	u32		 	 ncmd;
	u32		 	 ncmd_min_read;

#if defined(TILEPCI_ENDP)
	struct page 		*mapped_pages[MAX_MAPPED_PAGES_PER_XFER];	
	tilegxpci_buf_info_t	 buf_info;
	unsigned long 		 offset;
	unsigned long 		 start_page_addr;
	unsigned long		 start_pfn;
	unsigned long		 end_pfn;
	unsigned long		 page_count;
	int			 nmapped_pages;
	void			*va;
#endif /* defined(TILEPCI_ENDP) */

	ret = 0;
	stream = filp->private_data;
	q = stream->cmd_queue;

	switch (cmd) {
	case TILEPCI_IOC_GET_NCMD:
		ret = put_user(q->ncmd_wanted, (u32 __user *)arg);
		break;
	case TILEPCI_IOC_SET_NCMD:
		ncmd = (u32)arg;
		HID_WRK_TRACE("Setting ZC channel %d ncmds to %d\n",
			      q->chan, ncmd);
		ret = tlr_zc_cmd_q_ncmd_set(stream, ncmd);
		break;
	case TILEPCI_IOC_GET_MIN_READ_COMPS:
		HID_WRK_TRACE("Getting ZC channel %d ncmd_min_read %d\n",
			      q->chan, q->ncmd_min_read);
		ret = put_user(q->ncmd_min_read, (u32 __user *)arg);
		break;
	case TILEPCI_IOC_SET_MIN_READ_COMPS:
		ncmd_min_read = (u32)arg;
		HID_WRK_TRACE("Setting ZC channel %d ncmds to %d\n",
			      q->chan, ncmd_min_read);
		ret = 0;
		q->ncmd_min_read = ncmd_min_read;
		if (tlr_list_len(&q->comp_q, &q->lock) >= q->ncmd_min_read) {
			wmb();  /* ensure visibility before waking */
			wake_up_interruptible(&q->comp_queue);
		}
		break;
#if defined(TILEPCI_ENDP)
        case TILEPCI_IOC_REG_BUF:
		ret = copy_from_user(&buf_info, 
				     (tilegxpci_buf_info_t __user *)arg,
				     sizeof(tilegxpci_buf_info_t));
		if (ret > 0)
			return -EFAULT;
                HID_WRK_TRACE("Getting user-space buffer %p with %lld bytes\n",
                      	      (void*) buf_info.va, buf_info.size);

		/* 
		 * Determine which pages are being accessed and bump ref counts. 
		 */
		offset = (unsigned long)buf_info.va & ~PAGE_MASK;
		start_page_addr = (unsigned long)buf_info.va & PAGE_MASK;
		start_pfn = start_page_addr >> PAGE_SHIFT;
		end_pfn = ((unsigned long)buf_info.va + buf_info.size - 1) >> 
			PAGE_SHIFT;
		page_count = end_pfn - start_pfn + 1;

		down_read(&current->mm->mmap_sem);
		/* We have to set writable so that all the pages are populated. */
		nmapped_pages = get_user_pages(current, current->mm,
					       start_page_addr,
					       page_count,
					       1, 0,
				      	       mapped_pages, NULL);
		up_read(&current->mm->mmap_sem);
		if (nmapped_pages <= 0)
			return -EINVAL;

                va = page_address(mapped_pages[0]);
		ret = gxio_trio_register_page(&stream->trio_state->trio,
                                              stream->trio_state->trio.asid,
                                              va,
                                              buf_info.size,
                                              0);
		if (ret < 0)
			TRACE("register failed\n");

                break;
#endif /* defined(TILEPCI_ENDP) */
	default:
		ret = -ENOTTY;
		HID_ERR_TRACE("Exit ENOTTY\n");
		goto exit;
		break;
	}

 exit:
	return ret;
}


#if defined(TILEPCI_ENDP)
#ifdef CONFIG_COMPAT
static long tlr_zc_compat_ioctl(struct file *filp,
                                unsigned int cmd, unsigned long arg)
{
	/* Sign-extend the argument so it can be used as a pointer. */
#ifdef HAVE_UNLOCKED_IOCTL
        return tlr_zc_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
#else
        return tlr_zc_ioctl(filp->f_dentry->d_inode, filp, cmd,
                            (unsigned long)compat_ptr(arg));
#endif
}
#endif
#endif /* defined(TILEPCI_ENDP) */


static ssize_t tlr_zc_read(struct file *filp, char __user *buf, size_t count,
			   loff_t *f_pos)
{
	ssize_t			 ret;
	struct tlr_pcie_dev	*tlr;
	struct tlr_zc_cmd_q	*q;
	int			 err;
	int			 i;
	int			 have_mutex;
	int			 wait;
	u32		 	 nxfer;
	tilepci_xfer_comp_t	*xfer;
	struct tlr_zc_cmd	*cmd;
	struct tlr_zc_stream    *stream = filp->private_data;

	ret = 0;
	q = stream->cmd_queue;
	tlr = q->tlr;
	have_mutex = FALSE;
	wait = (filp->f_flags & O_NONBLOCK) == 0;

	HID_FOP_TRACE("Entered ZC channel %d: wait %d\n",
		      q->chan, wait);

	HID_FOP_TRACE("q[%p]: type %d state %d chan %d\n",
		      q, q->type, q->state, q->chan);
	if (count == 0) {
		ret = 0;
		HID_EX_TRACE("Exit\n");
		goto exit;
	}

	if (!access_ok(VERIFY_WRITE, buf, count)) {
		ret = -EFAULT;
		HID_ERR_TRACE("Exit EFAULT\n");
		goto exit;
	}

	nxfer = count / sizeof(tilepci_xfer_comp_t);

	if ((nxfer * sizeof(tilepci_xfer_comp_t)) != count) {
		ret = -EINVAL;
		HID_ERR_TRACE("Exit EINVAL\n");
		goto exit;
	}

	/* Grab the command queue read lock. */
	err = down_interruptible(&q->rd_xfer_mutex);
	if (err != 0) {
		HID_ERR_TRACE("Exit ERESTARTSYS\n");
		ret = -ERESTARTSYS;
		goto exit;
	}
	have_mutex = TRUE;

	/* Abandon this operation if the queue is poisoned due to reset. */
	if (q->chip_reset_poison) {
		ret = -ENXIO;
		HID_ERR_TRACE("Exit ENXIO\n");
		goto exit;
	}

	for (i = 0; i < nxfer; i++) {
		xfer = &q->rd_xfer_comps[i];

		if (i >= q->ncmd_min_read)
			wait = FALSE;

		err = tlr_zc_cmd_q_get_comp(q, wait, nxfer - i, &cmd);
		if (err != 0) {
			if (i == 0) {
				ret = err;
				HID_ERR_TRACE("Exit %d\n", (int)ret);
				goto exit;
			} else {
				ret = 0;
				break;
			}
		}

		xfer->addr = cmd->usr_addr;
		xfer->len = cmd->comp_len;
		xfer->cookie = cmd->cookie;
		xfer->flags = cmd->flags;

		tlr_zc_cmd_q_put_free(q, cmd);
	}

	ret = i * sizeof(*xfer);

	err = copy_to_user(buf, q->rd_xfer_comps, ret);
	if (err != 0) {
		HID_ERR_TRACE("Exit EFAULT\n");
		ret = -EFAULT;
		goto exit;

	}

	HID_EX_TRACE("Exit %d\n", (int)ret);

 exit:
	if (have_mutex) {
		up(&q->rd_xfer_mutex);
		have_mutex = FALSE;
	}

	return ret;
}


static ssize_t tlr_zc_write(struct file *filp, const char __user *buf,
			    size_t count, loff_t *f_pos)
{
	struct tlr_zc_stream 	 *stream = filp->private_data;
	struct tlr_list		  tmp_cmds;	
	struct tlr_pcie_dev	 *tlr;
	struct tlr_zc_cmd	 *cmd;
	struct tlr_zc_cmd_q	 *q;
	tilepci_xfer_req_t	 *xfer;	
	ssize_t			  ret;	
	u32		 	  i, nxfer;
	int			  err, have_mutex, valid, wait;

#if defined(TILEPCI_ENDP)
	struct gxpci_queue_state *shared_queue_state;
	struct tlr_zc_stream     *shared_stream;
        gxio_trio_dma_queue_t    *dma_queue_desc; 
        spinlock_t 		 *lock;
	unsigned long 		  flags;
        unsigned int 		  dma_cmds_completed;
        unsigned int 		  index, index_start, index_end;
        volatile uint16_t 	  hw_cnt;
        uint16_t 		  batch_completions;
#endif /* definded(TILEPCI_ENDP) */

	ret = 0;
	q = stream->cmd_queue;
	tlr = q->tlr;
	have_mutex = FALSE;
	wait = (filp->f_flags & O_NONBLOCK) == 0;
	tlr_list_init(&tmp_cmds);

	HID_FOP_TRACE("Entered ZC channel %d: wait %d\n",
		      q->chan, wait);

	if (count == 0) {
		ret = 0;
		HID_EX_TRACE("Exit\n");
		goto exit;
	}

	nxfer = count / sizeof(tilepci_xfer_req_t);

	if ((nxfer * sizeof(tilepci_xfer_req_t)) != count) {
		ret = -EINVAL;
		HID_ERR_TRACE("Exit EINVAL\n");
		goto exit;
	}

	if (nxfer > q->ncmd_wanted) {
		ret = -ENOBUFS;
		HID_ERR_TRACE("Exit ENOBUF\n");
		goto exit;
	}

	/* Grab the command queue write lock. */
	err = down_interruptible(&q->wr_xfer_mutex);
	if (err != 0) {
		HID_ERR_TRACE("Exit ERESTARTSYS\n");
		ret = -ERESTARTSYS;
		goto exit;
	}
	have_mutex = TRUE;

	/* Abandon this operation if the queue is poisoned due to reset. */
	if (q->chip_reset_poison) {
		ret = -ENXIO;
		HID_ERR_TRACE("Exit ENXIO\n");
		goto exit;
	}

	err = copy_from_user(q->wr_xfer_reqs, buf, count);
	if (err < 0) {
		ret = -EFAULT;
		HID_ERR_TRACE("Exit EFAULT\n");
		goto exit;
	}

	for (i = 0; i < nxfer; i++) {
		xfer = &q->wr_xfer_reqs[i];
		valid = tlr_xfer_validate(q, xfer);
		if (!valid) {
			ret = -EFAULT;
			HID_ERR_TRACE("Exit  EFAULT\n");
			goto exit;
		}
	}

	/*
	 * At this point, the xfers are validated, so everything should
	 * proceed without static problems. Dynamic problems may still occur
	 * i.e. the user could still interrupt, etc
	 */

	for (i = 0; i < nxfer; i++) {
		xfer = &q->wr_xfer_reqs[i];

		err = tlr_zc_cmd_q_get_free(q, wait, &cmd);
		if (err != 0) {
			if ((i != 0) && (err == -EAGAIN)) {
				break;
			} else {
				ret = err;
				goto exit;
			}
		}

		__tlr_list_add_zc_cmd(&tmp_cmds, cmd);

		err = tlr_xfer_cvt(q, xfer, cmd);
		if (err != 0) {
			ret = err;
			goto exit;
		}
	}

	tlr_zc_cmd_q_pend_list(q, &tmp_cmds);

	/* ...and then post the buffers, adding SoC if needed. */
	post_cmds(q);
	ret = i * sizeof(tilepci_xfer_req_t);

#if defined(TILEPCI_ENDP)
	if (q->type == TLR_ZC_CMD_H2T) 
		dma_queue_desc = 
	        	&stream->h2t_dma_resource->pull_dma_queue_desc;
	else 
		dma_queue_desc = 
	        	&stream->t2h_dma_resource->pull_dma_queue_desc;

	lock = &stream->trio_state->trio_queue_lock;
	
	/* Lock down. */	
	spin_lock_irqsave(lock, flags);

        gxpci_zc_check_tile_cmds(stream);       	

	/* Unlock. */					
	spin_unlock_irqrestore(lock, flags);

	/* Lock down. */	
	spin_lock_irqsave(lock, flags);

        hw_cnt = gxio_trio_read_dma_queue_complete_count(dma_queue_desc);
        batch_completions = (uint16_t)(hw_cnt - 
        	(stream->trio_state->cmd_dma_chan_dequeued &
                 TRIO_PUSH_DMA_REGION_VAL__COUNT_RMASK));

	/* Dispatch all the pull DMA completions. */
	index_start = stream->trio_state->cmd_dma_chan_dequeued;
	index_end = index_start + batch_completions;
	for (i = index_start; i != index_end; i++) {
		index = stream->trio_state->cmd_dma_chans[i & 
			(GXPCI_STREAM_PULL_DMA_RING_LEN - 1)];
		if (q->type == TLR_ZC_CMD_H2T) {
			shared_stream = tlr_get_zc_h2t_stream(tlr, index);
			shared_queue_state = &shared_stream->h2t_queue_state;
		}
		else {
			shared_stream = tlr_get_zc_t2h_stream(tlr, index);
			shared_queue_state = &shared_stream->t2h_queue_state;
		}
	
		/* Update the number of completed pull DMA. */
		dma_cmds_completed = shared_queue_state->fetch_dmas_completed;
		shared_queue_state->host_cmds_posted +=
			shared_queue_state->host_cmds_cnt_array[
				dma_cmds_completed & 
					(PCIE_CMD_QUEUE_ENTRIES - 1)];
	
		shared_queue_state->fetch_dmas_completed++;
	}
	stream->trio_state->cmd_dma_chan_dequeued += batch_completions;

	gxpci_zc_parse_host_cmds(stream);

	/* Unlock. */					
	spin_unlock_irqrestore(lock, flags);

	/* Lock down. */	
	spin_lock_irqsave(lock, flags);

	gxpci_zc_process_host_cmds(stream);

	/* Unlock. */					
	spin_unlock_irqrestore(lock, flags);
#endif /* defined(TILEPCI_ENDP) */

	HID_EX_TRACE("Exit  ret %d\n", (int)ret);

 exit:
	tlr_zc_cmd_q_free_list(q, &tmp_cmds);
	if (have_mutex) {
		up(&q->wr_xfer_mutex);
		have_mutex = FALSE;
	}

	return ret;
}


static unsigned int tlr_zc_poll(struct file *filp, poll_table *table)
{
	unsigned int		 ret;
	int			 err;
	struct tlr_zc_stream    *stream;
	struct tlr_zc_cmd_q	*q;
	int			 have_rd_mutex;
	int			 have_wr_mutex;

	HID_FOP_TRACE("Entered\n");

	ret = 0;
	stream = filp->private_data;
	q = stream->cmd_queue;
	have_rd_mutex = FALSE;
	have_wr_mutex = FALSE;

	/* Add wait queues to the poll table; we don't actually wait here. */
	poll_wait(filp, &q->free_queue, table);
	poll_wait(filp, &q->comp_queue, table);

	/*
	 * Grab both the read and write semaphores so that this operation is
	 * ordered with respect to any other processes that may be reading
	 * or writing.  Are we allowed to return -ERESTARTSYS here?  Can't
	 * seem to find the appropriate documentation...
	 */
	err = down_interruptible(&q->rd_xfer_mutex);
	if (err != 0) {
		ret = -ERESTARTSYS;
		HID_ERR_TRACE("Exit ERESTARTSYS\n");
		goto exit;
	}
	have_rd_mutex = TRUE;

	err = down_interruptible(&q->wr_xfer_mutex);
	if (err != 0) {
		ret = -ERESTARTSYS;
		HID_ERR_TRACE("Exit ERESTARTSYS\n");
		goto exit;
	}
	have_wr_mutex = TRUE;

	/* Abandon this operation if the queue is poisoned due to reset. */
	if (q->chip_reset_poison) {
		ret = POLLERR;
		HID_ERR_TRACE("Exit  ENXIO\n");
		goto exit;
	}

	if (!tlr_list_empty(&q->comp_q, &q->lock))
		ret |= (POLLIN | POLLRDNORM);

	if (!tlr_list_empty(&q->free_q, &q->lock))
		ret |= (POLLOUT | POLLWRNORM);


 exit:
	if (have_wr_mutex) {
		up(&q->wr_xfer_mutex);
		have_wr_mutex = FALSE;
	}

	if (have_rd_mutex) {
		up(&q->rd_xfer_mutex);
		have_rd_mutex = FALSE;
	}

	HID_EX_TRACE("Exit\n");
	return ret;
}


static int tlr_zc_release(struct inode *inode, struct file *filp)
{
	int			 ret;
	struct tlr_zc_stream    *stream;
	struct tlr_zc_cmd_q	*q;
#if defined(TILEPCI_ENDP)
	struct semaphore	*mutex;
	spinlock_t 		*lock;
	unsigned long		 flags;
#endif 

	ret = 0;
	stream = filp->private_data;
	q = stream->cmd_queue;
	
#if defined(TILEPCI_ENDP)
	mutex = &stream->trio_state->trio_queue_mutex;
	lock = &stream->trio_state->trio_queue_lock;
#endif         

	HID_FOP_TRACE("Entered ZC channel %d\n", q->chan);

	/* Grab the queue read and write locks. */
	down(&q->rd_xfer_mutex);
	down(&q->wr_xfer_mutex);

	if (--q->open_count == 0) {
		/*
		 * Send a channel reset, then start discarding any completions
		 * that come back.  If we're already in reset, this has no
		 * effect.
		 */
		tlr_zc_cmd_q_start_reset(q);
		tlr_zc_cmd_q_enable_reset_discard(q);

		/*
		 * Clear chip_reset_poison before ncmd_set() so it doesn't
		 * assert.
		 */
		q->chip_reset_poison = 0;

		ret = __tlr_zc_cmd_q_ncmd_set(stream, TLR_ZC_CMD_Q_NCMD_INIT);

#if defined(TILEPCI_ENDP)
		/* Grab the TRIO-shared queue lock. */
	        down(mutex);

	        /* Lock down. */
        	spin_lock_irqsave(lock, flags);        	
#endif 
                /* Release per stream resources. */
                release_zc_stream(stream);

		/*
		 * Clear the command queue ready flag so that upon the next
		 * initial open, the queue is re-initialized by
		 * tlr_zc_cmd_q_init().
		 */
		q->is_ready = FALSE;

		wmb();  /* ensure memory visibility */
        
#if defined(TILEPCI_ENDP)
		/* Unlock. */
	        spin_unlock_irqrestore(lock, flags);

		up(mutex);
#endif 
	}

	HID_EX_TRACE("Exit OK\n");

	up(&q->wr_xfer_mutex);
	up(&q->rd_xfer_mutex);
	return ret;
}


static struct file_operations tlr_zc_ops = {
	.owner = THIS_MODULE,
	.read = tlr_zc_read,
	.write = tlr_zc_write,
	.poll = tlr_zc_poll,
	.release = tlr_zc_release,
#ifdef HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl = tlr_zc_ioctl,
#else
	.ioctl = tlr_zc_ioctl,
#endif
#if defined(TILEPCI_ENDP)
#ifdef CONFIG_COMPAT
	.compat_ioctl = tlr_zc_compat_ioctl,
#endif
#endif /* defined(TILEPCI_ENDP) */
	.mmap = tlr_zc_mmap,
};


int tlr_zc_open(struct inode *inode, struct file *filp)
{
        struct tlr_pcie_dev     *tlr =
                container_of(inode->i_cdev, struct tlr_pcie_dev, cdev);
	struct tlr_zc_cmd_q	*q;
	struct tlr_zc_stream	*stream;
#if defined(TILEPCI_ENDP)
	struct semaphore	*lock;
#endif
	tlr_zc_cmd_type_t 	 cmd_type;
	int			 ret;
	int			 err;
	int 			 minor;
	int			 stream_index;
	int			 have_rd_mutex;
	int			 have_wr_mutex;

#if defined(TILEPCI_HOST)
	if (tlr->drv_mismatch)
		return -EPERM;
#endif

	minor = MINOR(inode->i_rdev);
	if (minor >= FIRST_ZC_H2T_MINOR &&
	    minor <= LAST_ZC_H2T_MINOR) {
	    	stream_index = minor - FIRST_ZC_H2T_MINOR;
		stream = tlr_get_zc_h2t_stream(tlr, stream_index);
		cmd_type = TLR_ZC_CMD_H2T;
	} 
	else if (minor >= FIRST_ZC_T2H_MINOR &&
		 minor <= LAST_ZC_T2H_MINOR) {
		stream_index = minor - FIRST_ZC_T2H_MINOR;
		stream = tlr_get_zc_t2h_stream(tlr, stream_index);
		cmd_type = TLR_ZC_CMD_T2H;
	}
	else {
		ERR("Illegal minor number\n");
		return -ENXIO;
	}

#if defined(TILEPCI_ENDP)
	/* 
	 * This mutex is used in open time because we have to allocate some 
	 * memory, request interrupt and etc where spinlock can not work. 
	 */
	lock = &stream->trio_state->trio_queue_mutex;
#endif 	

	/* Set the private data to point at our queue. */
	ret = 0;
	have_rd_mutex = FALSE;
	have_wr_mutex = FALSE;
	q = stream->cmd_queue;
	if (q == NULL) {
		ERR("NULL cmd_q during open: pci stream = %d\n", stream_index);
		ret = -ENXIO;
		HID_ERR_TRACE("Exit ENXIO\n");
		goto exit;
	}
	filp->private_data = stream;

	/* Use the zero copy read, write, etc. */
	filp->f_op = &tlr_zc_ops;

	HID_FOP_TRACE("Enter: pci stream = %d\n", stream_index);

	/* Grab the queue read lock. */
	err = down_interruptible(&q->rd_xfer_mutex);
	if (err != 0) {
		HID_ERR_TRACE("Exit ERESTARTSYS\n");
		ret = -ERESTARTSYS;
		goto exit;
	}
	have_rd_mutex = TRUE;

	/* Grab the queue write lock. */
	err = down_interruptible(&q->wr_xfer_mutex);
	if (err != 0) {
		HID_ERR_TRACE("Exit ERESTARTSYS\n");
		ret = -ERESTARTSYS;
		goto exit;
	}
	have_wr_mutex = TRUE;

	/*
	 * If 'chip_reset_poison', some other file handle is
	 * still open on a dead queue; don't allow any more
	 * opens until that one goes away.
	 */
	if (q->chip_reset_poison) {
		ret = -ENXIO;
		HID_ERR_TRACE("Exit ENXIO - chip_reset_poison\n");
		goto exit;
	}

	if (!q->is_ready) {
#if defined(TILEPCI_ENDP)
		/* Grab the TRIO-shared queue lock. */
	        err = down_interruptible(lock);
        	if (err != 0) {
                	HID_ERR_TRACE("Exit ERESTARTSYS\n");
                	ret = -ERESTARTSYS;
                	goto exit;
        	}
#endif 
		ret = init_zc_stream(stream, cmd_type);
#if defined(TILEPCI_ENDP)
		/* Release the TRIO-shared queue lock. */
		up(lock);
#endif 
		if (ret != 0) {
			HID_ERR_TRACE("EXIT, init_zc_stream() failed\n");
			goto exit;
		}

		/*
		 * make sure the queue is actually initialized;
		 * we may have reset the chip and lost our connection.
		 */
		ret = tlr_zc_cmd_q_init(q);
		if (ret != 0) {
			HID_ERR_TRACE("Exit, init failed\n");
			goto exit;
		}

	}

	if (++q->open_count == 1) {
		/*
		 * Wait for the channel to reach the 'need start of
		 * connection' state.
		 */
		ret = tlr_zc_cmd_q_wait_for_need_soc(q);
		if (ret != 0) {
			HID_ERR_TRACE("Exit, need soc\n");
			goto exit;
		}

		ret = __tlr_zc_cmd_q_ncmd_set(stream, TLR_ZC_CMD_Q_NCMD_OPEN);
		if (ret != 0) {
			HID_ERR_TRACE("Exit, ncmd_set()\n");
			goto exit;
		}

		q->nreq_cur = 0;
		q->ncomp_cur = 0;
		q->ncomp_bytes_cur = 0;
		q->nreq_bytes_cur = 0;
	}

	HID_EX_TRACE("Exit ret %d\n", ret);

 exit:
	if (have_wr_mutex) {
		up(&q->wr_xfer_mutex);
		have_wr_mutex = FALSE;
	}
	if (have_rd_mutex) {
		up(&q->rd_xfer_mutex);
		have_rd_mutex = FALSE;
	}

	return ret;
}


/*
 * Helper function for releasing some or all of the zero-copy semaphores.
 * 'count' specifies the number of command queues to be released; this will
 * unlock queues 0 through count - 1.
 */
void release_zc_semaphores(struct tlr_pcie_dev *tlr, int count)
{
	int i;
	struct tlr_zc_stream *stream;
	struct tlr_zc_cmd_q *q;
	for (i = count - 1; i >= 0; i--) {
		stream = tlr_get_zc_h2t_stream(tlr, i);
		q = stream->cmd_queue;
		if (q) {
			up(&q->wr_xfer_mutex);
			up(&q->rd_xfer_mutex);
		}
	}

	for (i = count - 1; i >= 0; i--) {
		stream = tlr_get_zc_t2h_stream(tlr, i);
		q = stream->cmd_queue;
		if (q) {
			up(&q->wr_xfer_mutex);
			up(&q->rd_xfer_mutex);
		}
	}
}


/* Helper function for grabbing all the zero-copy semaphores. */
int grab_all_zc_semaphores(struct tlr_pcie_dev *tlr)
{
	int i;
	int ret;
	struct tlr_zc_stream *h2t_stream;
	struct tlr_zc_stream *t2h_stream;
	struct tlr_zc_cmd_q *h2t_q;
	struct tlr_zc_cmd_q *t2h_q;

	ret = 0;
	for (i = 0; i < GXPCI_HOST_ZC_QUEUE_COUNT; i++) {
		h2t_stream = tlr_get_zc_h2t_stream(tlr, i);
		h2t_q = h2t_stream->cmd_queue;
		if (h2t_q) {
			if (down_interruptible(&h2t_q->rd_xfer_mutex)) {
				ret = -ERESTARTSYS;
				goto err_exit;
			}

			if (down_interruptible(&h2t_q->wr_xfer_mutex)) {
				up(&h2t_q->rd_xfer_mutex);
				ret = -ERESTARTSYS;
				goto err_exit;
			}
		}

		t2h_stream = tlr_get_zc_t2h_stream(tlr, i);
		t2h_q = t2h_stream->cmd_queue;
		if (t2h_q) {
			if (down_interruptible(&t2h_q->rd_xfer_mutex)) {
				ret = -ERESTARTSYS;
				goto err_exit;
			}

			if (down_interruptible(&t2h_q->wr_xfer_mutex)) {
				up(&t2h_q->rd_xfer_mutex);
				ret = -ERESTARTSYS;
				goto err_exit;
			}
		}
		
	}
	return ret;

 err_exit:
	release_zc_semaphores(tlr, i);
	return ret;
}


int tlr_zc_init(struct tlr_pcie_dev *tlr)
{
	struct pci_dev *pci_dev = tlr->pci_dev;
	struct tlr_zc_stream *stream;
	int num_devs = GXPCI_HOST_ZC_QUEUE_COUNT;
	int err = -ENOMEM;
	int i, j;
#if defined(TILEPCI_ENDP)
	struct tlr_zc_stream *shared_stream;
	int shared_index;
#endif /* defined(TILEPCI_ENDP) */

	/* H2T ZC stream init. */ 
	for (i = 0; i < num_devs; i++) {
		stream = tlr_get_zc_h2t_stream(tlr, i);
		stream = (struct tlr_zc_stream*)kmalloc(sizeof(*stream),
				  			GFP_KERNEL);
		if (stream == NULL)
			goto stream_alloc_zc_h2t_failed;

		tlr_set_zc_h2t_stream(tlr, i, stream);
		memset(stream, 0, sizeof(*stream));

		stream->tlr = tlr;
		stream->pci_dev = pci_dev;
		stream->index = i;

#if defined(TILEPCI_ENDP)
		/* Initialize TRIO-shared queue resources. */
		if (i < ZC_QUEUE_TRIO_NUM) {
			stream->trio_state = 
				kmalloc(sizeof(struct trio_resource), 
					GFP_KERNEL);
			if (stream->trio_state == NULL)
				goto stream_alloc_zc_h2t_failed;
			memset(stream->trio_state, 0, 
			       sizeof(struct trio_resource));				

			sema_init(&stream->trio_state->trio_queue_mutex, 1);
			spin_lock_init(&stream->trio_state->trio_queue_lock);

			stream->h2t_dma_resource = 
				kmalloc(sizeof(struct gxpci_dma_resource), 
					GFP_KERNEL);
			if (stream->h2t_dma_resource == NULL)
				goto stream_alloc_zc_h2t_failed;
			memset(stream->h2t_dma_resource, 0, 
			       sizeof(struct gxpci_dma_resource));						
		}
		else { /* Share with other queues. */
			shared_index = i % ZC_QUEUE_TRIO_NUM; 
			shared_stream = 
				tlr_get_zc_h2t_stream(tlr, shared_index);
			
			stream->trio_state = shared_stream->trio_state;
			stream->h2t_dma_resource = 
				shared_stream->h2t_dma_resource;
		}
#else
        	/* 
		 * Get some memory for the device's command and completion 
		 * arrays. 
		 */
        	stream->h2t_buffer_cmd_array =
                	pci_alloc_consistent(pci_dev, 
					     PCIE_HOST_BUFFER_CMD_ARRAY_SIZE,
                                             &stream->h2t_buffer_cmd_handle); 
        	stream->h2t_completion_array =
                	pci_alloc_consistent(pci_dev, 
					     PCIE_HOST_COMPLETION_ARRAY_SIZE,
                                     	     &stream->h2t_completion_handle); 

        	if ((stream->h2t_buffer_cmd_array == 0) || 
		    (stream->h2t_completion_array == 0))
                	goto array_alloc_zc_h2t_failed;

		memset(stream->h2t_buffer_cmd_array, 0, 
		       PCIE_HOST_BUFFER_CMD_ARRAY_SIZE);
        	memset(stream->h2t_completion_array, 0, 
		       PCIE_HOST_COMPLETION_ARRAY_SIZE);

		/* Map in the host queue registers. */
                err = -EIO;
                stream->h2t_regs = ioremap(pci_resource_start(pci_dev, 0) +
			GXPCI_HOST_ZC_QUEUE_H2T_REGS_OFFSET +
                        GXPCI_HOST_CHAR_REGS_MAP_SIZE * i,
                        sizeof(struct gxpci_host_queue_regs));
                if (stream->h2t_regs == NULL)
                        goto ioremap_zc_h2t_failed;

                /* Map in the interrupt triggering bus address. */
                stream->intr_regs = ioremap(pci_resource_start(pci_dev, 0) +
                        GXPCI_HOST_ZC_QUEUE_MMI_REGS_OFFSET,
                        sizeof(uint64_t));
                if (stream->intr_regs == NULL)
                        goto ioremap_intr_zc_h2t_failed;
#endif /* !defined(TILEPCI_ENDP) */

		err = tlr_zc_stream_init(stream, TLR_ZC_CMD_H2T);
		if (err != 0)
			goto stream_init_zc_h2t_failed;

                continue;

stream_init_zc_h2t_failed:	
#if defined(TILEPCI_HOST)
		if (stream->intr_regs)
			iounmap(stream->intr_regs);
ioremap_intr_zc_h2t_failed:
		if (stream->h2t_regs)
			iounmap(stream->h2t_regs);
ioremap_zc_h2t_failed:
array_alloc_zc_h2t_failed:
	        if (stream->h2t_buffer_cmd_array)
        	        pci_free_consistent(pci_dev, 
        	        		    PCIE_HOST_BUFFER_CMD_ARRAY_SIZE,
					    stream->h2t_buffer_cmd_array,
                        	            stream->h2t_buffer_cmd_handle);

	        if (stream->h2t_completion_array)
        	        pci_free_consistent(pci_dev, 
        	        		    PCIE_HOST_COMPLETION_ARRAY_SIZE,
                	                    stream->h2t_completion_array,
                        	            stream->h2t_completion_handle);
#endif /* defined(TILEPCI_HOST) */ 
stream_alloc_zc_h2t_failed:
#if defined(TILEPCI_ENDP)
		if (stream->h2t_dma_resource)
			kfree(stream->h2t_dma_resource);
		if (stream->trio_state)			
			kfree(stream->trio_state);
#endif /* defined(TILEPCI_ENDP) */ 
		if (stream)			
			kfree(stream);
		tlr_set_zc_h2t_stream(tlr, i, NULL);
		
		break;
	}

	/* If no successful H2T streams, return err. */
	if (i == 0)
		return err;

	/* T2H ZC stream init. */
	for (j = 0; j < num_devs; j++) {
		stream = tlr_get_zc_t2h_stream(tlr, j);
		stream = (struct tlr_zc_stream*)kmalloc(sizeof(*stream),
							GFP_KERNEL);
		if (stream == NULL)
			goto stream_alloc_zc_t2h_failed;

		tlr_set_zc_t2h_stream(tlr, j, stream);
		memset(stream, 0, sizeof(*stream));

		stream->tlr = tlr;
		stream->pci_dev = pci_dev;
		stream->index = j;

#if defined(TILEPCI_ENDP)
		/* Initialize TRIO-shared queue resources. */		
                if (j < ZC_QUEUE_TRIO_NUM) {
			stream->trio_state = 
				kmalloc(sizeof(struct trio_resource),
					GFP_KERNEL);
			if (stream->trio_state == NULL)
				goto stream_alloc_zc_t2h_failed; 
			memset(stream->trio_state, 0, 
			       sizeof(struct trio_resource));
			       
			sema_init(&stream->trio_state->trio_queue_mutex ,1);
			spin_lock_init(&stream->trio_state->trio_queue_lock);

			stream->t2h_dma_resource = 
				kmalloc(sizeof(struct gxpci_dma_resource), 
					GFP_KERNEL);
			if (stream->t2h_dma_resource == NULL)
				goto stream_alloc_zc_t2h_failed;
			memset(stream->t2h_dma_resource, 0, 
			       sizeof(struct gxpci_dma_resource));
                }
                else { /* Share with other queues. */
                        shared_index = j % ZC_QUEUE_TRIO_NUM;
       			shared_stream = 
				tlr_get_zc_t2h_stream(tlr, shared_index);
				
			stream->trio_state = shared_stream->trio_state;
			stream->t2h_dma_resource =
				shared_stream->t2h_dma_resource;				                                  
                }
#else
        	/* 
		 * Get some memory for the device's command and completion 
		 * arrays. 
		 */
        	stream->t2h_buffer_cmd_array =
                	pci_alloc_consistent(pci_dev, 
					     PCIE_HOST_BUFFER_CMD_ARRAY_SIZE,
                                             &stream->t2h_buffer_cmd_handle); 
        	stream->t2h_completion_array =
                	pci_alloc_consistent(pci_dev, 
					     PCIE_HOST_COMPLETION_ARRAY_SIZE,
                                     	     &stream->t2h_completion_handle); 

        	if ((stream->t2h_buffer_cmd_array == 0) || 
		    (stream->t2h_completion_array == 0))
                	goto array_alloc_zc_t2h_failed;

		memset(stream->t2h_buffer_cmd_array, 0, 
		       PCIE_HOST_BUFFER_CMD_ARRAY_SIZE);
        	memset(stream->t2h_completion_array, 0, 
		       PCIE_HOST_COMPLETION_ARRAY_SIZE);

                /* Map in the host queue registers. */
                err = -EIO;
                stream->t2h_regs = ioremap(pci_resource_start(pci_dev, 0) +
			GXPCI_HOST_ZC_QUEUE_T2H_REGS_OFFSET +
                        GXPCI_HOST_CHAR_REGS_MAP_SIZE * j,
                        sizeof(struct gxpci_host_queue_regs));
                if (stream->t2h_regs == NULL)
                        goto ioremap_zc_t2h_failed;

                /* Map in the interrupt triggering bus address. */
                stream->intr_regs = ioremap(pci_resource_start(pci_dev, 0) +
                        GXPCI_HOST_ZC_QUEUE_MMI_REGS_OFFSET,
                        sizeof(uint64_t));
                if (stream->intr_regs == NULL)
                        goto ioremap_intr_zc_t2h_failed;
#endif /* !defined(TILEPCI_ENDP) */ 

		err = tlr_zc_stream_init(stream, TLR_ZC_CMD_T2H);
		if (err != 0)
			goto stream_init_zc_t2h_failed;

                continue;

stream_init_zc_t2h_failed:
#if defined(TILEPCI_HOST)	
		if (stream->intr_regs)
			iounmap(stream->intr_regs);
ioremap_intr_zc_t2h_failed:
		if (stream->t2h_regs)
			iounmap(stream->t2h_regs);
ioremap_zc_t2h_failed:
array_alloc_zc_t2h_failed:
	        if (stream->t2h_buffer_cmd_array)
        	        pci_free_consistent(pci_dev, 
        	        		    PCIE_HOST_BUFFER_CMD_ARRAY_SIZE,
					    stream->t2h_buffer_cmd_array,
                        	            stream->t2h_buffer_cmd_handle);

	        if (stream->t2h_completion_array)
        	        pci_free_consistent(pci_dev, 
        	        		    PCIE_HOST_COMPLETION_ARRAY_SIZE,
                	                    stream->t2h_completion_array,
                        	            stream->t2h_completion_handle);
#endif /* defined(TILEPCI_HOST) */ 
stream_alloc_zc_t2h_failed:
#if defined(TILEPCI_ENDP)		
		if (stream->t2h_dma_resource)
			kfree(stream->t2h_dma_resource);
		if (stream->trio_state)			
			kfree(stream->trio_state);
#endif /* defined(TILEPCI_ENDP) */ 
		if (stream)			
			kfree(stream);
		tlr_set_zc_t2h_stream(tlr, j, NULL);

		break;
	}

	/* 
	 * If no successful T2H streams, clean up H2T stream resources then 
	 * return err. 
	 */
	if (j == 0) {
		for (i = 0; i < num_devs; i++) {
			stream = tlr_get_zc_h2t_stream(tlr, i);
			if (stream) {	
#if defined(TILEPCI_HOST)				
				if (stream->intr_regs)
					iounmap(stream->intr_regs);
				if (stream->h2t_regs)
					iounmap(stream->h2t_regs);

		        	if (stream->h2t_buffer_cmd_array)
        		       		pci_free_consistent(pci_dev, 
        		       			PCIE_HOST_BUFFER_CMD_ARRAY_SIZE,
						stream->h2t_buffer_cmd_array,
                       	        		stream->h2t_buffer_cmd_handle);

		        	if (stream->h2t_completion_array)
        		       		pci_free_consistent(pci_dev, 
        		       			PCIE_HOST_COMPLETION_ARRAY_SIZE,
                        	            	stream->h2t_completion_array,
                       	        	    	stream->h2t_completion_handle);
#else
                                if (stream->h2t_dma_resource)
                                	kfree(stream->h2t_dma_resource);
				if (stream->trio_state)			
					kfree(stream->trio_state);
#endif /* !defined(TILEPCI_HOST) */ 	
				if (stream)			
					kfree(stream);
				tlr_set_zc_h2t_stream(tlr, i, NULL);
			}
		}

		return err;
	}

	return 0;
	
}


/*
 * Reset all write (and read?) streams so that they contain no data.
 * Also, poison any open file handles so that the holders must close
 * and open a new session before getting more data.
 *
 * This method must be called while the following are true:
 *  - the caller holds all read and write mutexes
 *  - the worker thread has been descheduled
 *  - is_ready = 0 so that no interrupts will be processed
 *  - the chip has been reset but not booted
 */
void tlr_zc_chip_reset(struct tlr_pcie_dev* tlr)
{
	int i;
	struct tlr_zc_cmd_q *q;

	for (i = 0; i < GXPCI_HOST_ZC_QUEUE_COUNT; i++) {
		q = tlr->zc_h2t_streams[i]->cmd_queue;
		if (q) {
#if defined(TILEPCI_HOST)
			/* Free irq. */
			if (q->irq_num >= 0)
				gxpci_free_irq(tlr->zc_h2t_streams[i]);
#endif /* defined(TILEPCI_HOST) */

			if (q->open_count > 0)
				q->chip_reset_poison = 1;

			if (q->is_ready)
				tlr_zc_cmd_q_free(q);
		}

		q = tlr->zc_t2h_streams[i]->cmd_queue;
		if (q) {
#if defined(TILEPCI_HOST)
			/* Free irq. */
			if (q->irq_num >= 0)
				gxpci_free_irq(tlr->zc_t2h_streams[i]);
#endif /* defined(TILEPCI_HOST) */

			if (q->open_count > 0)
                                q->chip_reset_poison = 1;

			if (q->is_ready)
				tlr_zc_cmd_q_free(q);
		}
	}
}


void tlr_zc_free(struct tlr_pcie_dev* tlr)
{
	int i;
        struct tlr_zc_cmd *cmds;
        struct tlr_zc_cmd_q *q;

        for (i = 0; i < GXPCI_HOST_ZC_QUEUE_COUNT; i++) {
	        cmds = tlr->zc_h2t_streams[i]->cmds;
                if (cmds)
	                tlr_free_mem(cmds, tlr->zc_h2t_streams[i]->cmds_sz);

	 	cmds = tlr->zc_t2h_streams[i]->cmds;
                if (cmds)
                        tlr_free_mem(cmds, tlr->zc_t2h_streams[i]->cmds_sz);
        }

	for (i = 0; i < GXPCI_HOST_ZC_QUEUE_COUNT; i++) {
		q = tlr->zc_h2t_streams[i]->cmd_queue;
                if (q) {
	                kfree(q);
                        tlr->zc_h2t_streams[i]->cmd_queue = NULL;
                }
 
                q = tlr->zc_t2h_streams[i]->cmd_queue;
                if (q) {
                        kfree(q);
                        tlr->zc_t2h_streams[i]->cmd_queue = NULL;
		}
	}
}
#endif /* #if GXPCI_HOST_ZC_QUEUE_COUNT */
