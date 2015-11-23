/*
 * Common Packet Queue driver code shared by the PF and the VF driver.
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

static const char driver_name[] = DRIVER_NAME_STRING;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37)
#define init_MUTEX(_m) sema_init(_m,1)
#endif

/** VM_RESERVED is no longer in Linux 3.7.x and up. */
#ifndef VM_RESERVED
#define VM_RESERVED	(VM_DONTEXPAND | VM_DONTDUMP)
#endif

// The PCIE version that this driver was written for.
#define DRIVER_VERSION \
  PCIE_VERSION_DEF(TILEGXPCI_MAJOR, TILEGXPCI_MINOR)

/*
 * The largest this can be set to is the maximum allocation size
 * in the kernel (PAGE_SIZE << (MAX_ORDER-1)); to increase that limit,
 * see the code setting MAX_ORDER in <linux/mmzone.h>.
 * Also see the HOST_RD_SEGMENT_MAX_SIZE test in tilegxpci_host.h.
 */
#if HOST_PQ_SEGMENT_MAX_SIZE > (PAGE_SIZE << (MAX_ORDER - 1))
# error HOST_PQ_SEGMENT_MAX_SIZE set too large for this configuration
#endif

/*
 * Flag indicating if the Packet Queue buffer allocation is NUMA aware.
 * Right now, this can be set only on hosts with IOMMU disabled, i.e. no
 * "intel_iommu=on" in the kernel parameter line.
 */
int pq_numa_capable = 0;
module_param(pq_numa_capable, int, S_IRUGO);
MODULE_PARM_DESC(pq_numa_capable,
	"Flag indicating if the Packet Queue buffer allocation is NUMA aware.");

/* Per-port host Packet Queue H2T queue number. */
static int pq_h2t_queues =
#ifdef TILEPCI_VF
	GXPCI_HOST_PQ_VF_H2T_COUNT;
#else
	GXPCI_HOST_PQ_H2T_COUNT;
#endif
module_param(pq_h2t_queues, int, S_IRUGO);
MODULE_PARM_DESC(pq_h2t_queues,
	"Number of H2T queues per Packet Queue interface.");

/* Per-port host Packet Queue T2H queue number. */
static int pq_t2h_queues =
#ifdef TILEPCI_VF
	GXPCI_HOST_PQ_VF_T2H_COUNT;
#else
	GXPCI_HOST_PQ_T2H_COUNT;
#endif
module_param(pq_t2h_queues, int, S_IRUGO);
MODULE_PARM_DESC(pq_t2h_queues,
	"Number of T2H queues per Packet Queue interface.");

/*
 * To support different size (in size[KMG]), NUMA node and H2T/T2H ring sharing
 * configurations for all the Packet Queue DMA ring buffers, we need to
 * configure [size, nid, share_h2t_t2h_ring] of the Packet Queue interfaces in
 * both T2H and H2T directions.
 * The Packet Queue T2H DMA ring buffers are configured with
 * "pq_t2h_mems=[<size>][,<numa_node>][,<share_h2t_t2h_ring>]
 *              [:[<size>][,<numa_node>][,<share_h2t_t2h_ring>]]*",
 * and the Packet Queue H2T DMA ring buffers are configured with
 * "pq_h2t_mems=[<size>][,<numa_node>][,<share_h2t_t2h_ring>]
 *              [:[<size>][,<numa_node>][,<share_h2t_t2h_ring>]]*",
 * with the relative position of the 3-tuple in the parameter string indicating
 * the Packet Queue interface number. In particular, the 3-tuple setting for the
 * H2T/T2H queues with the same interface number must be identical.
 *
 * Note: <size> (in size[KMG]) is always PAGE_SIZE aligned, and its MAX 
 * value is (HOST_PQ_SEGMENT_MAX_SIZE * HOST_PQ_SEGMENT_MAX_NUM); if it is
 * not specified, the default size of HOST_PQ_SEGMENT_MAX_SIZE is allocated.
 * numa_node is ranged from 0 to (MAX_NUMNODES - 1); if it is not specified, or
 * not online, the DMA ring buffer is allocated from the current NUMA node.
 * share_h2t_t2h_ring is set to 0 by default, which means that the H2T ring
 * buffer is separate from the T2H ring buffer; otherwise, a single DMA ring
 * buffer is shared for those two channels.
 */
static char *pq_t2h_mems;
module_param(pq_t2h_mems, charp, S_IRUGO);
MODULE_PARM_DESC(pq_t2h_mems, "Size (in size[KMG]), NUMA node and "
	"share_h2t_t2h_ring configurations for the Packet Queue T2H DMA buffer "
	"allocation.");

static char *pq_h2t_mems;
module_param(pq_h2t_mems, charp, S_IRUGO);
MODULE_PARM_DESC(pq_h2t_mems, "Size (in size[KMG]), NUMA node and "
	"share_h2t_t2h_ring configurations for the Packet Queue H2T DMA buffer "
	"allocation.");

/*
 * Size (in size[KMG]) of a single PA-contiguous Packet Queue segment to
 * allocate. If not specified, the default size of HOST_PQ_SEGMENT_MAX_SIZE is
 * used. The specified value must not be larger than HOST_PQ_SEGMENT_MAX_SIZE.
 */
static char *pq_ring_buf_size;
module_param(pq_ring_buf_size, charp, S_IRUGO); 
MODULE_PARM_DESC(pq_ring_buf_size, "Size in size[KMG] of a single "
		 "PA-contiguous Packet Queue segment, MAX 4M which is also the "
		 "default.");

/*
 * Size of a single PA-contiguous Packet Queue segment, which is 4MB by default.
 * This size can be changed with pq_ring_buf_size.
 */
static unsigned long pq_segment_size = HOST_PQ_SEGMENT_MAX_SIZE;

// Functions exported by packet_queue_vf.c
extern int tlr_packet_queue_open(struct tlr_pcie_dev *tlr, struct file *filp,
				 struct tlr_packet_queue_state *pq,
				 int channel);
extern void tlr_packet_queue_free(struct tlr_packet_queue_state *pq);
extern void tlr_packet_queue_free_shared(struct tlr_packet_queue_state *h2t_pq,
					 struct tlr_packet_queue_state *t2h_pq);

static int pq_parse_config(struct tlr_pcie_dev* tlr)
{
	unsigned long segment_size, ring_size;
	int num_segments, i;
	char pq_param_string[1024] = "";
	char *pq_sub_opt_ptr;
	char *pq_opt_ptr;
	char *pq_ptr;
	char *tmp;

#ifdef TILEPCI_VF
	if (pq_h2t_queues > GXPCI_HOST_PQ_VF_H2T_COUNT) {
		dev_err(&tlr->pci_dev->dev,
			"VF PQ H2T queue number invalid, use default value.\n");
		pq_h2t_queues = GXPCI_HOST_PQ_VF_H2T_COUNT;
	}
	if (pq_t2h_queues > GXPCI_HOST_PQ_VF_T2H_COUNT) {
		dev_err(&tlr->pci_dev->dev,
			"VF PQ T2H queue number invalid, use default value.\n");
		pq_t2h_queues = GXPCI_HOST_PQ_VF_T2H_COUNT;
	}
#else
	/* Packet Queue H2T/T2H queue number configurations. */
	int total_pq_queues = pq_h2t_queues + pq_t2h_queues;
	if (total_pq_queues == 0 ||
	    total_pq_queues > GXPCI_HOST_PQ_MAX_QUEUE_NUM) {
		dev_err(&tlr->pci_dev->dev,
			"PQ queue number invalid, use default value.\n");
		pq_h2t_queues = GXPCI_HOST_PQ_H2T_COUNT;
		pq_t2h_queues = GXPCI_HOST_PQ_T2H_COUNT;
	}
#endif

	tlr->pq_h2t_queues = pq_h2t_queues;
	tlr->pq_t2h_queues = pq_t2h_queues;

	/* Packet Queue single PA-contiguous segment size. */
	if (pq_ring_buf_size) {
		segment_size =
			ALIGN(memparse(pq_ring_buf_size, &tmp), PAGE_SIZE);
		if (!segment_size || (segment_size > pq_segment_size))
			dev_err(&tlr->pci_dev->dev, "PQ segment "
				"size invalid, use default value.\n");
		else
			pq_segment_size = segment_size;
	}

	/* Packet Queue H2T queue DMA ring buffer configurations. */
	if (pq_h2t_mems)
		strncpy(pq_param_string, pq_h2t_mems, sizeof(pq_param_string));
	pq_ptr = pq_param_string;

	for (i = 0; i < tlr->pq_h2t_queues; i++) {

		/* Initialize default configurations. */
		tlr->pq_h2t[i].ring_size = pq_segment_size;
		tlr->pq_h2t[i].numa_node = numa_node_id();
		tlr->pq_h2t[i].share_h2t_t2h_ring = 0;

		if ((pq_opt_ptr = strsep(&pq_ptr, ":")) != NULL) {
			pq_sub_opt_ptr = strsep(&pq_opt_ptr, ",");
			if (pq_sub_opt_ptr)
				tlr->pq_h2t[i].ring_size =
					memparse(pq_sub_opt_ptr, &tmp);
			pq_sub_opt_ptr = strsep(&pq_opt_ptr, ",");
			if (pq_sub_opt_ptr)
				tlr->pq_h2t[i].numa_node =
					simple_strtol(pq_sub_opt_ptr, NULL, 0);
			pq_sub_opt_ptr = strsep(&pq_opt_ptr, ",");
			if (pq_sub_opt_ptr)
				tlr->pq_h2t[i].share_h2t_t2h_ring =
					simple_strtol(pq_sub_opt_ptr, NULL, 0);
		}

		/* Error handlings. */
		if (tlr->pq_h2t[i].ring_size == 0)
			tlr->pq_h2t[i].ring_size = pq_segment_size;

		if (tlr->pq_h2t[i].ring_size % PAGE_SIZE) {
			dev_err(&tlr->pci_dev->dev,
				"PQ ring size not page-aligned, fixed.\n");
			tlr->pq_h2t[i].ring_size =
				ALIGN(tlr->pq_h2t[i].ring_size, PAGE_SIZE);
		}

		if (!node_online(tlr->pq_h2t[i].numa_node)) {
			dev_err(&tlr->pci_dev->dev,
				"PQ NUMA node invalid, use default value.\n");
			tlr->pq_h2t[i].numa_node = numa_node_id();
		}

		if (tlr->pq_h2t[i].share_h2t_t2h_ring) {
			tlr->pq_h2t[i].share_h2t_t2h_ring = 1;
			/* Shared buffer requires a pair of Packet Queues. */
			if (i >= tlr->pq_t2h_queues) {
				dev_err(&tlr->pci_dev->dev,
					"PQ shared ring H2T index larger "
					"than T2H, use separate ring.\n");
				tlr->pq_h2t[i].share_h2t_t2h_ring = 0;
			}
		}

		/* The number of segments must be a power of 2. */
		tlr->pq_h2t[i].num_segments =
			(tlr->pq_h2t[i].ring_size - 1) / pq_segment_size + 1;
		if (tlr->pq_h2t[i].num_segments &
		    (tlr->pq_h2t[i].num_segments - 1)) {
			dev_err(&tlr->pci_dev->dev,
				"PQ segment number must be a power of 2.\n");
			return -EINVAL;
		}
	
		if (tlr->pq_h2t[i].num_segments > HOST_PQ_SEGMENT_MAX_NUM) {
			dev_err(&tlr->pci_dev->dev,
				"PQ segment number too large.\n");
			return -EINVAL;
		}

		tlr->pq_h2t[i].segment_size = pq_segment_size;
		tlr->pq_h2t[i].segment_size_order =
			ffs(tlr->pq_h2t[i].segment_size) - 1;
	}

	/* Packet Queue T2H queue DMA ring buffer configurations. */
	if (pq_t2h_mems)
		strncpy(pq_param_string, pq_t2h_mems, sizeof(pq_param_string));
	pq_ptr = pq_param_string;

	for (i = 0; i < tlr->pq_t2h_queues; i++) {

		/* Initialize default configurations. */
		tlr->pq_t2h[i].ring_size = pq_segment_size;
		tlr->pq_t2h[i].numa_node = numa_node_id();
		tlr->pq_t2h[i].share_h2t_t2h_ring = 0;

		if ((pq_opt_ptr = strsep(&pq_ptr, ":")) != NULL) {
			pq_sub_opt_ptr = strsep(&pq_opt_ptr, ",");
			if (pq_sub_opt_ptr)
				tlr->pq_t2h[i].ring_size =
					memparse(pq_sub_opt_ptr, &tmp);
			pq_sub_opt_ptr = strsep(&pq_opt_ptr, ",");
			if (pq_sub_opt_ptr)
				tlr->pq_t2h[i].numa_node =
					simple_strtol(pq_sub_opt_ptr, NULL, 0);
			pq_sub_opt_ptr = strsep(&pq_opt_ptr, ",");
			if (pq_sub_opt_ptr)
				tlr->pq_t2h[i].share_h2t_t2h_ring =
					simple_strtol(pq_sub_opt_ptr, NULL, 0);
		}

		/* Error handlings. */
		if (tlr->pq_t2h[i].ring_size == 0)
			tlr->pq_t2h[i].ring_size = pq_segment_size;

		if (tlr->pq_t2h[i].ring_size % PAGE_SIZE) {
			dev_err(&tlr->pci_dev->dev,
				"PQ ring size not page-aligned, fixed.\n");
			tlr->pq_t2h[i].ring_size =
				ALIGN(tlr->pq_t2h[i].ring_size, PAGE_SIZE);
		}

		if (!node_online(tlr->pq_t2h[i].numa_node)) {
			dev_err(&tlr->pci_dev->dev,
				"PQ NUMA node invalid, use default value.\n");
			tlr->pq_t2h[i].numa_node = numa_node_id();
		}

		if (tlr->pq_t2h[i].share_h2t_t2h_ring) {
			tlr->pq_t2h[i].share_h2t_t2h_ring = 1;
			/* Shared buffer requires a pair of Packet Queues. */
			if (i >= tlr->pq_h2t_queues) {
				dev_err(&tlr->pci_dev->dev,
					"PQ shared ring T2H index larger "
					"than H2T, use separate ring.\n");
				tlr->pq_t2h[i].share_h2t_t2h_ring = 0;
			}
		}

		/* The number of segments must be a power of 2. */
		tlr->pq_t2h[i].num_segments =
			(tlr->pq_t2h[i].ring_size - 1) / pq_segment_size + 1;
		if (tlr->pq_t2h[i].num_segments &
		    (tlr->pq_t2h[i].num_segments - 1)) {
			dev_err(&tlr->pci_dev->dev,
				"PQ segment number must be a power of 2.\n");
			return -EINVAL;
		}
	
		if (tlr->pq_t2h[i].num_segments > HOST_PQ_SEGMENT_MAX_NUM) {
			dev_err(&tlr->pci_dev->dev,
				"PQ segment number too large.\n");
			return -EINVAL;
		}

		tlr->pq_t2h[i].segment_size = pq_segment_size;
		tlr->pq_t2h[i].segment_size_order =
			ffs(tlr->pq_t2h[i].segment_size) - 1;
	}

	/* Packet Queue shared DMA ring buffer error handlings. */
	for (i = 0; i < MIN(tlr->pq_h2t_queues, tlr->pq_t2h_queues); i++) {

		if (tlr->pq_h2t[i].share_h2t_t2h_ring !=
		    tlr->pq_t2h[i].share_h2t_t2h_ring) {
			dev_err(&tlr->pci_dev->dev,
				"PQ shared ring configuration incompatible "
				"for interface %d, use separate ring.\n", i);
			tlr->pq_h2t[i].share_h2t_t2h_ring = 0;
			tlr->pq_t2h[i].share_h2t_t2h_ring = 0;
			continue;
		}

		if (tlr->pq_h2t[i].share_h2t_t2h_ring) {

			if (tlr->pq_h2t[i].ring_size !=
			    tlr->pq_t2h[i].ring_size) {
				ring_size = MIN(tlr->pq_h2t[i].ring_size,
						tlr->pq_t2h[i].ring_size);
				num_segments = MIN(tlr->pq_h2t[i].num_segments,
						   tlr->pq_t2h[i].num_segments);
				dev_err(&tlr->pci_dev->dev,
					"PQ shared ring sizes incompatible "
					"for interface %d, use size %lu\n",
					i, ring_size);
				tlr->pq_h2t[i].ring_size = ring_size;
				tlr->pq_t2h[i].ring_size = ring_size;
				tlr->pq_h2t[i].num_segments = num_segments;
				tlr->pq_t2h[i].num_segments = num_segments;
			}

			if (tlr->pq_h2t[i].numa_node !=
			    tlr->pq_t2h[i].numa_node) {
				dev_err(&tlr->pci_dev->dev,
					"PQ shared ring NUMA nodes "
					"incompatible for interface %d, "
					"use NUMA node %d\n",
					i, tlr->pq_h2t[i].numa_node);
				tlr->pq_t2h[i].numa_node =
					tlr->pq_h2t[i].numa_node;
			}
		}
	}

	return 0;
}

static void pq_queue_monitor(struct tlr_pcie_dev *tlr)
{
	struct gxpci_queue_pair_status *queue_pair_status;
	int queue;

	/* Monitor the packet queues, T2H queues first. */
	for (queue = 0; queue < tlr->pq_t2h_queues; queue++) {
		struct gxpci_host_pq_regs_drv *tile_regs;

		queue_pair_status = &tlr->queue_sts_array->pq_sts_t2h[queue];

		/*
		 * If both ends has GXPCI_CHAN_RESET, both are ready for
		 * reset and we set GXPCI_CHAN_RESET_ACK to both. Note that
		 * this is where both queues should end up under normal
		 * conditions.
		 */
		if ((queue_pair_status->tile_queue_status ==
			GXPCI_CHAN_RESET) &&
			(queue_pair_status->host_queue_status ==
			GXPCI_CHAN_RESET)) {
			queue_pair_status->tile_queue_status =
				GXPCI_CHAN_RESET_ACK;
			queue_pair_status->host_queue_status =
				GXPCI_CHAN_RESET_ACK;
		} else if (queue_pair_status->tile_queue_status == 
				GXPCI_CHAN_RESET) {
			/*
			 * Tile side initiated the reset. We alert the host
			 * queue by setting GXPCI_CHAN_RESET to the host
			 * user-visible status.
			 */
			struct tlr_pq_status *queue_status;

			queue_status = tlr->pq_t2h[queue].queue_status;

			/*
			 * If the host side app hasn't started, do nothing
			 * here and the tile side's release function would
			 * time out and return.
			 */
			if (queue_pair_status->host_queue_opened &&
				(queue_status->status !=
				GXPCI_CHAN_UNINITIALIZED)) {
				queue_status->status = GXPCI_CHAN_RESET;

				/*
				 * Upon detecting GXPCI_CHAN_RESET, the user app
				 * should call close() to synchronize the reset
				 * sequence between the two ends.
				 */
			}
		} else if (queue_pair_status->host_queue_status == 
				GXPCI_CHAN_RESET) {
			/*
			 * Check for the special case where the tile application
			 * exits due to a signal. The tile app, via the EP
			 * driver, sets host_queue_status to GXPCI_CHAN_RESET
			 * before setting tile_queue_status to
			 * GXPCI_CHAN_UNINITIALIZED. Here we just notify the
			 * host app of the reset event.
			 *
			 * Note that we fall through to the common case.
			 */
			if (queue_pair_status->tile_queue_status ==
				GXPCI_CHAN_UNINITIALIZED) {
				struct tlr_pq_status *queue_status;

				queue_status = tlr->pq_t2h[queue].queue_status;

				/*
				 * If the host app hasn't started yet,
				 * don't set GXPCI_CHAN_RESET to its queue
				 * status.
				 */
				if (queue_pair_status->host_queue_opened)
					queue_status->status = GXPCI_CHAN_RESET;

				/*
				 * Clear the host_queue_status, otherwise it
				 * will cause the next tile app to be reset.
				 */
				queue_pair_status->host_queue_status =
					GXPCI_CHAN_UNINITIALIZED;

				/*
				 * Upon detecting GXPCI_CHAN_RESET, the user app
				 * should call close() to synchronize the reset
				 * sequence between the two ends.
				 */
			}

			/*
			 * Host side initiated the reset. We alert the tile
			 * queue by setting GXPCI_CHAN_RESET to the tile
			 * user-visible status. If the host app hasn't started
			 * yet, do nothing here. For the sake of correctness,
			 * do not write to the tile's status if the tile app
			 * hasn't started yet.
			 */
			if (queue_pair_status->host_queue_opened &&
			    queue_pair_status->tile_queue_opened) {
				tile_regs = tlr->pq_t2h[queue].regs;
				writel(GXPCI_CHAN_RESET,
				       &tile_regs->queue_status);
			}

			/*
			 * Upon detecting GXPCI_CHAN_RESET, the user app
			 * should call close() to synchronize the reset
			 * sequence between the two ends.
			 */
		}
	}

	/* Monitor the packet queues, H2T. */
	for (queue = 0; queue < tlr->pq_h2t_queues; queue++) {
		struct gxpci_host_pq_regs_drv *tile_regs;

		/* Look at H2T queues. */
		queue_pair_status = &tlr->queue_sts_array->pq_sts_h2t[queue];

		/*
		 * If both ends has GXPCI_CHAN_RESET, both are ready for
		 * reset and we set GXPCI_CHAN_RESET_ACK to both. Note that
		 * this is where both queues should end up under normal
		 * conditions.
		 */
		if ((queue_pair_status->tile_queue_status ==
			GXPCI_CHAN_RESET) &&
			(queue_pair_status->host_queue_status ==
			GXPCI_CHAN_RESET)) {
			queue_pair_status->tile_queue_status =
				GXPCI_CHAN_RESET_ACK;
			queue_pair_status->host_queue_status =
				GXPCI_CHAN_RESET_ACK;
		} else if (queue_pair_status->tile_queue_status == 
				GXPCI_CHAN_RESET) {
			/*
			 * Tile side initiated the reset. We alert the host
			 * queue by setting GXPCI_CHAN_RESET to the host
			 * user-visible status.
			 */
			struct tlr_pq_status *queue_status;

			queue_status = tlr->pq_h2t[queue].queue_status;

			/*
			 * If the host side app hasn't started, do nothing
			 * here and the tile side's release function would
			 * time out and return.
			 */
			if (queue_pair_status->host_queue_opened &&
				(queue_status->status !=
				GXPCI_CHAN_UNINITIALIZED)) {
				queue_status->status = GXPCI_CHAN_RESET;

				/*
				 * Upon detecting GXPCI_CHAN_RESET, the user app
				 * should call close() to synchronize the reset
				 * sequence between the two ends.
				 */
			}
		} else if (queue_pair_status->host_queue_status == 
				GXPCI_CHAN_RESET) {
			/*
			 * Check for the special case where the tile application
			 * exits due to a signal. The tile app, via the EP
			 * driver, sets host_queue_status to GXPCI_CHAN_RESET
			 * before setting tile_queue_status to
			 * GXPCI_CHAN_UNINITIALIZED. Here we just notify the
			 * host app of the reset event.
			 *
			 * Note that we fall through to the common case.
			 */
			if (queue_pair_status->tile_queue_status ==
				GXPCI_CHAN_UNINITIALIZED) {
				struct tlr_pq_status *queue_status;

				queue_status = tlr->pq_h2t[queue].queue_status;

				/*
				 * If the host app hasn't started yet,
				 * don't set GXPCI_CHAN_RESET to its queue
				 * status.
				 */
				if (queue_pair_status->host_queue_opened)
					queue_status->status = GXPCI_CHAN_RESET;

				/*
				 * Clear the host_queue_status, otherwise it
				 * will cause the next tile app to be reset.
				 */
				queue_pair_status->host_queue_status =
					GXPCI_CHAN_UNINITIALIZED;

				/*
				 * Upon detecting GXPCI_CHAN_RESET, the user app
				 * should call close() to synchronize the reset
				 * sequence between the two ends.
				 */
			}

			/*
			 * Host side initiated the reset. We alert the tile
			 * queue by setting GXPCI_CHAN_RESET to the tile
			 * user-visible status. If the host app hasn't started
			 * yet, do nothing here. For the sake of correctness,
			 * do not write to the tile's status if the tile app
			 * hasn't started yet.
			 */
			if (queue_pair_status->host_queue_opened &&
			    queue_pair_status->tile_queue_opened) {
				tile_regs = tlr->pq_h2t[queue].regs;
				writel(GXPCI_CHAN_RESET,
				       &tile_regs->queue_status);
			}

			/*
			 * Upon detecting GXPCI_CHAN_RESET, the user app
			 * should call close() to synchronize the reset
			 * sequence between the two ends.
			 */
		}
	}
}

static void pq_queue_init(struct tlr_pcie_dev *tlr)
{
	int i;

#ifdef TILEPCI_VF
	tlr->pq_h2t_regs_offset = GXPCI_VF_HOST_PQ_H2T_REGS_OFFSET;
	tlr->pq_drv_regs_map_size = GXPCI_VF_HOST_PQ_REGS_DRV_MAP_SIZE;
	tlr->pq_app_regs_map_size = GXPCI_VF_HOST_PQ_REGS_APP_MAP_SIZE;
#else
	tlr->pq_h2t_regs_offset = GXPCI_HOST_PQ_H2T_REGS_OFFSET;
	tlr->pq_drv_regs_map_size = GXPCI_HOST_PQ_REGS_DRV_MAP_SIZE;
	tlr->pq_app_regs_map_size = GXPCI_HOST_PQ_REGS_APP_MAP_SIZE;
#endif

	for (i = 0; i < tlr->pq_h2t_queues; i++) {
		tlr->pq_h2t[i].tlr = tlr;
		init_MUTEX(&tlr->pq_h2t[i].mutex);
		spin_lock_init(&tlr->pq_h2t[i].lock);
	}

	for (i = 0; i < tlr->pq_t2h_queues; i++) {
		tlr->pq_t2h[i].tlr = tlr;
		init_MUTEX(&tlr->pq_t2h[i].mutex);
		spin_lock_init(&tlr->pq_t2h[i].lock);
	}

	for (i = 0; i < MIN(tlr->pq_h2t_queues, tlr->pq_t2h_queues); i++)
		init_MUTEX(&tlr->pq_mutex[i]);
}

static void pq_queue_free(struct tlr_pcie_dev *tlr)
{
	int i;

	for (i = 0; i < tlr->pq_h2t_queues; i++) {
		if (tlr->pq_h2t[i].pq_segment[0] &&
		    tlr->pq_h2t[i].share_h2t_t2h_ring == 0)
			tlr_packet_queue_free(&tlr->pq_h2t[i]);
		if (tlr->pq_h2t[i].regs) {
			iounmap(tlr->pq_h2t[i].regs);
			tlr->pq_h2t[i].regs = NULL;
		}
	}

	for (i = 0; i < tlr->pq_t2h_queues; i++) {
		if (tlr->pq_t2h[i].pq_segment[0] &&
		    tlr->pq_t2h[i].share_h2t_t2h_ring == 0)
			tlr_packet_queue_free(&tlr->pq_t2h[i]);
		if (tlr->pq_t2h[i].regs) {
			iounmap(tlr->pq_t2h[i].regs);
			tlr->pq_t2h[i].regs = NULL;
		}
	}

	/* Free shared Packet Queue DMA ring buffers. */
	for (i = 0; i < MIN(tlr->pq_h2t_queues, tlr->pq_t2h_queues); i++)
		if (tlr->pq_h2t[i].share_h2t_t2h_ring &&
		    tlr->pq_h2t[i].pq_segment[0])
			tlr_packet_queue_free_shared(&tlr->pq_h2t[i],
						     &tlr->pq_t2h[i]);
}
