/*
 * gxpci_host_netdev_ctrl.h
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
 * Driver specific netdev_ctrl functions/structures.
 *
 */

#ifndef __GXPCI_HOST_NETDEV_CTRL_H__
#define __GXPCI_HOST_NETDEV_CTRL_H__

/*
 * Tilera netdev control channel between host kernel & TileGx card that
 * communicates over PCIe. Uses reserved char stream channel
 * /dev/tilegxpci_<MAJOR>/1.
 */

#include <stdbool.h>

#if defined(TILEPCI_HOST) && defined(GXPCI_HOST_SDNIC)

/*
 * COPY from the tile side.
 * TODO: Move this into gxpci_nic structure
 */
typedef union if_tag_s
{
	struct {
#define IF_TYPE_ETH   0
#define IF_TYPE_VLAN  1
		uint16_t type:6;          /* If type. */
		uint16_t inst:10;         /* Interface instance. */
	};

	uint16_t u_tag;
} if_tag_t;

/* Include the protocol header after if_tag_t declaration
 * due to dependency.
 */
#include "tilegxpci_netdev_ctrl.h"

/* forward declaration */
struct net_device;

typedef struct tlr_netdev_ctrl_s
{
        /* State of the ctrl channel. */
        bool up;
        /* Discovery is done. */
        bool discovered;
        /* File structure handle for control channel. */
        struct file *filp;
        /* Work structure to poll and async read. */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
        struct delayed_work netdev_ctrl_work;
#else
        struct work_struct netdev_ctrl_work;
#endif
        /* Cache the tlr_pcie_dev. */
        struct tlr_pcie_dev *tlr;
        /* Semaphore for req_rsp/notif reads. */
        struct semaphore rsem;
} tlr_netdev_ctrl_t;

/* Host side function declartions. */
int tlr_netdev_ctrl_populate(struct net_device * netdev);

int tlr_netdev_ctrl_init(struct tlr_pcie_dev *tlr);

int tlr_netdev_ctrl_close(struct tlr_pcie_dev *dev);

int tlr_netdev_ctrl_proc_entry_create(void);

int tlr_netdev_ctrl_get_mac_address(tlr_netdev_ctrl_t *netdev_ctrl,
                                    int if_num, uint8_t *dev_addr);

/* Exit the driver usage all together, on all the boards. */
int tlr_netdev_ctrl_exit(void);

/* Application specific calls. */
int tlr_ifconfig_req_rsp(struct net_device *netdev, uint32_t op_code,
                         char *req_buf, size_t req_size,
                         char *rsp_buf, size_t rsp_size);

int tlr_ethtool_req_rsp(struct net_device *netdev, uint32_t op_code,
                        char *req_buf, size_t req_size,
                        char *rsp_buf, size_t rsp_size);

#endif /* TILPCI_HOST && TILEPCI_HOST_SDNIC */

#endif /* __GXPCI_HOST_NETDEV_CTRL_H__ */

