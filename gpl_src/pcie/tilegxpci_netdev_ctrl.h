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
 * Tilera host NIC ctrl interface.
 * The control channel defines and the communication channel between
 * the host and the tile.
 * The tile side user space driver listens to these messages and responds
 *
 */

#ifndef __TILEGX_NETDEV_CTRL_H__
#define __TILEGX_NETDEV_CTRL_H__

#include <stdbool.h>

/*
 * tlr_netdev_ctrl_hdr_t - Ctrl plane communication header.
 * @App code - to demux at first level
 * @op_code - op_code inside application which helps re-use op-codes
 * @flags- identifie if its a req/rsp/notif/error
 * @msg_token - token id to uniquely id each msg
 * @payload_len - length of the payload
 */
typedef struct tlr_netdev_ctrl_hdr_s
{
	uint8_t app_code;     /**< Application code. */
	uint32_t msg_token;   /**< Msg token. */
	uint32_t op_code;     /**< OP-code. */
	uint16_t flags;       /**< Flags - req_rsp or notif. */
	if_tag_t if_tag;      /**< Port index. */
	uint16_t payload_len; /**< Length of the payload. */
	uint16_t status;      /**< SUCCESS or failure/err-code. */
} __attribute ((packed)) tlr_netdev_ctrl_hdr_t;

/*
 * tlr_netdev_ctrl_frame_t - Frame includes the header and payload,
 *			     payload len is encoded in the header.
 */
typedef struct tlr_netdev_ctrl_frame_s
{
	tlr_netdev_ctrl_hdr_t hdr;
	uint8_t payload[0]; 
} __attribute ((packed)) tlr_netdev_ctrl_frame_t;

#define TLR_NETDEV_CTRL_BUF_SIZE(_payload_size) \
	(sizeof(tlr_netdev_ctrl_hdr_t) + _payload_size)

/*
 * Application specific definitions for the appcode - size uint8_t.
 */
enum
{
	TLR_NETDEV_CTRL  = 1, /**< Control message. */
	TLR_ETHTOOL      = 2, /**< Ethtool. */
	TLR_IFCONFIG     = 3, /**< ifconfig. */
	TLR_APP_CODE_MAX = 4, /**< Max - last appcode. */
};

/*
 * TLR_NETDEV_CTRL opcodes - size uint32_t.
 */
typedef enum tlr_netdev_ctrl_op_codes_s
{
	TLR_NETDEV_CTRL_DISCOVER  = 1, /**< Discover control message. */
	TLR_NETDEV_CTRL_LINK_UP   = 2, /**< Link up notification. */
	TLR_NETDEV_CTRL_LINK_DOWN = 3, /**< Link Down notification. */
} tlr_netdev_ctrl_op_codes_t;


/* netdev_ctrl header flags. */
#define TLR_NETDEV_CTRL_FL_REQ		0x0001 /* req msg */
#define TLR_NETDEV_CTRL_FL_RSP		0x0002 /* rsp msg */
#define TLR_NETDEV_CTRL_FL_NOTIF	0x0004 /* Notification */
#define TLR_NETDEV_CTRL_FL_ERR		0x0008 /* Error */
#define TLR_NETDEV_CTRL_TIMEOUT		4000     /* 4000 milliseconds */
#define TLR_NETDEV_CTRL_ONE_TICK	500      /* 500 milliseconds */
#define TLR_NETDEV_CTRL_POLL_TIMEOUT	(2 * HZ) /* poll every 2 secs */
#define TLR_NETDEV_CTRL_BOOT_TIMEOUT	(5 * HZ) /* poll every 5 secs */
#define TLR_HDR_SIZE			sizeof(tlr_netdev_ctrl_hdr_t)

/* Header include causes export errors. Redefining. */
#define ETH_ALEN  6

/* Link level parameters. */
struct tlr_netdev_ctrl_if_params
{
	uint8_t mac_addr[ETH_ALEN]; /* Port's Mac address. */
};

/* Tile capability structure. */
struct tlr_tile_capability
{
#define NUM_IFS 8
	uint64_t netdev_features; /* Type netdev_features_t. */
	uint32_t num_links;       /* Number of links. */
	struct tlr_netdev_ctrl_if_params link_params[NUM_IFS]; 
};

#endif /* __TILEGX_NETDEV_CTRL_H__ */
