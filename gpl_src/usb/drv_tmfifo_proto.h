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

/**
 * @file
 *
 * Definition of the protocol used between a host system and a Tile target
 * when communicating over the tile-monitor FIFO in the rshim.
 */
#ifndef _SYS_HV_DRV_TMFIFO_PROTO_H
#define _SYS_HV_DRV_TMFIFO_PROTO_H

#ifdef __KERNEL__
#include <linux/stddef.h>
#else
#include <stddef.h>
#endif

/** Size of the packet header. */
#define TMFIFO_PKT_HDR_LEN 2

/** Size of a control message. */
#define TMFIFO_CTL_LEN 8

/** Current version of the protocol. */
#define TMFIFO_PROTO_VERS 1

//
// XXX Eventually, a full description of each version of the protocol goes
// here.
//
	/*
	 * Data flows between the host and tile, and vice versa, as
	 * sequences of packets.  A packet comprises a 2-byte header,
	 * followed by data.  Right now the header is just a little-endian
	 * count of the number of data bytes following it, which must be at
	 * least 1.  If we start supporting multiple channel numbers, the
	 * header will probably also contain those, and might grow.  The
	 * packetization is done only to allow us to send data blocks which
	 * are not a multiple of 8 bytes long, since the hardware only
	 * supports that alignment; it is not done to provide message
	 * boundaries.  Thus, a driver write request may be contained in
	 * multiple packets, and a packet may contain data from multiple
	 * requests.
	 */



/** First phase of the initialization 3-way handshake.  Sent from the
 *  host to the tile.  Byte 3 is the largest version of the protocol that
 *  the host can handle. */
#define TMFIFO_CTL_INIT0   0

/** Second phase of the initialization 3-way handshake.  Sent from the
 *  tile to the host in response to INIT0.  Byte 3 is the minimum of the
 *  protocol version in the INIT0, and the largest version of the protocol
 *  that the tile can handle.  After this message has been sent, the tile's
 *  transmit behavior must conform to the version of the protocol in this
 *  message, and the host must interpret data received subsequent to this
 *  message accordingly. */
#define TMFIFO_CTL_INIT1   1

/** Third phase of the initialization 3-way handshake.  Sent from the host
 *  to the tile in response to INIT1.  Byte 3 is the minimum of the protocol
 *  versions in the INIT0 and INIT1.  handle.  After this message has been
 *  sent, the host's transmit behavior must conform to the version of the
 *  protocol in this message, and the tile must interpret data received
 *  subsequent to this message accordingly. */
#define TMFIFO_CTL_INIT2   2

#endif /* _SYS_HV_DRV_TMFIFO_PROTO_H */
