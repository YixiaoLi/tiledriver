/*
 * tilegxpci_host_vf.h
 *
 * Virtual function Host-driver function and structure declarations.
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
#ifndef __TLR_GXPCI_HOST_VF_H__
#define __TLR_GXPCI_HOST_VF_H__ 

#define TILEPCI_VF

#include "tilegxpci_shared_code.h"

/** The vendor ID for all Tilera processors. */
#define TILERA_VENDOR_ID		0x1a41

/** The Virtual Function device ID for the Gx36 processor. */
#define TILERA_GX36_DEV_VF_ID		0x0201

#endif
