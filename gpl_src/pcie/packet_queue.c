/*
 * packet_queue.c - Tilera TILEGx host-side driver routines for the
 *                  user-space PCIe Packet Queue.
 *
 * This source code is derived for code provided in "Linux Device
 * Drivers" by Alessandro Rubini and Jonathan Corbet, published by
 * O'Reilly & Associates.
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/pagemap.h>

#include <asm/page.h>
#include <asm/pgtable.h> 		/* io_remap_page_range */
#include <asm/uaccess.h> 

#include "tilegxpci.h"
#include "tilegxpci_host.h"
#include "tilegxpci_version.h"

#include "packet_queue_common.c"
