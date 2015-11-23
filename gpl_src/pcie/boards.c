/*
 * boards.c - board-specific support code, mostly for supporting chip reset.
 *
 * The "board" here really means a Tilera chip, not a physical circuit board
 * which may contain one or more chips.
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
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <asm/uaccess.h> 
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/version.h>

#include "tilegxpci.h"
#include "tilegxpci_host.h"

/* Number of times to retrain the link. */
#define GXPCI_LINK_RETRAIN_RETRIES	2

static LIST_HEAD(board_list);
static int num_boards;
DECLARE_MUTEX(board_list_mutex);
 
/*
 * Array of pointers to all the Tilera chips in the system.
 */
static tlr_board_t *tlr_chips[MAX_CHIPS_PER_DOMAIN];


/**********************************************************************/
/*                            Support Routines                        */
/**********************************************************************/

/* We assume that the ability to read the RSHIM device info means that the
 * link is up. */
static int
link_is_down(struct pci_dev* dev)
{
	struct tlr_pcie_dev *tlr = dev_get_drvdata(&dev->dev);

	/* Check to see whether device is responsive. */
	if (((rshim_reg_read(tlr, RSH_DEV_INFO) >> RSH_DEV_INFO__TYPE_SHIFT) &
		RSH_DEV_INFO__TYPE_RMASK) != RSH_DEV_INFO__TYPE_VAL_RSHIM) {
		dev_err(&dev->dev, "RSHIM access failure, link is down?\n");

		return TRUE;
	}

	return FALSE;
}

/**********************************************************************/
/*              Generic Implementations of Board Functions            */
/**********************************************************************/

/* Returns TRUE if any of the board's PCI links are down. */
int
generic_is_link_down(tlr_board_t* board)
{
	int i;
	
	for (i = 0; i < board->num_ports; i++) {
		if (link_is_down(board->ports[i]->pci_dev))
			return TRUE;
	}
	return FALSE;
}

/* Returns TRUE if the board has already booted (potentially
 * via some mechanism other than PCI). */
int
generic_has_booted(tlr_board_t* board)
{
	/*
	 * For now, always return TRUE.
	 */
	return TRUE;
}

/* Perform a secondary bus reset on a particular device. */
static void
tlr_start_secondary_reset(struct pci_dev* dev)
{
	u16 rmw;
	struct pci_dev* bridge_port = dev->bus->self;

	pci_read_config_word(bridge_port, PCI_BRIDGE_CONTROL, &rmw);
	rmw |= PCI_BRIDGE_CTL_BUS_RESET;
	pci_write_config_word(bridge_port, PCI_BRIDGE_CONTROL, rmw);
}

/* Finish a secondary bus reset on a particular device. */
static void
tlr_finish_secondary_reset(struct pci_dev* dev)
{
	u16 rmw;
	struct pci_dev* bridge_port = dev->bus->self;

	pci_read_config_word(bridge_port, PCI_BRIDGE_CONTROL, &rmw);
	rmw &= ~PCI_BRIDGE_CTL_BUS_RESET;
	pci_write_config_word(bridge_port, PCI_BRIDGE_CONTROL, rmw);
}

/* Retrain the PCIe link. */
static void
tlr_retrain_link(struct pci_dev* dev)
{
	int retrain_tries = 0;
	struct tlr_pcie_dev *tlr;

	get_link_speed_width(dev);

	tlr = pci_get_drvdata(dev);
	while ((tlr->link_speed < tlr->expected_link_speed) ||
	       (tlr->link_width < tlr->expected_link_width)) {

		u16 rmw;
		int ppos;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
		const int PCI_EXP_LNKCTL_RL = 0x20;
#endif
		struct pci_dev *bridge_port = dev->bus->self;

		dev_warn(&dev->dev,
			 "LINK_SPEED_GEN %d LINK_WIDTH %d, retrain ...\n",
			 tlr->link_speed, tlr->link_width);

		ppos = pci_find_capability(bridge_port, PCI_CAP_ID_EXP);

		pci_read_config_word(bridge_port, ppos + PCI_EXP_LNKCTL, &rmw);
		rmw |= PCI_EXP_LNKCTL_RL;
		pci_write_config_word(bridge_port, ppos + PCI_EXP_LNKCTL, rmw);

		msleep(1000);

		get_link_speed_width(dev);
		if ((tlr->link_speed == tlr->expected_link_speed) &&
		    (tlr->link_width == tlr->expected_link_width))
			break;
		else if (++retrain_tries == GXPCI_LINK_RETRAIN_RETRIES)
				break;
	}
}

/*
 * Attempt to reset the TILE via the RSHIM, including the TRIO and the PCIe
 * ports.
 */
static int
reset_all(tlr_board_t * board)
{
	int i;

#define PCIE_PREBOOTER

	/* Use port 0 to send the reset command. */
	struct tlr_pcie_dev* tlr = board->ports[0];

	/* Reset everyting. */
	rshim_reg_write(tlr, RSH_RESET_MASK, 0);

#ifdef PCIE_PREBOOTER
	/* Make the chip reboot from SROM which contains the prebooter. */
	rshim_reg_write(tlr, RSH_BOOT_CONTROL,
			RSH_BOOT_CONTROL__BOOT_MODE_VAL_SPI);
#else
	/* Make sure the chip doesn't reboot from SROM or I2C. */
	rshim_reg_write(tlr, RSH_BOOT_CONTROL, 0);
#endif

	/* Hit the soft reset "button." */
	rshim_reg_write(tlr, RSH_RESET_CONTROL,
			RSH_RESET_CONTROL__RESET_CHIP_VAL_KEY);

	/*
	 * Toggle the Secondary Bus Reset bit in the upstream port so that
	 * it doesn't report the link state transition from DL_Active to
	 * DL_Inactive, resulting from the Gx PCIe reset, as a Surprise Down
	 * error.
	 */
	for (i = 0; i < board->num_ports; i++)
		tlr_start_secondary_reset(board->ports[i]->pci_dev);

	/* Wait for the chip to reset, then bring the links back up. */
	msleep(TLR_RESET_WAIT_MS);

	for (i = 0; i < board->num_ports; i++)
		tlr_finish_secondary_reset(board->ports[i]->pci_dev);

	msleep(TLR_RESET_WAIT_MS);

	for (i = 0; i < board->num_ports; i++)
		tlr_retrain_link(board->ports[i]->pci_dev);

	return 0;
}

/*
 * Attempt to reset the TILE via the RSHIM, excluding the TRIO and the PCIe
 * ports to keep the link up.
 */
static int
reset_without_pcie(tlr_board_t * board)
{
	int i;

	/* Use port 0 to send the reset command. */
	struct tlr_pcie_dev* tlr = board->ports[0];

	/* Trigger RSHIM SWINT3 to clean up TRIO resources. */
	rshim_reg_write(tlr, RSH_SWINT, RSH_INT_VEC0_RTC__SWINT3_MASK);

	/* Give some time for the TRIO resource cleanup. */
	msleep(2 * TLR_RESET_WAIT_MS);

	/* Reset everyting, except TRIO and the PCIe MACs. */
	rshim_reg_write(tlr, RSH_RESET_MASK, TRIO_PCIE_MAC_MASK);

	/* Make sure the chip doesn't reboot from SROM or I2C. */
	rshim_reg_write(tlr, RSH_BOOT_CONTROL, 0);

	/* Hit the soft reset "button." */
	rshim_reg_write(tlr, RSH_RESET_CONTROL,
			RSH_RESET_CONTROL__RESET_CHIP_VAL_KEY);

	msleep(TLR_RESET_WAIT_MS);

	for (i = 0; i < board->num_ports; i++)
		tlr_retrain_link(board->ports[i]->pci_dev);

	return 0;
}

/*
 * Attempt to reset the TILE via the RSHIM.
 * It returns an error if an attempt to reset fails.
 */
int
generic_do_reset(tlr_board_t * board)
{
	if (gxpci_reset_all)
		return reset_all(board);
	else
		return reset_without_pcie(board);
}

/**********************************************************************/
/*                      Board Probing and Listing                     */
/**********************************************************************/

/* Given a PCI device reported by linux's probe() invocation, figure
 * out which board it belongs to.  This routine is responsible for
 * detecting cases where one chip or board has more than one PCIe EP
 * links and thus is probe()'d once for each port.
 */
int
tlr_map_device_to_board(struct tlr_pcie_dev* tlr)
{	
	int i = 0;
	int result = 0;
	u32 port_index = 0;
	u64 scratch_val = 0;
	int match_found = 0;
	tlr_board_t *board = NULL;

	down(&board_list_mutex);

	/* Check if the chip limit is exceeded. */
	if (num_boards == MAX_CHIPS_PER_DOMAIN) {
		dev_err(&tlr->pci_dev->dev,
			"MAX_CHIPS_PER_DOMAIN is exceeded, ignore this port.\n");
		result = -ENOMEM;
		goto exit;
	}

	/* Write this port's link index to the RSHIM Scratch register. */
	rshim_reg_write(tlr, RSH_SCRATCHPAD, tlr->link_index);

	/*
	 * Make sure that the MMIO write above is issued before
	 * the subsequent reads.
	 */
	smp_mb();

	/*
	 * We loop through all the chips that have been detected and read
	 * from each chip's RSHIM Scratch register. If it contains our
	 * link index value, this is the chip that we belong to. If no match
	 * is found, we belong to a new chip.
	 *
	 * We also guard against the unlikely case that a port can be linked
	 * to two chips, which in theory can happen if some non-PCI agents
	 * (HW or SW) write to the Scratch register on a separate chip with
	 * a value that matches our link index during the PCI bus probe stage,
	 * which really should not happen because no known Scratch register
	 * users are active during this short period.
	 */
	for (i = 0; i < num_boards; i++) {
		board = tlr_chips[i];

		scratch_val = rshim_reg_read(board->ports[0], RSH_SCRATCHPAD);
		if (scratch_val == tlr->link_index) {
			if (match_found) {
				dev_err(&tlr->pci_dev->dev, "Port found on "
					"two chips, ignore this port.\n");
				result = -EIO;
				break;
			}

			match_found = 1;
			port_index = board->num_ports++;
			board->ports[port_index] = tlr;

			tlr->board = board;
			tlr->port_index = port_index;
		}
	}

	/* If this is the first port on a chip, allocate the board struct. */
	if (match_found == 0) {
		board = kmalloc(sizeof(*board), GFP_KERNEL);
		if (board == NULL) {
			result = -ENOMEM;
			goto exit;
		}
		memset(board, 0, sizeof(*board));
		board->locked_pid = -1;
		board->num_ports = 1;
		board->ports[0] = tlr;
		board->is_link_down = generic_is_link_down;
		board->has_booted = &generic_has_booted;
		board->do_reset = &generic_do_reset;
		spin_lock_init(&board->rshim_reg_lock);
		list_add_tail(&board->list, &board_list);
		board->board_index = num_boards++;

		tlr_chips[board->board_index] = board;
		tlr->board = board;
		tlr->port_index = 0;
	}

 exit:
	up(&board_list_mutex);
	return result;
}

/* Invoked when a board is removed (if hotplug is supported) or when
 * the driver is being removed via rmmod.  When the last of a boards
 * ports is remove()'d, the routine needs to free any allocated
 * resources and remove the board from the global list-of-boards. */  
void
tlr_unmap_device_from_board(struct tlr_pcie_dev* tlr)
{
	tlr_board_t* board = tlr->board;

	down(&board_list_mutex);

	board->num_ports--;

	if (board->num_ports == 0) {
		list_del(&board->list);
		kfree(board);
		num_boards--;
	}
		
	up(&board_list_mutex);
}

static void*
tlr_board_seq_start(struct seq_file *s, loff_t *pos)
{
	int count;
	struct list_head* cursor;

	/* Grab the mutex, we will release during stop(). */
	if (down_interruptible(&board_list_mutex))
		return ERR_PTR(ERESTARTSYS);
	
	if (*pos >= num_boards)
		return NULL;

	count = 0;
	list_for_each(cursor, &board_list) {
		if (count == *pos)
			return list_entry(cursor, tlr_board_t, list);
		count++;
	}
	
	return NULL;
}

static void*
tlr_board_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	tlr_board_t* board = (tlr_board_t*) v;
	
	(*pos)++;
	if (*pos >= num_boards)
		return NULL;
	
	return list_entry(board->list.next, tlr_board_t, list);
}

static void
tlr_board_seq_stop(struct seq_file *s, void *v)
{
	up(&board_list_mutex);
}

static int
tlr_board_seq_show(struct seq_file *s, void *v)
{
	tlr_board_t* board = (tlr_board_t*) v;
	int i;

	for (i = 0; i < board->num_ports; i++) {
		seq_printf(s, "%d ", MAJOR(board->ports[i]->first_dev));
	}
	seq_printf(s, "\n");

	return 0;
}

static struct seq_operations tlr_board_seq_ops = {
	.start = tlr_board_seq_start,
	.next = tlr_board_seq_next,
	.stop = tlr_board_seq_stop,
	.show = tlr_board_seq_show,
};

/* Open a sequence file that prints information about all the boards
 * in the system, in particular the major number associated with each
 * PCIE port on a board. */
static int
tlr_proc_boards_open(struct inode *inode, struct file *file)
{      
	return seq_open(file, &tlr_board_seq_ops);
}

struct file_operations tlr_proc_boards_ops = {
	.owner = THIS_MODULE,
	.open = tlr_proc_boards_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

#ifdef GXPCI_HOST_SDNIC
/* Cleanup the netdev ctrl channel during exit. */
int
tlr_netdev_ctrl_exit(void)
{
	tlr_board_t *board;
	int i;

	for (i = 0; i < num_boards; i++) {
		board = tlr_chips[i];
		if (board && board->netdev_ctrl.up)
			tlr_netdev_ctrl_close(board->ports[0]);
	}

	return 0;
}
#endif /* GXPCI_HOST_SDNIC */
