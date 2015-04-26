/*
 * ez8139.c
 * Copyright (C) 2015 Sinan Akpolat
 *
 * realtek 8139 driver written to explore device driver programming
 * This file is distributed under GNU GPLv2, see LICENSE file.
 * If you haven't received a file named LICENSE see <http://www.gnu.org/licences>
 *
 * ez8139 driver is distributed WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE
 *
 * This is a code written solely for training purposes,
 * under any circumstances it should not be run on a production system.
 */

/* Some portions of code is copied and modified from original 8139cp.c file */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/pci.h>

#define DRV_NAME "ez8139" 

static DEFINE_PCI_DEVICE_TABLE(ez8139_pci_tbl) = {
{   PCI_DEVICE(PCI_VENDOR_ID_REALTEK, PCI_DEVICE_ID_REALTEK_8139), }, //we are only interested in Realtek 8139 PCI ID.
{ } //terminate array with NULL
};


static int ez8139_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	printk(KERN_ALERT "A pci device for ez8139 probed\n");
	return 0;
}

static void ez8139_remove(struct pci_dev *pdev)
{
	printk(KERN_ALERT "A pci device for ez8139 is being removed\n");
}

static struct pci_driver ez8139_pci = {
	.name = DRV_NAME,
	.id_table = ez8139_pci_tbl,
	.probe = ez8139_probe,
	.remove = ez8139_remove
};

static int __init ez8139_init(void) 
{
	int ret = pci_register_driver(&ez8139_pci);
	printk(KERN_ALERT "ez8139 driver registered in pci subsystem!\n");
	return ret;
}

static void __exit ez8139_exit(void) 
{
	pci_unregister_driver(&ez8139_pci);
	printk(KERN_ALERT "ez8139 driver unregistered from pci subsystem...\n");
}

module_init(ez8139_init);
module_exit(ez8139_exit);

MODULE_LICENSE("GPL");
