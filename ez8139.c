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
#include <linux/byteorder/generic.h> //for le to cpu conversions

#define DRV_NAME "ez8139"
#define PRINT_DEV_LOC " rtl8139 device on pci bus %d pci slot %d.\n", pdev->bus->number, PCI_SLOT(pdev->devfn)

struct ez8139_priv {
    void __iomem *regs; //regs addr in kernel's virtual address space (mapped from IO addr space)
};

static int init_pci_regs(struct pci_dev *pdev);	//initialize PCI dev ready for device register IO
static void pci_regs_test(void __iomem *regs);	//test PCI register access on device

static void deinit_pci_regs(struct pci_dev *pdev);	//free regions and disable the device

static int init_pci_regs(struct pci_dev *pdev)
{
	struct ez8139_priv *priv;
	resource_size_t pci_regs;	//offset of device register addresses in IO addr space

	int ret;

	priv = (struct ez8139_priv *) pci_get_drvdata(pdev);

	//enable the pci device for use
	ret = pci_enable_device(pdev);
	if (ret) {
		printk(KERN_ALERT "Could not enable" PRINT_DEV_LOC);
		return ret;
	}

	//register the regions in all BARs to this driver
	ret = pci_request_regions(pdev, DRV_NAME);
	if (ret) { //this may fail if a driver for rtl8139 took the device but did not release regions yet.
		printk("Unable to register PCI regions for" PRINT_DEV_LOC);
		pci_disable_device(pdev);
		return ret; 
	}

	//get the beginning address of device registers (presented in BAR1)
	pci_regs = pci_resource_start(pdev,1);
	if (!pci_regs)
	{
		printk("Unable to access IO registers over PCI for" PRINT_DEV_LOC);
		pci_release_regions(pdev);
		pci_disable_device(pdev);
		return -EIO;
	}

	//we are going to access device registers over mmap'd IO (MMIO).
	//basically we are mapping CPU's IO address space to kernel's virtual address space (because Intel CPU's work that way)
	priv->regs = ioremap(pci_regs, pci_resource_len(pdev,1)); //map a chunk of IO addr space as big as device's registers (known to PCI) to virt addr space
	if (!priv->regs)
	{
		printk("Unable to map IO registers to virt addr space for" PRINT_DEV_LOC);
		pci_release_regions(pdev);
		pci_disable_device(pdev);
		return -EIO;
	}

	return 0;
}

static void pci_regs_test(void __iomem *regs)
{
	__u32 mac1 = 0;
	__u32 mac2 = 0;

	__u8 mac_whole[6]; //just for testing

	//if the PCI IO is properly initialized we can access registers detailed in datasheet (section 6) over PCI
	//let's read mac addr (IDR0-6), datasheet says we can only read this 4-bytes at once
	//we will use the mmap'd and then translated virt addr returned by ioremap
	//we could have also read this from eeprom directly as is the case with original 8139 driver
	mac1 = ioread32(regs);
	mac2 = ioread32(regs + 4);
	printk(KERN_INFO "First MAC part read from io regs is %08x - untranslated\n", mac1);
	printk(KERN_INFO "Second MAC part read from io regs is %08x - untranslated\n", mac2);

	//PCI is a le bus therefore we need to translate everything we read to make sense of them

	mac_whole[0] = le32_to_cpu(mac1) & 0xFF;
	mac_whole[1] = (le32_to_cpu(mac1) >> 8) & 0xFF;
	mac_whole[2] = (le32_to_cpu(mac1) >> 16) & 0xFF;
	mac_whole[3] = (le32_to_cpu(mac1) >> 24) & 0xFF;
	mac_whole[4] = le32_to_cpu(mac2) & 0xFF;
	mac_whole[5] = (le32_to_cpu(mac2) >>8) & 0xFF;

	printk(KERN_INFO "MAC addr read from io regs is %02x:%02x:%02x:%02x:%02x:%02x - translated\n", \
							mac_whole[0], mac_whole[1], mac_whole[2], \
							mac_whole[3], mac_whole[4], mac_whole[5]);
}

static void deinit_pci_regs(struct pci_dev *pdev)
{
	printk("Releasing IO regions and disabling the" PRINT_DEV_LOC);
	pci_release_regions(pdev); //release the mmap'd region so that another driver can use it if necessary.
	pci_disable_device(pdev);
}

static int ez8139_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int ret;
	struct ez8139_priv *priv;

	printk(KERN_INFO "A pci device for ez8139 probed\n");

	priv = (struct ez8139_priv*) kmalloc(sizeof(struct ez8139_priv), GFP_KERNEL);
	if(!priv)
	{
		printk("Unable to allocate kernel mem for" PRINT_DEV_LOC);
		return -ENOMEM;
	}
	pci_set_drvdata(pdev, priv); //set pdev's priv
	printk(KERN_DEBUG "private section is allocated at %p for rtl8139 device on pci bus %d pci slot %d.\n", \
								priv, pdev->bus->number, PCI_SLOT(pdev->devfn)); 

	ret = init_pci_regs(pdev);
	if(ret)
	{
		kfree(pci_get_drvdata(pdev));
	}

	pci_regs_test(priv->regs); //test if device registers are accessible

	return ret;
}

static void ez8139_remove(struct pci_dev *pdev)
{
	struct ez8139_priv *priv;
	priv = (struct ez8139_priv *) pci_get_drvdata(pdev);

	printk(KERN_INFO "A pci device for ez8139 is being removed\n");
	deinit_pci_regs(pdev);
	kfree(pci_get_drvdata(pdev));
	return;
}

static DEFINE_PCI_DEVICE_TABLE(ez8139_pci_tbl) = {
{   PCI_DEVICE(PCI_VENDOR_ID_REALTEK, PCI_DEVICE_ID_REALTEK_8139), }, //we are only interested in Realtek 8139 PCI ID.
{ } //terminate array with NULL
};

static struct pci_driver ez8139_pci = {
	.name = DRV_NAME,
	.id_table = ez8139_pci_tbl,
	.probe = ez8139_probe,
	.remove = ez8139_remove
};

static int __init ez8139_init(void) 
{
	int ret = pci_register_driver(&ez8139_pci);
	printk(KERN_INFO "ez8139 driver registered in pci subsystem!\n");
	return ret;
}

static void __exit ez8139_exit(void) 
{
	pci_unregister_driver(&ez8139_pci);
	printk(KERN_INFO "ez8139 driver unregistered from pci subsystem...\n");
}

module_init(ez8139_init);
module_exit(ez8139_exit);

MODULE_LICENSE("GPL");
