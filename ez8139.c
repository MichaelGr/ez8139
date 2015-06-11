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
#include <linux/delay.h> //to be able to wait
#include <linux/proc_fs.h> //to interact with userspace
#include <linux/seq_file.h> //to print via proc interface

#define DRV_NAME "ez8139"
#define PRINT_DEV_LOC " rtl8139 device on pci bus %d pci slot %d.\n", pdev->bus->number, PCI_SLOT(pdev->devfn)

//some frequently used register addrs
#define CPCR 0xE0 //c+ mode command register
#define CMDR 0x37 //normal command register
#define DTCCR 0x10 //tally dump registers, 8 bytes long

//various bit values
#define DAC_BIT (1<<4) //DAC bit in C+ command register
#define DTCCR_CMD_BIT (1<<3) //set to 1 to start tally counter dump, then read until set back to 0 by device
#define CMD_RES_BIT (1<<4) //set to 1 to reset the device (needed during init to start with a clean slate)
#define CMD_TX_EN_BIT (1<<2) //enable TX in command register
#define CMD_RX_EN_BIT (1<<3) //enable RX in command register
#define CP_TX_EN_BIT 1 //enable TX in C+ mode
#define CP_RX_EN_BIT (1<<1) //enable RX in C+ mode


//some sizes
#define TALLY_DUMP_SIZE 64 //size of tally dump (contents are in ch6.3 (DTCCR) in datasheet)

struct ez8139_priv {
	void __iomem *regs; //regs addr in kernel's virtual address space (mapped from IO addr space)
	dma_addr_t tally_dump_bus; //the address we will tell the bus
	void *tally_dump_virt; //the address we will use as CPU
};

static int init_pci_regs(struct pci_dev *pdev);	//initialize PCI dev ready for device register IO
static void pci_regs_test(void __iomem *regs);	//test PCI register access on device

static void deinit_pci_regs(struct pci_dev *pdev);	//free regions and disable the device

static int init_pci_dma(struct pci_dev *pdev); //init DMA

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
	u32 mac1, mac2;
	u64 mac_whole;

	//if the PCI IO is properly initialized we can access registers detailed in datasheet (section 6) over PCI
	//let's read mac addr (IDR0-6), datasheet says we can only read this 4-bytes at once
	//we will use the mmap'd and then translated virt addr returned by ioremap
	//we could have also read this from eeprom directly as is the case with original 8139 driver
	mac1 = ioread32(regs);
	mac2 = ioread32(regs + 4);

	mac_whole = ((u64) mac2 << 32) | mac1;
	printk(KERN_INFO "Complete MAC address read from io regs is %pM - translated\n", &mac_whole);
}

static void deinit_pci_regs(struct pci_dev *pdev)
{
	printk("Releasing IO regions and disabling the" PRINT_DEV_LOC);
	pci_release_regions(pdev); //release the mmap'd region so that another driver can use it if necessary.
	pci_disable_device(pdev);
}

static int init_pci_dma(struct pci_dev *pdev)
{
	u16 cmdregval;
	int rc;
	struct ez8139_priv *priv;
	priv = (struct ez8139_priv *) pci_get_drvdata(pdev);

	printk(KERN_DEBUG "priv section is at %p\n", priv);

	pci_set_mwi(pdev); //set mwi bit in command register
	//9.13.3 in datasheet
	//device writes RAM large chunks and then invalidates them in cache addrs
	//not sure if this works (or even need to work) in QEMU
	//this only works in RX queues according to datasheet

	rc = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64));
	if(rc) printk (KERN_ALERT "Unable to set consistent DMA mask to 64" PRINT_DEV_LOC);
	rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(64));
	if(rc) printk (KERN_ALERT "Unable to set DMA mask to 64" PRINT_DEV_LOC);

	//not sure if Linux cares about DAC, setting it nevertheless
	iowrite16(DAC_BIT, priv->regs + (CPCR));
	cmdregval = ioread16(priv->regs + (CPCR));
	printk(KERN_INFO "C+ command reg value after setting DAC: %x\n", cmdregval);

	pci_set_master(pdev); //enable bus mastering for device and read and set again latency timer 
	//linux only sets latency timer to 64 or 255 I understand?	

	//we won't write anything, but since dma_alloc_coherent is so easy ...
	priv->tally_dump_virt = dma_alloc_coherent(&pdev->dev, TALLY_DUMP_SIZE, &priv->tally_dump_bus, GFP_KERNEL);
	/*	pdev->dev, //this is the device struct kernel knows
	 *	TALLY_DUMP_SIZE, // (64) size of tally dump - still takes up one page :(
	 *	&priv->tally_dump_bus, //allocated addr translated for bus, will be filled by this function
	 */
	printk(KERN_INFO "I registered %p for tally dump ops\n", priv->tally_dump_virt);

	return 0;
}

static int show_dma_test_entry(struct seq_file *file, void *seq)
{
	unsigned char *cp = 0;
	int i = 0;

	printk(KERN_INFO "printing tally dump dma result\n");
	printk(KERN_INFO "tally dump virt addr in private data %p\n", file->private);

	cp = (unsigned char *) file->private;

	for(i=0;i<8;i++)
	{
		seq_printf(file, "tally%d: %x:%x:%x:%x-%x:%x:%x:%x\n", i,
				*(cp+(i*8)), *(cp+(i*8)+1), *(cp+(i*8)+2), *(cp+(i*8)+3),
				*(cp+(i*8)+4), *(cp+(i*8)+5), *(cp+(i*8)+6), *(cp+(i*8)+7));
	}
	return 0;
}

static int open_dma_test_entry(struct inode *inode, struct file *file)
{
	struct pci_dev *pdev = 0;
	int bus_num = 0;
	int slot_num = 0;
	int i;

	struct ez8139_priv *priv;

	printk(KERN_INFO "proc entry %s is being read\n", file->f_dentry->d_name.name);

	//extract bus and slot num from filename
	sscanf(file->f_dentry->d_name.name, "dma_test_p%d_s%d", &bus_num, &slot_num);

	printk(KERN_INFO "scanf result: %d, %d\n", bus_num, slot_num);

	//TODO: test this by specifically placing the device to large number in QEMU config
	pdev = pci_get_domain_bus_and_slot(0,bus_num,PCI_DEVFN(slot_num,0));

	if(!pdev)
	{
		printk(KERN_ALERT "dev not found for %s\n", file->f_dentry->d_name.name);
		return -EFAULT;
	}

	priv = (struct ez8139_priv *) pci_get_drvdata(pdev);

	iowrite32(priv->tally_dump_bus >> 32, priv->regs + DTCCR + 4);
	iowrite32((priv->tally_dump_bus & DMA_BIT_MASK(32)) | DTCCR_CMD_BIT, priv->regs + DTCCR); //mark 4th bit to command dumping to the addr

	for(i=0;i<1000;i++) {
		if((ioread32(priv->regs + DTCCR) & DTCCR_CMD_BIT) == 0)
		{
			printk(KERN_INFO "got DTCCR_CMD_BIT back to 0 again\n");
			break;
		}
		udelay(10);
	}

	if(ioread32(priv->regs + DTCCR) & DTCCR_CMD_BIT) {
		printk(KERN_ALERT "waited for 1 sec but no result :(\n");
		return -EFAULT;
	}

	return single_open(file, show_dma_test_entry, priv->tally_dump_virt);
}

static int close_dma_test_entry(struct inode *inode, struct file *file)
{
	struct pci_dev *pdev = 0;
	int bus_num = 0;
	int slot_num = 0;

	sscanf(file->f_dentry->d_name.name, "dma_test_p%d_s%d", &bus_num, &slot_num);
	pdev = pci_get_domain_bus_and_slot(0,bus_num,PCI_DEVFN(slot_num,0));
	if(pdev)
		pci_dev_put(pdev);

	return 0;
}

static const struct file_operations dma_test_entry_fops = {
	.open = open_dma_test_entry,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = close_dma_test_entry
};

static struct proc_dir_entry* create_dma_test_entry(struct pci_dev *pdev)
{
	struct proc_dir_entry *new_dma_test_entry;
	char entry_name[18]; // format: dma_test_pX_sY where X<256, Y<32

	snprintf(entry_name, 18, "dma_test_p%d_s%d", pdev->bus->number, PCI_SLOT(pdev->devfn));

	new_dma_test_entry = proc_create(entry_name, 0, NULL, &dma_test_entry_fops);
	if(!new_dma_test_entry)
	{
		printk(KERN_ALERT "Unable to create a proc entry to test dma" PRINT_DEV_LOC);
	}

	return new_dma_test_entry;
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

	init_pci_dma(pdev);

	create_dma_test_entry(pdev); //no problem if fails

	//TODO: need to write a function like cp_init_hw in the original driver
	//we need to reset and re-configure the device

	//In QEMU this is enough to see packets coming to the device	
	iowrite8(CMD_TX_EN_BIT | CMD_RX_EN_BIT, priv->regs + CMDR); //write 1 to read & write enable pins in normal cmd reg

	return ret;
}

static void ez8139_remove(struct pci_dev *pdev)
{
	char entry_name[18];
	struct ez8139_priv *priv;
	priv = (struct ez8139_priv *) pci_get_drvdata(pdev);

	snprintf(entry_name, 18, "dma_test_p%d_s%d", pdev->bus->number, PCI_SLOT(pdev->devfn));
	remove_proc_entry(entry_name, NULL);
	dma_free_coherent(&pdev->dev, TALLY_DUMP_SIZE, priv->tally_dump_virt, priv->tally_dump_bus);

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
