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

#include <linux/pci.h> //for pci device
#include <linux/etherdevice.h> //for net device
#include <linux/interrupt.h> //interrupt handling

#include <linux/byteorder/generic.h> //for le to cpu conversions
#include <linux/delay.h> //to be able to wait
#include <linux/proc_fs.h> //to interact with userspace
#include <linux/seq_file.h> //to print via proc interface

#define DRV_NAME "ez8139"
#define PRINT_DEV_LOC " rtl8139 device on pci bus %d pci slot %d.\n", \
	pdev->bus->number, PCI_SLOT(pdev->devfn)

//some frequently used register addrs
#define CPCR 0xE0 //c+ mode command register
#define CMDR 0x37 //normal command register
#define DTCCR 0x10 //tally dump registers, 8 bytes long
#define IMR 0x3C //interrupt mask registers, 2 bytes long
#define ISR 0x3E //interrupt status registers, 2 bytes long
#define RCR 0x44 //Rx config registers, 4 bytes long

//various bit values
#define DAC_BIT (1<<4) //DAC bit in C+ command register
#define DTCCR_CMD_BIT (1<<3)	//set to 1 to start tally counter dump,
				//then read until set back to 0 by device
#define CMD_RES_BIT (1<<4) 	//set to 1 to reset the device
				//(needed during init to start with a clean slate)
#define CMD_TX_EN_BIT (1<<2) //enable TX in command register
#define CMD_RX_EN_BIT (1<<3) //enable RX in command register
#define CP_TX_EN_BIT 1 //enable TX in C+ mode
#define CP_RX_EN_BIT (1<<1) //enable RX in C+ mode
#define ISR_ROK 1 //raised if "new packet received" interrupt is issued (RX OK)
#define IMR_ROK 1 //mask "new packet received" interrupt
#define RCR_AB (1<<3) //accept broadcast packets
#define RCR_APM (1<<1) //accept packets with matching destination MAC address
#define RCR_AALL 0x3F //accept all packets


//some sizes
#define TALLY_DUMP_SIZE 64 	//size of tally dump
				//(its contents are in ch6.3 (DTCCR) in datasheet)

struct ez8139_pci_priv {
	void __iomem *regs; 	//regs addr in kernel's virtual address space
				//(mapped from IO addr space)
	dma_addr_t tally_dump_bus; //the address we will tell the bus
	void *tally_dump_virt; //the address we will use as CPU
};

struct ez8139_net_priv {
	struct net_device *dev;
	struct pci_dev *pdev;
};

static int init_pci_regs(struct pci_dev *pdev);	//initialize PCI dev for device register IO
static u64 pci_regs_test(void __iomem *regs);	//test PCI register access on device

static void deinit_pci_regs(struct pci_dev *pdev); //free regions and disable the device

static int init_pci_dma(struct pci_dev *pdev); //init DMA

static int init_pci_regs(struct pci_dev *pdev)
{
	struct ez8139_pci_priv *pp;
	resource_size_t pci_regs; //offset of device register addresses in IO addr space

	int ret;

	pp = (struct ez8139_pci_priv *) pci_get_drvdata(pdev);

	//enable the pci device for use
	ret = pci_enable_device(pdev);
	if (ret) {
		printk(KERN_ALERT "Could not enable" PRINT_DEV_LOC);
		return ret;
	}

	//register the regions in all BARs to this driver
	ret = pci_request_regions(pdev, DRV_NAME);
	if (ret) {
		//this may fail if a driver for rtl8139 took the device
		//but did not release regions yet.
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

	pp->regs = ioremap(pci_regs, pci_resource_len(pdev,1));
	//map a chunk of IO addr space as big as device's registers (known to PCI)
	//to virt addr space

	if (!pp->regs)
	{
		printk("Unable to map IO registers to virt addr space for" PRINT_DEV_LOC);
		pci_release_regions(pdev);
		pci_disable_device(pdev);
		return -EIO;
	}

	return 0;
}

static u64 pci_regs_test(void __iomem *regs)
{
	u32 mac1, mac2;
	u64 mac_whole;

	//if the PCI IO is properly initialized we can access registers
	//detailed in datasheet (section 6) over PCI.
	//let's read mac addr (IDR0-6),
	//datasheet says we can only read this 4-bytes at once.
	//we will use the mmap'd and then translated virt addr returned by ioremap
	//we could have also read this from eeprom directly
	//as is the case with original 8139 driver.

	mac1 = ioread32(regs);
	mac2 = ioread32(regs + 4);

	mac_whole = ((u64) mac2 << 32) | mac1;
	printk(KERN_INFO "Complete MAC address read from io regs is %pM - translated\n", \
			&mac_whole);

	return mac_whole;
}

static void deinit_pci_regs(struct pci_dev *pdev)
{
	printk("Releasing IO regions and disabling the" PRINT_DEV_LOC);
	//release the mmap'd region so that another driver can use it if necessary.
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static int init_pci_dma(struct pci_dev *pdev)
{
	u16 cmd_reg;
	int rc;
	struct ez8139_pci_priv *pp;
	pp = (struct ez8139_pci_priv *) pci_get_drvdata(pdev);

	printk(KERN_DEBUG "priv section is at %p\n", pp);

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
	cmd_reg = ioread16(pp->regs + (CPCR));
	iowrite16(cmd_reg | DAC_BIT, pp->regs + (CPCR));
	cmd_reg = ioread16(pp->regs + (CPCR));
	printk(KERN_INFO "C+ command reg value after setting DAC: %x\n", cmd_reg);

	pci_set_master(pdev);	//enable bus mastering for device and
				//read and set again latency timer
				//linux only sets latency timer to 64 or 255 I understand?

	//we won't write anything, but since dma_alloc_coherent is so easy ...
	pp->tally_dump_virt = dma_alloc_coherent(&pdev->dev, TALLY_DUMP_SIZE, &pp->tally_dump_bus, GFP_KERNEL);
	/*	pdev->dev - this is the device struct kernel knows
	 *	TALLY_DUMP_SIZE (64) - size of tally dump - still takes up one page :(
	 *	&pp->tally_dump_bus - allocated addr translated for bus,
	 *				will be filled by this function
	 */
	printk(KERN_INFO "I registered %p for tally dump ops\n", pp->tally_dump_virt);

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

	struct ez8139_pci_priv *pp;

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

	pp = (struct ez8139_pci_priv *) pci_get_drvdata(pdev);

	iowrite32(pp->tally_dump_bus >> 32, pp->regs + DTCCR + 4);
	iowrite32((pp->tally_dump_bus & DMA_BIT_MASK(32)) | DTCCR_CMD_BIT, \
			pp->regs + DTCCR); //mark 4th bit to command dumping to the addr

	for(i=0;i<1000;i++) {
		if((ioread32(pp->regs + DTCCR) & DTCCR_CMD_BIT) == 0)
		{
			printk(KERN_INFO "got DTCCR_CMD_BIT back to 0 again\n");
			break;
		}
		udelay(10);
	}

	if(ioread32(pp->regs + DTCCR) & DTCCR_CMD_BIT) {
		printk(KERN_ALERT "waited for 1 sec but no result :(\n");
		return -EFAULT;
	}

	return single_open(file, show_dma_test_entry, pp->tally_dump_virt);
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

	snprintf(entry_name, 18, "dma_test_p%d_s%d", \
			pdev->bus->number, PCI_SLOT(pdev->devfn));

	new_dma_test_entry = proc_create(entry_name, 0, NULL, &dma_test_entry_fops);
	if(!new_dma_test_entry)
	{
		printk(KERN_ALERT "Unable to create a proc entry to test dma" PRINT_DEV_LOC);
	}

	return new_dma_test_entry;
}

static irqreturn_t ez8139_interrupt(int irq, void *dev_instance)
{
	struct ez8139_net_priv *np = netdev_priv((struct net_device *) dev_instance);
	struct pci_dev *pdev = np->pdev;
	struct ez8139_pci_priv *pp = (struct ez8139_pci_priv *) pci_get_drvdata(pdev);
	u16 isr_reg = 0;

	//TODO: handle interrupt and schedule related bottom half tasklet

	printk(KERN_DEBUG "Got interrupt on" PRINT_DEV_LOC);

	isr_reg = ioread16(pp->regs + ISR);
	printk(KERN_DEBUG "Status of the ISR: %d\n", isr_reg);
	if(isr_reg & ISR_ROK) {
		iowrite16(isr_reg | ISR_ROK, pp->regs + ISR); //setting the bit makes it reset
		printk(KERN_DEBUG "Handled rx interrupt on" PRINT_DEV_LOC);
		return IRQ_HANDLED;
	} else {
		printk(KERN_DEBUG "Umm, it was not for" PRINT_DEV_LOC);
		return IRQ_NONE;
	}
}

static int ez8139_open(struct net_device *dev)
{
	struct ez8139_net_priv *np = netdev_priv(dev);
	struct pci_dev *pdev = np->pdev;
	struct ez8139_pci_priv *pp = (struct ez8139_pci_priv *) pci_get_drvdata(pdev);
	const int irq = pdev->irq;
	u16 imr_reg = 0;
	u8 cp_reg = 0;
	u8 cmd_reg = 0;
	int ret;

	printk(KERN_DEBUG "Setting up" PRINT_DEV_LOC);
	printk(KERN_DEBUG "It is supposed to be on irq line %d on pci\n", irq);

	ret = request_irq(irq, ez8139_interrupt, IRQF_SHARED, dev->name, dev);

	if(!ret) {
		imr_reg = ioread16(pp->regs + IMR);
		iowrite16(imr_reg | IMR_ROK, pp->regs + IMR); //send us an interrupt when Rx successful
	} else
		printk(KERN_ALERT "failed to register on irq line %d\n", irq);

	cp_reg = ioread8(pp->regs + CPCR);
	cmd_reg = ioread8(pp->regs + CMDR);
	iowrite8(cp_reg | CP_TX_EN_BIT | CP_RX_EN_BIT, pp->regs + CPCR);
	iowrite8(cp_reg | CMD_TX_EN_BIT | CMD_RX_EN_BIT, pp->regs + CMDR);
	//set read & write enable pins in normal cmd reg
	iowrite32(ioread32(pp->regs + RCR) | RCR_AALL, pp->regs + RCR);

	//TODO: return necessary error code on problem
	return ret;
}

static int ez8139_stop(struct net_device *dev)
{
	struct ez8139_net_priv *np = netdev_priv(dev);
	struct pci_dev *pdev = np->pdev;
	struct ez8139_pci_priv *pp = (struct ez8139_pci_priv *) pci_get_drvdata(pdev);
	u16 imr_reg = 0;
	u8 cp_reg = 0;
	u8 cmd_reg = 0;

	printk(KERN_DEBUG "Setting down" PRINT_DEV_LOC);

	//TODO: first check if irq is registered successfully
	free_irq(np->pdev->irq, dev);

	imr_reg = ioread16(pp->regs + IMR);
	iowrite16(imr_reg & ~IMR_ROK, pp->regs + IMR); //mask out Rx OK interrupts

	cp_reg = ioread8(pp->regs + CPCR);
	cmd_reg = ioread8(pp->regs + CMDR);
	iowrite8(cp_reg & ~(CP_TX_EN_BIT | CP_RX_EN_BIT), pp->regs + CPCR);
	iowrite8(cp_reg & ~(CMD_TX_EN_BIT | CMD_RX_EN_BIT), pp->regs + CMDR);

	return 0;
}

static netdev_tx_t ez8139_tx_start(struct sk_buff *skb, struct net_device *dev)
{
	struct ez8139_net_priv *np = netdev_priv(dev);
	struct pci_dev *pdev = np->pdev;

	printk(KERN_DEBUG "Got packet to trasmit for" PRINT_DEV_LOC);

	return NETDEV_TX_OK;
}

static struct net_device_ops ez8139_ndo = {
	.ndo_open = ez8139_open,
	.ndo_stop = ez8139_stop,
	.ndo_start_xmit = ez8139_tx_start,
};

static int ez8139_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int ret;
	struct ez8139_pci_priv *pp;
	struct net_device *dev;
	struct ez8139_net_priv *np;
	u64 dev_mac;

	printk(KERN_INFO "A pci device for ez8139 probed\n");

	pp = (struct ez8139_pci_priv*) kmalloc(sizeof(struct ez8139_pci_priv), GFP_KERNEL);
	if(!pp)	{
		printk("Unable to allocate kernel mem for" PRINT_DEV_LOC);
		return -ENOMEM;
	}
	pci_set_drvdata(pdev, pp); //set pdev's priv
	printk(KERN_DEBUG "private section is allocated at %p \
			for rtl8139 device on pci bus %d pci slot %d.\n", \
			pp, pdev->bus->number, PCI_SLOT(pdev->devfn));

	ret = init_pci_regs(pdev);
	if(ret)
	{
		kfree(pci_get_drvdata(pdev));
	}

	dev_mac = pci_regs_test(pp->regs); //get mac to test if device registers are accessible

	init_pci_dma(pdev);

	create_dma_test_entry(pdev); //no problem if fails

	//create netdev
	dev = alloc_etherdev(sizeof(struct ez8139_net_priv));
	if(!dev) {
		printk("Unable to allocate kernel mem for" PRINT_DEV_LOC);
		return -ENOMEM;
	}

	//let pci device know the net device
	SET_NETDEV_DEV(dev, &pdev->dev);

	np = netdev_priv(dev);

	np->pdev = pdev;
	np->dev = dev;

	dev->netdev_ops = &ez8139_ndo;
	dev->destructor = free_netdev;

	memcpy(dev->dev_addr, &dev_mac, 6);

	ret = register_netdev(dev);

	//TODO: need to write a function like cp_init_hw in the original driver
	//we need to reset and re-configure the device

	return ret;
}

static void ez8139_remove(struct pci_dev *pdev)
{
	char entry_name[18];
	struct ez8139_pci_priv *pp;
	pp = (struct ez8139_pci_priv *) pci_get_drvdata(pdev);


	snprintf(entry_name, 18, "dma_test_p%d_s%d", \
			pdev->bus->number, PCI_SLOT(pdev->devfn));
	remove_proc_entry(entry_name, NULL);

	dma_free_coherent(&pdev->dev, TALLY_DUMP_SIZE, pp->tally_dump_virt, pp->tally_dump_bus);

	printk(KERN_INFO "A pci device for ez8139 is being removed\n");
	deinit_pci_regs(pdev);
	kfree(pci_get_drvdata(pdev));

	return;
}

//we are only interested in Realtek 8139 PCI ID.
static DEFINE_PCI_DEVICE_TABLE(ez8139_pci_tbl) = {
{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, PCI_DEVICE_ID_REALTEK_8139), },
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
