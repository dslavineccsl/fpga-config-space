#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/aer.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/spinlock.h>
#include <asm/byteorder.h>

#include "pcie_wb.h"
#include "wishbone.h"

#if defined(__BIG_ENDIAN)
#define endian_addr(width, shift) (sizeof(wb_data_t)-width)-shift
#elif defined(__LITTLE_ENDIAN)
#define endian_addr(width, shift) shift
#else
#error "unknown machine byte order (endian)"
#endif


//define DBG_PRINT_EN


static unsigned int pmcintx      = 0; /* module parameter, force INTx interrupt for PCI/PMC card */
static unsigned int ebsbaren       = 0; /* module parameter, enable BAR2 for pci eb slave */

//#define FNAME if (unlikely(debug_fname)) printk(KERN_ALERT PCIE_WB ": %s\n", __FUNCTION__);
#define FNAME

static unsigned int debug_rw     = 1; /* module parameter, enable debug prints */
static unsigned int debug_offst  = 1; /* module parameter, enable debug prints */

static unsigned int debug_irqcfg = 0; /* module parameter, enable debug prints in irq handler*/
static unsigned int debug_irqh   = 0; /* module parameter, enable debug prints in irq handler*/

static unsigned int debug_cfg_rw = 1; /* module parameter, enable debug prints */
static unsigned int debug_cycle  = 1; /* module parameter, enable debug prints */
static unsigned int debug_rqrpl  = 1; /* module parameter, enable debug prints */
static unsigned int debug_fname  = 1; /* module parameter, enable debug prints */


static unsigned int cyctout  = 0xfffffff0;     /* module parameter, xwb cycle timeout */




#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,28)

/* Missing in 2.6.28. Present in 2.6.29. */
static void compat_pci_clear_master(struct pci_dev *dev)
{
	u16 old_cmd, cmd;
  
  FNAME

	pci_read_config_word(dev, PCI_COMMAND, &old_cmd);
	cmd = old_cmd & ~PCI_COMMAND_MASTER;
	pci_write_config_word(dev, PCI_COMMAND, cmd);
	dev->is_busmaster = false;
}

/* Override with backwards compatible version */
#define pci_clear_master compat_pci_clear_master
#endif

static void pcie_int_enable(struct pcie_wb_dev *dev, int on)
{
	int enable;
  FNAME

	/* enable/disable interrupts for pmc device */
	if(dev->pci_dev->device == PMC_WB_DEVICE_ID){
		iowrite32((on ? 0x20000000UL:0) + 0x10000000UL, dev->pci_res[0].addr + CONTROL_REGISTER_HIGH);
		ioread32(dev->pci_res[0].addr + CONTROL_REGISTER_HIGH); /*dummy read to be sure that write was executed*/
	}
	/* enable/disable interrupts for pcie device */
	else{
		enable = on && !dev->msi;
		iowrite32((enable ? 0x20000000UL:0) + 0x10000000UL, dev->pci_res[0].addr + CONTROL_REGISTER_HIGH);
  }
  
#ifdef DBG_PRINT_EN  
	if (unlikely(debug_irqcfg)) printk(KERN_ALERT PCIE_WB ": %s: interrupt on=%d\n", __FUNCTION__, on);  
#endif  
  
}

static void wb_cycle(struct wishbone* wb, int on)
{
	struct pcie_wb_dev* dev;
	unsigned char* control;
  
  FNAME

	dev = container_of(wb, struct pcie_wb_dev, wb);
	control = dev->pci_res[0].addr;

#ifdef DBG_PRINT_EN
	if (unlikely(debug_cycle)) printk(KERN_ALERT PCIE_WB ": %s: cycle(%d)\n", __FUNCTION__, on);
#endif

	iowrite32((on?0x80000000UL:0) + 0x40000000UL, control + CONTROL_REGISTER_HIGH);
}

static void wb_byteenable(struct wishbone* wb, unsigned char be)
{
	struct pcie_wb_dev* dev;

	FNAME

	dev = container_of(wb, struct pcie_wb_dev, wb);

	switch (be) {
	case 0x1:
		dev->width = 1;
		dev->shift = 0;
		dev->low_addr = endian_addr(1, 0);
		break;
	case 0x2:
		dev->width = 1;
		dev->shift = 8;
		dev->low_addr = endian_addr(1, 1);
		break;
	case 0x4:
		dev->width = 1;
		dev->shift = 16;
		dev->low_addr = endian_addr(1, 2);
		break;
	case 0x8:
		dev->width = 1;
		dev->shift = 24;
		dev->low_addr = endian_addr(1, 3);
		break;
	case 0x3:
		dev->width = 2;
		dev->shift = 0;
		dev->low_addr = endian_addr(2, 0);
		break;
	case 0xC:
		dev->width = 2;
		dev->shift = 16;
		dev->low_addr = endian_addr(2, 2);
		break;
	case 0xF:
		dev->width = 4;
		dev->shift = 0;
		dev->low_addr = endian_addr(4, 0);
		break;
	default:
		/* noop -- ignore the strange bitmask */
		break;
	}
}

static void wb_write(struct wishbone* wb, wb_addr_t addr, wb_data_t data)
{
	struct pcie_wb_dev* dev;
	unsigned char* control;
	unsigned char* window;
	wb_addr_t window_offset;
  
  FNAME

	dev = container_of(wb, struct pcie_wb_dev, wb);
	control = dev->pci_res[0].addr;
	window = dev->pci_res[1].addr;

	window_offset = addr & WINDOW_HIGH;
	if (unlikely(window_offset != dev->window_offset)) {
		iowrite32(window_offset, control + WINDOW_OFFSET_LOW);

#ifdef DBG_PRINT_EN    
    if (unlikely(debug_offst)) printk(KERN_DEBUG PCIE_WB ": %s: changing window offset, pre=0x%08X post=0x%08X\n", 
      __FUNCTION__, dev->window_offset, window_offset);
#endif
    
		dev->window_offset = window_offset;
	}

	switch (dev->width) {
	case 4:
  
#ifdef DBG_PRINT_EN  
		if (unlikely(debug_rw)) printk(KERN_DEBUG PCIE_WB ": %s: iowrite32\tA:0x%08X\tD:0x%08X\n", 
      __FUNCTION__, addr & ~3, data);
#endif
    
		iowrite32(data, window + (addr & WINDOW_LOW));
		break;
	case 2:
  
#ifdef DBG_PRINT_EN  
		if (unlikely(debug_rw)) printk(KERN_DEBUG PCIE_WB ": %s: iowrite16\tA:0x%08X\tD:0x%04x\n", 
      __FUNCTION__, (addr & ~2) + dev->low_addr, data >> dev->shift);
#endif
    
		iowrite16(data >> dev->shift, window + (addr & WINDOW_LOW) + dev->low_addr);
		break;
	case 1:
  
#ifdef DBG_PRINT_EN  
		if (unlikely(debug_rw)) printk(KERN_DEBUG PCIE_WB ": %s: iowrite8\tA:0x%08X\tD:0x%02x\n", 
      __FUNCTION__, (addr & ~1) + dev->low_addr, data >> dev->shift);
#endif
    
		iowrite8 (data >> dev->shift, window + (addr & WINDOW_LOW) + dev->low_addr);
		break;
	}
}

static wb_data_t wb_read(struct wishbone* wb, wb_addr_t addr)
{
	wb_data_t out;
	struct pcie_wb_dev* dev;
	unsigned char* control;
	unsigned char* window;
	wb_addr_t window_offset;
  FNAME

	dev = container_of(wb, struct pcie_wb_dev, wb);
	control = dev->pci_res[0].addr;
	window = dev->pci_res[1].addr;

	window_offset = addr & WINDOW_HIGH;
	if (unlikely(window_offset != dev->window_offset)) {

#ifdef DBG_PRINT_EN    
    if (unlikely(debug_offst)) printk(KERN_DEBUG PCIE_WB " %s: changing window offset, pre=0x%08X post=0x%08X\n", 
      __FUNCTION__, dev->window_offset, window_offset);    
#endif
    
		iowrite32(window_offset, control + WINDOW_OFFSET_LOW);
		dev->window_offset = window_offset;
	}

	switch (dev->width) {
	case 4:
		out = ((wb_data_t)ioread32(window + (addr & WINDOW_LOW)));
    
#ifdef DBG_PRINT_EN    
		if (unlikely(debug_rw)) printk(KERN_ALERT PCIE_WB ": %s: ioread32\tA:0x%08X\tD:0x%08X\n", 
      __FUNCTION__, addr & ~3, out);
#endif
    
		break;
	case 2:
		out = ((wb_data_t)ioread16(window + (addr & WINDOW_LOW) + dev->low_addr)) << dev->shift;
    
#ifdef DBG_PRINT_EN    
		if (unlikely(debug_rw)) printk(KERN_ALERT PCIE_WB ": %s: ioread16\tA:0x%08X\tD:0x%04x\n", 
      __FUNCTION__, addr & ~2, out);
#endif    
		break;
	case 1:
		out = ((wb_data_t)ioread8 (window + (addr & WINDOW_LOW) + dev->low_addr)) << dev->shift;

#ifdef DBG_PRINT_EN    
		if (unlikely(debug_rw)) printk(KERN_ALERT PCIE_WB ": %s: ioread8\tA:0x%08X\tD:0x%02x\n", 
      __FUNCTION__, addr & ~1, out);
#endif
    
		break;
	default: /* technically should be unreachable */
		out = 0;
		break;
	}

	mb(); /* ensure serial ordering of non-posted operations for wishbone */

	return out;
}

static wb_data_t wb_read_cfg(struct wishbone *wb, wb_addr_t addr)
{
	wb_data_t out;
	struct pcie_wb_dev* dev;
	unsigned char* control;
  
  FNAME

	dev = container_of(wb, struct pcie_wb_dev, wb);
	control = dev->pci_res[0].addr;

	switch (addr) {
	case 0:  out = ioread32(control + ERROR_FLAG_HIGH);   break;
	case 4:  out = ioread32(control + ERROR_FLAG_LOW);    break;
	case 8:  out = ioread32(control + SDWB_ADDRESS_HIGH); break;
	case 12: out = ioread32(control + SDWB_ADDRESS_LOW);  break;
  
  case 0xF0: out = ioread32(control + CYCLE_TIMEOUT_CONTROL);  break;
  case 0xF4: out = ioread32(control + CYCLE_TIMEOUT_MAX);  break;
  case 0xF5: iowrite32(1, control + CYCLE_TIMEOUT_MAX); out=0; break; // clear max cycle time counter
  
	default: out = 0; break;
	}

	mb(); /* ensure serial ordering of non-posted operations for wishbone */

#ifdef DBG_PRINT_EN 
  if (unlikely(debug_cfg_rw)) printk(KERN_ALERT PCIE_WB ": %s: A=0x%08X D=0x%08X\n", 
    __FUNCTION__, addr, out);
#endif

	return out;
}

/*
static void wb_set_num_of_cycle_ops(struct wishbone *wb, wb_data_t data)
{
	struct pcie_wb_dev* dev;
	unsigned char* control;
  
  FNAME

	dev = container_of(wb, struct pcie_wb_dev, wb);
	control = dev->pci_res[0].addr;

  if (unlikely(debug_cfg_rw)) printk(KERN_ALERT PCIE_WB ": %s: A=0x%08X D=0x%08X\n", __FUNCTION__, addr, data);
  iowrite32(data, control + CYCLE_OP_COUNT);
}
*/


static int wb_request(struct wishbone *wb, struct wishbone_request *req)
{
	struct pcie_wb_dev* dev;
	unsigned char* control;
	uint32_t ctl;
	int out;

	FNAME

	dev = container_of(wb, struct pcie_wb_dev, wb);
	control = dev->pci_res[0].addr;

	ctl        = ioread32(control + MASTER_CTL_HIGH);
	req->addr  = ioread32(control + MASTER_ADR_LOW);
	req->data  = ioread32(control + MASTER_DAT_LOW);
	req->mask  = ctl & 0xf;
	req->write = (ctl & 0x40000000) != 0;

	out = (ctl & 0x80000000) != 0;

#ifdef DBG_PRINT_EN
  if (unlikely(debug_rqrpl)) printk(KERN_ALERT PCIE_WB ": %s: C=0x%08X  A=0x%08X D=0x%08X M=0x%08X W=0x%08X O=0x%08X\n",
    __FUNCTION__, ctl, req->addr, req->data, req->mask, req->write, out);
#endif

  
	if (out){ 
    
#ifdef DBG_PRINT_EN    
    if (unlikely(debug_rqrpl)) printk(KERN_ALERT PCIE_WB ": %s: WR A=MASTER_CTL_HIGH=0x0%08X D=1\n", 
      __FUNCTION__, MASTER_CTL_HIGH);
#endif
    
    iowrite32(1, control + MASTER_CTL_HIGH); /* dequeue operation */
  }

#ifdef DBG_PRINT_EN  
  if (unlikely(debug_rqrpl)) printk(KERN_ALERT PCIE_WB ": %s: Reenabling IRQs\n", __FUNCTION__);
#endif
  
	pcie_int_enable(dev, 1);

	return out;
}

static void wb_reply(struct wishbone *wb, int err, wb_data_t data)
{
	struct pcie_wb_dev* dev;
	unsigned char* control;

	dev = container_of(wb, struct pcie_wb_dev, wb);
	control = dev->pci_res[0].addr;

#ifdef DBG_PRINT_EN
  if (unlikely(debug_rqrpl)) printk(KERN_ALERT PCIE_WB ": %s: WR A=MASTER_DAT_LOW=0x0%08X D=0x0%08X\n", 
    __FUNCTION__, MASTER_DAT_LOW, data);
#endif
  
	iowrite32(data, control + MASTER_DAT_LOW);

#ifdef DBG_PRINT_EN  
  if (unlikely(debug_rqrpl)) printk(KERN_ALERT PCIE_WB ": %s: WR A=MASTER_CTL_HIGH=0x0%08X D=0x0%08X\n", 
    __FUNCTION__, MASTER_CTL_HIGH, err+2);
#endif
  
	iowrite32(err+2, control + MASTER_CTL_HIGH);
}

// =============================================================================

static void wb_cycle_ebs(struct wishbone* wb, int on)
{
	struct pcie_wb_dev* dev;
	unsigned char* control;
  wb_data_t out;
  
  FNAME

	dev = container_of(wb, struct pcie_wb_dev, wb);
	control = dev->pci_res[0].addr;

#ifdef DBG_PRINT_EN
	if (unlikely(debug_cycle)) printk(KERN_ALERT PCIE_WB ": %s: setting EBS cycle to (%d)\n", __FUNCTION__, on);
#endif

//  msleep(10);
	iowrite32((on?0x80000000UL:0) + 0x40000000UL, control + EB_SLAVE_CFG_REG_CYCLE);

  mb();
  
#ifdef DBG_PRINT_EN
//  msleep(10);
  out = ioread32(control + EB_SLAVE_CFG_REG_CYCLE);
//  msleep(10);
	if (unlikely(debug_cycle)) printk(KERN_ALERT PCIE_WB ": %s: EBS cycle set to 0x%08X\n", __FUNCTION__, out);
//  msleep(10);
#endif

}



static void wb_write_ebs(struct wishbone* wb, wb_addr_t addr, wb_data_t data)
{
	struct pcie_wb_dev* dev;
	unsigned char* control;
	unsigned char* window;
	wb_addr_t window_offset;
  
  FNAME

	dev = container_of(wb, struct pcie_wb_dev, wb);
	control = dev->pci_res[0].addr;
    
#ifdef DBG_PRINT_EN  
		if (unlikely(debug_rw)) printk(KERN_DEBUG PCIE_WB ": %s: iowrite32\tA:0x%08X\tD:0x%08X\n", 
      __FUNCTION__, EB_RX_FIFO_DATA, data);
#endif
    
		iowrite32(data, control + EB_RX_FIFO_DATA);
}

static wb_data_t wb_read_ebs(struct wishbone* wb, wb_addr_t addr)
{
	wb_data_t out;
	struct pcie_wb_dev* dev;
	unsigned char* control;
	unsigned char* window;
	wb_addr_t window_offset;
  FNAME

	dev = container_of(wb, struct pcie_wb_dev, wb);
//	control = dev->pci_res[0].addr;
	control = dev->pci_res[0].addr;

	out = ((wb_data_t)ioread32(control + EB_TX_FIFO_DATA));
	mb(); //ensure serial ordering of non-posted operations for wishbone
#ifdef DBG_PRINT_EN    
		if (unlikely(debug_rw)) printk(KERN_ALERT PCIE_WB ": %s: ioread32\tA:0x%08X\tD:0x%08X\n", 
      __FUNCTION__, EB_TX_FIFO_DATA, out);
#endif    

	return out;
}


// =============================================================================


static const struct wishbone_operations wb_ops = {
	.owner      = THIS_MODULE,
	.cycle      = wb_cycle,
	.byteenable = wb_byteenable,
	.write      = wb_write,
	.read       = wb_read,
	.read_cfg   = wb_read_cfg,
	.request    = wb_request,
	.reply      = wb_reply,
  .cycle_ebs  = wb_cycle_ebs,
  .write_ebs  = wb_write_ebs,
  .read_ebs   = wb_read_ebs
};

static irqreturn_t irq_handler(int irq, void *dev_id)
{
	struct pcie_wb_dev *dev = dev_id;
	unsigned char* wb_conf;
	uint32_t wb_cfg_data;

	/* if card is PMC with IntX interrupts  */
	/* it is likely that irq line is shared */
	if (!(dev->msi) && (dev->pci_dev->device == PMC_WB_DEVICE_ID)){
		/* check that pmc card has requested IRQ */
		/* if it has not then exit               */
		wb_conf = dev->pci_res[2].addr;
		wb_cfg_data = ioread32(wb_conf + WB_CONF_ISR_REG);
		if (!(wb_cfg_data & WB_CONF_IRQ_STATUS_MASK)){
			return IRQ_NONE;
		}
	}

#ifdef DBG_PRINT_EN
  if (debug_irqh) printk(KERN_ALERT ": %s: IRQ handled\n", __FUNCTION__);
#endif
  
	pcie_int_enable(dev, 0); /* disable IRQ on Etherbone layer - Etherbone */
	wishbone_slave_ready(&dev->wb);

	return IRQ_HANDLED;
}

static int setup_bar(struct pci_dev* pdev, struct pcie_wb_resource* res, int bar)
{
	res->start = pci_resource_start(pdev, bar);
	res->end = pci_resource_end(pdev, bar);
	res->size = res->end - res->start + 1;

 if (debug_rw) printk(KERN_INFO PCIE_WB ": %s: BAR%d  0x%lx - 0x%lx\n", __FUNCTION__, bar, res->start, res->end);

	if ((pci_resource_flags(pdev, 0) & IORESOURCE_MEM) == 0) {
		printk(KERN_ALERT PCIE_WB ": %s: BAR%d is not a memory resource\n", __FUNCTION__, bar);
		return -ENOMEM;
	}

	if (!request_mem_region(res->start, res->size, PCIE_WB)) {
		printk(KERN_ALERT PCIE_WB ": %s: BAR%d: request_mem_region failed\n", __FUNCTION__, bar);
		return -ENOMEM;
	}

	res->addr = ioremap_nocache(res->start, res->size);
	if (debug_rw) printk(KERN_ALERT PCIE_WB ": %s: BAR%d: ioremap to %lx\n", __FUNCTION__, bar, (unsigned long)res->addr);

	return 0;
}

static void destroy_bar(struct pcie_wb_resource* res)
{
// if (debug) printk(KERN_ALERT ": %s: released io 0x%lx\n", __FUNCTION__, res->start);

	iounmap(res->addr);
	release_mem_region(res->start, res->size);
}

static int probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	/* Do probing type stuff here.
	 * Like calling request_region();
	 * reading BARs
	 * reading IRQ
	 * register char dev
	 */
	struct pcie_wb_dev *dev;
	unsigned char* control;
  unsigned char* control_eb;
	unsigned char* wb_conf;

//	if(unlikely(debug)){
		printk(KERN_INFO PCIE_WB ": %s:-----------------------------\n", __FUNCTION__);
		printk(KERN_INFO PCIE_WB ": %s: PCI Device info\n"             , __FUNCTION__);
		printk(KERN_INFO PCIE_WB ": %s: vendor        : %04x\n"        , __FUNCTION__, pdev->vendor);
		printk(KERN_INFO PCIE_WB ": %s: device        : %04x\n"        , __FUNCTION__, pdev->device);
		printk(KERN_INFO PCIE_WB ": %s: PCIe capable  : %04x\n"        , __FUNCTION__, pdev->pcie_cap);
		printk(KERN_INFO PCIE_WB ": %s: irq number    : %d\n"          , __FUNCTION__, pdev->irq);
		printk(KERN_INFO PCIE_WB ": %s:-----------------------------\n", __FUNCTION__);
//	}

	if (pdev->revision != 0x01) {
		printk(KERN_ALERT PCIE_WB ": %s: revision ID wrong!\n", __FUNCTION__);
		goto fail_out;
	}

	if (pci_enable_device(pdev) < 0) {
		printk(KERN_ALERT PCIE_WB ": %s: could not enable device!\n", __FUNCTION__);
		goto fail_out;
	}

	dev = kmalloc(sizeof(struct pcie_wb_dev), GFP_KERNEL);
	if (!dev) {
		printk(KERN_ALERT PCIE_WB "%s: could not allocate memory for pcie_wb_dev structure!\n", __FUNCTION__);
		goto fail_disable;
	}

	/* Initialize structure */
	dev->pci_dev = pdev;
	dev->msi = 1;
	dev->window_offset = 0;
	dev->low_addr = 0;
	dev->width = 4;
	dev->shift = 0;

	dev->wb.wops = &wb_ops;
	dev->wb.parent = &pdev->dev;
	dev->wb.mask = 0xffff;

//	dev->wb_ebs.wops = &wb_ops;
//	dev->wb_ebs.parent = &pdev->dev;
//	dev->wb_ebs.mask = 0xffff;



	pci_set_drvdata(pdev, dev);

  /* check which device is being installed: PMC or PCIe and setup bars accordingly */
	if (pdev->device == PMC_WB_DEVICE_ID) {
		printk(KERN_INFO PCIE_WB ": %s: Requesting BARs for PMC Device : %04x:%04x\n", __FUNCTION__, pdev->vendor, pdev->device);
		/* BAR1 - etherbone configuration space */
		if (setup_bar(pdev, &dev->pci_res[0], 1) < 0) goto fail_free;
		/* BAR2 - wishbone */
		if (setup_bar(pdev, &dev->pci_res[1], 2) < 0) goto fail_bar0;
		/* BAR0 -  PCI/WB bridge configuration space */
		if (setup_bar(pdev, &dev->pci_res[2], 0) < 0) goto fail_bar1;
    
//    if(ebsbaren){
//		  /* BAR3 - PCI EB slave to XWB master */
//		  if (setup_bar(pdev, &dev->pci_res[3], 3) < 0) goto fail_bar2;
//    }
    
	}
	else{
		printk(KERN_INFO PCIE_WB ": %s: Requesting BARs for PCIe Device : %04x:%04x\n", __FUNCTION__, pdev->vendor, pdev->device);
		/* BAR0 - etherbone configuration space */
		if (setup_bar(pdev, &dev->pci_res[0], 0) < 0) goto fail_free;
		/* BAR1 - Xwishbone */
		if (setup_bar(pdev, &dev->pci_res[1], 1) < 0) goto fail_bar0;
    
//    if(ebsbaren){
//		  /* BAR2 - PCI EB slave to XWB master */
//		  if (setup_bar(pdev, &dev->pci_res[3], 3) < 0) goto fail_bar2;
//    }
	}

	/* Initialize device registers */
	control    = dev->pci_res[0].addr;
  printk(KERN_INFO PCIE_WB ": %s: Init CFG registers\n", __FUNCTION__);
	iowrite32(0, control + WINDOW_OFFSET_LOW);
	iowrite32(0, control + CONTROL_REGISTER_HIGH);
  printk(KERN_INFO PCIE_WB ": %s: Init CFG registers done\n", __FUNCTION__);  
  
  // set max timeout for XWB cycle op
  printk(KERN_INFO PCIE_WB ": %s: Init CFG registers: CYC TOUT\n", __FUNCTION__);
  iowrite32(cyctout, control + CYCLE_TIMEOUT_CONTROL);
  printk(KERN_INFO PCIE_WB ": %s: Init CFG registers: CYC TOUT done\n", __FUNCTION__);
  
//  if(ebsbaren){
    // set max timeout for EB cycle
//    control_eb = dev->pci_res[0].addr;
//    printk(KERN_INFO PCIE_WB ": %s: Init EB CFG registers: CYC TOUT 0x%08X\n", __FUNCTION__, control_eb);
    iowrite32(cyctout, control + EB_SLAVE_CYLE_TOUT_MAX);
    printk(KERN_INFO PCIE_WB ": %s: Init EB CFG registers: CYC TOUT done\n", __FUNCTION__);
//  }
  
	/* Configure interrupts*/
	/* configure if device is PCIe and wants MSI */
  printk(KERN_INFO PCIE_WB ": %s: Configuring interrupts...\n", __FUNCTION__);
	if(pdev->device != PMC_WB_DEVICE_ID){
		if(pdev->pcie_cap && dev->msi){
			pci_set_master(pdev); /* enable bus mastering => needed for MSI */
			printk(KERN_INFO PCIE_WB ": %s: Enabled bus mastering for PCI Device : %04x:%04x\n", __FUNCTION__, pdev->vendor, pdev->device);

			/* enable message signaled interrupts */
			if (pci_enable_msi(pdev) != 0) {
			/* resort to legacy interrupts if MSI enable failed*/
			printk(KERN_ALERT PCIE_WB ": %s: Could not enable MSI interrupting (using legacy)\n", __FUNCTION__);
			dev->msi = 0;
			pci_clear_master(pdev);
			pci_intx(pdev, 1); /* enable legacy INTx interrupts for PCIe device*/
			printk(KERN_INFO PCIE_WB ": %s: Enabled legacy interrupts for PCIe Device : %04x:%04x\n", __FUNCTION__, pdev->vendor, pdev->device);
		}
		else{
			/* disable legacy interrupts when using MSI */
			printk(KERN_INFO PCIE_WB ": %s: Enabled MSI, disabling INTx interrupts for PCIe Device : %04x:%04x\n", __FUNCTION__, pdev->vendor, pdev->device);
			pci_intx(pdev, 0);
			}
		}
	}
	else{
		/* configure interrupts for pmc device */
		wb_conf = dev->pci_res[2].addr;

		/* Enable INTx interrupts on PMC device */
		if(pmcintx){
			iowrite32(1, wb_conf + WB_CONF_ICR_REG); /* enable wishbone interrupts to the PCI core */
			iowrite32(0x10, control + PMC_IRQ_CONTROL); /* enable INTx interrupts to wishbone */

			dev->msi = 0;
			pci_intx(pdev, 1); /* enable INTx interrupts on PMC device */
			printk(KERN_INFO PCIE_WB ": %s: Enabled INTx interrupts for PMC Device : %04x:%04x\n", __FUNCTION__, pdev->vendor, pdev->device);
	}
	else{ /* enable MSI */
		pci_set_master(pdev); /* enable bus mastering => needed for MSI */

			/* enable message signaled interrupts */
			if (pci_enable_msi(pdev) != 0) {
				printk(KERN_ALERT PCIE_WB ": %s: Could not enable MSI interrupting on PMC device\n", __FUNCTION__);
				dev->msi = 0;
				pci_clear_master(pdev);
			}
			else{
				iowrite32(0x1, control + PMC_IRQ_CONTROL); /* enable MSI interrupts to wishbone */
			}
		}
	}

	if (wishbone_register(&dev->wb) < 0) {
		printk(KERN_ALERT PCIE_WB ": %s: could not register wishbone bus\n", __FUNCTION__);
		goto fail_msi;
	}

//  if(ebsbaren){
//    if (wishbone_register_ebs(&dev->wb_ebs) < 0) {
//      printk(KERN_ALERT PCIE_WB ": %s: could not register wishbone ebs bus\n", __FUNCTION__);
//      goto fail_msi_ebs;
//    }
//  }

	if (request_irq(pdev->irq, irq_handler, IRQF_SHARED, "pcie_wb", dev) < 0) {
		printk(KERN_ALERT PCIE_WB ": %s: could not register interrupt handler\n", __FUNCTION__);
		goto fail_reg;
	}

	/* Enable interrupts from wishbone */
	pcie_int_enable(dev, 1);

  return 0;

//fail_msi_ebs:
//  wishbone_unregister_ebs(&dev->wb_ebs);
fail_reg:
	wishbone_unregister(&dev->wb);
fail_msi:
	if (dev->msi) {
		pci_intx(pdev,0 );
		pci_disable_msi(pdev);
	}
/*fail_master:*/
	pci_clear_master(pdev);
//fail_bar2:
//  if(ebsbaren) destroy_bar(&dev->pci_res[3]);
fail_bar1:
	destroy_bar(&dev->pci_res[1]);
fail_bar0:
	destroy_bar(&dev->pci_res[0]);
fail_free:
	kfree(dev);
fail_disable:
	pci_disable_device(pdev);
fail_out:
	return -EIO;
}

static void remove(struct pci_dev *pdev)
{
	struct pcie_wb_dev *dev;

//if(unlikely(debug)){
		printk(KERN_INFO PCIE_WB ": %s:-------------------------\n", __FUNCTION__);
		printk(KERN_INFO PCIE_WB ": %s: Removing PCI Device : \n"  , __FUNCTION__);
		printk(KERN_INFO PCIE_WB ": %s: vendor        : %04x\n"    , __FUNCTION__, pdev->vendor);
		printk(KERN_INFO PCIE_WB ": %s: device        : %04x\n"    , __FUNCTION__, pdev->device);
		printk(KERN_INFO PCIE_WB ": %s: PCIe capable  : %04x\n"    , __FUNCTION__, pdev->pcie_cap);
		printk(KERN_INFO PCIE_WB ": %s: irq number    : %d\n"      , __FUNCTION__, pdev->irq);
		printk(KERN_INFO PCIE_WB ": %s:-------------------------\n", __FUNCTION__);
//  }

  dev = pci_get_drvdata(pdev);
	/* disable/remove interrupts*/
  printk(KERN_ALERT PCIE_WB ": %s: Disabling XWB interrupts\n", __FUNCTION__);
	pcie_int_enable(dev, 0);
  
  printk(KERN_ALERT PCIE_WB ": %s: Unregistering Wishbone devide\n", __FUNCTION__);
//  if(ebsbaren) wishbone_unregister_ebs(&dev->wb_ebs);

    printk(KERN_ALERT PCIE_WB ": %s: Freeing interrupt\n", __FUNCTION__);
	free_irq(dev->pci_dev->irq, dev);

    printk(KERN_ALERT PCIE_WB ": %s: Unregistering Wishbone device\n", __FUNCTION__);    
	wishbone_unregister(&dev->wb);
  
  printk(KERN_ALERT PCIE_WB ": %s: Disabling device interrupts\n", __FUNCTION__);
	if(dev->msi){
		pci_disable_msi(pdev);
		pci_clear_master(pdev);
	}else{
		pci_intx(pdev, 0);
	}

  printk(KERN_ALERT PCIE_WB ": %s: Destroying BARs\n", __FUNCTION__);
//  if(ebsbaren){
//    destroy_bar(&dev->pci_res[3]);
//  }

	if(pdev->device == PMC_WB_DEVICE_ID){
		destroy_bar(&dev->pci_res[2]);
  }
  destroy_bar(&dev->pci_res[1]);
	destroy_bar(&dev->pci_res[0]);
	kfree(dev);

	printk(KERN_INFO PCIE_WB ": %s: Removed Device %04x:%04x\n", __FUNCTION__, pdev->vendor, pdev->device);
	pci_disable_device(pdev);
}

static struct pci_device_id ids[] = {
	{ PCI_DEVICE(PCIE_WB_VENDOR_ID, PCIE_WB_DEVICE_ID), },
  { PCI_DEVICE(PCIE_WB_VENDOR_ID, PMC_WB_DEVICE_ID ), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, ids);

static struct pci_driver pcie_wb_driver = {
	.name = PCIE_WB,
	.id_table = ids,
	.probe = probe,
	.remove = remove,
};

static int __init pcie_wb_init(void)
{
	return pci_register_driver(&pcie_wb_driver);
}

static void __exit pcie_wb_exit(void)
{
	pci_unregister_driver(&pcie_wb_driver);
}


MODULE_AUTHOR("Stefan Rauch <s.rauch@gsi.de> Dusan Slavinec <dusan.slavinec@cosylab.com>");
MODULE_DESCRIPTION("GSI Altera-Wishbone bridge driver");

module_param(debug_rw, int, 0644);
MODULE_PARM_DESC(debug_rw, "Enable read/write debug prints");

module_param(debug_offst, int, 0644);
MODULE_PARM_DESC(debug_offst, "Enable offset change debug prints");

module_param(debug_irqcfg, int, 0644);
MODULE_PARM_DESC(debug_irqcfg, "Enable irq configuration debug prints");

module_param(debug_irqh, int, 0644);
MODULE_PARM_DESC(debug_irqh, "Enable irq handler debug prints");

module_param(debug_cfg_rw, int, 0644);
MODULE_PARM_DESC(debug_cfg_rw, "Enable WB configuration debug prints");

module_param(debug_cycle, int, 0644);
MODULE_PARM_DESC(debug_cycle, "Enable Cycle debug prints");

module_param(debug_rqrpl, int, 0644);
MODULE_PARM_DESC(debug_rqrpl, "Enable Request/Reply debug prints");

module_param(debug_fname, int, 0644);
MODULE_PARM_DESC(debug_fname, "Enable function name debug prints");

module_param(pmcintx, int, 0644);
MODULE_PARM_DESC(pmcintx, "Force INTx interrupt for PMC card");

module_param(ebsbaren, int, 0644);
MODULE_PARM_DESC(ebsbaren, "Enable BAR for PCIe EB slave");

module_param(cyctout, int, 0644);
MODULE_PARM_DESC(cyctout, "Wishbone cycle timeout");


MODULE_LICENSE("GPL");
MODULE_VERSION(PCIE_WB_VERSION);

module_init(pcie_wb_init);
module_exit(pcie_wb_exit);
