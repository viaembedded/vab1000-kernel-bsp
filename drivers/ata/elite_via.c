/*
 * elite_via.c 	- Driver for SATA controller of VIA ELITE ARM SOC
 *			  (C) 2012 VIA Technology Inc
 *
 *
 * Driver for SATA controller of VIA ELITE ARM SOC
 * This controller is based on VIA VT3456 SATA controller
 * In VIA ELITE ARM SOC, it's no longer a PCI device 
 * But a standard platform device, and no more South bridges.
 * Drivers must be re-designed.
 *
 *
 * Version 1.0
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/gfp.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/errno.h>

#include <linux/blkdev.h>
#include <scsi/scsi_host.h>
#include <linux/libata.h>
#include <linux/of_device.h>
#include <linux/of.h>

#include <asm/irq.h>

#define DRV_NAME "elite_via"
#define DRV_VERSION "0.0.1"

enum {
	VIA_BAD_PREQ	= 0x01, /* Crashes if PREQ# till DDACK# set */
	VIA_BAD_CLK66	= 0x02, /* 66 MHz clock doesn't work correctly */
	VIA_SET_FIFO	= 0x04, /* Needs to have FIFO split set */
	VIA_NO_UNMASK	= 0x08, /* Doesn't work with IRQ unmasking on */
	VIA_BAD_ID	    = 0x10, /* Has wrong vendor ID (0x1107) */
	VIA_BAD_AST	    = 0x20, /* Don't touch Address Setup Timing */
	VIA_NO_ENABLES	= 0x40, /* Has no enablebits */
	VIA_SATA_PATA	= 0x80, /* SATA/PATA combined configuration */
	/* additional flags for sata clter */
	VIA_IDFLAG_SINGLE = (1 << 8), /* single channel controller */
};

#define DEVICE_ID_VIA_ELITE   (0x9001)
#define DEVICE_ID_VIA_ANON    (0xffff)

/******************************************/
/**
 * Registers Offset
 */
#define  REG_DEVICE_ID_OFFSET       (0x02)  /* PCI device ID */
#define  REG_COMMAND_BASE_OFFSET    (0x10)  /* 0x10-0x13 */
#define  REG_CONTROL_BASE_OFFSET    (0x14)  /* 0x14-0x17 */
#define  REG_BMDMA_BASE_OFFSET      (0x20)  /* 0x20-0x23, Bus Master Base */

/*
 * platform resources for SATA controller
 * This could be defined and set in BSP
 *
 * Base address for SATA: 0xD800:D000 - 0xD800:D7FF
 * Irq num defined in mach-elite/irqs.h
 */
 
#define  ELITE_SATA_PHYS_BASE  (0xD800D000)
#define  ELITE_SATA_PHYS_LENS  (0x800)
#define  CONFIG_SPACE_LEN      (0x100)

#define ELITE_SATA_COMMAND_BASE (0xd800d1f0) /*Command IO Register Base Address*/
#define ELITE_SATA_CONTROL_BASE (0xd800d3f4) /*Device Control IO Register Base Address*/
#define ELITE_SATA_BMDMA_BASE   (0xd800d400) /*Bus master IO regeister Base Address*/

/*
 * Elite sata controller data structure
 */
struct elite_sata_ctl {
	unsigned int phybase;
	unsigned int physize;
	unsigned int config_len;
	void __iomem    *mmio_base; /* mmio base address */
	int irq;                    /* Interrupt */

	struct device   *dev;        /* Generic device interface */   
	void            *private_data; /* Private item container */
};

/*
 * Data structure from old PCI driver
 * VIA SouthBridge chips & hardware infomations
 */
struct via_isa_bridge {
	const char     *name;
	unsigned int   id;
	unsigned char  rev_min;
	unsigned char  rev_max;
	unsigned char  udma_mask;
	unsigned int   flags;
    
	/* private controller data structure */
	void *private_data;
} via_isa_bridges[] = {
	{ "vt3499",	DEVICE_ID_VIA_ELITE, 0x00, 0x2f, ATA_UDMA6, 
	VIA_BAD_AST | VIA_SATA_PATA | VIA_IDFLAG_SINGLE, NULL},
	{ "vtxxxx",	DEVICE_ID_VIA_ANON, 0x00, 0x2f, ATA_UDMA6, 
	VIA_BAD_AST | VIA_SATA_PATA | VIA_IDFLAG_SINGLE, NULL},
	{ NULL }
};

struct via_port {
	unsigned char cached_device;
};

static inline struct elite_sata_ctl *host_to_elite_clter(struct ata_host *host)
{
	struct via_isa_bridge * tmp;
	tmp = (struct via_isa_bridge *)host->private_data; 

	return (struct elite_sata_ctl *)tmp->private_data;
}
/******************************************/

/* Porting ATA_PCI APIs to low-level driver */

/**
 * IO R/W API by mmio
 */
static inline void via_read_config_byte(struct elite_sata_ctl *sata_ctl,
	int offset, unsigned char *val)
{
	*val = ioread8(sata_ctl->mmio_base + offset);
}

static inline void via_read_config_word(struct elite_sata_ctl *sata_ctl,
	int offset, unsigned short *val)
{
	*val = ioread16(sata_ctl->mmio_base + offset);
}

static inline void via_read_config_dword(struct elite_sata_ctl *sata_ctl,
	int offset, unsigned int *val)
{
	*val = ioread32(sata_ctl->mmio_base + offset);
}

static inline void via_write_config_byte(struct elite_sata_ctl *sata_ctl,
	int offset, unsigned char val)
{
	iowrite8(val, (sata_ctl->mmio_base + offset));
}

static inline void via_write_config_word(struct elite_sata_ctl *sata_ctl,
	int offset, unsigned short val)
{
	iowrite16(val, (sata_ctl->mmio_base + offset));
}

static inline void via_write_config_dword(struct elite_sata_ctl *sata_ctl,
	int offset, unsigned int val)
{
	iowrite32(val, (sata_ctl->mmio_base + offset));
}

static inline void via_controller_base_init(struct elite_sata_ctl *sata_ctl)
{
        via_write_config_byte(sata_ctl,0x4,0x7);/*Bus Master/Memory Space Access/IO Space Access*/
        via_write_config_byte(sata_ctl,0x9,0x8b);/*Select IDE Native Mode*/
        via_write_config_dword(sata_ctl,0x10,ELITE_SATA_COMMAND_BASE);/*Set Command IO Register Base Address*/
        via_write_config_dword(sata_ctl,0x14,ELITE_SATA_CONTROL_BASE);/*Set Device Control IO Register Base Address*/
        via_write_config_dword(sata_ctl,0x20,ELITE_SATA_BMDMA_BASE);/*Set Bus Master IO Register Base Address*/
        via_write_config_byte(sata_ctl,0xb8,0x91);/*OOB Kick off*/
}

static inline void via_hitachi_patch(struct elite_sata_ctl *sata_ctl)
{
        unsigned char val;

        //Rx44[2]=0
        via_read_config_byte(sata_ctl,0x44,&val);
        val &= 0xFB;
        via_write_config_byte(sata_ctl,0x44,val);

        //Rx49[6]=0
        via_read_config_byte(sata_ctl,0x49,&val);
        val &= 0xBF;
        via_write_config_byte(sata_ctl,0x49,val);
}

static const struct ata_port_info *via_sff_find_valid_pi(
	const struct ata_port_info * const *ppi)
{
	int i;

	/* look up the first valid port_info */
	for (i = 0; i < 2 && ppi[i]; i++)
		if (ppi[i]->port_ops != &ata_dummy_port_ops)
			return ppi[i];

	return NULL;
}

/**
 *	via_sff_init_host - acquire native PCI ATA resources and init host
 *	@host: target ATA host
 *	@priv_data: private sata_ctler information
 *
 *	Acquire native PCI ATA resources for @host and initialize the
 *	first two ports of @host accordingly.  Ports marked dummy are
 *	skipped and allocation failure makes the port dummy.
 *
 *	Note that native PCI resources are valid even for legacy hosts
 *	as we fix up pdev resources array early in boot, so this
 *	function can be used for both native and legacy SFF hosts.
 *
 *	LOCKING:
 *	Inherited from calling layer (may sleep).
 *
 *	RETURNS:
 *	0 if at least one port is initialized, -ENODEV if no port is
 *	available.
 */
static int via_sff_init_host(struct ata_host *host, void *priv_data)
{
	struct device *gdev = host->dev;
	struct elite_sata_ctl *ctl = (struct elite_sata_ctl *)priv_data;

	unsigned int mask = 0;
	unsigned int cmd_addr = 0, ctl_addr = 0;
	int i;

	/* init port addresses, only primary channel available on ELITE SOC */
	for (i = 0; i < host->n_ports; i++) {
		struct ata_port *ap = host->ports[i];

		/* secondary channel is dummy */
		if (ata_port_is_dummy(ap))
			continue;

		/* In X86 && PCI config space, base address registers 
		 * MUST end with bit[1:0] (01b) when it's mapped to IO space,
		 * BUT in Elite ARM SOC, we think this rule not working,
		 * so end with bit[1:0] (00b)
		 */
		/* restrict to only 1st channel */
		if (i == 0) {
			/* default cmd_addr 0xD1F1, actually 0x01F0 */
			via_read_config_dword(ctl, REG_COMMAND_BASE_OFFSET, &cmd_addr);
			ap->ioaddr.cmd_addr = 
				(void __iomem *)(ctl->mmio_base + (cmd_addr & 0x0FFE));

			/* Defined in ATA spec, ctl_addr = ctl_base + 2
			 * which ctl_base is always with bit[1:0] (00b)
			 * default ctl_addr 0xD3F5, actually 0x03F6
			 */
			via_read_config_dword(ctl, REG_CONTROL_BASE_OFFSET, &ctl_addr);
			ap->ioaddr.altstatus_addr = 
			ap->ioaddr.ctl_addr = (void __iomem *)
			(ctl->mmio_base + (ctl_addr & 0x0FFE) + ATA_PCI_CTL_OFS );

			ata_sff_std_ports(&ap->ioaddr);
			ata_port_desc(ap, "cmd 0x%x ctl 0x%x", 
			(unsigned int)(ap->ioaddr.cmd_addr), (unsigned int)(ap->ioaddr.ctl_addr));
		}

		mask |= 1 << i;
	}

	if (!mask) {
		dev_printk(KERN_ERR, gdev, "no available native port\n");
		return -ENODEV;
	}

	return 0;
}

/**
 *	via_sff_prepare_host - helper to prepare PCI PIO-only SFF ATA host
 *	@pdev: target PCI device
 *	@ppi: array of port_info, must be enough for two ports
 *	@r_host: out argument for the initialized ATA host
 *	@priv_data: private sata_clter information
 *
 *	Helper to allocate PIO-only SFF ATA host for @pdev, acquire
 *	all PCI resources and initialize it accordingly in one go.
 *
 *	LOCKING:
 *	Inherited from calling layer (may sleep).
 *
 *	RETURNS:
 *	0 on success, -errno otherwise.
 */
static int via_sff_prepare_host(struct platform_device *pdev,
			     const struct ata_port_info * const *ppi,
			     struct ata_host **r_host,  void* priv_data)
{
	struct ata_host *host;
	int rc;

	if (!devres_open_group(&pdev->dev, NULL, GFP_KERNEL))
		return -ENOMEM;

	host = ata_host_alloc_pinfo(&pdev->dev, ppi, 2);
	if (!host) {
		dev_printk(KERN_ERR, &pdev->dev,
			   "failed to allocate ATA host\n");
		rc = -ENOMEM;
		goto err_out;
	}

	rc = via_sff_init_host(host, priv_data);
	if (rc)
		goto err_out;

	devres_remove_group(&pdev->dev, NULL);
	*r_host = host;
	return 0;

err_out:
	devres_release_group(&pdev->dev, NULL);
	return rc;
}


static void via_bmdma_nodma(struct ata_host *host, const char *reason)
{
	int i;

	dev_printk(KERN_ERR, host->dev, "BMDMA: %s, falling back to PIO\n",
		   reason);

	for (i = 0; i < 2; i++) {
		host->ports[i]->mwdma_mask = 0;
		host->ports[i]->udma_mask = 0;
	}
}

/**
 *	via_bmdma_init - acquire PCI BMDMA resources and init ATA host
 *	@host: target ATA host
 *	@priv_data: private sata_clter information
 *
 *	Acquire PCI BMDMA resources and initialize @host accordingly.
 *
 *	LOCKING:
 *	Inherited from calling layer (may sleep).
 */
static void via_bmdma_init(struct ata_host *host, void *priv_data)
{
	struct device *gdev = host->dev;
	struct elite_sata_ctl *sata_ctl;
	int i, rc;
	unsigned int bmdma;

	sata_ctl = (struct elite_sata_ctl *)priv_data;

	rc = dma_set_mask(gdev, ATA_DMA_MASK);
	if (rc)
		via_bmdma_nodma(host, "failed to set dma mask");
	if (!rc) {
		rc = dma_set_coherent_mask(gdev, ATA_DMA_MASK);
		if (rc)
			via_bmdma_nodma(host,
					"failed to set consistent dma mask");
	}
    
	for (i = 0; i < 2; i++) {
		struct ata_port *ap = host->ports[i];
		if (ata_port_is_dummy(ap))
			continue;

		/* 
		 * init bus master registers, restrict to only 1st channel
		 */
		if (i == 0)
		{
			via_read_config_dword(sata_ctl, REG_BMDMA_BASE_OFFSET, &bmdma);
			ap->ioaddr.bmdma_addr = 
				(void __iomem *)(sata_ctl->mmio_base + (bmdma & 0x0FFE));
		}

		if ((!(ap->flags & ATA_FLAG_IGN_SIMPLEX)) &&
			(ioread8(ap->ioaddr.bmdma_addr + 2) & 0x80))
			host->flags |= ATA_HOST_SIMPLEX;

		ata_port_desc(ap, "bmdma 0x%x", (unsigned int)(ap->ioaddr.bmdma_addr));
	}

	return;
}

/**
 *	via_bmdma_prepare_host - helper to prepare PCI BMDMA ATA host
 *	@pdev: target PCI device
 *	@ppi: array of port_info, must be enough for two ports
 *	@r_host: out argument for the initialized ATA host
 * 	@priv_data: private sata_clter infomation
 *
 *	Helper to allocate BMDMA ATA host for @pdev, acquire all PCI
 *	resources and initialize it accordingly in one go.
 *
 *	LOCKING:
 *	Inherited from calling layer (may sleep).
 *
 *	RETURNS:
 *	0 on success, -errno otherwise.
 */
static int via_bmdma_prepare_host(struct platform_device *pdev,
				const struct ata_port_info * const * ppi,
				struct ata_host **r_host, void *priv_data)
{
	int rc;

	rc = via_sff_prepare_host(pdev, ppi, r_host, priv_data);
	if (rc)
		return rc;

	if(!priv_data)
	{
		printk(KERN_ERR "No sata_clter infomation\n");
		return -EINVAL;
	}

	via_bmdma_init(*r_host, priv_data);
	return 0;
}

/**
 *	via_elite_bmdma_init_one - Initialize/register BMDMA PCI IDE controller
 *	@pdev: Controller to be initialized
 *	@ppi: array of port_info, must be enough for two ports
 *	@sht: scsi_host_template to use when registering the host
 *	@host_priv: some board & hardware information, which is holding 
 *              private sata controller data structure
 *	@hflags: host flags
 *
 *	This function is similar to ata_pci_sff_init_one() but also
 *	takes care of BMDMA initialization.
 *
 *	LOCKING:
 *	Inherited from PCI layer (may sleep).
 *
 *	RETURNS:
 *	Zero on success, negative on errno-based value on error.
 */
static int via_bmdma_init_one(struct platform_device *pdev,
			const struct ata_port_info * const * ppi,
			struct scsi_host_template *sht, void *host_priv,
			int hflags)
{
	struct device *dev = &pdev->dev;
	struct elite_sata_ctl *sata_ctl;
	const struct ata_port_info *pi;
	struct ata_host *host = NULL;
	int rc;

	DPRINTK("ENTER\n");

	if(!host_priv)
	{
		printk(KERN_ERR "No valid sata_ctler info, host_priv null\n");
		return -EINVAL;
	}
	else
		sata_ctl = (struct elite_sata_ctl *)
			(((struct via_isa_bridge *)host_priv)->private_data);

	if(!sata_ctl)
	{
		printk(KERN_ERR "No valid sata_ctler info, sata_ctl null\n");
		return -EINVAL;
	}

	pi = via_sff_find_valid_pi(ppi);
	if (!pi) {
		dev_printk(KERN_ERR, &pdev->dev,
			   "no valid port_info specified\n");
		return -EINVAL;
	}

	if (!devres_open_group(dev, NULL, GFP_KERNEL))
		return -ENOMEM;

	/* prepare and activate BMDMA host */
	rc = via_bmdma_prepare_host(pdev, ppi, &host, (void*)sata_ctl);
	if (rc)
		goto out;

	/*
	 * In host_priv, we have "struct via_isa_bridge"
	 * In host_priv->private_data, we have "struct elite_sata_clter"
	 */
	host->private_data = host_priv;
	host->flags |= hflags;

	/**
	 * As for native mode only, use default API instead of 
	 * "ata_pci_sff_activate_host", use NULL instead of "IRQF_SHARED".
	 */
	rc = ata_host_activate(host, sata_ctl->irq, ata_bmdma_interrupt, 
		IRQF_TRIGGER_NONE, sht);
	sata_ctl->private_data = (void*)host;
out:
	if (rc == 0)
		devres_remove_group(&pdev->dev, NULL);
	else
		devres_release_group(&pdev->dev, NULL);

	return rc;
}

/**
 *	via_config_fifo		-	set up the FIFO
 *
 *	Set the FIFO properties for this device if necessary. Used both on
 *	set up and on and the resume path
 */
static void via_config_fifo(struct platform_device *pdev, unsigned int flags)
{
	return;
}

/**
 *	via_cable_detect	-	cable detection
 *	@ap: ATA port
 *
 *	Perform cable detection. Actually for the VIA case the BIOS
 *	already did this for us. We read the values provided by the
 *	BIOS. If you are using an 8235 in a non-PC configuration you
 *	may need to update this code.
 *
 *	Hotplug also impacts on this.
 */

static int via_cable_detect(struct ata_port *ap) {
	const struct via_isa_bridge *config = ap->host->private_data;

	if ((config->flags & VIA_SATA_PATA) && ap->port_no == 0)
		return ATA_CBL_SATA;
    
	//by-default
	return ATA_CBL_PATA40;
}

static int via_pre_reset(struct ata_link *link, unsigned long deadline)
{
	struct ata_port *ap = link->ap;
	struct elite_sata_ctl *sata_ctl = host_to_elite_clter(ap->host);
	const struct via_isa_bridge *config = ap->host->private_data;
	unsigned char val, mask = 0;

	if (!(config->flags & VIA_NO_ENABLES)) {
	/* Only 1st channel available, Just test primary channel bit1
	 * in SATA Channel Enable register
	 */
		mask = 1<<1;
		via_read_config_byte(sata_ctl, 0x40, &val);
		if(!(val & mask)){
			printk("Primary Channel IO Access Disable\n");
			return -ENONET;
		}
	}

	return ata_sff_prereset(link, deadline);
}


/**
 *	via_do_set_mode	-	set transfer mode data
 *	@ap: ATA interface
 *	@adev: ATA device
 *	@mode: ATA mode being programmed
 *	@set_ast: Set to program address setup
 *	@udma_type: UDMA mode/format of registers
 *
 *	Program the VIA registers for DMA and PIO modes. Uses the ata timing
 *	support in order to compute modes.
 *
 *	FIXME: Hotplug will require we serialize multiple mode changes
 *	on the two channels.
 */

static void via_do_set_mode(struct ata_port *ap, struct ata_device *adev,
			    int mode, int set_ast, int udma_type)
{
	return;
    
	/* Not useful for Elite SOC or VT3456/VT3410 chipset */
#if 0
	struct ata_device *peer = ata_dev_pair(adev);
	struct ata_timing t, p;
	static int via_clock = 33333;	/* Bus clock in kHZ */
	unsigned long T =  1000000000 / via_clock;
	unsigned long UT = T;
	int ut;
	int offset = 3 - (2*ap->port_no) - adev->devno;

	switch (udma_type) {
	case ATA_UDMA4:
		UT = T / 2; break;
	case ATA_UDMA5:
		UT = T / 3; break;
	case ATA_UDMA6:
		UT = T / 4; break;
	}

	/* Calculate the timing values we require */
	ata_timing_compute(adev, mode, &t, T, UT);

	/* We share 8bit timing so we must merge the constraints */
	if (peer) {
		if (peer->pio_mode) {
			ata_timing_compute(peer, peer->pio_mode, &p, T, UT);
			ata_timing_merge(&p, &t, &t, ATA_TIMING_8BIT);
		}
	}

	/* Address setup is programmable but breaks on UDMA133 setups */
	if (set_ast) {
		u8 setup;	/* 2 bits per drive */
		int shift = 2 * offset;

		pci_read_config_byte(pdev, 0x4C, &setup);
		setup &= ~(3 << shift);
		setup |= (clamp_val(t.setup, 1, 4) - 1) << shift;
		pci_write_config_byte(pdev, 0x4C, setup);
	}

	/* Load the PIO mode bits */
	pci_write_config_byte(pdev, 0x4F - ap->port_no,
		((clamp_val(t.act8b, 1, 16) - 1) << 4) | (clamp_val(t.rec8b, 1, 16) - 1));
	pci_write_config_byte(pdev, 0x48 + offset,
		((clamp_val(t.active, 1, 16) - 1) << 4) | (clamp_val(t.recover, 1, 16) - 1));

	/* Load the UDMA bits according to type */
	switch (udma_type) {
	case ATA_UDMA2:
	default:
		ut = t.udma ? (0xe0 | (clamp_val(t.udma, 2, 5) - 2)) : 0x03;
		break;
	case ATA_UDMA4:
		ut = t.udma ? (0xe8 | (clamp_val(t.udma, 2, 9) - 2)) : 0x0f;
		break;
	case ATA_UDMA5:
		ut = t.udma ? (0xe0 | (clamp_val(t.udma, 2, 9) - 2)) : 0x07;
		break;
	case ATA_UDMA6:
		ut = t.udma ? (0xe0 | (clamp_val(t.udma, 2, 9) - 2)) : 0x07;
		break;
	}

	/* Set UDMA unless device is not UDMA capable */
	if (udma_type) {
		u8 udma_etc;

		pci_read_config_byte(pdev, 0x50 + offset, &udma_etc);

		/* clear transfer mode bit */
		udma_etc &= ~0x20;

		if (t.udma) {
			/* preserve 80-wire cable detection bit */
			udma_etc &= 0x10;
			udma_etc |= ut;
		}

		pci_write_config_byte(pdev, 0x50 + offset, udma_etc);
	}
#endif

}

static void via_set_piomode(struct ata_port *ap, struct ata_device *adev)
{
	const struct via_isa_bridge *config = ap->host->private_data;
	int set_ast = (config->flags & VIA_BAD_AST) ? 0 : 1;

	via_do_set_mode(ap, adev, adev->pio_mode, set_ast, config->udma_mask);
}

static void via_set_dmamode(struct ata_port *ap, struct ata_device *adev)
{
	const struct via_isa_bridge *config = ap->host->private_data;
	int set_ast = (config->flags & VIA_BAD_AST) ? 0 : 1;

	via_do_set_mode(ap, adev, adev->dma_mode, set_ast, config->udma_mask);
}

/**
 *	via_mode_filter		-	filter buggy device/mode pairs
 *	@dev: ATA device
 *	@mask: Mode bitmask
 *
 *	We need to apply some minimal filtering for old controllers and at least
 *	one breed of Transcend SSD. Return the updated mask.
 *
 *	We only support ELITE sata controller(VT3456) for now
 */

static unsigned long via_mode_filter(struct ata_device *dev, unsigned long mask)
{
/* Not useful for ELITE SOC */
#if 0
	struct ata_host *host = dev->link->ap->host;
	const struct via_isa_bridge *config = host->private_data;
	unsigned char model_num[ATA_ID_PROD_LEN + 1];

	if (config->id == PCI_DEVICE_ID_VIA_82C586_0) {
		ata_id_c_string(dev->id, model_num, ATA_ID_PROD, sizeof(model_num));
		if (strcmp(model_num, "TS64GSSD25-M") == 0) {
			ata_dev_printk(dev, KERN_WARNING,
	"disabling UDMA mode due to reported lockups with this device.\n");
			mask &= ~ ATA_MASK_UDMA;
		}
	}

	if (dev->class == ATA_DEV_ATAPI &&
	    dmi_check_system(no_atapi_dma_dmi_table)) {
		ata_dev_printk(dev, KERN_WARNING, "controller locks up on ATAPI DMA, forcing PIO\n");
		mask &= ATA_MASK_PIO;
	}
#endif

	return mask;
}

/**
 *	via_tf_load - send taskfile registers to host controller
 *	@ap: Port to which output is sent
 *	@tf: ATA taskfile register set
 *
 *	Outputs ATA taskfile to standard ATA host controller.
 *
 *	Note: This is to fix the internal bug of via chipsets, which
 *	will reset the device register after changing the IEN bit on
 *	ctl register
 */
static void via_tf_load(struct ata_port *ap, const struct ata_taskfile *tf)
{
	struct ata_ioports *ioaddr = &ap->ioaddr;
	struct via_port *vp = ap->private_data;
	unsigned int is_addr = tf->flags & ATA_TFLAG_ISADDR;
	int newctl = 0;

	if (tf->ctl != ap->last_ctl) {
		iowrite8(tf->ctl, ioaddr->ctl_addr);
		ap->last_ctl = tf->ctl;
		ata_wait_idle(ap);
		newctl = 1;
	}

	if (tf->flags & ATA_TFLAG_DEVICE) {
		iowrite8(tf->device, ioaddr->device_addr);
		vp->cached_device = tf->device;
	} else if (newctl)
		iowrite8(vp->cached_device, ioaddr->device_addr);

	if (is_addr && (tf->flags & ATA_TFLAG_LBA48)) {
		WARN_ON_ONCE(!ioaddr->ctl_addr);
		iowrite8(tf->hob_feature, ioaddr->feature_addr);
		iowrite8(tf->hob_nsect, ioaddr->nsect_addr);
		iowrite8(tf->hob_lbal, ioaddr->lbal_addr);
		iowrite8(tf->hob_lbam, ioaddr->lbam_addr);
		iowrite8(tf->hob_lbah, ioaddr->lbah_addr);
		VPRINTK("hob: feat 0x%X nsect 0x%X, lba 0x%X 0x%X 0x%X\n",
			tf->hob_feature,
			tf->hob_nsect,
			tf->hob_lbal,
			tf->hob_lbam,
			tf->hob_lbah);
	}

	if (is_addr) {
		iowrite8(tf->feature, ioaddr->feature_addr);
		iowrite8(tf->nsect, ioaddr->nsect_addr);
		iowrite8(tf->lbal, ioaddr->lbal_addr);
		iowrite8(tf->lbam, ioaddr->lbam_addr);
		iowrite8(tf->lbah, ioaddr->lbah_addr);
		VPRINTK("feat 0x%X nsect 0x%X lba 0x%X 0x%X 0x%X\n",
			tf->feature,
			tf->nsect,
			tf->lbal,
			tf->lbam,
			tf->lbah);
	}

	ata_wait_idle(ap);
}

static int via_port_start(struct ata_port *ap)
{
	struct via_port *vp;
	struct elite_sata_ctl *sata_ctl = host_to_elite_clter(ap->host);

	int ret = ata_bmdma_port_start(ap);
	if (ret < 0)
		return ret;

	vp = (struct via_port*)devm_kzalloc(sata_ctl->dev, sizeof(struct via_port), GFP_KERNEL);
	if (vp == NULL)
		return -ENOMEM;
	ap->private_data = vp;

	return 0;
}

static struct scsi_host_template via_sht = {
	ATA_BMDMA_SHT(DRV_NAME),
};

static struct ata_port_operations via_port_ops = {
	.inherits       = &ata_bmdma_port_ops,
	.cable_detect   = via_cable_detect,
	.set_piomode    = via_set_piomode,
	.set_dmamode    = via_set_dmamode,
	.prereset       = via_pre_reset,
	.sff_tf_load    = via_tf_load,
	.port_start	= via_port_start,
	.mode_filter    = via_mode_filter,
};

static struct ata_port_operations via_port_ops_noirq = {
	.inherits       = &via_port_ops,
	.sff_data_xfer  = ata_sff_data_xfer_noirq,
};


/**
 * Platform Driver APIs
 */
 
#ifdef CONFIG_PM
/**
 *	PM suspend/resume API
 */
static void via_controller_reinit(struct elite_sata_ctl* sata_ctl)
{
	via_controller_base_init(sata_ctl);
	via_hitachi_patch(sata_ctl);
}

static int via_elite_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ata_host *host = dev_get_drvdata(&pdev->dev);

	return ata_host_suspend(host, state);
}

static int via_elite_resume(struct platform_device *pdev)
{
	struct ata_host *host = dev_get_drvdata(&pdev->dev);
	struct elite_sata_ctl *sata_ctl = host_to_elite_clter(host);

	via_controller_reinit(sata_ctl);
	ata_host_resume(host);

	return 0;
}
#endif

/**
 *	via_elite_remove
 */
static int via_elite_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ata_host *host = dev_get_drvdata(dev);
	struct elite_sata_ctl *sata_ctl = host_to_elite_clter(host);
	struct resource *res = NULL;

	if(sata_ctl == NULL) {
		printk("via_elite_remove failed: sata_ctl NULL\n");
		return -1;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res != NULL) {
		release_mem_region(res->start, resource_size(res));
	}

	if(sata_ctl->mmio_base)
		iounmap(sata_ctl->mmio_base);
    
	if (sata_ctl->irq)
		devm_free_irq(dev, sata_ctl->irq, host);

	ata_host_detach(host);

	kfree(sata_ctl);

	return 0;
}

/**
 *	via_elite_probe
 *
 */
static int via_elite_probe(struct platform_device *pdev)
{
	/* Early VIA without UDMA support */
	static const struct ata_port_info via_mwdma_info = {
		.flags = ATA_FLAG_SLAVE_POSS,
		.pio_mask = ATA_PIO4,
		.mwdma_mask = ATA_MWDMA2,
		.port_ops = &via_port_ops
	};
	/* Ditto with IRQ masking required */
	static const struct ata_port_info via_mwdma_info_borked = {
		.flags = ATA_FLAG_SLAVE_POSS,
		.pio_mask = ATA_PIO4,
		.mwdma_mask = ATA_MWDMA2,
		.port_ops = &via_port_ops_noirq,
	};
	/* VIA UDMA 33 devices (and borked 66) */
	static const struct ata_port_info via_udma33_info = {
		.flags = ATA_FLAG_SLAVE_POSS,
		.pio_mask = ATA_PIO4,
		.mwdma_mask = ATA_MWDMA2,
		.udma_mask = ATA_UDMA2,
		.port_ops = &via_port_ops
	};
	/* VIA UDMA 66 devices */
	static const struct ata_port_info via_udma66_info = {
		.flags = ATA_FLAG_SLAVE_POSS,
		.pio_mask = ATA_PIO4,
		.mwdma_mask = ATA_MWDMA2,
		.udma_mask = ATA_UDMA4,
		.port_ops = &via_port_ops
	};
	/* VIA UDMA 100 devices */
	static const struct ata_port_info via_udma100_info = {
		.flags = ATA_FLAG_SLAVE_POSS,
		.pio_mask = ATA_PIO4,
		.mwdma_mask = ATA_MWDMA2,
		.udma_mask = ATA_UDMA5,
		.port_ops = &via_port_ops
	};
	/* UDMA133 with bad AST (All current 133) */
	static const struct ata_port_info via_udma133_info = {
		.flags = ATA_FLAG_SLAVE_POSS,
		.pio_mask = ATA_PIO4,
		.mwdma_mask = ATA_MWDMA2,
		.udma_mask = ATA_UDMA6,
		.port_ops = &via_port_ops
	};

	struct elite_sata_ctl *sata_ctl = NULL;
	struct resource *res = NULL;
	struct ata_host *host;
	const struct ata_port_info *ppi[] = { NULL, NULL };
	struct via_isa_bridge *config;

	unsigned char  enable;
	unsigned int   timing;
	int err = 0, irq;
	unsigned short dev_id = 0;

	sata_ctl = (struct elite_sata_ctl*)kzalloc(sizeof(struct elite_sata_ctl), GFP_KERNEL);
	if (!sata_ctl) {
		printk(KERN_ERR "Cannot allocate ctl structure\n");
		err = -ENOMEM;
		goto fail;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_ERR "Cannot get iomem resource\n");
		err = -ENXIO;
		goto fail;
	}

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (res == NULL) {
		printk(KERN_ERR "Req mem region failed\n");
		err = -EBUSY;
		goto fail;
	}

	sata_ctl->mmio_base = ioremap(res->start, resource_size(res));
	if (sata_ctl->mmio_base == NULL) {
		printk(KERN_ERR "IO map failed\n");
		err = -ENXIO;
		goto fail;
	}
	sata_ctl->phybase = ELITE_SATA_PHYS_BASE;
	sata_ctl->physize = ELITE_SATA_PHYS_LENS;
	sata_ctl->config_len = CONFIG_SPACE_LEN;
	sata_ctl->dev = &pdev->dev;

	/* Get irq resource, request irq in ata_host_activate() */
	/* resource index that of this type in resource array */
	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		printk(KERN_ERR "Req irq resource failed\n");
		err = -EBUSY;
		goto fail;
	}
	sata_ctl->irq = irq;

	/*
	 * search for proper hardware info, add sata_ctl as private_data
	 */
	via_read_config_word(sata_ctl, REG_DEVICE_ID_OFFSET, &dev_id);
	for (config = via_isa_bridges; config->id != DEVICE_ID_VIA_ANON;
		config++)
	{
		/*
		 * For ELITE SOC only, DEVICE ID "0x9001"
		 */
		if(config->id == dev_id)
			break;
	}

	/* if single channel set 2nd to dummy */
	if(config->flags & VIA_IDFLAG_SINGLE)
		ppi[1] = &ata_dummy_port_info;

	config->private_data = (void*)sata_ctl;

	if (!(config->flags & VIA_NO_ENABLES)) {
		/* 0x40 low bits indicate enabled channels */
		via_read_config_byte(sata_ctl, 0x40 , &enable);
		enable &= 3;
		if (enable == 0) {
			err = -ENODEV;
			printk(KERN_ERR "SATA CTL NOT ENABLED\n");
            
			goto fail;
		}
	}

	/* Initialize the FIFO for the enabled channels. */
	//via_config_fifo(pdev, config->flags);

	/* Clock set up */
	switch (config->udma_mask) {
	case 0x00:
		if (config->flags & VIA_NO_UNMASK)
			ppi[0] = &via_mwdma_info_borked;
		else
			ppi[0] = &via_mwdma_info;
		break;
	case ATA_UDMA2:
		ppi[0] = &via_udma33_info;
		break;
	case ATA_UDMA4:
		ppi[0] = &via_udma66_info;
		break;
	case ATA_UDMA5:
		ppi[0] = &via_udma100_info;
		break;
	case ATA_UDMA6:
		ppi[0] = &via_udma133_info;
		break;
	default:
		WARN_ON(1);
		err = -ENODEV;
		goto fail;
 	}

	if (config->flags & VIA_BAD_CLK66) {
		/* Disable the 66MHz clock on problem devices */
		via_read_config_dword(sata_ctl, 0x50, &timing);
		timing &= ~0x80008;
		via_write_config_dword(sata_ctl, 0x50, timing);
	}
	
	via_controller_base_init(sata_ctl);
	via_hitachi_patch(sata_ctl);
	err = via_bmdma_init_one(pdev, ppi, &via_sht, (void *)config, 0);
	if (err) {
		printk(KERN_ERR "via_bmdma_init_one FAILED!");
		goto fail;
	}
        
	host = platform_get_drvdata(pdev);
	return 0;

fail:
	if(sata_ctl)
	{
		if(sata_ctl->mmio_base)
			iounmap(sata_ctl->mmio_base);
	}

	if(res)
		release_mem_region(res->start, resource_size(res));

	if(sata_ctl)
		kfree(sata_ctl);

	return err;
}

#ifdef CONFIG_OF
static const struct of_device_id elite_sata_match[] = {
	{ .compatible = "s3graphics,elite1000-sata" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, elite_sata_match);
#endif

static struct platform_device_id elite_sata_driver_ids[] = {
	{
		.name  = "elite-sata",
		.driver_data	= (kernel_ulong_t)NULL,
	}, {
		.name  = "elite-sata.0",
		.driver_data	= (kernel_ulong_t)NULL,
	}, { /* sentinel */ },
};


static struct platform_driver elite_sata_driver = {
	.probe   = via_elite_probe,
	.remove  = via_elite_remove,
	.suspend = via_elite_suspend,
	.resume  = via_elite_resume,
	.id_table  = elite_sata_driver_ids,
	.driver  = {
		.name = "s3graphics-elite-sata",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(elite_sata_match),
	},
};

static int __init via_elite_init(void)
{
	int ret;

	/* register platform device & driver */
	ret = platform_driver_register(&elite_sata_driver);
	if(ret)
		printk(KERN_ERR "ELITE SATA platform drv register failed, ret:%d\n",ret);

	return 0;
}

static void __exit via_elite_exit(void)
{
	platform_driver_unregister(&elite_sata_driver);

	return;
}

MODULE_AUTHOR("VIA technology");
MODULE_DESCRIPTION("driver for VIA Elite SOC SATA");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(via_elite_init);
module_exit(via_elite_exit);

