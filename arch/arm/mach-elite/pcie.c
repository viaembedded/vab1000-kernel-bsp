/*
 *  linux/arch/arm/mach_elite/pci.c
 *
 *	This program is free software: you can redistribute it and/or modify it under the
 *	terms of the GNU General Public License as published by the Free Software Foundation,
 *	either version 2 of the License, or (at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful, but WITHOUT
 *	ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *	PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *	You should have received a copy of the GNU General Public License along with
 *	this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/ptrace.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/ioport.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <asm/irq.h>
#include <asm/system.h>

#include <asm/mach/pci.h>

#include "pcie.h"

#define	HDMI_AUDIO_DEV 0
#define HDMI_BUS_NUM 3
#define PCI_SPACE_SIZE 256

static unsigned char audio_pci_cfg_shadow[PCI_SPACE_SIZE];

struct pcie_port {
	u8			index;
	u8			root_bus_nr;
	void __iomem		*base;
	char			mem_space_name[16];
	char			io_space_name[16];
	struct resource		res[2];
	int 			header_index;
	int 			enumerated;
	int 			scan_finished;
	unsigned short  regs[16];
};

static struct pcie_port pcie_port[3];
static int num_pcie_ports = 0;
static struct resource pcie_io_space;
static struct resource pcie_mem_space;

static int read_hdmi_audio_config(int where, int size, u32 *value)
{

	if (where >= PCI_SPACE_SIZE) {
		printk("err: read_hdmi_audio_config!\n");
	} 

	switch (size) {
	case 1:
		*value = audio_pci_cfg_shadow[where];
		break;
	case 2:
		*value = *((unsigned short*)(audio_pci_cfg_shadow + (where & ~1)));
		break;
	case 4:
		*value = *((u32*)(audio_pci_cfg_shadow + (where & ~3)));
		break;

	}

	return PCIBIOS_SUCCESSFUL;
}

static int write_hdmi_audio_config(int where, int size, u32 value)
{
	if (where >= PCI_SPACE_SIZE) {
		printk("err: write_hdmi_audio_config!\n");
	}
#if 0
	switch (size) {
	case 1:
		audio_pci_cfg_shadow[where] = value;
		break;
	case 2:
		*((unsigned short*)(audio_pci_cfg_shadow + (where & ~1))) = value;
		break;
	case 4:
		*((u32*)(audio_pci_cfg_shadow + (where & ~3))) = value;
		break;
	}
#endif
	return PCIBIOS_SUCCESSFUL;
}

static void __init elite_pcie_preinit(void)
{
	int i;
	u32 size_each_mem;
	u32 size_each_io;
	u32 start_mem;
	u32 start_io;

	pcie_io_space.name = "PCIe I/O Space";
	pcie_io_space.start = ELITE_PCIE_IO_PHYS_BASE(0);
	pcie_io_space.end =
		ELITE_PCIE_IO_PHYS_BASE(0) + ELITE_PCIE_IO_PHYS_SIZE * 3 - 1;
	pcie_io_space.flags = IORESOURCE_IO;
	if (request_resource(&iomem_resource, &pcie_io_space))
		panic("can't allocate PCIe I/O space");

	pcie_mem_space.name = "PCIe MEM Space";
	pcie_mem_space.start = ELITE_PCIE_MEM_PHYS_BASE;
	pcie_mem_space.end =
		ELITE_PCIE_MEM_PHYS_BASE + ELITE_PCIE_MEM_SIZE - 1;
	pcie_mem_space.flags = IORESOURCE_MEM;
	if (request_resource(&iomem_resource, &pcie_mem_space))
		panic("can't allocate PCIe MEM space");

	switch (num_pcie_ports) {
	case 0:
		size_each_mem = 0;
		size_each_io = 0;
		break;

	case 1:
		size_each_mem = SZ_255M;
		size_each_io = SZ_64K;
		break;

	case 2:
		size_each_mem = SZ_116M; 
		size_each_io = SZ_32K;
		break;

	case 3:
		size_each_mem = SZ_77M;
		size_each_io = SZ_21K;
		break;

	default:
		panic("invalid number of PCIe ports");
	}

	start_mem = ELITE_PCIE_MEM_PHYS_BASE;
	start_io =  PCIBIOS_MIN_IO;

	for (i = 0; i < num_pcie_ports; i++) {
		struct pcie_port *pp = pcie_port + i;

		snprintf(pp->io_space_name, sizeof(pp->io_space_name),
			"PCIe %d I/O", pp->index);
		pp->io_space_name[sizeof(pp->io_space_name) - 1] = 0;
		pp->res[0].name = pp->io_space_name;
		pp->res[0].start = start_io;
		pp->res[0].end = start_io + size_each_io - 1;
		pp->res[0].flags = IORESOURCE_IO;

		snprintf(pp->mem_space_name, sizeof(pp->mem_space_name),
			"PCIe %d MEM", pp->index);
		pp->mem_space_name[sizeof(pp->mem_space_name) - 1] = 0;
		pp->res[1].name = pp->mem_space_name;
		pp->res[1].flags = IORESOURCE_MEM;
		pp->res[1].start = start_mem;
		pp->res[1].end = start_mem + size_each_mem - 1;

		start_mem += size_each_mem;
		start_io += size_each_io;
	}
}


static void __init elite_pcie_postinit(void)
{
	int i;

	for (i = 0; i < num_pcie_ports; i++) {
		struct pcie_port *pp = pcie_port + i;
	/* 0x300 is the special register in the root port that is designed by HW to solve the 
	* problem of 0xC0000000~0xCFFFFFFF I/O space address range while elite-1K only support 64k
	* I/O address space. The value set in this register defines the upper 16bits of the I/O space 
	* of this root port. Combied with the I/O address in the EP as the lower 16bits under
	* this port, 32bits I/O address can be achieved.
	*/
		if (pp->index == 0) {
			writew(0xcfff, pp->base + 0x300);
		}else{
			writew(0xcfff, pp->base + 0x300);

		}
	}
}
static int __init elite_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct pcie_port *pp;

	if (nr >= num_pcie_ports)
		return 0;

	pp = &pcie_port[nr];
	pp->root_bus_nr = sys->busnr;
	
	pci_add_resource_offset(&sys->resources, &pp->res[0], sys->io_offset);
	pci_add_resource_offset(&sys->resources, &pp->res[1], sys->mem_offset);

	return 1;
}

static struct pcie_port *bus_to_port(int bus)
{
	int i;
	
	for (i = num_pcie_ports - 1; i >= 0; i--) {
		int rbus = pcie_port[i].root_bus_nr;
		if (rbus != -1 && rbus <= bus)
			break;
	}

	return i >= 0 ? pcie_port + i : NULL;
}


static bool dev_id_found(struct pci_bus *bus, unsigned int devfn)
{
	int i,j;
	u32 dev_id;
	void __iomem *dev_id_offset;
	dev_id = (devfn << 8)|(bus->number );

	for (i = 0; i < num_pcie_ports; i++) {
		if (pcie_port[i].enumerated == 0)
			return false;
			 
		dev_id_offset = pcie_port[i].base + BUS_DEV_FUNC_ID_SECTION_OFFSET;
		for(j = 0;j < 16; j++){
			if(dev_id == readw(dev_id_offset + j*2))
			{
				return true;
			}
		}	
	}
		return false;
}

static void __iomem* get_dev_cfg_base_addr(struct pci_bus *bus, unsigned int devfn)
{
	int i;
	int j = 0;
	u32 dev_id;
	void __iomem *dev_id_offset;

	dev_id = (devfn << 8)|(bus->number );

	for (i = 0; i < num_pcie_ports; i++) {
		dev_id_offset = pcie_port[i].base + BUS_DEV_FUNC_ID_SECTION_OFFSET;
		for(j = 0;j < 16; j++){
			if(dev_id == readw(dev_id_offset + j * 2))
				break;
		}
		if(j < 16) break;
	}
	if(i < num_pcie_ports && j < 16)
		return pcie_port[i].base + j * DEVICE_CFG_SIZE;
	else
		return 0;
}


static int
elite_pcie_read_config(struct pci_bus *bus, unsigned int devfn, int where, int size,
		  u32 * value)
{
	int i;
	bool found = false;
	u32 dev_id;
	void __iomem* dev_cfg_base_addr;
	struct pcie_port *pp;

	dev_id = (devfn << 8)|(bus->number );

	if (dev_id == ((HDMI_AUDIO_DEV<<8) | HDMI_BUS_NUM) ) {
		return read_hdmi_audio_config(where, size, value);
	}

	found = dev_id_found(bus,devfn);
	if(found){
		dev_cfg_base_addr = get_dev_cfg_base_addr(bus,devfn);

		switch (size) {
		case 1:
			*value = readb((dev_cfg_base_addr + where));
			break;
		case 2:
			*value = readw((dev_cfg_base_addr + (where & ~1)));
			break;
		case 4:
			*value = readl((dev_cfg_base_addr + (where & ~3)));
			break;
		}
	} else {//setup the device id table in the root port cfg space

		for (i = 0; i < num_pcie_ports; i++) {
			pp = pcie_port + i ;			

			if(pp->scan_finished == 1)
				continue;

			if(pp->header_index == 16)
				continue;

			writew(dev_id, pp->base + BUS_DEV_FUNC_ID_SECTION_OFFSET + (pp->header_index)*2);
			*value = readl(pp->base + (pp->header_index) * DEVICE_CFG_SIZE + PCI_VENDOR_ID);

			 if (*value == 0xffff0001){
				writew(0, pp->base + BUS_DEV_FUNC_ID_SECTION_OFFSET + (pp->header_index) *2);
				return PCIBIOS_SUCCESSFUL;
			  } else if (*value != 0xffffffff && *value != 0x00000000 && 
			            *value != 0x0000ffff && *value != 0xffff0000 ){
				pp->header_index++;
				pp->enumerated = 1;
				break;
			} else {
				writew(0, pp->base + BUS_DEV_FUNC_ID_SECTION_OFFSET + (pp->header_index) * 2);
				return PCIBIOS_DEVICE_NOT_FOUND;
			}		
		}
	} 

	return PCIBIOS_SUCCESSFUL;
}

static int
elite_pcie_write_config(struct pci_bus *bus, unsigned int devfn, int where, int size,
		   u32 value)
{
	void __iomem* dev_cfg_base_addr;
	u32 dev_id;

	dev_id = (devfn << 8)|(bus->number );
	dev_cfg_base_addr = get_dev_cfg_base_addr(bus,devfn);

	if (dev_id == ((HDMI_AUDIO_DEV<<8)|HDMI_BUS_NUM)) {
		return write_hdmi_audio_config(where, size, value);
	}

	switch (size) {
	case 1:
		writeb(value, dev_cfg_base_addr + where);
		break;
	case 2:
		writew(value, dev_cfg_base_addr + (where & ~1));
		break;
	case 4:
		writel(value, dev_cfg_base_addr + (where & ~3));
		break;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int __init elite_pcie_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	struct pcie_port *pp = bus_to_port(dev->bus->number);

	if(pp->index == 0)
		return IRQ_ELITE_PCIE_PORT0;
	else if(pp->index == 1)
		return IRQ_ELITE_PCIE_PORT1;
	else if(pp->index == 2)
		return IRQ_ELITE_PCIE_PORT2;
        else
                return IRQ_GFX_INTB;
}

static struct pci_ops elite_pcie_ops = {
	.read = elite_pcie_read_config,
	.write = elite_pcie_write_config,
};

static struct pci_bus __init*
elite_pcie_scan_bus(int nr, struct pci_sys_data *sys)
{
	struct pci_bus *bus;

	if (nr < num_pcie_ports) {
		bus = pci_scan_root_bus(NULL, sys->busnr, &elite_pcie_ops, sys,
					&sys->resources);
		pcie_port[nr].scan_finished = 1;
		
	} else {
		bus = NULL;
		BUG();
	}

	return bus;
}

static struct hw_pci elite_pcie __initdata = {
	.setup = elite_pcie_setup,
	.swizzle = pci_std_swizzle,
	.map_irq = elite_pcie_map_irq,
	.scan = elite_pcie_scan_bus,
	.preinit = elite_pcie_preinit,
	.postinit = elite_pcie_postinit,
};

static void __init add_pcie_port(int index, unsigned long base)
{
	u32 value;
	struct pcie_port *pp = &pcie_port[num_pcie_ports++];
	printk(KERN_INFO "Elite PCIe port %d: ", index);
	pp->index = index;
	pp->header_index = 0;
	pp->enumerated = 0;
 	pp->scan_finished = 0;
	pp->root_bus_nr = -1;
	pp->base = (void __iomem*)ioremap_nocache(base, ELITE_PCIE_PORTS_SIZE);
//	printk("pp->base %d    0x%x\n",index,pp->base);

	value = readl(pp->base+0x250);
	value &= ~(1 << 2);
	writel(value, pp->base+ 0x250);
	usleep_range(10000,15000);
}

void elite_pcie_ports_suspend(void)
{
    int i, j, value;
	
	struct pcie_port *pp = NULL;
	for (i = 0; i < num_pcie_ports; i++)
	{
		pp = &pcie_port[i];
		if (pp->base != NULL)	
		{
			for (j = 0; j < 16; j++)
			{
				pp->regs[j] = readw(pp->base + 0x260 + j*2);
			}

		}
	}

}

void elite_pcie_ports_resume(void)
{
    int i, j, value;
	struct pcie_port *pp = NULL;
	for (i = 0; i < num_pcie_ports; i++)
	{
		pp = &pcie_port[i];
		if (pp->base != NULL)	
		{
		    printk("pp->base+0x250=%x\n",(pp->base+0x250));
			value = readl(pp->base+0x250);
			printk("RP before 0x250 0x%x\n",value);
			value &= ~(1 << 2);  //Power On 
			printk("RP middle 0x250 0x%x\n",value);
			writel(value, pp->base+ 0x250);
			usleep_range(10000,15000);
			value = readl(pp->base+0x250);
			printk("RP after 0x250 0x%x\n",value);
			
			for (j = 0; j < 16; j++)
			{
				writew(pp->regs[j],pp->base + 0x260 + j*2);
			}
		}
	}

}

static void init_hdmi_audio()
{
	memset(audio_pci_cfg_shadow, 0xff, PCI_SPACE_SIZE);
	*(u32*)audio_pci_cfg_shadow = 0x903f5333;

	*((u32*)audio_pci_cfg_shadow+1) = 0x00100006;
	*((u32*)audio_pci_cfg_shadow+2) = 0x04030001;
	*((u32*)audio_pci_cfg_shadow+3) = 0x00800008;
	*((u32*)audio_pci_cfg_shadow+4) = 0xD8090000;//base addr
	*((u32*)audio_pci_cfg_shadow+5) = 0x00000000;
	*((u32*)audio_pci_cfg_shadow+6) = 0x0;
	*((u32*)audio_pci_cfg_shadow+7) = 0x0;
	*((u32*)audio_pci_cfg_shadow+8) = 0x00000000;
	*((u32*)audio_pci_cfg_shadow+9) = 0x0;

	*((u32*)audio_pci_cfg_shadow+0xa) = 0x0;
	*((u32*)audio_pci_cfg_shadow+0xb) = 0x90455333;
	*((u32*)audio_pci_cfg_shadow+0xc) = 0x0;
	*((u32*)audio_pci_cfg_shadow+0xd) = 0x000000dc;

}

static void __init elite_pcie_ports_init(int init_port0, int init_port1, int init_port2)
{
	if (init_port0)
		add_pcie_port(0, ELITE_PCIE0_BASE_ADDR);

	if (init_port1)
		add_pcie_port(1, ELITE_PCIE1_BASE_ADDR);

	if (init_port2)
		add_pcie_port(2, ELITE_PCIE2_BASE_ADDR);
}

static int __init elite_pcie_init(void)
{
	printk("ELITE PCIE INIT\n");
	init_hdmi_audio();
	elite_pcie_ports_init(1,1,0);
	elite_pcie.nr_controllers = num_pcie_ports;
	pci_common_init(&elite_pcie);
	return 0;
}

subsys_initcall(elite_pcie_init);
