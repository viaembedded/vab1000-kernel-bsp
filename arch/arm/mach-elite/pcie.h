#ifndef ELITE_PCIE_H
#define ELITE_PCIE_H

#define ELITE_PCIE0_BASE_ADDR     0xd80b0000
#define ELITE_PCIE1_BASE_ADDR     0xd80c0000
#define ELITE_PCIE2_BASE_ADDR     0xd80d0000
#define ELITE_PCIE_PORTS_SIZE       SZ_64K	

#define BUS_DEV_FUNC_ID_SECTION_OFFSET  0x0260

#define DEVICE_CFG_SIZE			   SZ_4K


#define ELITE_PCIE_IO_PHYS_BASE(i)  (0xcfff0000 + ((i) << 16))
#define ELITE_PCIE_IO_PHYS_SIZE 	 SZ_64K 


#define ELITE_PCIE_IO_BASE      0xcffe0000
#define ELITE_PCIE_IO_SIZE 	 SZ_128K


#define SZ_255M   			 (SZ_256M -SZ_1M)
#if 0
#define ELITE_PCIE_MEM_PHYS_BASE  0xc1800000
#define ELITE_PCIE_MEM_SIZE		SZ_232M

#define SZ_232M		0xce800000	
#define SZ_116M		0xc7400000	
#define SZ_77M		0xc4d00000	
#else
#define ELITE_PCIE_MEM_PHYS_BASE  0xc0000000
#define ELITE_PCIE_MEM_SIZE		SZ_255M 
#endif

#define SZ_1M				0x00100000
#define SZ_2M				0x00200000
#define SZ_4M				0x00400000
#define SZ_8M				0x00800000
#define SZ_16M				0x01000000
#define SZ_32M				0x02000000
#define SZ_64M				0x04000000
#define SZ_128M				0x08000000
#define SZ_256M				0x10000000
#define SZ_512M				0x20000000

#define SZ_232M		0xce800000	
#define SZ_96M	        (SZ_64M+SZ_32M)	
#define SZ_116M		(SZ_128M-SZ_8M-SZ_4M)
#define SZ_77M		(SZ_64M+SZ_8M+SZ_4M+SZ_1M)	


#define SZ_1K				0x00000400
#define SZ_2K				0x00000800
#define SZ_4K				0x00001000
#define SZ_8K				0x00002000
#define SZ_16K				0x00004000
#define SZ_32K				0x00008000
#define SZ_64K				0x00010000

#define SZ_21K				(SZ_16K+SZ_4K+SZ_1K)



#define IRQ_ELITE_PCIE_PORT0	9 + 32
#define IRQ_ELITE_PCIE_PORT1	11 + 32
#define IRQ_ELITE_PCIE_PORT2	13 + 32

#endif
