/*++
Copyright (c) 2008  WonderMedia Technologies, Inc.   All Rights Reserved.

This PROPRIETARY SOFTWARE is the property of WonderMedia Technologies, Inc.
and may contain trade secrets and/or other confidential information of
WonderMedia Technologies, Inc. This file shall not be disclosed to any third
party, in whole or in part, without prior written consent of WonderMedia.

THIS PROPRIETARY SOFTWARE AND ANY RELATED DOCUMENTATION ARE PROVIDED AS IS,
WITH ALL FAULTS, AND WITHOUT WARRANTY OF ANY KIND EITHER EXPRESS OR IMPLIED,
AND WonderMedia TECHNOLOGIES, INC. DISCLAIMS ALL EXPRESS OR IMPLIED WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR
NON-INFRINGEMENT.

Module Name:

	$Workfile: elite_nand.c $

Abstract:

	POST functions, called by main().

Revision History:
--*/

#ifndef __NFC_H__
#define __NFC_H__

#if (CONFIG_MTD_NAND_HM_ECC == 1)
#if (CONFIG_MTD_NAND_PAGE_SIZE != 512)
#define NAND_BBT_BCH_ECC
#else
#undef NAND_BBT_BCH_ECC
#endif
#else
#define NAND_BBT_BCH_ECC
#endif

#ifndef DWORD
#define DWORD	unsigned int
#endif

#define MAX_PRODUCT_NAME_LENGTH 0x20
#define NAND_TYPE_MLC 1
#define NAND_TYPE_SLC 0

#define	ELITE_NFC_REG_0           0x00 
#define	ELITE_NFC_REG_1           0x04 
#define	ELITE_NFC_REG_2           0x08 
#define	ELITE_NFC_REG_3           0x0c 
#define	ELITE_NFC_REG_4           0x10 
#define	ELITE_NFC_REG_5           0x14 
#define	ELITE_NFC_REG_6           0x18 
#define	ELITE_NFC_REG_7           0x1c 
#define	ELITE_NFC_REG_8           0x20 
#define	ELITE_NFC_REG_9           0x24 
#define	ELITE_NFC_REG_A          0x28
#define	ELITE_NFC_REG_B          0x2c
#define	ELITE_NFC_REG_C          0x30
#define	ELITE_NFC_REG_D          0x34
#define	ELITE_NFC_REG_E          0x38
#define	ELITE_NFC_REG_F          0x3c
#define	ELITE_NFC_REG_10         0x40
#define	ELITE_NFC_REG_11         0x44
#define	ELITE_NFC_REG_12         0x48 
#define	ELITE_NFC_REG_13         0x4c 
#define	ELITE_NFC_REG_14         0x50 
#define	ELITE_NFC_REG_15         0x54 
#define	ELITE_NFC_REG_16         0x58 
#define	ELITE_NFC_REG_17         0x5c 
#define	ELITE_NFC_REG_18         0x60 
#define	ELITE_NFC_REG_19         0x64
#define	ELITE_NFC_REG_1A        0x68
#define	ELITE_NFC_REG_1B        0x6c
#define	ELITE_NFC_REG_1C        0x70
#define	ELITE_NFC_REG_1D        0x74
#define	ELITE_NFC_REG_1E        0x78
#define	ELITE_NFC_REG_1F         0x7c
#define	ELITE_NFC_REG_20         0x80
#define	ELITE_NFC_REG_21         0x84
#define	ELITE_NFC_REG_22         0x88
#define	ELITE_NFC_REG_23         0x8c
#define	ELITE_NFC_REG_24         0x90
#define	ELITE_NFC_REG_25         0x94
#define	ELITE_NFC_REG_26         0x98
#define	ELITE_NFC_REG_27         0x9c
#define	ELITE_NFC_REG_28         0xa0
#define	ELITE_NFC_REG_29         0xa4
#define	ELITE_NFC_REG_2A        0xa8
#define	ELITE_NFC_REG_2B        0xac
#define	ELITE_NFC_REG_2C        0xb0
#define	ELITE_NFC_REG_2D        0xb4
#define	ELITE_NFC_REG_2E        0xb8
#define	ELITE_NFC_REG_2F        0xbc
#define	ELITE_NFC_REG_30         0xc0
#define	ELITE_NFC_REG_31         0xc4
#define	ELITE_NFC_REG_32         0xc8


/*NAND PDMA*/
#define ELITE_NFC_DMA_GCR	0x100
#define ELITE_NFC_DMA_IER		0x104
#define ELITE_NFC_DMA_ISR		0x108
#define ELITE_NFC_DMA_DESPR	0x10C
#define ELITE_NFC_DMA_RBR	0x110
#define ELITE_NFC_DMA_DAR	0x114
#define ELITE_NFC_DMA_BAR	0x118
#define ELITE_NFC_DMA_CPR	0x11C
#define ELITE_NFC_DMA_CCR	0X120

/*NAND PDMA - DMA_GCR : DMA Global Control Register*/
#define NAND_PDMA_GCR_DMA_EN	        0x00000001      /* [0] -- DMA controller enable */
#define NAND_PDMA_GCR_SOFTRESET	0x00000100      /* [8] -- Software rest */

/*NAND PDMA - DMA_IER : DMA Interrupt Enable Register*/
#define NAND_PDMA_IER_INT_EN		0x00000001      /* [0] -- DMA interrupt enable */

/*NAND PDMA - DMA_ISR : DMA Interrupt Status Register*/
#define NAND_PDMA_IER_INT_STS	0x00000001      /* [0] -- DMA interrupt status */

/*NAND PDMA - DMA_DESPR : DMA Descriptor base address Pointer Register*/
/*NAND PDMA - DMA_RBR : DMA Residual Bytes Register*/
#define NAND_PDMA_RBR_End		0x80000000      /* [31] -- DMA descriptor end flag */
#define NAND_PDMA_RBR_Format		0x40000000      /* [30] -- DMA descriptor format */

/*NAND PDMA - DMA_DAR : DMA Data Address Register*/
/*NAND PDMA - DMA_BAR : DMA Rbanch Address Register*/
/*NAND PDMA - DMA_CPR : DMA Command Pointer Register*/
/*NAND PDMA - DMA_CCR : DMAContext Control Register for Channel 0*/
#define NAND_PDMA_READ					0x00
#define NAND_PDMA_WRITE					0x01
#define NAND_PDMA_CCR_RUN				0x00000080
#define NAND_PDMA_CCR_IF_TO_PERIPHERAL     0x00000000
#define NAND_PDMA_CCR_PERIPHERAL_TO_IF	0x00400000
#define NAND_PDMA_CCR_EVT_CODE			0x0000000f
#define NAND_PDMA_CCR_EVT_NO_STATUS		0x00000000
#define NAND_PDMA_CCR_EVT_FF_UNDERRUN	0x00000001
#define NAND_PDMA_CCR_EVT_FF_OVERRUN	        0x00000002
#define NAND_PDMA_CCR_EVT_DESP_READ		0x00000003
#define NAND_PDMA_CCR_EVT_DATA_RW		0x00000004
#define NAND_PDMA_CCR_EVT_EARLY_END		0x00000005
#define NAND_PDMA_CCR_EVT_SUCCESS		        0x0000000f

#define ECC_FIFO_0    0x1c0
#define ECC_FIFO_1    0x1c4
#define ECC_FIFO_2    0x1c8
#define ECC_FIFO_3    0x1cc
#define ECC_FIFO_4    0x1d0
#define ECC_FIFO_5    0x1d4
#define ECC_FIFO_6    0x1d8
#define ECC_FIFO_7    0x1dc
#define ECC_FIFO_8    0x1e0
#define ECC_FIFO_9    0x1e4
#define ECC_FIFO_A   0x1e8
#define ECC_FIFO_B   0x1ec
#define ECC_FIFO_C   0x1f0
#define ECC_FIFO_D   0x1f4
#define ECC_FIFO_E   0x1f8
#define ECC_FIFO_F   0x1fc

/*Status Registers*/
#define ELITE_NFC_DATAPORT	             0x00

/*Control Registers */
#define ELITE_NFC_COMCTRL	             0x04
#define ELITE_NFC_COMPORT0	             0x08
#define ELITE_NFC_COMPORT1_2	     0x0c
#define ELITE_NFC_COMPORT3_4	     0x10
#define ELITE_NFC_COMPORT5_6	     0x14
#define ELITE_NFC_COMPORT7	             0x18
#define ELITE_NFC_COMPORT8_9             0x1c
#define ELITE_NFC_DMA_COUNTER	     0x20
#define ELITE_NFC_SMC_ENABLE	     0x24

/*Status Registers*/
#define ELITE_NFC_MISC_STAT_PORT      0x28

/*Status Control Registers*/
#define ELITE_NFC_HOST_STAT_CHANGE 0x2c

/*Control Registers*/
#define ELITE_NFC_SMC_DMA_COUNTER   0x30
#define ELITE_NFC_CALC_CTRL                     0x34
#define ELITE_NFC_CALC_NUM	                  0x38
#define ELITE_NFC_CALC_NUM_QU	          0x3c
#define ELITE_NFC_REMAINDER	                  0x40
#define ELITE_NFC_CHIP_ENABLE_CTRL    0x44
#define ELITE_NFC_NAND_TYPE_SEL            0x48
#define ELITE_NFC_REDUNT_ECC_STAT_MASK    0x4c
#define ELITE_NFC_READ_CYCLE_PULE_CTRL    0x50
#define ELITE_NFC_MISC_CTRL	                              0x54
#define ELITE_NFC_DUMMY_CTRL	                      0x58
#define ELITE_NFC_PAGESIZE_DIVIDER_SEL         0x5c
#define ELITE_NFC_RW_STROBE_TUNE	              0x60
#define ELITE_NFC_BANK18_ECC_STAT_MASK     0x64

/*Status Registers*/
#define ELITE_NFC_ODD_BANK_PARITY_STAT	    0x68
#define ELITE_NFC_EVEN_BANK_PARITY_STAT         0x6c
#define ELITE_NFC_REDUNT_AREA_PARITY_STAT    0x70
#define ELITE_NFC_IDLE_STAT	         0x74
#define ELITE_NFC_PHYS_ADDR	         0x78

/*Status Control Registers*/
#define ELITE_NFC_REDUNT_ECC_STAT	          0x7c
#define ELITE_NFC_BANK18_ECC_STAT	          0x80

/*Control Registers*/
#define ELITE_NFC_TIMER_COUNTER_CONFIG   0x84
#define ELITE_NFC_NANDFLASH_BOOT	             0x88
#define ELITE_NFC_ECC_BCH_CTRL	                     0x8c
#define ELITE_NFC_ECC_BCH_INT_MASK	             0x90

/*Status Control Registers*/
#define ELITE_NFC_ECC_BCH_INT_STAT1              0x94

/*Status Registers*/
#define ELITE_NFC_ECC_BCH_INT_STAT2 0x98
#define ELITE_NFC_ECC_BCH_ERR_POS1	0x9c
#define ELITE_NFC_ECC_BCH_ERR_POS2	0xa0
#define ELITE_NFC_ECC_BCH_ERR_POS3	0xa4
#define ELITE_NFC_ECC_BCH_ERR_POS4	0xa8
#define ELITE_NFC_ECC_BCH_ERR_POS5	0xac
#define ELITE_NFC_ECC_BCH_ERR_POS6	0xb0
#define ELITE_NFC_ECC_BCH_ERR_POS7	0xb4
#define ELITE_NFC_ECC_BCH_ERR_POS8	0xb8

/* PDMA Descriptor short*/
typedef struct {
	unsigned long volatile req_count:16;		 /* bit 0 -15 -Request count   */
	unsigned long volatile i:1;				 /* bit 16    -interrupt      */
	unsigned long volatile reserve:13;		         /* bit 17-29 -reserved     */
	unsigned long volatile format:1;		         /* bit 30    -The descriptor format    */
	unsigned long volatile end:1;			         /* bit 31    -End flag of descriptor list*/
	unsigned long volatile data_buffer_addr:32;/* bit 31    -Data Buffer address  */
} nand_pdma_desc_short;

/*PDMA Descriptor long*/
typedef struct 
{
	unsigned long volatile req_count:16;		 /* bit 0 -15 -Request count    */
	unsigned long volatile i:1;				 /* bit 16    -interrupt    */
	unsigned long volatile reserve:13;		         /* bit 17-29 -reserved      */
	unsigned long volatile format:1;		         /* bit 30    -The descriptor format     */
	unsigned long volatile end:1;			         /* bit 31    -End flag of descriptor list*/
	unsigned long volatile data_buffer_addr:32;/* bit 31-0  -Data Buffer address       */
	unsigned long volatile branch_addr:32;	 /* bit 31-2  -Descriptor Branch address	*/
	unsigned long volatile reserve0:32;		 /* bit 31-0  -reserved */
} nand_pdma_desc_long;


#define ECC1BIT 0
#define ECC4BIT 1
#define ECC8BIT 2
#define ECC12BIT 3
#define ECC16BIT 4
#define ECC24BIT 5
#define ECC40BIT 6
#define ECC24BIT_PER_1K 5
#define ECC40BIT_PER_1K 6
#define ECC44BIT_PER_1K 7
#define ECC44BIT               8
#define ECC1BIT_BIT_COUNT   32
#define ECC4BIT_BIT_COUNT   52
#define ECC8BIT_BIT_COUNT   104
#define ECC12BIT_BIT_COUNT 156
#define ECC16BIT_BIT_COUNT 208
#define ECC24BIT_PER_1K_BIT_COUNT 336
#define ECC40BIT_PER_1K_BIT_COUNT 560
#define ECC44BIT_PER_1K_BIT_COUNT 616
#define ECC44BIT_BIT_COUNT    576
#define ECC1BIT_BYTE_COUNT    4
#define ECC4BIT_BYTE_COUNT    8
#define ECC8BIT_BYTE_COUNT    16
#define ECC12BIT_BYTE_COUNT  20
#define ECC16BIT_BYTE_COUNT  26
#define ECC24BIT_PER_1K_BYTE_COUNT 42
#define ECC40BIT_PER_1K_BYTE_COUNT 70
#define ECC44BIT_PER_1K_BYTE_COUNT 77
#define ECC44BIT_BYTE_COUNT 72
#define ECC_MOD_SEL 7
#define MAX_ERR_BIT 44

/* ECC BCH interrupt mask */
#define ECC_BCH_INTERRUPT_ENABLE   0x101
#define ECC_BCH_INTERRUPT_DISABLE  0x0

/* cfg_1 */
#define SP_CMD_EN              0x400   /*reid set cmd registers 0x08~0x1c*/  
#define DPAHSE_DISABLE  0x80     /* disable data phase */
#define DATA_READ_EN      0x40     /* enable data read */
#define END_WITH_CMD_EN  0x10  /* enable end with command */
#define DPAHSE_DISABLE      0x80   /* disable data phase */
#define NAND2NFC                    0x40   /* direction : nand to controller */
#define NFC2NAND                    0x00   /* direction : controller to controller */
#define SING_RW	                      0x20   /* enable signal read/ write command */
#define MUL_CMDS                  0x10   /* support cmd+addr+cmd */
#define NFC_TRIGGER            0x01   /* start cmd&addr sequence */

/* cfg_9 */
#define NFC_CMD_RDY 0x04
#define OOB_RW             0x02

/* cfg_a */
#define NFC_BUSY	0x02   /* command and data is being transfer in flash I/O */
#define FLASH_RDY    0x01  /* flash is ready */

/* cfg_b */
#define B2R	              0x08     /* status form busy to ready */

/* cfg_12 */
#define WP_DISABLE   (1<<4) /* disable write protection */
#define OLDDATA_EN   (1<<2) /* enable old data structure */
#define WIDTH_8	        0
#define WIDTH_16      (1<<3)
#define PAGE_512       0
#define PAGE_2K         1
#define PAGE_4K         2
#define PAGE_8K         3
#define DIRECT_MAP   (1<<5)
#define USE_SW_ECC  0x40
#define USE_HW_ECC  0
//#define CHECK_ALLFF  (1<<6)

/* cfg_1d */
#define NFC_IDLE	   0x01
#define DMA_SINGNAL 0
#define DMA_INC4          0x10
#define DMA_INC8          0x20

/* cfg_23 */
#define READ_RESUME 0x100
#define ECC_MODE        7

/* cfg_25 status */
#define ERR_CORRECT 0x100
#define BCH_ERR            0x1

/* cfg_26 status */
#define BANK_DR            0x800
#define BANK_NUM        0x700
#define BCH_ERR_CNT  0x3F

#define ADDR_COLUMN              1
#define ADDR_PAGE                     2
#define ADDR_COLUMN_PAGE 3

#define WRITE_NAND_COMMAND(d, adr) do { *(volatile unsigned char *)(adr) = (unsigned char)(d); } while (0)
#define WRITE_NAND_ADDRESS(d, adr)   do { *(volatile unsigned char *)(adr) = (unsigned char)(d); } while (0)

#define SOURCE_CLOCK 25
#define MAX_SPEED_MHZ 65
#define MAX_READ_DELAY 9   /* 8.182 = tSKEW 3.606 + tDLY 4.176 + tSETUP 0.4 */
#define MAX_WRITE_DELAY 9 /* 8.72 = tDLY 10.24 - tSKEW 1.52*/

#define FIRST_4K_218     0
#define SECOND_4K_218 4314 /*4096 + 218 */

/* Nand Product Table. We take HYNIX_NF_HY27UF081G2A for example.*/
struct elite_nand_flash_dev{
	 DWORD flash_id;                   /*composed by 4 bytes of ID. For example:0xADF1801D*/
	 DWORD block_count;            /*block count of one chip. For example: 1024*/
	 DWORD page_size;                /*page size. For example:2048(other value can be 512 or 4096)*/
	 DWORD spare_size;               /*spare area size. For example:16(almost all kinds of nand is 16)*/
	 DWORD block_size;               /*block size = dwPageSize * PageCntPerBlock. For example:131072*/
	 DWORD address_cycle;          /*address cycle 4 or 5*/
	 DWORD bb_page0_position;  /*BI0 page postion in block*/
	 DWORD bb_page1_position;  /*BI1 page postion in block*/
	 DWORD bb_info_offset;         /*BI offset in page*/
	 DWORD data_width;              /*data with X8 or X16*/
	 DWORD page_program_limit;     /*chip can program PAGE_PROGRAM_LIMIT times within the same page*/
	 DWORD seq_row_read_support; /*whether support sequential row read, 1 = support 0 = not support*/
	 DWORD seq_page_program;       /*chip need sequential page program in a block. 1 = need*/
	 DWORD nand_type;       /*MLC or SLC*/
	 DWORD ecc_bit_num;   /*ECC bit number needed*/
	 DWORD rw_timming;    /*NFC Read/Write Pulse width and Read/Write hold time. default =0x12121010*/
	 char product_name[MAX_PRODUCT_NAME_LENGTH]; /*product name. for example "HYNIX_NF_HY27UF081G2A"*/
	 unsigned long options;
};

extern int add_mtd_device(struct mtd_info* mtd);
int elite_nand_init_pdma(struct mtd_info *mtd);
int elite_nand_free_pdma(struct mtd_info *mtd);
int elite_nand_alloc_desc_pool(unsigned long *desc_addr);
int elite_nand_init_short_desc(unsigned long *desc_addr, unsigned int req_count, unsigned long *buffer_addr);
int elite_nand_init_long_desc(unsigned long *desc_addr, unsigned int req_count, unsigned long *buffer_addr,unsigned long *branch_addr, int end);
int elite_nand_config_pdma(struct mtd_info *mtd, unsigned long *desc_addr, unsigned int dir);
int elite_nand_pdma_handler(struct mtd_info *mtd);
void elite_nand_hamming_ecc_1bit_correct(struct mtd_info *mtd);
void elite_bch4bit_data_ecc_correct(struct mtd_info *mtd,int column, unsigned int page_addr);
void elite_bch4bit_redunt_ecc_correct(struct mtd_info *mtd);
void elite_copy_filename (char *dst, char *src, int size);
int elite_get_sys_para(char *var_name, unsigned char *varval, int *var_len);
int elite_set_sys_para(char *var_name, char *varval);

/* Chip can not auto increment pages */
#define NAND_NO_AUTOINCR	0x00000001
#define NAND_CHIPOPTIONS_MSK	(0x0000ffff & ~NAND_NO_AUTOINCR)

#endif

