/*
	linux/include/asm-arm/arch-elite/dma.h

	Copyright (c) 2012 S3Graphics Technologies, Inc.

	This program is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software Foundation,
	either version 2 of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful, but WITHOUT
	ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
	PARTICULAR PURPOSE.  See the GNU General Public License for more details.
	You should have received a copy of the GNU General Public License along with
	this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef __MARCH_DMA_H
#define __MARCH_DMA_H

#include <asm/sizes.h>
#include "types.h"

#define dmach_t unsigned int

/*
 * This is the maximum DMA address that can be DMAd to.
 */

#define DMA_MEM_REG_OFFSET		0x100
#define DMA_CHAN_REG_SIZE		0x20
#define DMA_IF0RBR_CH			0x00
#define DMA_IF0DAR_CH			0x04
#define	DMA_IF0BAR_CH			0x08
#define DMA_IF0CPR_CH			0x0c
#define DMA_IF1RBR_CH			0x10
#define DMA_IF1DAR_CH			0x14
#define DMA_IF1BAR_CH			0x18
#define DMA_IF1CPR_CH			0x1c

#define	DMA_GCR				0x40
#define DMA_IER				0x48
#define DMA_ISR				0x4c
#define DMA_TMR				0x50
#define DMA_CCR				0x80

/*
 * Maximum physical DMA buffer size
 */

#define MAX_DMA_SIZE			SZ_16K
#define CUT_DMA_SIZE			SZ_4K

/***********************************************
* The WM3437 has 16 internal DMA channels.
*************************************************/

#define DMA_CHANNELS    16
#define MAX_DMA_CHANNELS DMA_CHANNELS
#define DMA_INT0 IRQ_DMA_CH_0
#define DMA_INT1 IRQ_DMA_CH_1
#define DMA_INT2 IRQ_DMA_CH_2
#define DMA_INT3 IRQ_DMA_CH_3
#define DMA_INT4 IRQ_DMA_CH_4
#define DMA_INT5 IRQ_DMA_CH_5
#define DMA_INT6 IRQ_DMA_CH_6
#define DMA_INT7 IRQ_DMA_CH_7
#define DMA_INT8 IRQ_DMA_CH_8
#define DMA_INT9 IRQ_DMA_CH_9
#define DMA_INT10 IRQ_DMA_CH_10
#define DMA_INT11 IRQ_DMA_CH_11
#define DMA_INT12 IRQ_DMA_CH_12
#define DMA_INT13 IRQ_DMA_CH_13
#define DMA_INT14 IRQ_DMA_CH_14
#define DMA_INT15 IRQ_DMA_CH_15

/************************************
*
*  DMA GLOBAL CONTROL
*
*************************************/
#define DMA_SW_RST	BIT8
#define DMA_BIG_ENDIAN	BIT1
#define DMA_GLOBAL_EN	BIT0
/************************************
*
*  DMA_INTERRUPT ENABLE
*
*************************************/
#define CH00_INT_EN	BIT0
#define CH01_INT_EN	BIT1
#define CH02_INT_EN	BIT2
#define CH03_INT_EN	BIT3
#define CH04_INT_EN	BIT4
#define CH05_INT_EN	BIT5
#define CH06_INT_EN	BIT6
#define CH07_INT_EN	BIT7
#define CH08_INT_EN	BIT8
#define CH09_INT_EN	BIT9
#define CH10_INT_EN	BIT10
#define CH11_INT_EN	BIT11
#define CH12_INT_EN	BIT12
#define CH13_INT_EN	BIT13
#define CH14_INT_EN	BIT14
#define CH15_INT_EN	BIT15
#define ALL_INT_EN	0x0000FFFF

/************************************
*
*  DMA_INTERRUPT STATUS
*
*************************************/
#define CH00_INT_STS	BIT0
#define CH01_INT_STS	BIT1
#define CH02_INT_STS	BIT2
#define CH03_INT_STS	BIT3
#define CH04_INT_STS	BIT4
#define CH05_INT_STS	BIT5
#define CH06_INT_STS	BIT6
#define CH07_INT_STS	BIT7
#define CH08_INT_STS	BIT8
#define CH09_INT_STS	BIT9
#define CH10_INT_STS	BIT10
#define CH11_INT_STS	BIT11
#define CH12_INT_STS	BIT12
#define CH13_INT_STS	BIT13
#define CH14_INT_STS	BIT14
#define CH15_INT_STS	BIT15
#define ALL_INT_CLEAR	0x0000FFFF

/************************************
*
*  DMA SCHEDULE SCHEME
*
*************************************/
#define SCHEDULE_RR_DISABLE	BIT0
#define TIMER_1_SHIFT	16
#define TIMER_2_SHIFT	8
/************************************
*
*  DMA_CCR SETTIGN
*
*************************************/
/*WRAP MODE [31:30]*/
#define DMA_WRAP_1	0x00000000
#define DMA_WRAP_2	BIT30
#define DMA_WRAP_4	BIT31
#define DMA_WRAP_8	(BIT31|BIT30)
#define DMA_WRQP_MASK	0xC0000000
/*BUST[29:28]*/
#define DMA_BURST_1	0x00000000
#define DMA_BURST_4	BIT28
#define DMA_BURST_8	BIT29
#define DMA_BURST_MASK	0x30000000
/*TRANSFER SIZE[[27:26]*/
#define DMA_SIZE_8	0x00000000
#define DMA_SIZE_16	BIT26
#define DMA_SIZE_32	BIT27
#define DMA_SIZE_MASK	0x0C000000
/*/1/2 addr mode[25:24]*/
#define DMA_WRAP_MODE	0x00000000
#define DMA_INC_MODE	BIT24
#define DMA_SG_MODE	BIT25
#define DMA_ADDR_MODE_MASK	0x03000000
/*SW REQUEST ENABLE[23]*/
#define DMA_SW_REQ	BIT23
#define DMA_SW_REQ_MASK	0x00800000
/*DMA 1/2 TRANS DIRECTION[22]*/
#define DEVICE_TO_MEM	BIT22
#define DEVICE_TO_MEM_MASK	0x00400000
/*DMA REQ NUM[20:16]*/
#define DMA_REQ_ID_SHIFT	16
#define DMA_REQ_ID_MASK		0x000F8000

/*DMA complete BIT*/
#define DMA_P0_COMPLETE	BIT9	   /*0:complete 1:did't complete*/
#define DMA_P1_COMPLETE	BIT8

#define SYSTEM_DMA_RUN	BIT7
#define DMA_RUN_MASK	0x00000080
#define DMA_WAKE BIT6
#define DMA_WAKE_MASK	0x00000040
#define DMA_ACTIVE	BIT4
#define DMA_ACTIVE_MASK	0x00000010
#define DMA_USER_SET_MASK	0xFFFE0000
#define SYSTEM_DMA_REQ_EN BIT11

/*DMA ERROR ID[3:0]*/
#define DMA_EVT_ID_MASK	0x0000000F
#define DMA_EVT_NO_STATUS	0
#define DMA_EVT_REG	1
#define DMA_EVT_FF_UNDERRUN	2
#define DMA_EVT_FF_OVERRUN	3
#define DMA_EVT_DESP_READ	4
#define DMA_EVT_DESP_WRITE	5
#define DMA_EVT_MR_READ		6
#define DMA_EVT_MR_WRITE	7
#define DMA_EVT_DATA_READ	8
#define DMA_EVT_DATA_WRITE	9
#define DMA_EVT_SUCCESS		15

/*DMA UPDATE MEMORY REG. BIT*/
#define DMA_UP_MEMREG_EN BIT5

/*****************************************
	DMA descript setting
******************************************/
#define DMA_DES_END	BIT31
#define DMA_FORMAT_DES1	BIT30
#define DMA_INTEN_DES	BIT29
#define DMA_DONE_DES    BIT16
#define DMA_DES0_SIZE 0x8	/*8 byte*/
#define DMA_DES1_SIZE 0x10	/*16 byte*/
#define DMA_DES_REQCNT_MASK 0xFFFF


#define DMA_CTRL_CFG_BASE_ADDR      0xD8001800


#define ELITE_DMA_1_BURST 0 
#define ELITE_DMA_2_BURST  1
#define ELITE_DMA_4_BURST  2
#define ELITE_DMA_8_BURST  3


#define ELITE_DMA_BURST_LEN_SINGLE  0
#define ELITE_DMA_BURST_LEN_INCR4  1
#define ELITE_DMA_BURST_LEN_INCR8  2

#define ELITE_DMA_RANS_SIZE_8BIT 0
#define ELITE_DMA_RANS_SIZE_16BIT 1
#define ELITE_DMA_RANS_SIZE_32BIT 2

#define ELITE_DMA_ADDR_MODE_WRAP 0
#define ELITE_DMA_ADDR_MODE_INC 1
#define ELITE_DMA_ADDR_MODE_SG 2

#define ELITE_DMA_REQ_SW 1
#define ELITE_DMA_REQ_HW 0

#define ELITE_DMA_TRANS_IF0_TO_IF12 0
#define ELITE_DMA_TRANS_IF12_TO_IF0 1

/*
 * All possible devices a DMA channel can be attached to.
 */

enum dma_device_e {
	UART_0_TX_DMA_REQ = 0,/* uart0*/
	UART_0_RX_DMA_REQ = 1,/* uart0*/
	UART_1_TX_DMA_REQ = 2,/* uart1*/
	UART_1_RX_DMA_REQ = 3,/* uart1*/
	SPI0_DMA_TX_REQ = 4, /*spi0tx*/
	SPI0_DMA_RX_REQ = 5, /*spi0rx*/
	AHB1_AUD_DMA_REQ_0 = 16,
	AHB1_AUD_DMA_REQ_1 = 17,
	AHB1_AUD_DMA_REQ_2 = 18,
	AHB1_AUD_DMA_REQ_3 = 19,
	AHB1_AUD_DMA_REQ_4 = 20,
	AHB1_AUD_DMA_REQ_5 = 21,
	AHB1_AUD_DMA_REQ_6 = 22,
	AHB1_AUD_DMA_REQ_7 = 23,
	SF_DMA_TX_REQ = 30, /*sftx */
	SF_DMA_RX_REQ = 31, /*sftx */
	MEMORY_DMA_REQ = 32,/* memory*/
	DEVICE_RESERVED = 33 /* reserved*/
};

/*
 * DMA device configuration structure
 * when memory to memory
 *	MIF0addr : source address
 *	MIF1addr : destination address
 * when device to memory or memory to device
 *	MIF0addr : memory address
 *	MIF1addr : device FIFO address
 */

struct dma_device_cfg_s {
	enum dma_device_e device_reqtype;
	unsigned long default_ccr;
	unsigned long mif0_addr;
	unsigned long mif1_addr;
	unsigned int  chunk_size;
};

/*
* DMA descriptor registers
*/

struct dma_des_fmt0 {
	volatile unsigned long req_cnt;
	volatile unsigned long data_addr;
};
struct dma_des_fmt1 {
	volatile unsigned long req_cnt;
	volatile unsigned long data_addr;
	volatile unsigned long branch_addr;
	volatile unsigned long reserved;

};

struct dma_req_s {
	dmach_t channel;
	unsigned int device;
	char *device_id;
	unsigned long addr_wrp_bnd:2;
	unsigned long bst_len:2;
	unsigned long trans_size:2;
	unsigned long if12_addr_mode:2;
	unsigned long sw_req:1;
	unsigned long trans_dir:1;
	unsigned long fifo_addr;
	unsigned int  chunk_size;
	void (*callback)(void *data);
	void *callback_data;
};

extern int elite_request_dma(struct dma_req_s *dma_req);

extern void elite_free_dma(dmach_t ch);

extern int elite_start_dma(dmach_t ch, dma_addr_t dma_ptr, dma_addr_t dma_ptr2, unsigned int size);

extern void elite_reset_dma(dmach_t ch);

extern void elite_clear_dma(dmach_t ch);

extern void elite_stop_dma(dmach_t ch);

extern void elite_resume_dma(dmach_t ch);

//extern struct dma_mem_reg_group_s elite_get_dma_pos_info(dmach_t ch);

extern unsigned int elite_get_dma_pos(dmach_t ch);

extern int elite_dma_busy(dmach_t ch);

//extern void elite_dump_dma_regs(dmach_t ch);

#endif 
