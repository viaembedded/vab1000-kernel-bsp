/*
 * linux/drivers/input/rmtctl/elite_rmtctl.h
 * s3 input remote control driver
 *
 *
 * This program is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 2 of the License, or 
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __ELT_CIR_H__
#define __ELT_CIR_H__

#define CIR_BASE_ADDR						0xD8270000

#define IRSWRST								0x00			// [0x00] IR Software Reset register
#define IRCTL								0x04			// [0x04] IR Control register
#define IRCTL_2								0x08			// [0x08] IR Control register
#define IRSTS								0x0c			// [0x0c] IR Status register
#define IRDATA(x)							(0x10+x*0x4)		// [0x10-0x20] IR Received Data register
#define PARAMETER(x)						(0x24+x*0x4)		// [0x24-0x3c]IR Parameter Register for Remote Controller Vendor "NEC"
#define NEC_REPEAT_TIME_OUT_CTRL			0x40			// [0X40] 
#define NEC_REPEAT_TIME_OUT_COUNT		0x44			// [0X44] 
#define NEC_REPEAT_TIME_OUT_STS			0x48			// [0X48]
#define NEC_2ND_HEADER_PREFIX_BIT_NUM	0x4c			// [0X4C]
#define JVC_CONTI_CTRL						0x50			// [0X50] 
#define JVC_CONTI_CNT						0x54			// [0X54] 
#define JVC_CONTI_STS						0x58			// [0X58] 
#define INT_MASK_CTRL						0x60			// [0X60] 
#define INT_MASK_COUNT						0x64			// [0X64] 
#define INT_MASK_STS						0x68			// [0X68] 
#define WAKEUP_CMD1(x)						(0x70+x*0x4)		// [0X70-0x80]
#define WAKEUP_CMD2(x)						(0x84+x*0x4)		// [0X84-0x94] 
#define WAKEUP_CTRL						0x98			// [0X98] 
#define WAKEUP_STS							0x9c			// [0X9C] 
#define IRFSM								0xa0			// [0Xa0] 
#define IRHSPMC								0xa4			// [0xa4] IR Host-Synchronous-Pulse Measure Counter register
#define IRHSPTC								0xa8			// [0xa8] IR Host-Synchronous-Pulse Tolerance Counter register

/* this structure should define in another header file 
  * located at mach-elite/include/mach/
  */
struct elitexxx_cir_platform_data {
	u32 wakeup;
};

#endif //__ELT_CIR_H__

