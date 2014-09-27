/*
 * linux/drivers/input/rmtctl/elite_rmtctl.c
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
#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <asm/mach-types.h>
#include <asm/irq.h>
#include <mach/types.h>
#include <mach/iomap.h>
#include <mach/wakeup_types.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "elite_rmtctl.h"

#define CIR_HZ 32768
#define CIR_clock_period (1*1000000000/CIR_HZ)
//#define APB_HZ 24000000
#define APB_HZ 200000000
#define APB_clock_period (1*1000000000/APB_HZ)

/* enable & status bits */
#define WAKEUP_SD3_CARD_DETECT_STAT_EN	(1UL<<30)
#define WAKEUP_SD0_CARD_DETECT_STAT_EN	(1UL<<29)
#define WAKEUP_CIR_STAT_EN		(1UL<<22)
#define WAKEUP_USB_DEV_ATTACH_STAT_EN	(1UL<<21)
#define WAKEUP_USB_HOST_STAT_EN			(1UL<<20)
#define WAKEUP_RTC_STAT_EN			(1UL<<15)
#define WAKEUP_POWER_BTN_STAT_EN		(1UL<<14)
#define WAKEUP_INTERNAL_IP3_STAT_EN		(1UL<<7)
#define WAKEUP_INTERNAL_IP2_STAT_EN		(1UL<<6)
#define WAKEUP_INTERNAL_IP1_STAT_EN		(1UL<<5)
#define WAKEUP_INTERNAL_IP0_STAT_EN		(1UL<<4)
#define WAKEUP_GPIO3_STAT_EN			(1UL<<3)
#define WAKEUP_GPIO2_STAT_EN			(1UL<<2)
#define WAKEUP_GPIO1_STAT_EN			(1UL<<1)
#define WAKEUP_GPIO0_STAT_EN			(1UL<<0)

enum CIR_TYPE {
    NEC = 0, // NEC / JVC / PZ_OCN
    PANASONIC,
    SONY,
    PHILIPS_RC5,
    PHILIPS_RC6,
    PHILIPS_RCMM,
};

enum mode {
    MODE_PZ_OCN,
    MODE_PHILIPS_RC5,
    MODE_NEC,
    MODE_JVC,
    MODE_PANASONIC,
    MODE_SONY,
    MODE_PHILIPS_RC6_20_BIT,
    MODE_PHILIPS_RC6_148_BIT,
    MODE_PHILIPS_RCMM_12_BIT,
    MODE_PHILIPS_RCMM_24_BIT,
    MODE_PHILIPS_RCMM_32_BIT,
};

enum NEC2GEN {
    NEC2GEN_NEC,
    NEC2GEN_PZ_OCN, //use NEC to run PZ_OCN.
    NEC2GEN_JVC, //use NEC to run JVC.
};

struct keycode_map {
    u32 hwcode;
    u32 keycode;
};

struct elite_cir_dev {
    struct device * dev;
    struct input_dev *idev;
    struct clk *clk;
    struct clk *ahbclk;
    struct keycode_map *kmap;
    unsigned long rate;
    unsigned long ahb_rate;
    int kmap_size;
    void __iomem *regs_base;
    int irq;
    //unsigned long volatile __jiffy_data prev_jiffies;
    /* CIR configuration */
    enum CIR_TYPE  irtype;
    u32 mod_using_nec2gen;
    u32 ir_nec_sub_f;
    u32 ir_nec_as_pz_ocn;
    u32 ir_rec_bits;
    u32 ir_rc6mod_msk;
    u32 ir_inv;
    u32 ir_sw_en_on;
    u32 en_rcv_more_bit;
    u32 en_rcv_less_bit;
    /* mode    : {1st_bit, 2nd_bit}
        * submode : {13th_bit, 14th_bit} 
        */
    u32 ir_rcmm_mod_msk; //set 1 to bit[?] to mask PHILIPS RCMM mode_? detection. (enable mode_1, disable mode_0,2,3)
    u32 ir_rcmm_mod00_3rd_to_12th_bit_chk; //set 1 to enable check customer ID
    u32 ir_rcmm_mod00_3rd_to_12th_bit_val; //set customer ID 
    u32 ir_rcmm_mod00_submode_msk;  //set 1 to bit[?] to mask PHILIPS RCMM submode_? detection. (disable submode_0,1,2,3)  
    u32 ir_rcmm_mod00_submode_addr_msk;  //set 1 to bit[?] to mask PHILIPS RCMM submode_* address_? detection. (disable submode_* address_0,1,2,3)
    /* Parameters */
    u32 REG_SynP;
    u32 REG_SynP_;
    u32 REG_SynP_Sub_;
    u32 REG_SynP_Subx_;
    u32 REG_T_REG_1T;
    u32 REG_T_REG_2T;
    u32 REG_X_REG_2T;
    u32 REG_NEC_REPEAT_TIMEOUT_EN;
    u32 REG_NEC_REPEAT_TIMEOUT_PERIOD_CNT;
    u32 REG_JVC_CONTI_CMD_EN;
    u32 REG_JVC_CONTI_CMD_PERIOD_CNT;
    u32 REG_NEC_2ND_HEADER_PREFIX_BIT_NUM;
    u32 REG_NEC_TO_DO_PZ_OCN;
    
};

//elite1k-520016c1-BreakSong-02 static int cir_mode = MODE_PZ_OCN;
//elite1k-520016c1-BreakSong-02-start
static int cir_mode = MODE_NEC;
//elite1k-520016c1-BreakSong-02-end
static int last_wakeup_type = 0;

static int wakeup_cfg = 0; /* 1: wakeup by any key; 0: depends on CMD */

static void __iomem *reg_cir_base;

static char *cir_proc_name="cir_mode";
static struct proc_dir_entry *cir_proc_entry;
static char *last_wakeup_proc_name="last_wakeup";
static struct proc_dir_entry *last_wakeup_proc_entry;
extern unsigned int last_wakeup_status;
static int wakeup_bits[] = {
    WAKEUP_GPIO0_STAT_EN,
    WAKEUP_GPIO1_STAT_EN,
    WAKEUP_GPIO2_STAT_EN,
    WAKEUP_GPIO3_STAT_EN,
    WAKEUP_INTERNAL_IP0_STAT_EN,
    WAKEUP_INTERNAL_IP1_STAT_EN,
    WAKEUP_INTERNAL_IP2_STAT_EN,
    WAKEUP_INTERNAL_IP3_STAT_EN,
    WAKEUP_POWER_BTN_STAT_EN,
    WAKEUP_RTC_STAT_EN,
    WAKEUP_USB_HOST_STAT_EN,
    WAKEUP_USB_DEV_ATTACH_STAT_EN,
    WAKEUP_CIR_STAT_EN,
    WAKEUP_SD0_CARD_DETECT_STAT_EN,
    WAKEUP_SD3_CARD_DETECT_STAT_EN,
};

#define cir_writel(value, reg) \
    __raw_writel(value, reg_cir_base + (reg))
#define cir_readl(reg) \
    __raw_readl(reg_cir_base + (reg))

#define atoi(str)  simple_strtoul(((str != NULL) ? str : ""), NULL, 0)

static struct keycode_map  pzocn_kmap[] = {
    {.hwcode = 0x0FF0, .keycode = 538,},//KEY_POWER
    {.hwcode = 0x0CF3, .keycode = 542,},//KEY_MUTE
    {.hwcode = 0x0DF2, .keycode = 518,},//KEY_VOLUMEUP
    {.hwcode = 0x2DD2, .keycode = 514,},//KEY_VOLUMEDOWN
    {.hwcode = 0x2FD0, .keycode = 530,},//KEY_CHANNELDOWN
    {.hwcode = 0x2ED1, .keycode = 534,},//KEY_CHANNELUP
    {.hwcode = 0x0EF1, .keycode = 522,},//KEY_EXIT
    {.hwcode = 0x0BF4, .keycode = 526,},//KEY_BACK
    {.hwcode = 0x1CE3, .keycode = 576,},//KEY_PAGEUP
    {.hwcode = 0x26D9, .keycode = 584,},//KEY_PAGEDOWN
    {.hwcode = 0x30CF, .keycode = 539,}, //KEY_PVR
    {.hwcode = 0x2AD5, .keycode = 535,},//KEY_PROGRAM
    {.hwcode = 0x2BD4, .keycode = 596,},//KEY_UP
    {.hwcode = 0x29D6, .keycode = 580,}, //KEY_LEFT
    {.hwcode = 0x28D7, .keycode = 604,}, //KEY_RIGHT
    {.hwcode = 0x25DA, .keycode = 592,}, //KEY_DOWN
    {.hwcode = 0x2CD3, .keycode = 588,},//KEY_OK
    {.hwcode = 0x1BE4, .keycode = 523,},//KEY_INFO
    {.hwcode = 0x423E, .keycode = 581,},//KEY_REWIND
    {.hwcode = 0x5A58, .keycode = 589,},//KEY_PLAY
    {.hwcode = 0x5C60, .keycode = 597,},//KEY_STOP
    {.hwcode = 0x324F, .keycode = 605,},//KEY_FASTFORWARD
    {.hwcode = 0x18E7, .keycode = 577,},//KEY_REFRESH
    {.hwcode = 0x2860, .keycode = 515,},//KEY_CONNECT
    {.hwcode = 0x24DB, .keycode = 543,},//KEY_FAVORITES
    {.hwcode = 0x8612, .keycode = 520,},//KEY_RECORD
    {.hwcode = 0x351A, .keycode = 519,},//KEY_CONFIG
    {.hwcode = 0x3378, .keycode = 512,},//KEY_FIND
    {.hwcode = 0x6512, .keycode = 600,},//KEY_HELP
    {.hwcode = 0x492A, .keycode = 517,},//KEY_SOUND
    {.hwcode = 0x16E9, .keycode = 527,},//KEY_TV
    {.hwcode = 0x17E8, .keycode = 531,},//KEY_AUDIO
    {.hwcode = 0x19E6, .keycode = 545,}, /*KEY_SD*/
    {.hwcode = 0x1AE5, .keycode = 546,}, /*KEY_HD*/
    {.hwcode = 0x21DE, .keycode = 585,},//KEY_1
    {.hwcode = 0x23DC, .keycode = 593,},//KEY_2
    {.hwcode = 0x22DD, .keycode = 601,},//KEY_3
    {.hwcode = 0x1DE2, .keycode = 525,},//KEY_4
    {.hwcode = 0x1EE1, .keycode = 533,},//KEY_5
    {.hwcode = 0x1FE0, .keycode = 541,},//KEY_6
    {.hwcode = 0x20DF, .keycode = 521,},//KEY_7
    {.hwcode = 0x11EE, .keycode = 529,},//KEY_8
    {.hwcode = 0x12ED, .keycode = 537,},//KEY_9
    {.hwcode = 0x14EB, .keycode = 524,},//"*"
    {.hwcode = 0x13EC, .keycode = 532,},//KEY_0
    {.hwcode = 0x15EA, .keycode = 540,}, //"#"
};

//elite1k-520016c1-BreakSong-02 static struct keycode_map  nec_kmap[] = {
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x1A, .keycode = 538,},//KEY_POWER
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x1E, .keycode = 542,},//KEY_MUTE
//elite1k-520016c1-BreakSong-02   {.hwcode = 0x06, .keycode = 518,},//KEY_VOLUMEUP
//elite1k-520016c1-BreakSong-02   {.hwcode = 0x02, .keycode = 514,},//KEY_VOLUMEDOWN
//elite1k-520016c1-BreakSong-02  {.hwcode = 0x12, .keycode = 530,},//KEY_CHANNELDOWN
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x16, .keycode = 534,},//KEY_CHANNELUP
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x0A, .keycode = 522,},//KEY_EXIT
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x0E, .keycode = 526,},//KEY_BACK
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x40, .keycode = 576,},//KEY_PAGEUP
//elite1k-520016c1-BreakSong-02   {.hwcode = 0x48, .keycode = 584,},//KEY_PAGEDOWN
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x1B, .keycode = 539,}, //KEY_PVR
//elite1k-520016c1-BreakSong-02     {.hwcode = 0x17, .keycode = 535,},//KEY_PROGRAM
//elite1k-520016c1-BreakSong-02     {.hwcode = 0x54, .keycode = 596,},//KEY_UP
//elite1k-520016c1-BreakSong-02     {.hwcode = 0x44, .keycode = 580,},//KEY_LEFT
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x5C, .keycode = 604,},//KEY_RIGHT
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x50, .keycode = 592,},//KEY_DOWN
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x4C, .keycode = 588,},//KEY_OK
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x0B, .keycode = 523,},//KEY_INFO
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x45, .keycode = 581,},//KEY_REWIND
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x4D, .keycode = 589,},//KEY_PLAY
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x55, .keycode = 597,},//KEY_STOP
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x5D, .keycode = 605,},//KEY_FASTFORWARD
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x41, .keycode = 577,},//KEY_REFRESH
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x03, .keycode = 515,},//KEY_CONNECT
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x1F, .keycode = 543,},//KEY_FAVORITES
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x08, .keycode = 520,},//KEY_RECORD
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x07, .keycode = 519,},//KEY_CONFIG
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x00, .keycode = 512,},//KEY_FIND
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x58, .keycode = 600,},//KEY_HELP
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x05, .keycode = 517,},//KEY_SOUND
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x0F, .keycode = 527,},//KEY_TV
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x13, .keycode = 531,},//KEY_AUDIO
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x21, .keycode = 545,}, /*KEY_SD*/
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x22, .keycode = 546,}, /*KEY_HD*/
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x49, .keycode = 585,},//KEY_1
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x51, .keycode = 593,},//KEY_2
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x59, .keycode = 601,},//KEY_3
//elite1k-520016c1-BreakSong-02   {.hwcode = 0x0D, .keycode = 525,},//KEY_4
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x15, .keycode = 533,},//KEY_5
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x1D, .keycode = 541,},//KEY_6
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x09, .keycode = 521,},//KEY_7
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x11, .keycode = 529,},//KEY_8
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x19, .keycode = 537,},//KEY_9
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x0C, .keycode = 524,},// "*"
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x14, .keycode = 532,},//KEY_0
//elite1k-520016c1-BreakSong-02    {.hwcode = 0x1C, .keycode = 540,}, //"#"
//elite1k-520016c1-BreakSong-02};

//elite1k-520016c1-BreakSong-02-start
static struct keycode_map  nec_kmap[] = {
    
    //{.hwcode = 0x00, .keycode = 0,},//KEY_RESERVED,      /* TOUR */
    //{.hwcode = 0x02, .keycode = 0,},//KEY_RESERVED,      /* IRIS+ */
    //{.hwcode = 0x05, .keycode = 0,},//KEY_RESERVED,      /* AUTO */
    //{.hwcode = 0x06, .keycode = 0,},//KEY_RESERVED,      /* RESERVED-16 */
    //{.hwcode = 0x0C, .keycode = 0,},//KEY_RESERVED,      /* LOCK */
    //{.hwcode = 0x41, .keycode = 0,},//KEY_RESERVED,      /* PIP */
    //{.hwcode = 0x4B, .keycode = 0,},//KEY_RESERVED,      /* +10 */
    //{.hwcode = 0x4D, .keycode = 0,},//KEY_RESERVED,      /* RESERVED -9 */
    //{.hwcode = 0x52, .keycode = 0,},//KEY_RESERVED,      /* OSD */
    //{.hwcode = 0x53, .keycode = 0,},//KEY_RESERVED,      /* PTZ */
    //{.hwcode = 0x54, .keycode = 0,},//KEY_RESERVED,      /* IRIS- */
    //{.hwcode = 0x55, .keycode = 0,},//KEY_RESERVED,      /* RESERVED -13 */
    //{.hwcode = 0x5D, .keycode = 0,},//KEY_RESERVED,      /* RESERVED -4 */
    {.hwcode = 0x4A, .keycode = 0x174,},//KEY_ZOOM,      /* ZOOM */
    {.hwcode = 0x4F, .keycode = 1,},//KEY_ESC            /* ESC  */
    {.hwcode = 0x1F, .keycode = 2,},//KEY_1 
    {.hwcode = 0x1C, .keycode = 3,},//KEY_2 
    {.hwcode = 0x1D, .keycode = 4,},//KEY_3 
    {.hwcode = 0x1A, .keycode = 5,},//KEY_4 
    {.hwcode = 0x59, .keycode = 6,},//KEY_5 
    {.hwcode = 0x40, .keycode = 7,},//KEY_6 
    {.hwcode = 0x16, .keycode = 8,},//KEY_7 
    {.hwcode = 0x46, .keycode = 9,},//KEY_8 
    {.hwcode = 0x19, .keycode = 10,},//KEY_9 
    {.hwcode = 0x0D, .keycode = 11,},//KEY_0
    {.hwcode = 0x5A, .keycode = 28,},//KEY_ENTER
    {.hwcode = 0x01, .keycode = 113,},//KEY_MUTE 
    {.hwcode = 0x5C, .keycode = 114,},//KEY_VOLUMEDOWN  
    {.hwcode = 0x5B, .keycode = 115,},//KEY_VOLUMEUP
    {.hwcode = 0x5F, .keycode = 139,},//KEY_MENU
    {.hwcode = 0x0B, .keycode = 167,},//KEY_RECORD       /* REC */
    {.hwcode = 0x5E, .keycode = 168,},//KEY_REWIND  
    {.hwcode = 0x57, .keycode = 207,},//KEY_PLAY 
    {.hwcode = 0x4E, .keycode = 208,},//KEY_FASTFORWARD
    
    //For android environment, we don't want have any reserved key, therefore, we define 59~71 keycode to use it.
    {.hwcode = 0x00, .keycode = 59,},//KEY_RESERVED,      /* TOUR */
    {.hwcode = 0x02, .keycode = 60,},//KEY_RESERVED,      /* IRIS+ */
    {.hwcode = 0x05, .keycode = 61,},//KEY_RESERVED,      /* AUTO */
    {.hwcode = 0x06, .keycode = 62,},//KEY_RESERVED,      /* RESERVED-16 */
    {.hwcode = 0x0C, .keycode = 63,},//KEY_RESERVED,      /* LOCK */
    {.hwcode = 0x41, .keycode = 64,},//KEY_RESERVED,      /* PIP */
    {.hwcode = 0x4B, .keycode = 65,},//KEY_RESERVED,      /* +10 */
    {.hwcode = 0x4D, .keycode = 66,},//KEY_RESERVED,      /* RESERVED -9 */
    {.hwcode = 0x52, .keycode = 67,},//KEY_RESERVED,      /* OSD */
    {.hwcode = 0x53, .keycode = 68,},//KEY_RESERVED,      /* PTZ */
    {.hwcode = 0x54, .keycode = 69,},//KEY_RESERVED,      /* IRIS- */
    {.hwcode = 0x55, .keycode = 70,},//KEY_RESERVED,      /* RESERVED -13 */
    {.hwcode = 0x5D, .keycode = 71,},//KEY_RESERVED,      /* RESERVED -4 */
         
};
//elite1k-520016c1-BreakSong-02-end 


static struct keycode_map  rc5_kmap[] = {
    {.hwcode = 0x10, .keycode = 538,},//KEY_POWER
    {.hwcode = 0x11, .keycode = 542,},//KEY_MUTE
    {.hwcode = 0x17, .keycode = 518,},//KEY_VOLUMEUP
    {.hwcode = 0x16, .keycode = 514,},//KEY_VOLUMEDOWN
    {.hwcode = 0x19, .keycode = 530,},//KEY_CHANNELDOWN
    {.hwcode = 0x1A, .keycode = 534,},//KEY_CHANNELUP
    {.hwcode = 0x14, .keycode = 522,},//KEY_EXIT
    {.hwcode = 0x15, .keycode = 526,},//KEY_BACK
    {.hwcode = 0x28, .keycode = 576,},//KEY_PAGEUP
    {.hwcode = 0x29, .keycode = 584,},//KEY_PAGEDOWN
    {.hwcode = 0x22, .keycode = 539,}, //KEY_PVR
    {.hwcode = 0x04, .keycode = 535,},//KEY_PROGRAM
    {.hwcode = 0x23, .keycode = 596,},//KEY_UP
    {.hwcode = 0x25, .keycode = 580,},//KEY_LEFT
    {.hwcode = 0x26, .keycode = 604,},//KEY_RIGHT
    {.hwcode = 0x24, .keycode = 592,},//KEY_DOWN
    {.hwcode = 0x27, .keycode = 588,},//KEY_OK
    {.hwcode = 0x20, .keycode = 523,},//KEY_INFO
    {.hwcode = 0x2C, .keycode = 581,},//KEY_REWIND
    {.hwcode = 0x2D, .keycode = 589,},//KEY_PLAY
    {.hwcode = 0x2E, .keycode = 597,},//KEY_STOP
    {.hwcode = 0x2F, .keycode = 605,},//KEY_FASTFORWARD
    {.hwcode = 0x2B, .keycode = 577,},//KEY_REFRESH
    {.hwcode = 0x21, .keycode = 515,},//KEY_CONNECT
    {.hwcode = 0x02, .keycode = 543,},//KEY_FAVORITES
    {.hwcode = 0x03, .keycode = 520,},//KEY_RECORD
    {.hwcode = 0x18, .keycode = 519,},//KEY_CONFIG
    {.hwcode = 0x12, .keycode = 512,},//KEY_FIND
    {.hwcode = 0x2A, .keycode = 600,},//KEY_HELP
    {.hwcode = 0x1C, .keycode = 517,},//KEY_SOUND
    {.hwcode = 0x05, .keycode = 527,},//KEY_TV
    {.hwcode = 0x06, .keycode = 531,},//KEY_AUDIO
    {.hwcode = 0x07, .keycode = 545,}, /*KEY_SD*/
    {.hwcode = 0x08, .keycode = 546,}, /*KEY_HD*/
    {.hwcode = 0x31, .keycode = 585,},//KEY_1
    {.hwcode = 0x32, .keycode = 593,},//KEY_2
    {.hwcode = 0x33, .keycode = 601,},//KEY_3
    {.hwcode = 0x34, .keycode = 525,},//KEY_4
    {.hwcode = 0x35, .keycode = 533,},//KEY_5
    {.hwcode = 0x36, .keycode = 541,},//KEY_6
    {.hwcode = 0x37, .keycode = 521,},//KEY_7
    {.hwcode = 0x38, .keycode = 529,},//KEY_8
    {.hwcode = 0x39, .keycode = 537,},//KEY_9
    {.hwcode = 0x09, .keycode = 524,},//"*"
    {.hwcode = 0x30, .keycode = 532,},//KEY_0
    {.hwcode = 0x0A, .keycode = 540,}, //"#"
};
#if 0
static struct keycode_map  pzocn_kmap[] = {
    .hwcode = 0x0FF0, .keycode = KEY_POWER,},
    .hwcode = 0x0CF3, .keycode = KEY_MUTE,},
    .hwcode = 0x0DF2, .keycode = KEY_VOLUMEUP,},
    .hwcode = 0x2DD2, .keycode = KEY_VOLUMEDOWN,},
    .hwcode = 0x2FD0, .keycode = KEY_CHANNELDOWN,},
    .hwcode = 0x2ED1, .keycode = KEY_CHANNELUP,},
    .hwcode = 0x0EF1, .keycode = KEY_EXIT,},
    .hwcode = 0x0BF4, .keycode = KEY_BACK,},
    .hwcode = 0x1CE3, .keycode = KEY_PAGEUP,},
    .hwcode = 0x26D9, .keycode = KEY_PAGEDOWN,},
    .hwcode = 0x30CF, .keycode = KEY_PVR,}, 
    .hwcode = 0x2AD5, .keycode = KEY_PROGRAM,},
    .hwcode = 0x2BD4, .keycode = KEY_UP,},
    .hwcode = 0x29D6, .keycode = KEY_LEFT,}, //?? diff from the OCN Spec
    .hwcode = 0x28D7, .keycode = KEY_RIGHT,}, //?? diff from the OCN Spec
    .hwcode = 0x25DA, .keycode = KEY_DOWN,}, //?? diff from the OCN Spec
    .hwcode = 0x2CD3, .keycode = KEY_OK,},
    .hwcode = 0x1BE4, .keycode = KEY_INFO,},
    .hwcode = 0x423E, .keycode = KEY_REWIND,},
    .hwcode = 0x5A58, .keycode = KEY_PLAY,},
    .hwcode = 0x5C60, .keycode = KEY_STOP,},
    .hwcode = 0x324F, .keycode = KEY_FASTFORWARD,},
    .hwcode = 0x18E7, .keycode = KEY_REFRESH,},
    .hwcode = 0x2860, .keycode = KEY_CONNECT,},
    .hwcode = 0x24DB, .keycode = KEY_FAVORITES,},
    .hwcode = 0x8612, .keycode = KEY_RECORD,},
    .hwcode = 0x351A, .keycode = KEY_CONFIG,},
    .hwcode = 0x3378, .keycode = KEY_FIND,},
    .hwcode = 0x6512, .keycode = KEY_HELP,},
    .hwcode = 0x492A, .keycode = KEY_SOUND,},
    .hwcode = 0x16E9, .keycode = KEY_TV,},
    .hwcode = 0x17E8, .keycode = KEY_AUDIO,},
    .hwcode = 0x19E6, .keycode = KEY_RESERVED,}, /*KEY_SD*/
    .hwcode = 0x1AE5, .keycode = KEY_RESERVED,}, /*KEY_HD*/
    .hwcode = 0x21DE, .keycode = KEY_1,},
    .hwcode = 0x23DC, .keycode = KEY_2,},
    .hwcode = 0x22DD, .keycode = KEY_3,},
    .hwcode = 0x1DE2, .keycode = KEY_4,},
    .hwcode = 0x1EE1, .keycode = KEY_5,},
    .hwcode = 0x1FE0, .keycode = KEY_6,},
    .hwcode = 0x20DF, .keycode = KEY_7,},
    .hwcode = 0x11EE, .keycode = KEY_8,},
    .hwcode = 0x12ED, .keycode = KEY_9,},
    .hwcode = 0x14EB, .keycode = KEY_KPASTERISK,},//?? diff from the ocn spec
    .hwcode = 0x13EC, .keycode = KEY_0,},//?? diff from the ocn spec
    .hwcode = 0x15EA, .keycode = KEY_KPJPCOMMA,}, //"#"
};

static struct keycode_map  nec_kmap[] = {
    .hwcode = 0x1A, .keycode = KEY_POWER,},
    .hwcode = 0x1E, .keycode = KEY_MUTE,},
    .hwcode = 0x06, .keycode = KEY_VOLUMEUP,},
    .hwcode = 0x02, .keycode = KEY_VOLUMEDOWN,},
    .hwcode = 0x12, .keycode = KEY_CHANNELDOWN,},
    .hwcode = 0x16, .keycode = KEY_CHANNELUP,},
    .hwcode = 0x0A, .keycode = KEY_EXIT,},
    .hwcode = 0x0E, .keycode = KEY_BACK,},
    .hwcode = 0x40, .keycode = KEY_PAGEUP,},
    .hwcode = 0x48, .keycode = KEY_PAGEDOWN,},
    .hwcode = 0x1B, .keycode = KEY_PVR,}, 
    .hwcode = 0x17, .keycode = KEY_PROGRAM,},
    .hwcode = 0x54, .keycode = KEY_UP,},
    .hwcode = 0x44, .keycode = KEY_LEFT,},
    .hwcode = 0x5C, .keycode = KEY_RIGHT,},
    .hwcode = 0x50, .keycode = KEY_DOWN,},
    .hwcode = 0x4C, .keycode = KEY_OK,},
    .hwcode = 0x0B, .keycode = KEY_INFO,},
    .hwcode = 0x45, .keycode = KEY_REWIND,},
    .hwcode = 0x4D, .keycode = KEY_PLAY,},
    .hwcode = 0x55, .keycode = KEY_STOP,},
    .hwcode = 0x5D, .keycode = KEY_FASTFORWARD,},
    .hwcode = 0x41, .keycode = KEY_REFRESH,},
    .hwcode = 0x03, .keycode = KEY_CONNECT,},
    .hwcode = 0x1F, .keycode = KEY_FAVORITES,},
    .hwcode = 0x08, .keycode = KEY_RECORD,},
    .hwcode = 0x07, .keycode = KEY_CONFIG,},
    .hwcode = 0x00, .keycode = KEY_FIND,},
    .hwcode = 0x58, .keycode = KEY_HELP,},
    .hwcode = 0x05, .keycode = KEY_SOUND,},
    .hwcode = 0x0F, .keycode = KEY_TV,},
    .hwcode = 0x13, .keycode = KEY_AUDIO,},
    .hwcode = 0x21, .keycode = KEY_RESERVED,}, /*KEY_SD*/
    .hwcode = 0x22, .keycode = KEY_RESERVED,}, /*KEY_HD*/
    .hwcode = 0x49, .keycode = KEY_1,},
    .hwcode = 0x51, .keycode = KEY_2,},
    .hwcode = 0x59, .keycode = KEY_3,},
    .hwcode = 0x0D, .keycode = KEY_4,},
    .hwcode = 0x15, .keycode = KEY_5,},
    .hwcode = 0x1D, .keycode = KEY_6,},
    .hwcode = 0x09, .keycode = KEY_7,},
    .hwcode = 0x11, .keycode = KEY_8,},
    .hwcode = 0x19, .keycode = KEY_9,},
    .hwcode = 0x0C, .keycode = KEY_KPASTERISK,},
    .hwcode = 0x14, .keycode = KEY_0,},
    .hwcode = 0x1C, .keycode = KEY_KPJPCOMMA,}, //"#"
};

static struct keycode_map  rc5_kmap[] = {
    .hwcode = 0x10, .keycode = KEY_POWER,},
    .hwcode = 0x11, .keycode = KEY_MUTE,},
    .hwcode = 0x17, .keycode = KEY_VOLUMEUP,},
    .hwcode = 0x16, .keycode = KEY_VOLUMEDOWN,},
    .hwcode = 0x19, .keycode = KEY_CHANNELDOWN,},
    .hwcode = 0x1A, .keycode = KEY_CHANNELUP,},
    .hwcode = 0x14, .keycode = KEY_EXIT,},
    .hwcode = 0x15, .keycode = KEY_BACK,},
    .hwcode = 0x28, .keycode = KEY_PAGEUP,},
    .hwcode = 0x29, .keycode = KEY_PAGEDOWN,},
    .hwcode = 0x22, .keycode = KEY_PVR,}, 
    .hwcode = 0x04, .keycode = KEY_PROGRAM,},
    .hwcode = 0x23, .keycode = KEY_UP,},
    .hwcode = 0x25, .keycode = KEY_LEFT,},
    .hwcode = 0x26, .keycode = KEY_RIGHT,},
    .hwcode = 0x24, .keycode = KEY_DOWN,},
    .hwcode = 0x27, .keycode = KEY_OK,},
    .hwcode = 0x20, .keycode = KEY_INFO,},
    .hwcode = 0x2C, .keycode = KEY_REWIND,},
    .hwcode = 0x2D, .keycode = KEY_PLAY,},
    .hwcode = 0x2E, .keycode = KEY_STOP,},
    .hwcode = 0x2F, .keycode = KEY_FASTFORWARD,},
    .hwcode = 0x2B, .keycode = KEY_REFRESH,},
    .hwcode = 0x21, .keycode = KEY_CONNECT,},
    .hwcode = 0x02, .keycode = KEY_FAVORITES,},
    .hwcode = 0x03, .keycode = KEY_RECORD,},
    .hwcode = 0x18, .keycode = KEY_CONFIG,},
    .hwcode = 0x12, .keycode = KEY_FIND,},
    .hwcode = 0x2A, .keycode = KEY_HELP,},
    .hwcode = 0x1C, .keycode = KEY_SOUND,},
    .hwcode = 0x05, .keycode = KEY_TV,},
    .hwcode = 0x06, .keycode = KEY_AUDIO,},
    .hwcode = 0x07, .keycode = KEY_RESERVED,}, /*KEY_SD*/
    .hwcode = 0x08, .keycode = KEY_RESERVED,}, /*KEY_HD*/
    .hwcode = 0x31, .keycode = KEY_1,},
    .hwcode = 0x32, .keycode = KEY_2,},
    .hwcode = 0x33, .keycode = KEY_3,},
    .hwcode = 0x34, .keycode = KEY_4,},
    .hwcode = 0x35, .keycode = KEY_5,},
    .hwcode = 0x36, .keycode = KEY_6,},
    .hwcode = 0x37, .keycode = KEY_7,},
    .hwcode = 0x38, .keycode = KEY_8,},
    .hwcode = 0x39, .keycode = KEY_9,},
    .hwcode = 0x09, .keycode = KEY_KPASTERISK,},
    .hwcode = 0x30, .keycode = KEY_0,},
    .hwcode = 0x0A, .keycode = KEY_KPJPCOMMA,}, //"#"
};
#endif

#if  0
/* Media AD Player: key[0x00 - 0x1f] */
static struct keycode_map  adplayer_kmap[] = {
    .hwcode = 0, .keycode = KEY_POWER,},
    .hwcode = 1, .keycode = KEY_RESERVED,},
    .hwcode = 2, .keycode = KEY_RESERVED,},
    .hwcode = 3, .keycode = KEY_MUTE,},
    .hwcode = 4, .keycode = KEY_CLEAR,},
    .hwcode = 5, .keycode = KEY_UP,},
    .hwcode = 6, .keycode = KEY_ESC,},
    .hwcode = 7, .keycode = KEY_SCREEN,}, /* P/N */
    .hwcode = 8, .keycode = KEY_LEFT,},
    .hwcode = 9, .keycode = KEY_ENTER,},
    .hwcode = 10, .keycode = KEY_RIGHT,},
    .hwcode = 11, .keycode = KEY_SETUP,},
    .hwcode = 12, .keycode = KEY_F1,},
    .hwcode = 13, .keycode = KEY_DOWN,},
    .hwcode = 14, .keycode = KEY_F2,},
    .hwcode = 15, .keycode = KEY_STOP,},
    .hwcode = 16, .keycode = KEY_1,},
    .hwcode = 17, .keycode = KEY_2,},
    .hwcode = 18, .keycode = KEY_3,},
    .hwcode = 19, .keycode = KEY_TIME,},
    .hwcode = 20, .keycode = KEY_4,},
    .hwcode = 21, .keycode = KEY_5,},
    .hwcode = 22, .keycode = KEY_6,},
    .hwcode = 23, .keycode = KEY_PLAYPAUSE,},
    .hwcode = 24, .keycode = KEY_7,},
    .hwcode = 25, .keycode = KEY_8,},
    .hwcode = 26, .keycode = KEY_9,},
    .hwcode = 27, .keycode = KEY_VOLUMEUP,},
    .hwcode = 28, .keycode = KEY_0,},
    .hwcode = 29, .keycode = KEY_BACK,},
    .hwcode = 30, .keycode = KEY_FORWARD,},
    .hwcode = 31, .keycode = KEY_VOLUMEDOWN,},
};

/* VOS remote: key[0x00 - 0x1f] */
static struct keycode_map  vosremote_kmap[] = {
    .hwcode = 0, .keycode = KEY_RESERVED,},/* KARAOKE */
    .hwcode = 1, .keycode = KEY_RESERVED,},/* FUN- */
    .hwcode = 2, .keycode = KEY_RESERVED,},/* ANGLE */
    .hwcode = 3, .keycode = KEY_VOLUMEDOWN,},
    .hwcode = 4, .keycode = KEY_CLEAR,},
    .hwcode = 5, .keycode = KEY_0,},
    .hwcode = 6, .keycode = KEY_F1,},/* DIGEST */
    .hwcode = 7, .keycode = KEY_ZOOM,}, 
    .hwcode = 8, .keycode = KEY_7,},
    .hwcode = 9, .keycode = KEY_8,},
    .hwcode = 10, .keycode = KEY_NEXT,},
    .hwcode = 11, .keycode = KEY_VOLUMEUP,},
    .hwcode = 12, .keycode = KEY_4,},
    .hwcode = 13, .keycode = KEY_5,},
    .hwcode = 14, .keycode = KEY_POWER,},
    .hwcode = 15, .keycode = KEY_MUTE,},
    .hwcode = 16, .keycode = KEY_1,},
    .hwcode = 17, .keycode = KEY_2,},
    .hwcode = 18, .keycode = KEY_SUBTITLE,},
    .hwcode = 19, .keycode = KEY_RESERVED,},/* RETURN */
    .hwcode = 20, .keycode = KEY_RECORD,},
    .hwcode = 21, .keycode = KEY_RESERVED,},/* STEP */
    .hwcode = 22, .keycode = KEY_RESERVED,},/* A-B */
    .hwcode = 23, .keycode = KEY_RESERVED,},/* STEP B*/
    .hwcode = 24, .keycode = KEY_BACK,},
    .hwcode = 25, .keycode = KEY_PLAY,},
    .hwcode = 26, .keycode = KEY_EJECTCD,},
    .hwcode = 27, .keycode = KEY_RESERVED,},/* FF */
    .hwcode = 28, .keycode = KEY_LEFT,},
    .hwcode = 29, .keycode = KEY_DOWN,},
    .hwcode = 30, .keycode = KEY_F2,},/* Menu/PBC */
    .hwcode = 31, .keycode = KEY_PLAYPAUSE,},/* SF */
    .hwcode = 64, .keycode = KEY_AUDIO,},
    .hwcode = 65, .keycode = KEY_SETUP,},
    .hwcode = 66, .keycode = KEY_RESERVED,},/* FUN+ */
    .hwcode = 67, .keycode = KEY_RESERVED,},/* MARK */
    .hwcode = 68, .keycode = KEY_UP,},
    .hwcode = 69, .keycode = KEY_RESERVED,},/* +10 */
    .hwcode = 70, .keycode = KEY_RESERVED,},/* INVALID */
    .hwcode = 71, .keycode = KEY_RESERVED,},/* SURR */
    .hwcode = 72, .keycode = KEY_RIGHT,},
    .hwcode = 73, .keycode = KEY_9,},
    .hwcode = 74, .keycode = KEY_RESERVED,},/* INVALID */
    .hwcode = 75, .keycode = KEY_RESERVED,},/* VOCAL */
    .hwcode = 76, .keycode = KEY_TV,},
    .hwcode = 77, .keycode = KEY_6,},
    .hwcode = 78, .keycode = KEY_RESERVED,},/* INVALID */
    .hwcode = 79, .keycode = KEY_PROGRAM,},/* PROG */
    .hwcode = 80, .keycode = KEY_RESERVED,},/* DISPLAY */
    .hwcode = 81, .keycode = KEY_3,},
    .hwcode = 82, .keycode = KEY_RESERVED,},/* INVALID */
    .hwcode = 83, .keycode = KEY_RESERVED,},/* INVALID */
    .hwcode = 84, .keycode = KEY_GOTO,},
    .hwcode = 85, .keycode = KEY_PREVIOUS,},/* Prev/ASV- */
    .hwcode = 86, .keycode = KEY_RESERVED,},/* INVALID */
    .hwcode = 87, .keycode = KEY_RESERVED,},/* INVALID */
    .hwcode = 88, .keycode = KEY_RESERVED,},/* Repeat */
    .hwcode = 89, .keycode = KEY_STOP,},
    .hwcode = 90, .keycode = KEY_RESERVED,},/* INVALID */
    .hwcode = 91, .keycode = KEY_RESERVED,},/* INVALID */
    .hwcode = 92, .keycode = KEY_ENTER,},
    .hwcode = 93, .keycode = KEY_TITLE,},
};
#endif

static int elitexxx_cir_lookup(struct elite_cir_dev *cirdev, unsigned int hwcode)
{
    int i;

    for (i = 0; i < cirdev->kmap_size; i++)
        if (cirdev->kmap[i].hwcode == hwcode)
            return i;

    return -1;
}

static irqreturn_t elitexxx_cir_interrupt(int irq, void *dev_id)
{
    struct elite_cir_dev *cirdev = (struct elite_cir_dev *)dev_id;
    int index;
    unsigned int status, hwcode;
    unsigned int  rd_data1 = 0;
    unsigned int  rd_data2 = 0;
    unsigned int  rd_data3 = 0;
    unsigned int  rd_data4 = 0;
    unsigned int  rd_data5 = 0;

    unsigned int bit_num;
    unsigned int nec_repeat;

    /* Get IR status. */
    status = cir_readl(IRSTS);

    /* Check 'IR received data' flag. */
    if (!(status & 0x1))
        return IRQ_NONE;

    cir_writel(0x1, IRSTS);// CLEAR CIR_INTRQ

    bit_num = ( status >> 8) & 0xFF;
    nec_repeat = ( status >> 4) & 0x1;

    rd_data1 = cir_readl(IRDATA(0));
    rd_data2 = cir_readl(IRDATA(1));
    rd_data3 = cir_readl(IRDATA(2));
    rd_data4 = cir_readl(IRDATA(3));
    rd_data5 = cir_readl(IRDATA(4));

    pr_debug ("states=%x,CIR Receive Bit Count: %u\n",status,  bit_num );

    if ( cirdev->irtype == PHILIPS_RCMM ) {
        if ( bit_num == 12 )
            pr_debug ("CIR Detect PHILIPS_RCMM : mode_%d \n",( ( rd_data1 >> 10 ) & 0x3 ) );
        else if ( bit_num == 24 )
            pr_debug ("CIR Detect PHILIPS_RCMM : mode_%d , submode_%d \n",( ( rd_data1 >> 22 ) & 0x3 ) , ( ( rd_data1 >> 10 ) & 0x3 ) );
        else if ( bit_num == 32 )
            pr_debug ("CIR Detect PHILIPS_RCMM : mode_%d , submode_%d \n",( ( rd_data1 >> 30 ) & 0x3 ) , ( ( rd_data1 >> 18 ) & 0x3 ) );
    }

    if ( bit_num != cirdev->ir_rec_bits) {
        if ( (( cirdev->irtype == PHILIPS_RC5 ) && ((bit_num < cirdev->ir_rec_bits) && 
            (cirdev->en_rcv_less_bit == 1)))||((bit_num>cirdev->ir_rec_bits) && (cirdev->en_rcv_more_bit==1))) {
            pr_warn("***Warnning: CIR Receive Bit Count != %u\n", cirdev->ir_rec_bits);
        } else {
            pr_err("***ERROR: CIR Receive Bit Count != %u\n", 
            cirdev->ir_rec_bits);
        }
        return IRQ_HANDLED;
    }

    if ( nec_repeat ) {
        if (cirdev->mod_using_nec2gen == NEC2GEN_PZ_OCN)
            pr_debug ("1st CIR CODE is PZ_OCN REPEAT. Ignor it.\n");
        else
            pr_debug ("1st CIR CODE is NEC REPEAT. Ignor it.\n");

    }

    pr_debug ("Read CIR Receive Data Register :\n");
    pr_debug ("CIR Receive Data is: 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n", 
        rd_data5 , rd_data4 , rd_data3 , rd_data2 , rd_data1 );

    switch(cirdev->irtype)
    {
        case NEC :
            if (cirdev->ir_nec_as_pz_ocn)
            {
                if ((rd_data1 & 0xffffff) != 0x4E434F)
                {
                    pr_err("OCN user code error: received 0x%06x\n",(rd_data1 & 0xffffff));
                    return IRQ_HANDLED;
                }
                    hwcode = ((rd_data1>>24)<<8)|(rd_data2&0xff);
            }
            else
            {
                u8 not_keycode     = (rd_data1>>24) & 0xff;
                u8 keycode = (rd_data1>>16) & 0xff;
                if ((keycode ^ not_keycode) != 0xff) {
                    pr_err("NEC checksum error: received 0x%08x\n",rd_data1);
                return IRQ_HANDLED;
            }           
                     hwcode = ((rd_data1>>16)&0xff);
                 }
            break;
        case PHILIPS_RC5:
            hwcode = (rd_data1&0x3f);
            break;
        default:
            pr_err ("This protocol does not support now!\n");
            return IRQ_HANDLED;
    }

    index = elitexxx_cir_lookup(cirdev, hwcode);
    //elite1k-520016c1-BreakSong-02 if ((index >= 0) && (index < (sizeof(pzocn_kmap)/sizeof(pzocn_kmap[0])))) {
    //elite1k-520016c1-BreakSong-02-start
    if ((index >= 0) && (index < (sizeof(nec_kmap)/sizeof(nec_kmap[0])))) {
    //elite1k-520016c1-BreakSong-02-end
        pr_debug("IR report key %d\n" , cirdev->kmap[index].keycode);
        input_report_key(cirdev->idev, cirdev->kmap[index].keycode, 1);
        input_report_key(cirdev->idev, cirdev->kmap[index].keycode, 0);
        input_sync(cirdev->idev);
    }
    else
    {
        pr_err("Keymap lookup index out of range!!!\n");
    }

    return IRQ_HANDLED;
}

static void elitexxx_cir_hw_suspend(struct elite_cir_dev *cirdev)
{
    /* using power key as wake up source */
    if (cir_mode == MODE_PZ_OCN)
    {	
        cir_writel(0x0f4e434f, WAKEUP_CMD1(0));
        cir_writel(0x000000f0, WAKEUP_CMD1(1));
    }
    else if (cir_mode == MODE_PHILIPS_RC5)
    {   
        cir_writel(0x00000590, WAKEUP_CMD1(0));
        cir_writel(0x00000000, WAKEUP_CMD1(1));
    }
    else if(cir_mode == MODE_NEC)
    {   
        cir_writel(0xe51a7f00, WAKEUP_CMD1(0));
        cir_writel(0x00000000, WAKEUP_CMD1(1));
    }
    else 
    {
        pr_err("Elite cir_mode is error!!!\n");   
    }

    cir_writel(0x0, WAKEUP_CMD1(2));
    cir_writel(0x0, WAKEUP_CMD1(3));
    cir_writel(0x0, WAKEUP_CMD1(4));
    cir_writel(0xff00ff00, WAKEUP_CMD2(0));
    cir_writel(0x0, WAKEUP_CMD2(1));
    cir_writel(0x0, WAKEUP_CMD2(2));
    cir_writel(0x0, WAKEUP_CMD2(3));
    cir_writel(0x0, WAKEUP_CMD2(4));   
    if (wakeup_cfg)
        cir_writel(0x001, WAKEUP_CTRL);
    else
        cir_writel(0x101, WAKEUP_CTRL);  
}

static void elitexxx_cir_debug_print(struct elite_cir_dev *cirdev)
{
    pr_debug("REG_SynP = 0x%x\n", cirdev->REG_SynP);
    pr_debug("REG_SynP_= 0x%x\n", cirdev->REG_SynP_);
    pr_debug("REG_SynP_Sub_ = 0x%x\n", cirdev->REG_SynP_Sub_);
    pr_debug("REG_SynP_Subx_ = 0x%x\n", cirdev->REG_SynP_Subx_);
    pr_debug("REG_T_REG_1T = 0x%x\n", cirdev->REG_T_REG_1T);
    pr_debug("REG_T_REG_2T = 0x%x\n", cirdev->REG_T_REG_2T);
    pr_debug("REG_X_REG_2T = 0x%x\n", cirdev->REG_X_REG_2T);
    pr_debug("REG_NEC_REPEAT_TIMEOUT_EN = 0x%x\n", 
        cirdev->REG_NEC_REPEAT_TIMEOUT_EN);
    pr_debug("REG_NEC_REPEAT_TIMEOUT_PERIOD_CNT = 0x%x\n", 
        cirdev->REG_NEC_REPEAT_TIMEOUT_PERIOD_CNT);
    pr_debug("REG_JVC_CONTI_CMD_EN = 0x%x\n", 
        cirdev->REG_JVC_CONTI_CMD_EN);
    pr_debug("REG_JVC_CONTI_CMD_PERIOD_CNT = 0x%x\n", 
        cirdev->REG_JVC_CONTI_CMD_PERIOD_CNT);
    pr_debug("REG_NEC_2ND_HEADER_PREFIX_BIT_NUM = 0x%x\n", 
        cirdev->REG_NEC_2ND_HEADER_PREFIX_BIT_NUM);

    //------------------------------------
    // READ PARAMETERS
    //------------------------------------
    pr_debug ("SynP x%08x : Set to 0x%08x \n", 
        PARAMETER(0) , cir_readl ( PARAMETER(0) ));
    pr_debug ("SynP_ 0x%08x : Set to 0x%08x \n", 
        PARAMETER(1) , cir_readl ( PARAMETER(1) ));
    pr_debug ("SynP_Sub_ 0x%08x : Set to 0x%08x \n", 
        PARAMETER(2) , cir_readl ( PARAMETER(2) ));
    pr_debug ("SynP_Subx_ 0x%08x : Set to 0x%08x \n", 
        PARAMETER(3) , cir_readl ( PARAMETER(3) ));
    pr_debug ("T_REG_1T 0x%08x : Set to 0x%08x \n", 
        PARAMETER(4) , cir_readl ( PARAMETER(4) ));
    pr_debug ("T_REG_2T 0x%08x : Set to 0x%08x \n", 
        PARAMETER(5) , cir_readl ( PARAMETER(5) ));
    pr_debug ("X_REG_2T 0x%08x : Set to 0x%08x \n", 
        PARAMETER(6) , cir_readl ( PARAMETER(6) ));
    pr_debug ("NEC_REPEAT_TIMEOUT_CTRL 0x%08x : Set to 0x%08x \n", 
        NEC_REPEAT_TIME_OUT_CTRL, cir_readl(NEC_REPEAT_TIME_OUT_CTRL));
    pr_debug ("NEC_REPEAT_TIMEOUT_PERIOD_CNT 0x%08x : Set to 0x%08x \n", 
        NEC_REPEAT_TIME_OUT_COUNT , cir_readl( NEC_REPEAT_TIME_OUT_COUNT ));
    pr_debug ("NEC_2ND_HEADER_PREFIX_BIT_NUM 0x%08x : Set to 0x%08x \n", 
        NEC_2ND_HEADER_PREFIX_BIT_NUM , cir_readl( NEC_2ND_HEADER_PREFIX_BIT_NUM ));
    pr_debug ("JVC_CONTI_CMD_CTRL 0x%08x : Set to 0x%08x \n", 
        JVC_CONTI_CTRL, cir_readl( JVC_CONTI_CTRL ));
    pr_debug ("JVC_CONTI_CMD_PERIOD_CNT 0x%08x : Set to 0x%08x \n", 
        JVC_CONTI_CNT , cir_readl( JVC_CONTI_CNT ));
    pr_debug ("INTRQ_MASK_CTRL 0x%08x : Set to 0x%08x \n", 
        INT_MASK_CTRL, cir_readl( INT_MASK_CTRL ));
    pr_debug ("INTRQ_MASK_PERIOD_CNT 0x%08x : Set to 0x%08x \n", 
        INT_MASK_COUNT , cir_readl( INT_MASK_COUNT ));

    if ( cirdev->irtype == PHILIPS_RC6 )
        pr_debug ("IR_PHILIPS_RC6MODE_MASK = 0x%08x\n", 
        cirdev->ir_rc6mod_msk);
    else if ( cirdev->irtype == PHILIPS_RCMM ) {
        pr_debug ("IR_PHILIPS_RCMM_MODE_MASK = 0x%04x\n", 
            cirdev->ir_rcmm_mod_msk);
        pr_debug ("IR_PHILIPS_RCMM_MODE00_SUBMODE_MASK = 0x%04x\n", 
            cirdev->ir_rcmm_mod00_submode_msk);
    }
  
    pr_debug ("IR_CTRL2 0x%08x : Set to 0x%08x\n", 
        IRCTL_2, cir_readl(IRCTL_2));
    pr_debug ("IR_CTRL   0x%08x : Set to 0x%08x\n", 
        IRCTL, cir_readl(IRCTL));
}


static int elitexxx_cir_config(struct elite_cir_dev *cirdev, int mode)
{
    cirdev->kmap = NULL;
    cirdev->ir_inv = 0;
    cirdev->ir_sw_en_on = 1;
    cirdev->ir_nec_as_pz_ocn = 0x0;
    cirdev->ir_nec_sub_f = 0x1;
    cirdev->mod_using_nec2gen = NEC2GEN_NEC;

    cirdev->ir_rcmm_mod_msk = ~0xF;
    cirdev->ir_rcmm_mod00_3rd_to_12th_bit_chk = 0x0;
    cirdev->ir_rcmm_mod00_3rd_to_12th_bit_val = 0x0;
    cirdev->ir_rcmm_mod00_submode_msk = ~0xF;
    cirdev->ir_rcmm_mod00_submode_addr_msk = ~0xF;

    switch (mode) {
    /* fall though */
    case MODE_NEC:
    case MODE_PZ_OCN:
    case MODE_JVC:
        cirdev->irtype = NEC;
        if (MODE_PZ_OCN == mode) {
            cirdev->mod_using_nec2gen = NEC2GEN_PZ_OCN;
            cirdev->ir_nec_as_pz_ocn = 0x1;
            cirdev->ir_rec_bits = 40;
            cirdev->kmap = pzocn_kmap;
            cirdev->kmap_size = ARRAY_SIZE(pzocn_kmap);
        } else if (MODE_JVC == mode) {
            cirdev->mod_using_nec2gen = NEC2GEN_JVC;
            cirdev->ir_nec_sub_f = 0x0;
            cirdev->ir_rec_bits = 16;
        } else if (MODE_NEC == mode) {
            cirdev->ir_rec_bits = 32;
            cirdev->kmap = nec_kmap;
            cirdev->kmap_size = ARRAY_SIZE(nec_kmap);
        }
        break;
    case MODE_PANASONIC:
        cirdev->irtype = PANASONIC;
        cirdev->ir_rec_bits = 48;
        break;
    case MODE_SONY:
        cirdev->irtype = SONY;
        cirdev->ir_rec_bits = 12;
        break;
    case MODE_PHILIPS_RC5:
        cirdev->irtype = PHILIPS_RC5;
        cirdev->ir_rec_bits = 13;
        cirdev->kmap = rc5_kmap;
        cirdev->kmap_size = ARRAY_SIZE(rc5_kmap);
        break;
    case MODE_PHILIPS_RC6_20_BIT:
    case MODE_PHILIPS_RC6_148_BIT:
        cirdev->irtype = PHILIPS_RC6;
        cirdev->ir_rc6mod_msk = 0;
        if (MODE_PHILIPS_RC6_20_BIT == mode) {
            cirdev->ir_rec_bits = 20;
            cirdev->ir_rc6mod_msk = 0xbe;
        } else if(MODE_PHILIPS_RC6_148_BIT == mode) {
            cirdev->ir_rec_bits = 148;
            cirdev->ir_rc6mod_msk = 0xBF;
        }
        break;
    case MODE_PHILIPS_RCMM_12_BIT:
    case MODE_PHILIPS_RCMM_24_BIT:
    case MODE_PHILIPS_RCMM_32_BIT:
        cirdev->irtype = PHILIPS_RCMM;
        if (mode == MODE_PHILIPS_RCMM_12_BIT) {
            cirdev->ir_rec_bits = 12;
            cirdev->ir_rcmm_mod_msk = ~0x2;
            cirdev->ir_rcmm_mod00_3rd_to_12th_bit_chk = 0x0;
            cirdev->ir_rcmm_mod00_3rd_to_12th_bit_val = 0x0;
            cirdev->ir_rcmm_mod00_submode_msk = ~0x0;
            cirdev->ir_rcmm_mod00_submode_addr_msk = ~0x0;
        } else if(mode == MODE_PHILIPS_RCMM_24_BIT) {
            cirdev->ir_rec_bits = 24;
            cirdev->ir_rcmm_mod_msk = ~0x1;
            cirdev->ir_rcmm_mod00_3rd_to_12th_bit_chk = 0x1;
            cirdev->ir_rcmm_mod00_3rd_to_12th_bit_val = 0x3C6;
            cirdev->ir_rcmm_mod00_submode_msk = ~0x2;
            cirdev->ir_rcmm_mod00_submode_addr_msk = ~0x4;
        } else if(mode == MODE_PHILIPS_RCMM_32_BIT) {
            cirdev->ir_rec_bits = 32;
            cirdev->ir_rcmm_mod_msk = ~0x1;
            cirdev->ir_rcmm_mod00_3rd_to_12th_bit_chk = 0x1;
            cirdev->ir_rcmm_mod00_3rd_to_12th_bit_val = 0x293;
            cirdev->ir_rcmm_mod00_submode_msk = ~0x8;
            cirdev->ir_rcmm_mod00_submode_addr_msk = ~0x1;
        }
        break;
    default:
        pr_err("Unknown CIR mode\n");
        return -1;
    }

    return 0;
}

static inline void elite_cir_sw_reset(void)
{
    cir_writel(1, IRSWRST);
    cir_writel(0, IRSWRST);  
}

static void elitexxx_cir_hw_init(struct elite_cir_dev *cirdev)
{
    unsigned int wrdata;
    unsigned int intr_mask_period = 0 ;

    elitexxx_cir_config(cirdev, cir_mode);
    
    if ( cirdev->irtype == NEC ) {
        if (cirdev->mod_using_nec2gen == NEC2GEN_PZ_OCN) {
            pr_debug("\n* Remote : CIR PZ_OCN *\n");
            /* ( ( 3.64 * 1000000 / CIR_clock_period ) * ( 6 / 6.5 ) ) */
            //cirdev->REG_SynP =  3360000/CIR_clock_period;
            cirdev->REG_SynP =  3360*cirdev->rate/1000000;
            /* ( ( 1.80 * 1000000 / CIR_clock_period ) * ( 2.7 / 3.2 ) ) */
            cirdev->REG_SynP_=  1518*cirdev->rate/1000000;
            /* ( ( 3.64 * 1000000 / CIR_clock_period ) * ( 6 / 6.5 ) ) */
            //cirdev->REG_SynP_Sub_=  3360000/CIR_clock_period;
            cirdev->REG_SynP_Sub_=  3360*cirdev->rate/1000000;
            /* ( ( 3.64 * 1000000 / CIR_clock_period ) * ( 7 / 6.5 ) ) */
            cirdev->REG_SynP_Subx_ =  3920*cirdev->rate/1000000;
            /* ( ( 0.56 * 1000000 / CIR_clock_period ) * ( 0.5 )) */
            cirdev->REG_T_REG_1T = 280*cirdev->rate/1000000;
            /* ( ( 0.56 * 1000000 / CIR_clock_period ) * ( 1 )) */
            cirdev->REG_T_REG_2T = 560*cirdev->rate/1000000;
            /* ( ( 0.56 * 1000000 / CIR_clock_period ) * ( 1 )) */
            cirdev->REG_X_REG_2T = 560*cirdev->rate/1000000;

            cirdev->REG_NEC_REPEAT_TIMEOUT_EN = 1 ;
            /* ( 108 * 1000000 / APB_clock_period ) */
            cirdev->REG_NEC_REPEAT_TIMEOUT_PERIOD_CNT = 1079*2*(cirdev->ahb_rate/10000);
            cirdev->REG_JVC_CONTI_CMD_EN = 0 ;
            cirdev->REG_NEC_2ND_HEADER_PREFIX_BIT_NUM = 24 ;
            cirdev->REG_NEC_TO_DO_PZ_OCN = 1 ;
        } else if (cirdev->mod_using_nec2gen == NEC2GEN_JVC) {
            pr_debug("\n* Remote : CIR JVC *\n");
            /* ( ( 8.4400 * 1000000 / CIR_clock_period ) * ( 15.5 / 16 ) ) */
            cirdev->REG_SynP = 8130*cirdev->rate/1000000;
            /* ( ( 4.2200 * 1000000 / CIR_clock_period ) * ( 7.5 / 8 ) ) */
            cirdev->REG_SynP_=  3956*cirdev->rate/1000000;
            /* ( ( 0.5275 * 1000000 / CIR_clock_period ) * ( 0.5 )) */
            cirdev->REG_T_REG_1T = 263*cirdev->rate/1000000;
            /* ( ( 0.5275 * 1000000 / CIR_clock_period ) * ( 1 )) */
            cirdev->REG_T_REG_2T = 527*cirdev->rate/1000000;
            /* ( ( 0.5275 * 1000000 / CIR_clock_period ) * ( 1 )) */
            cirdev->REG_X_REG_2T = 527*cirdev->rate/1000000;
            cirdev->REG_NEC_REPEAT_TIMEOUT_EN = 0 ;
            /* ( 46.42 * 1000000 / CIR_clock_period ) */
            cirdev->REG_NEC_REPEAT_TIMEOUT_PERIOD_CNT = 46420*cirdev->rate/1000000;
            cirdev->REG_JVC_CONTI_CMD_EN = 1 ;
            /* ( 46.42 * 1000000 / CIR_clock_period ) */
            cirdev->REG_JVC_CONTI_CMD_PERIOD_CNT = 46420*cirdev->rate/1000000;
        } else { //NEC
            pr_debug("\n* Remote : CIR NEC *\n");
            /* ( ( 8.993 * 1000000 / CIR_clock_period ) * ( 14.0 / 15.5 ) ) */
            cirdev->REG_SynP = 8122*cirdev->rate/1000000;
            /* ( ( 4.642 * 1000000 / CIR_clock_period ) * ( 7.5 / 8 ) ) */
            cirdev->REG_SynP_= 4351*cirdev->rate/1000000;
            /* ( ( 2.321 * 1000000 / CIR_clock_period ) * ( 3.5 / 4 ) ) */
            cirdev->REG_SynP_Sub_= 2030*cirdev->rate/1000000;
            /* ( ( 2.321 * 1000000 / CIR_clock_period ) * ( 4.5 / 4 ) ) */
            cirdev->REG_SynP_Subx_ = 2611*cirdev->rate/1000000;
            /* ( ( 0.580 * 1000000 / CIR_clock_period ) * ( 0.5 )) */
            cirdev->REG_T_REG_1T = 290*cirdev->rate/1000000;
            /* ( ( 0.580 * 1000000 / CIR_clock_period ) * ( 1 )) */
            cirdev->REG_T_REG_2T = 580*cirdev->rate/1000000;
            /* ( ( 0.580 * 1000000 / CIR_clock_period ) * ( 1 )) */
            cirdev->REG_X_REG_2T = 580*cirdev->rate/1000000;
            cirdev->REG_NEC_REPEAT_TIMEOUT_EN = 1 ;
            /* ( 107.9 * 1000000 / APB_clock_period ) */
            cirdev->REG_NEC_REPEAT_TIMEOUT_PERIOD_CNT = 1079*2*(cirdev->ahb_rate/10000);
            cirdev->REG_JVC_CONTI_CMD_EN = 0 ;
        }
    }
    else if (cirdev->irtype == PANASONIC) {
        pr_debug("\n* Remote : CIR PANASONIC *\n");
        /* ( ( 3.56 * 1000000 / CIR_clock_period ) * ( 15.0 / 16 ) ) */
        cirdev->REG_SynP =  3337*cirdev->rate/1000000;
        /* ( ( 1.72 * 1000000 / CIR_clock_period ) * ( 7.0 / 8 ) ) */
        cirdev->REG_SynP_= 1505*cirdev->rate/1000000;
        /* ( ( 0.44 * 1000000 / CIR_clock_period ) * ( 0.5 )) */
        cirdev->REG_T_REG_1T = 220*cirdev->rate/1000000;
        /* ( ( 0.44 * 1000000 / CIR_clock_period ) * ( 1 )) */
        cirdev->REG_T_REG_2T = 440*cirdev->rate/1000000;
        /* ( ( 0.44 * 1000000 / CIR_clock_period ) * ( 1 )) */
        cirdev->REG_X_REG_2T = 440*cirdev->rate/1000000;
        cirdev->REG_NEC_REPEAT_TIMEOUT_EN = 0 ;
        cirdev->REG_JVC_CONTI_CMD_EN = 0 ;
    } 
    else if (cirdev->irtype == SONY) {
        pr_debug("\n* Remote : CIR SONY *\n");
        /* ( ( 2.40 * 1000000 / CIR_clock_period ) * ( 7.0 / 8 ) ) */
        cirdev->REG_SynP = 2100*cirdev->rate/1000000;
        /* ( ( 0.61 * 1000000 / CIR_clock_period ) * ( 0.5 )) */
        cirdev->REG_T_REG_1T = 305*cirdev->rate/1000000;
        /* ( ( 0.61 * 1000000 / CIR_clock_period ) * ( 1 )) */
        cirdev->REG_T_REG_2T = 610*cirdev->rate/1000000;
        /* ( ( 0.61 * 1000000 / CIR_clock_period ) * ( 1 ))*/
        cirdev->REG_X_REG_2T = 610*cirdev->rate/1000000;
        cirdev->REG_NEC_REPEAT_TIMEOUT_EN = 0 ;
        cirdev->REG_JVC_CONTI_CMD_EN = 0 ;
    } else if (( cirdev->irtype == PHILIPS_RC5 ) || ( cirdev->irtype == PHILIPS_RC6 )) {
        if ( cirdev->irtype == PHILIPS_RC5 )
            pr_debug("\n* Remote : CIR PHILIPS_RC5 *\n");
        else
            pr_debug("\n* Remote : CIR PHILIPS_RC6 *\n");
        /* ( ( 0.444 * 1000000 / CIR_clock_period ) * ( 0.5 ) ) */
        cirdev->REG_SynP = 222*cirdev->rate/1000000;
        /* ( ( 0.444 * 1000000 / CIR_clock_period ) * ( 1.5 ) ) */
        cirdev->REG_SynP_= 666*cirdev->rate/1000000;
        /* ( ( 0.444 * 1000000 / CIR_clock_period ) * ( 2.5 ) ) */
        cirdev->REG_SynP_Sub_= 1110*cirdev->rate/1000000;
        /* ( ( 0.444 * 1000000 / CIR_clock_period ) * ( 3.5 ) ) */
        cirdev->REG_SynP_Subx_ = 1554*cirdev->rate/1000000;
        /* ( ( 0.444 * 1000000 / CIR_clock_period ) * ( 4.5 ) ) */
        cirdev->REG_T_REG_1T = 1998*cirdev->rate/1000000;
        /* ( ( 0.444 * 1000000 / CIR_clock_period ) * ( 5.5 ) ) */
        cirdev->REG_T_REG_2T = 2442*cirdev->rate/1000000;
        /* ( ( 0.444 * 1000000 / CIR_clock_period ) * ( 6.5 ) ) */
        cirdev->REG_X_REG_2T = 2886*cirdev->rate/1000000;
        cirdev->REG_NEC_REPEAT_TIMEOUT_EN = 0 ;
        cirdev->REG_JVC_CONTI_CMD_EN = 0 ;
    } else if (cirdev->irtype == PHILIPS_RCMM) {
        pr_debug("\n* Remote : CIR PHILIPS_RCMM *\n");
        /* ( ( 0.027778 * 1000000 / CIR_clock_period ) * ( 15 )) */
        cirdev->REG_SynP = 41667*cirdev->rate/100000000;
        /* ( ( 0.027778 * 1000000 / CIR_clock_period ) * ( 2 )) */
        cirdev->REG_SynP_= 55556*cirdev->rate/1000000000;
        /* ( ( 0.027778 * 1000000 / CIR_clock_period ) * ( 10 )) */
        cirdev->REG_SynP_Sub_= 27778*cirdev->rate/100000000;
        /* ( ( 0.027778 * 1000000 / CIR_clock_period ) * ( 6 * 1.65)) */
        cirdev->REG_SynP_Subx_ = 27500*cirdev->rate/100000000;
        /* ( ( 0.027778 * 1000000 / CIR_clock_period ) * ( 6 )) */
        cirdev->REG_T_REG_1T = 16666*cirdev->rate/100000000;
        /* ( ( 0.027778 * 1000000 / CIR_clock_period ) * ( 6 * 0.52)) */
        cirdev->REG_T_REG_2T = 86667*cirdev->rate/1000000000;
        /* ( ( 0.027778 * 1000000 / CIR_clock_period ) * ( 2 )) */
        cirdev->REG_X_REG_2T = 55556*cirdev->rate/1000000000;
        cirdev->REG_NEC_REPEAT_TIMEOUT_EN = 0 ;
        cirdev->REG_JVC_CONTI_CMD_EN = 0 ;
    }

    /* CIR SW reset. */
    cir_writel(1, IRSWRST);
    cir_writel(0, IRSWRST);

    cir_writel(cirdev->REG_SynP, PARAMETER(0)); 
    cir_writel(cirdev->REG_SynP_, PARAMETER(1));
    cir_writel(cirdev->REG_SynP_Sub_, PARAMETER(2)); 
    cir_writel(cirdev->REG_SynP_Subx_, PARAMETER(3)); 
    cir_writel(cirdev->REG_T_REG_1T, PARAMETER(4)); 
    cir_writel(cirdev->REG_T_REG_2T, PARAMETER(5)); 
    cir_writel(cirdev->REG_X_REG_2T, PARAMETER(6));

    cir_writel(cirdev->REG_NEC_REPEAT_TIMEOUT_EN, 
        NEC_REPEAT_TIME_OUT_CTRL);
    cir_writel(cirdev->REG_NEC_REPEAT_TIMEOUT_PERIOD_CNT, 
        NEC_REPEAT_TIME_OUT_COUNT);
    cir_writel(cirdev->REG_JVC_CONTI_CMD_EN, 
        JVC_CONTI_CTRL);
    cir_writel(cirdev->REG_JVC_CONTI_CMD_PERIOD_CNT, 
        JVC_CONTI_CNT);
    cir_writel(cirdev->REG_NEC_2ND_HEADER_PREFIX_BIT_NUM, 
        NEC_2ND_HEADER_PREFIX_BIT_NUM);

    if ( cirdev->irtype == NEC ) {
        if (cirdev->mod_using_nec2gen == NEC2GEN_JVC)
            intr_mask_period = 0 ;
        else
            intr_mask_period = 300000000 ;
    } else if ( cirdev->irtype == PANASONIC ) {
        intr_mask_period = 100000000 ;
    } else if ( cirdev->irtype == SONY ) {
        intr_mask_period = 40000000 ;
    } if ( cirdev->irtype == PHILIPS_RC5 ) {
        intr_mask_period = 45000000 ;
    } else if ( cirdev->irtype == PHILIPS_RC6 ) {
        if ( cirdev->ir_rec_bits == 20 ) {
            intr_mask_period = 35000000 ;
        } else if ( cirdev->ir_rec_bits == 148 ) {
            intr_mask_period = 180000000 ;
        }
    } else if ( cirdev->irtype == PHILIPS_RCMM ) {
        if ( cirdev->ir_rec_bits == 12 ) {
            intr_mask_period = 8000000 ;
        } else if ( cirdev->ir_rec_bits == 24 ) {
            intr_mask_period = 12000000 ;
        } else if ( cirdev->ir_rec_bits == 32 ) {
            intr_mask_period = 15000000 ;
        }
    }
 
    if (cirdev->mod_using_nec2gen == NEC2GEN_JVC ) {
        cir_writel(0, INT_MASK_CTRL);
        cir_writel(0, INT_MASK_COUNT);
    } else {
        cir_writel(0x1, INT_MASK_CTRL);
        cir_writel((u32)(intr_mask_period/APB_clock_period), INT_MASK_COUNT);
    }

    wrdata = ((cirdev->ir_rcmm_mod00_submode_addr_msk & 0xf) << 28) |
        ((cirdev->ir_rcmm_mod00_submode_msk & 0xf) << 24) |
        ((cirdev->ir_rcmm_mod00_3rd_to_12th_bit_chk & 0x1) << 22) |
        ((cirdev->ir_rcmm_mod00_3rd_to_12th_bit_val & 0x3ff) << 12) |
        ((cirdev->ir_rcmm_mod_msk & 0xf) << 8) |(cirdev->ir_rc6mod_msk & 0xff);

    cir_writel(wrdata, IRCTL_2);

    wrdata = ((cirdev->ir_inv & 0x1) << 28) |
        ((cirdev->en_rcv_more_bit & 0x1) << 25) |
        ((cirdev->en_rcv_less_bit & 0x01) << 24) |
        ((cirdev->ir_rec_bits & 0xff) <<16) |
        ((cirdev->ir_nec_as_pz_ocn & 0x1) << 9) |
        ((cirdev->ir_nec_sub_f & 0x1) << 8) |
        ((cirdev->irtype & 0x7) << 4) |((cirdev->ir_sw_en_on & 0x1));

    cir_writel(wrdata, IRCTL);
    elitexxx_cir_debug_print(cirdev);

}

ssize_t elite_cir_mode_show(struct seq_file *m, void *v)
{
    seq_printf(m, "%d\n", cir_mode);
    return 0;
}
static ssize_t elite_last_wakeup_show(struct seq_file *m, void *v)
{
	int i;
	for(i = 0; i < (sizeof(wakeup_bits)/sizeof(wakeup_bits[0]));i++) {
		if (last_wakeup_status&wakeup_bits[i]) {
			last_wakeup_type = i + 1;
			break;
		}
	}
	seq_printf(m, "%d\n", last_wakeup_type);

	return 0;

}

ssize_t proc_elite_cir_write(struct file *file, const char __user *buf,
            size_t count, loff_t *pos)
{
    struct elite_cir_dev *cirdev = PDE(file->f_path.dentry->d_inode)->data;
    char k_buf[2];
    int ret;
    int new_cir_mode;
  
    if (count > 2) {
        pr_err("Invalid arguments! \n");
        return -EINVAL;
    }

    if (copy_from_user(k_buf,buf,count)) {
        ret =  -EFAULT;
        goto err;
    } else {
        new_cir_mode = atoi(k_buf);
        if (new_cir_mode > MODE_NEC) {
            pr_err("Invalid,cir_mode must be 0 or 1 or 2! \n");
            ret =  - EINVAL;
            goto err;
        } if (new_cir_mode != cir_mode) {
            cir_mode = new_cir_mode;
            elitexxx_cir_hw_init(cirdev);
        }
		
        return count;
    }
  err:
  return ret;
}

static int proc_elite_cir_open(struct inode *inode, struct file *file)
{
    return single_open(file, elite_cir_mode_show, PDE(inode)->data);
}

static int proc_elite_last_wakeup_open(struct inode *inode, struct file *file)
{
    return single_open(file, elite_last_wakeup_show, PDE(inode)->data);
}

static const struct file_operations elite_cir_proc_ops = {
    .owner = THIS_MODULE,
    .open = proc_elite_cir_open,
    .read = seq_read,
    .release = single_release,
    .write = proc_elite_cir_write,
};

static const struct file_operations elite_last_wakeup_proc_ops = {
    .owner = THIS_MODULE,
    .open = proc_elite_last_wakeup_open,
    .read = seq_read,
    .release = single_release,
};

int elite_cir_create_procfs(struct elite_cir_dev *cirdev)
{

    if (cir_mode > MODE_NEC) 
        return -1;

    cir_proc_entry = proc_create_data(cir_proc_name, 0666, NULL, &elite_cir_proc_ops, cirdev);

    if (!cir_proc_entry) 
            return -1;

    last_wakeup_proc_entry = proc_create_data(last_wakeup_proc_name, 0444, NULL, &elite_last_wakeup_proc_ops, NULL);
    
    if (!last_wakeup_proc_entry) 
        return -1;

    return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id elitexxx_cir_match[] = {
    {
        .compatible = "s3graphics, elite1000-cir",
    },
    {},
};
MODULE_DEVICE_TABLE(of, elitexxx_cir_match);
#endif

static int elitexxx_cir_probe(struct platform_device *pdev)
{
    int i;
    int ret;
    struct resource *res;
    struct elite_cir_dev *cirdev;
	const struct of_device_id *match;
	struct device_node *np = pdev->dev.of_node;

	match = of_match_device(of_match_ptr(elitexxx_cir_match), &pdev->dev);
	if (match) {
		of_property_read_u32(np, "s3g,cir-mode", &cir_mode);
		of_property_read_u32(np, "s3g,wakeup-cfg", &wakeup_cfg);
	}

    cirdev = devm_kzalloc(&pdev->dev, sizeof(struct elite_cir_dev), GFP_KERNEL);
    if (!cirdev) {
        dev_err(&pdev->dev, "Can't alloc memory for CIR dev\n");
        return -ENOMEM;
    }
    platform_set_drvdata(pdev, cirdev);

    cirdev->dev = &pdev->dev;
    cirdev->clk = clk_get(&pdev->dev, "cir");
    if (IS_ERR(cirdev->clk)) {
        dev_err(&pdev->dev, "no clock defined\n");
        return -ENODEV;
    }

    cirdev->ahbclk = clk_get_sys("ahb", "ahb");
    if (IS_ERR(cirdev->ahbclk)) {
        dev_err(&pdev->dev, "no clock AHB defined\n");
        return -ENODEV;
    }
    /* set for DTV board */
    __raw_writeb(__raw_readb(0xfe1104bd)|0xe0,0xfe1104be);//hw bug
    __raw_writeb(__raw_readb(0xfe1104fe)|0xe0,0xfe1104fe);

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "Get flatform resource failed\n");
        return -ENODEV;
    }
    if (!devm_request_mem_region(&pdev->dev, res->start, res->end - res->start, dev_name(&pdev->dev))) {
        dev_err(&pdev->dev, "Request memory region failed\n");
        return -ENOMEM;
    }

    cirdev->irq = platform_get_irq(pdev, 0);
    if (cirdev->irq == NO_IRQ) {
        dev_err(cirdev->dev, "Fail to fetch IRQ\n");
        return -ENXIO;
    }

    reg_cir_base = devm_ioremap(&pdev->dev, res->start, res->end - res->start) ;
    if (!reg_cir_base) {
        dev_err(&(pdev->dev), "ioremap failed\n");
        return -ENOMEM;
    }
    cirdev->regs_base = reg_cir_base;

    clk_enable(cirdev->clk); 
    cirdev->rate = CIR_HZ;    
    cirdev->ahb_rate = clk_get_rate(cirdev->ahbclk);
    pr_debug("cirdev->rate=%lu,cirdev->ahb_rate=%lu \n",cirdev->rate,cirdev->ahb_rate);
       
    /* Initial H/W */
    elitexxx_cir_hw_init(cirdev);

    if ((cirdev->idev = input_allocate_device()) == NULL) {
        dev_err(&(pdev->dev),"Alloc input device failed\n");
        return -ENOMEM;
    }

    set_bit(EV_KEY, cirdev->idev->evbit);
    if (!cirdev->kmap) {
        dev_err(&(pdev->dev), "Uninitialized key code mapping\n");
        return -ENODEV;
    }
    for (i = 0; i < cirdev->kmap_size; i++) {
        __set_bit(cirdev->kmap[i].keycode, cirdev->idev->keybit);
    }

    cirdev->idev->name = "elitexxx_cir";
    cirdev->idev->phys = "elitexxx_cir/input0";

    /* Register an input device. */
    if((ret = input_register_device(cirdev->idev)) < 0){
        dev_err(&(pdev->dev), "Fail to register input device\n");
        goto err_register;
    }

    /* Register an ISR */
    if((ret = devm_request_irq(&pdev->dev, cirdev->irq, elitexxx_cir_interrupt, 
            IRQF_SHARED | IRQF_DISABLED, "elitexxx_cir", cirdev)) <0 ) {
        dev_err(&(pdev->dev), "Fail to register interrupt\n");
        goto err_irq;
    }

	device_init_wakeup(&pdev->dev, 1);
#ifdef CONFIG_PROC_FS
	elite_cir_create_procfs(cirdev);
#endif
	dev_info(&(pdev->dev), "Elite CIR probed OK!\n");

    return 0;

err_irq:
    input_unregister_device(cirdev->idev);
err_register:
    input_free_device(cirdev->idev);
    clk_put(cirdev->clk);
    clk_put(cirdev->ahbclk);
    devm_kfree(&pdev->dev, cirdev);

    return ret;

}

static int elitexxx_cir_remove(struct platform_device *pdev)
{
    struct elite_cir_dev *cirdev;

    cirdev = platform_get_drvdata(pdev);
    input_unregister_device(cirdev->idev);
    input_free_device(cirdev->idev);
    platform_set_drvdata(pdev, NULL);
    clk_disable(cirdev->clk);
    clk_put(cirdev->clk);
    clk_put(cirdev->ahbclk);
    devm_kfree(&pdev->dev, cirdev);
    remove_proc_entry(cir_proc_name,cir_proc_entry);
     
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int elitexxx_cir_suspend(struct device *dev)
{
    struct elite_cir_dev *cirdev = dev_get_drvdata(dev);

    pr_debug("elitexxx_cir_suspend\n");

    if (device_may_wakeup(dev))
        enable_irq_wake(cirdev->irq);

    elitexxx_cir_hw_suspend(cirdev);

    disable_irq(cirdev->irq);

    return 0;
}

static int elitexxx_cir_resume(struct device *dev)
{
    struct elite_cir_dev *cirdev = dev_get_drvdata(dev);
    volatile  unsigned int regval;
    int i =0 ;
    pr_debug("enter elitexxx_cir_resume\n");

    /* set for DTV board */
    __raw_writeb(__raw_readb(0xfe1104bd)|0xe0,0xfe1104be);//hw bug
    __raw_writeb(__raw_readb(0xfe1104fe)|0xe0,0xfe1104fe);

    if (device_may_wakeup(dev))
        disable_irq_wake(cirdev->irq);

    elite_cir_sw_reset();

    /* Initial H/W */
    cir_writel(cir_readl(WAKEUP_CTRL) & ~(BIT0), WAKEUP_CTRL);

    for (i=0;i<10;i++) {
        regval = cir_readl(WAKEUP_STS) ;

        if (regval & BIT0) {
            cir_writel(cir_readl(WAKEUP_STS) | BIT4, WAKEUP_STS);
        } else {
            break;
        }
        msleep_interruptible(5);
    }

    regval = cir_readl(WAKEUP_STS) ;
    if (regval & BIT0)
        pr_err("CIR resume NG  WAKEUP_STS 0x%08x \n" ,regval);

    elitexxx_cir_hw_init(cirdev);
    enable_irq(cirdev->irq);
    /*wake up android */
    input_report_key(cirdev->idev, 538, 1);
    input_report_key(cirdev->idev, 538, 0);
    input_sync(cirdev->idev);

    pr_debug("exit elitexxx_cir_resume\n");

    return 0;
}
#endif


static struct platform_device_id elite_cir_driver_ids[] = {
	{
		.name  = "elite-cir",
		.driver_data	= (kernel_ulong_t)NULL,
	}, {
		.name  = "elite-cir.0",
		.driver_data	= (kernel_ulong_t)NULL,
	}, {
		/* sentinel */
	},
};

static SIMPLE_DEV_PM_OPS(elitexxx_cir_pm_ops,
            elitexxx_cir_suspend, elitexxx_cir_resume);

static struct platform_driver  elitexxx_cir_driver = {
    .probe = elitexxx_cir_probe,
    .remove = __devexit_p(elitexxx_cir_remove),
    .id_table  = elite_cir_driver_ids,
    .driver = {
        .name = "s3graphics-elite-cir",
        .owner = THIS_MODULE,
        .pm = &elitexxx_cir_pm_ops,
        .of_match_table = of_match_ptr(elitexxx_cir_match),
    },

};

module_platform_driver(elitexxx_cir_driver);

module_param(cir_mode, uint, 0444);
module_param(wakeup_cfg, uint, 0444);


MODULE_ALIAS("platform:elitexxx_cir");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("S3 Graphics, Inc.");
MODULE_DESCRIPTION("CIR driver for S3 E-litexxx SoC");

