/*
 * linux/arch/arm/mach-elite/pinctrl-elite.c
 *
 * E-lite1000 pin multiplexing configurations
 *
 * Copyright (C)  2012 S3 Graphics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>

#include <mach/pinconf-elite.h>


#include "pinctrl-elite.h"

#define DRIVER_NAME "s3graphics-elite-pinctrl"

/* Number of registers */
#define REG_NR_PULL_UPDW_EN	19
#define REG_NR_PULL_UPDW_CTRL0	19
#define REG_NR_PULL_UPDW_CTRL1	19
/* calculate register offset */
#define REG_ENABLE(x)			(((x)/8) + 0x40) //pin mux register
#define REG_OUTPUT_EN(x)		(((x)/8) + 0x80)
#define REG_OUTPUT_DAT(x)		(((x)/8) + 0xc0)
#define REG_PULL_UP_DOWN_EN(x)		(((x)/8) + 0x480)
#define REG_PULL_UP_DOWN_CTRL0(x)	(((x)/8) + 0x4c0)
#define REG_PULL_UP_DOWN_CTRL1(x)	(((x)/8) + 0x5c0)

#define REG_BIT(x)			((x) % 8)
/* registers offset */
#define REG_USB_OP_MODE			0x10c
#define REG_PIN_SHARE			0x200
#define REG_PAD_SD0_DRV_BASE		0x620 //drive strength
#define REG_PAD_DRV_REG_NUM		4
#define REG_PAD_SIG_VOLT		0x624

/* pin mux definition */
#define PMX_I2S_PCM_MASK		0x03
#define PMX_I2S_ADC			0x02
#define PMX_I2S_DAC			0x00
#define PMX_SPDIF			0x00
#define PMX_PCM				0x01

#define PMX_I2SMCLK_EN		(1<<2)
#define PMX_SATALED_EN		(1<<3)
#define PMX_KPAD_EN			~(1<<4)
#define PMX_PWM_EN			(1<<5)
#define PMX_SPI_EN			(1<<6)
#define PMX_UART0			(~((1<<7) | (1<<9)))
#define PMX_UART2			(1<<7)
#define PMX_TS1				(~(1<<8))
#define PMX_TS23			(1<<8)
#define PMX_SM1				(1<<9)
#define PMX_NAND			(~(1<<10))
#define PMX_SD2				(1<<10)


#define __GPIO_PIN(x)				(x)

#define ELITE_PIN_PWMOUT1_GPIO0		__GPIO_PIN(0)
#define ELITE_PIN_PWMOUT2_GPIO1		__GPIO_PIN(1)
#define ELITE_PIN_PWMOUT3_GPIO2		__GPIO_PIN(2)
#define ELITE_PIN_SM1OFF_GPIO3			__GPIO_PIN(3)
#define ELITE_PIN_SATA_LED0_GPIO4		__GPIO_PIN(4)
#define ELITE_PIN_SPI1CLK_GPIO5		__GPIO_PIN(5)
#define ELITE_PIN_SPI1MISO_GPIO6		__GPIO_PIN(6)
#define ELITE_PIN_SPI1MOSI_GPIO7		__GPIO_PIN(7)

#define ELITE_PIN_SPI1SS0N_GPIO8		__GPIO_PIN(8)
#define ELITE_PIN_SPI0SS1N_GPIO9		__GPIO_PIN(9)
#define ELITE_PIN_SPI0SS2N_GPIO10		__GPIO_PIN(10)
#define ELITE_PIN_SPI0SS3N_GPIO11		__GPIO_PIN(11)
/* Reseved */
//#define ELITE_PIN_GPIO12	__GPIO_PIN(12)
//#define ELITE_PIN_GPIO13	__GPIO_PIN(13)
//#define ELITE_PIN_GPIO14	__GPIO_PIN(14)
//#define ELITE_PIN_GPIO15	__GPIO_PIN(15)
/////////////////////////////////////////////////////////////////
#define ELITE_PIN_WAKEUP_GPIO16	__GPIO_PIN(16)
#define ELITE_PIN_WAKEUP_GPIO17	__GPIO_PIN(17)
#define ELITE_PIN_WAKEUP_GPIO18	__GPIO_PIN(18)
#define ELITE_PIN_WAKEUP_GPIO19	__GPIO_PIN(19)
/* Reseved */
//#define ELITE_PIN_WAKEUP_GPIO20	__GPIO_PIN(20)
#define ELITE_PIN_SUS_GPIO21	__GPIO_PIN(21) 
#define ELITE_PIN_SUS_GPIO22	__GPIO_PIN(22) 
#define ELITE_PIN_SUS_GPIO23	__GPIO_PIN(23) 
///////////////////////////////////////////////////////////////////
/* Reseved */
//#define ELITE_PIN_GPIO24		__GPIO_PIN(24)
//#define ELITE_PIN_GPIO25		__GPIO_PIN(25)
//#define ELITE_PIN_GPIO26		__GPIO_PIN(26)
//#define ELITE_PIN_GPIO27		__GPIO_PIN(27)
#define ELITE_PIN_SD0_CD_GPIO28		__GPIO_PIN(28)
//#define ELITE_PIN_SD3_CD_GPIO29		__GPIO_PIN(29) /* Removed from Elite1000 */
/* Reseved */
//#define ELITE_PIN_GPIO30		__GPIO_PIN(30)
//#define ELITE_PIN_GPIO31		__GPIO_PIN(31)
//////////////////////////////////////////////////////////////////
#define ELITE_PIN_SD0_PWRSW_GPIO32		__GPIO_PIN(32)
#define ELITE_PIN_SD0_CLK_GPIO33			__GPIO_PIN(33)
#define ELITE_PIN_SD0_CMD_GPIO34			__GPIO_PIN(34)
#define ELITE_PIN_SD0_WP_GPIO35			__GPIO_PIN(35)
#define ELITE_PIN_SD0_DATA0_GPIO36		__GPIO_PIN(36)
#define ELITE_PIN_SD0_DATA1_GPIO37		__GPIO_PIN(37)
#define ELITE_PIN_SD0_DATA2_GPIO38		__GPIO_PIN(38)
#define ELITE_PIN_SD0_DATA3_GPIO39		__GPIO_PIN(39)
///////////////////////////////////////////////////////////////////
#define ELITE_PIN_PCIE_RST0Z_GPIO40			__GPIO_PIN(40)
#define ELITE_PIN_PCIE_RST1Z_GPIO41			__GPIO_PIN(41)
#define ELITE_PIN_PCIE_CLK_REQ0Z_GPIO42		__GPIO_PIN(42)
#define ELITE_PIN_NAND_ALE_SD2CLK_GPIO43		__GPIO_PIN(43)
#define ELITE_PIN_NAND_CLE_SD2CMD_GPIO44		__GPIO_PIN(44)
#define ELITE_PIN_NAND_WEN_GPIO45			__GPIO_PIN(45)
#define ELITE_PIN_NAND_REN_GPIO46				__GPIO_PIN(46)
#define ELITE_PIN_NAND_DQS_GPIO47				__GPIO_PIN(47)
////////////////////////////////////////////////////////////////////
#define ELITE_PIN_NAND_WPN_SD2PWRSW_GPIO48	__GPIO_PIN(48)
#define ELITE_PIN_NAND_WPDN_SD2WP_GPIO49		__GPIO_PIN(49)
#define ELITE_PIN_NAND_RB0N_SD2CD_GPIO50			__GPIO_PIN(50)
#define ELITE_PIN_NAND_RB1N_GPIO51				__GPIO_PIN(51)
#define ELITE_PIN_NAND_CE0N_GPIO52				__GPIO_PIN(52)
#define ELITE_PIN_NAND_CE1N_GPIO53				__GPIO_PIN(53)
/* Reseved */
//#define ELITE_PIN_GPIO54		__GPIO_PIN(54)
//#define ELITE_PIN_GPIO55		__GPIO_PIN(55)
///////////////////////////////////////////////////////////////////
#define ELITE_PIN_NAND_IO0_SD2DATA0_GPIO56		__GPIO_PIN(56)
#define ELITE_PIN_NAND_IO1_SD2DATA1_GPIO57		__GPIO_PIN(57)
#define ELITE_PIN_NAND_IO2_SD2DATA2_GPIO58		__GPIO_PIN(58)
#define ELITE_PIN_NAND_IO3_SD2DATA3_GPIO59		__GPIO_PIN(59)
#define ELITE_PIN_NAND_IO4_SD2DATA4_GPIO60		__GPIO_PIN(60)
#define ELITE_PIN_NAND_IO5_SD2DATA5_GPIO61		__GPIO_PIN(61)
#define ELITE_PIN_NAND_IO6_SD2DATA6_GPIO62		__GPIO_PIN(62)
#define ELITE_PIN_NAND_IO7_SD2DATA7_GPIO63		__GPIO_PIN(63)
////////////////////////////////////////////////////////////
#define ELITE_PIN_SD1_PWRSW_GPIO64		__GPIO_PIN(64)
#define ELITE_PIN_SD1_CLK_GPIO65			__GPIO_PIN(65)
#define ELITE_PIN_SD1_CMD_GPIO66			__GPIO_PIN(66)
#define ELITE_PIN_SD1_WP_GPIO67			__GPIO_PIN(67)
#define ELITE_PIN_SD1_DATA0_GPIO68		__GPIO_PIN(68)
#define ELITE_PIN_SD1_DATA1_GPIO69		__GPIO_PIN(69)
#define ELITE_PIN_SD1_DATA2_GPIO70		__GPIO_PIN(70)
#define ELITE_PIN_SD1_DATA3_GPIO71		__GPIO_PIN(71)
//////////////////////////////////////////////////////////
#define ELITE_PIN_SD1_CD_GPIO72		__GPIO_PIN(72)
#define ELITE_PIN_SF_DI_GPIO73			__GPIO_PIN(73)
#define ELITE_PIN_SF_DO_GPIO74			__GPIO_PIN(74)
#define ELITE_PIN_SF_CS0N_GPIO75		__GPIO_PIN(75)
#define ELITE_PIN_SF_CS1N_GPIO76		__GPIO_PIN(76)
#define ELITE_PIN_SF_CLK_GPIO77		__GPIO_PIN(77)
/* Reseved */
//#define ELITE_PIN_GPIO78		__GPIO_PIN(78)
//#define ELITE_PIN_GPIO79		__GPIO_PIN(79)
////////////////////////////////////////////////////////////
#define ELITE_PIN_KPAD_ROW0_GPIO80		__GPIO_PIN(80)
#define ELITE_PIN_KPAD_ROW1_GPIO81		__GPIO_PIN(81)
#define ELITE_PIN_KPAD_ROW2_GPIO82		__GPIO_PIN(82)
#define ELITE_PIN_KPAD_ROW3_GPIO83		__GPIO_PIN(83)
#define ELITE_PIN_KPAD_COL0_GPIO84		__GPIO_PIN(84)
#define ELITE_PIN_KPAD_COL1_GPIO85		__GPIO_PIN(85)
#define ELITE_PIN_KPAD_COL2_GPIO86		__GPIO_PIN(86)
#define ELITE_PIN_KPAD_COL3_GPIO87		__GPIO_PIN(87)
/////////////////////////////////////////////////////////////
#define ELITE_PIN_SM_IOUC_GPIO88			__GPIO_PIN(88)
#define ELITE_PIN_SM_STROBE_GPIO89		__GPIO_PIN(89)
#define ELITE_PIN_SM_CMDVCC_GPIO90		__GPIO_PIN(90)
#define ELITE_PIN_SM_RSTIN_GPIO91			__GPIO_PIN(91)
#define ELITE_PIN_SM_OFF_GPIO92			__GPIO_PIN(92)
#define ELITE_PIN_UART1_TXD_GPIO93		__GPIO_PIN(93)
#define ELITE_PIN_UART1_RXD_GPIO94		__GPIO_PIN(94)
#define ELITE_PIN_UART0_RTS_UART2TXD_SM1IOUC_GPIO95		__GPIO_PIN(95)
///////////////////////////////////////////////////////////
#define ELITE_PIN_UART0_CTS_UART2RXD_SM1STROBE_GPIO96		__GPIO_PIN(96)
#define ELITE_PIN_UART0_RXD_SM1CMDVCC_GPIO97				__GPIO_PIN(97)
#define ELITE_PIN_UART0_TXD_SM1RSTIN_GPIO98					__GPIO_PIN(98)
#define ELITE_PIN_SPI0_CLK_GPIO99			__GPIO_PIN(99)
#define ELITE_PIN_SPI0_MOSI_GPIO100		__GPIO_PIN(100)
#define ELITE_PIN_SPI0_MISO_GPIO101		__GPIO_PIN(101)
#define ELITE_PIN_SPI0_SS0N_GPIO102		__GPIO_PIN(102)
/* Reseved */
//#define ELITE_PIN_GPIO103		__GPIO_PIN(103)
////////////////////////////////////////////////////////////
#define ELITE_PIN_I2S_DACMCLK_GPIO104		__GPIO_PIN(104)
#define ELITE_PIN_I2S_DACBCLK_GPIO105		__GPIO_PIN(105)
#define ELITE_PIN_I2S_DACLRC_GPIO106		__GPIO_PIN(106)
#define ELITE_PIN_I2S_DACDAT0_GPIO107		__GPIO_PIN(107)
#define ELITE_PIN_I2S_DACDAT1_I2SADCBCLK_PCMSYNC_GPIO108		__GPIO_PIN(108)
#define ELITE_PIN_I2S_DACDAT2_I2SADCLRC_PCMCLK_GPIO109			__GPIO_PIN(109)
#define ELITE_PIN_I2S_DACDAT3_PCMIN_GPIO110						__GPIO_PIN(110)
#define ELITE_PIN_I2S_ADCDAT0_GPIO111								__GPIO_PIN(111)
////////////////////////////////////////////////////////////
#define ELITE_PIN_I2S_ADCMCLK_I2SADCDAT1_PCMOUT_GPIO112		__GPIO_PIN(112)
#define ELITE_PIN_PCM_MCLK_I2SADCDAT2_GPIO113		__GPIO_PIN(113)
#define ELITE_PIN_SPDIFI_GPIO114		__GPIO_PIN(114)
#define ELITE_PIN_SPDIFO_I2SADCDAT3_GPIO115		__GPIO_PIN(115)
#define ELITE_PIN_I2C0_SCL_GPIO116		__GPIO_PIN(116)
#define ELITE_PIN_I2C0_SDA_GPIO117		__GPIO_PIN(117)
#define ELITE_PIN_I2C1_SCL_GPIO118		__GPIO_PIN(118)
#define ELITE_PIN_I2C1_SDA_GPIO119		__GPIO_PIN(119)
////////////////////////////////////////////////////////
/* Reseved */
//#define ELITE_PIN_DISP_CLKO_GPIO120		__GPIO_PIN(120)
//#define ELITE_PIN_DISP_CLKI_GPIO121		__GPIO_PIN(121)
//#define ELITE_PIN_GFX_GPIO3_GPIO122		__GPIO_PIN(122)
//#define ELITE_PIN_GFX_GPIO0_GPIO123		__GPIO_PIN(123)
//#define ELITE_PIN_DIU_I2C1_DAT_GPIO124		__GPIO_PIN(124)
//#define ELITE_PIN_DIU_I2C1_CLK_GPIO125		__GPIO_PIN(125)
//#define ELITE_PIN_DIU_I2C0_DAT_GPIO126		__GPIO_PIN(126)
//#define ELITE_PIN_DIU_I2C0_CLK_GPIO127		__GPIO_PIN(127)
////////////////////////////////////////////////////////
#define ELITE_PIN_TS_DIN0_GPIO128		__GPIO_PIN(128)
#define ELITE_PIN_TS_DIN1_TS2DAT_GPIO129		__GPIO_PIN(129)
#define ELITE_PIN_TS_DIN2_TS2CLK_GPIO130		__GPIO_PIN(130)
#define ELITE_PIN_TS_DIN3_TS2SYNC_GPIO131		__GPIO_PIN(131)
#define ELITE_PIN_TS_DIN4_TS2VALID_GPIO132		__GPIO_PIN(132)
#define ELITE_PIN_TS_DIN5_TS3CLK_GPIO133		__GPIO_PIN(133)
#define ELITE_PIN_TS_DIN6_TS3SYNC_GPIO134		__GPIO_PIN(134)
#define ELITE_PIN_TS_DIN7_TS3VALID_GPIO135		__GPIO_PIN(135)
//////////////////////////////////////////////////////////////
#define ELITE_PIN_TS_SYNC_GPIO136		__GPIO_PIN(136)
#define ELITE_PIN_TS_VALID_GPIO137		__GPIO_PIN(137)
#define ELITE_PIN_TS_CLK_GPIO138		__GPIO_PIN(138)
#define ELITE_PIN_TS3_DAT_GPIO139		__GPIO_PIN(139)
#define ELITE_PIN_TS1_DAT_GPIO140		__GPIO_PIN(140)
#define ELITE_PIN_TS1_CLk_GPIO141		__GPIO_PIN(141)
#define ELITE_PIN_TS1_SYNC_GPIO142		__GPIO_PIN(142)
#define ELITE_PIN_TS1_VALID_GPIO143		__GPIO_PIN(143)
//////////////////////////////////////////////////////////////
#define ELITE_PIN_AGC_OUT0_GPIO144		__GPIO_PIN(144)
#define ELITE_PIN_AGC_OUT1_GPIO145		__GPIO_PIN(145)
/* Reseved */
//#define ELITE_PIN_GPIO146		__GPIO_PIN(146)
//#define ELITE_PIN_GPIO147		__GPIO_PIN(147)
//#define ELITE_PIN_GPIO148		__GPIO_PIN(148)
//#define ELITE_PIN_GPIO149		__GPIO_PIN(149)
//#define ELITE_PIN_GPIO150		__GPIO_PIN(150)
//#define ELITE_PIN_GPIO151		__GPIO_PIN(151)
///////////////////////////////////////////////////////////////
#define ELITE_PIN_USB_SW0_GPIO152		__GPIO_PIN(480)
#define ELITE_PIN_USB_ATTA0_GPIO153		__GPIO_PIN(481)
#define ELITE_PIN_USB_OC0_GPIO154		__GPIO_PIN(482)
#define ELITE_PIN_USB_OC1_GPIO155		__GPIO_PIN(483)
#define ELITE_PIN_USB_OC2_GPIO156		__GPIO_PIN(484)
#define ELITE_PIN_USB_OC3_GPIO157		__GPIO_PIN(485)
/* Reseved */
//#define ELITE_PIN_GPIO158		__GPIO_PIN(486)
//#define ELITE_PIN_GPIO159		__GPIO_PIN(487)

#define NUM_GPIOS				(__GPIO_PIN(487) + 1)

/* Non-GPIO pins */

static inline u8 pinctrl_readb(struct elite_pinctrl *pctrl, u32 reg)
{
	return readb(pctrl->virt_base + reg);
}

static inline void pinctrl_writeb(struct elite_pinctrl *pctrl, u8 val, u32 reg)
{
	writeb(val, pctrl->virt_base + reg);
}

static inline u32 pinctrl_readl(struct elite_pinctrl *pctrl, u32 reg)
{
	return readl(pctrl->virt_base + reg);
}

static inline void pinctrl_writel(struct elite_pinctrl *pctrl, u32 val, u32 reg)
{
	writel(val, pctrl->virt_base + reg);
}

static void pinctrl_bit_writeb(struct elite_pinctrl *pctrl, u32 reg, int gpio, int value)
{
	u8 val;

	val = pinctrl_readb(pctrl, reg);
	if(value)
		pinctrl_writeb(pctrl, (1 << REG_BIT(gpio)) | val, reg);
	else
		pinctrl_writeb(pctrl, (~(1 << REG_BIT(gpio))) & val, reg);
}

static int pinctrl_bit_readb(struct elite_pinctrl *pctrl, u32 reg, int gpio)
{
	return (pinctrl_readb(pctrl, reg) >> REG_BIT(gpio)) & 0x01;
}

static void elite_pinctrl_func_en(struct elite_pinctrl *pctrl, unsigned gpio, int value)
{
	pinctrl_bit_writeb(pctrl, REG_ENABLE(gpio), gpio, value);
}

#if 0
static void elite_pinctrl_output_en(struct elite_pinctrl *pctrl, unsigned gpio, bool value)
{
	pinctrl_bit_writeb(pctrl, REG_OUTPUT_EN(gpio), gpio, value);
}
#endif

static void elite_pinctrl_pullupdown_en(struct elite_pinctrl *pctrl, unsigned gpio,
	int value)
{
	/* 
	 * map from range between gpio488 and gpio495 to range between gpio16 and gpio23
	 * due to the bad design of GPIO module
	 */
	if((gpio >= 16) && (gpio <= 23)) 
		gpio += 472;
	pinctrl_bit_writeb(pctrl, REG_PULL_UP_DOWN_EN(gpio), gpio, value);
}

static void elite_pinctrl_pullupdown_ctrl0_set(struct elite_pinctrl *pctrl, unsigned gpio, 
	enum elite_pinconf_pull value)
{
	if((gpio >= 16) && (gpio <= 23)) 
		gpio += 472;
	if(gpio == 28) { //HW BUG: SD0CD pin route to pin24
		pinctrl_bit_writeb(pctrl, REG_PULL_UP_DOWN_CTRL0(24), 24, value);
	} else
		pinctrl_bit_writeb(pctrl, REG_PULL_UP_DOWN_CTRL0(gpio), gpio, value);
}

static u8 elite_pinctrl_pullupdown_ctrl0_get(struct elite_pinctrl *pctrl, unsigned gpio)
{
	if((gpio >= 16) && (gpio <= 23)) 
		gpio += 472;

	if(gpio == 28) { //HW BUG: SD0CD pin route to pin24
		return pinctrl_bit_readb(pctrl, REG_PULL_UP_DOWN_CTRL0(24), 24);
	} else
		return pinctrl_bit_readb(pctrl, REG_PULL_UP_DOWN_CTRL0(gpio), gpio);
}

#if 0
static void elite_pinctrl_pullupdown_ctrl1_set(struct elite_pinctrl *pctrl, unsigned gpio, 
	enum elite_pinconf_pull value)
{
	if((gpio >= 16) && (gpio <= 23)) 
		gpio += 472;
	pinctrl_bit_writeb(pctrl, REG_PULL_UP_DOWN_CTRL1(gpio), gpio, value);
}

static void elite_pinctrl_pullupdown_ctrl1_get(struct elite_pinctrl *pctrl, unsigned gpio)
{
	if((gpio >= 16) && (gpio <= 23)) 
		gpio += 472;
	return pinctrl_bit_readb(pctrl, REG_PULL_UP_DOWN_CTRL1(gpio), gpio);
}
#endif

static void elite_pinctrl_pinmux_set(struct elite_pinctrl *pctrl, unsigned int value)
{
	pinctrl_writel(pctrl, value, REG_PIN_SHARE);
}

static u32 elite_pinctrl_pinmux_get(struct elite_pinctrl *pctrl)
{
	return pinctrl_readl(pctrl, REG_PIN_SHARE);
}

static void __sd0_drive_strength(struct elite_pinctrl *pctrl, 
	u32 reg, enum elite_pinconf_drvstrength drv)
{
	u8 value = (drv << 3) | drv;
	pinctrl_writeb(pctrl, value, reg);
}

static void elite_pinctrl_sd0_drive_strength_set(struct elite_pinctrl *pctrl, 
	enum elite_pinconf_drvstrength drv)
{
	u32 reg;
	
	for(reg= 0; reg<REG_PAD_DRV_REG_NUM; reg++) 
		__sd0_drive_strength(pctrl, reg + REG_PAD_SD0_DRV_BASE, drv);
}

static u8 elite_pinctrl_sd0_drive_strength_get(struct elite_pinctrl *pctrl)
{
	return pinctrl_readb(pctrl, REG_PAD_SD0_DRV_BASE) & 0x07;
}

static void elite_pinctrl_sd0_signal_voltage_set(struct elite_pinctrl *pctrl, 
	enum elite_pinconf_voltage volt)
{
	pinctrl_writeb(pctrl, volt, REG_PAD_SIG_VOLT);
}

static u8 elite_pinctrl_sd0_signal_voltage_get(struct elite_pinctrl *pctrl)
{
	return pinctrl_readb(pctrl, REG_PAD_SIG_VOLT) & 0x01;
}

static void elite_pinctrl_usb_mode_set(struct elite_pinctrl *pctrl, 
	enum elite_pinconf_usb_mode mode)
{
	pinctrl_writel(pctrl, (mode & 3) << 16, REG_USB_OP_MODE);
}

static u32 elite_pinctrl_usb_mode_get(struct elite_pinctrl *pctrl)
{
	return (pinctrl_readl(pctrl, REG_USB_OP_MODE) >> 16) & 3;
}

const struct pinctrl_pin_desc elite_pins[] = {
	PINCTRL_PIN(ELITE_PIN_PWMOUT1_GPIO0, "PWMOUT1_GPIO0"),
	PINCTRL_PIN(ELITE_PIN_PWMOUT2_GPIO1, "PWMOUT2_GPIO1"),
	PINCTRL_PIN(ELITE_PIN_PWMOUT3_GPIO2, "PWMOUT3_GPIO2"),
	PINCTRL_PIN(ELITE_PIN_SM1OFF_GPIO3, "SM1OFF_GPIO3"),
	PINCTRL_PIN(ELITE_PIN_SATA_LED0_GPIO4, "SATA_LED0_GPIO4"),
	PINCTRL_PIN(ELITE_PIN_SPI1CLK_GPIO5, "SPI1CLK_GPIO5"),
	PINCTRL_PIN(ELITE_PIN_SPI1MISO_GPIO6, "SPI1MISO_GPIO6"),
	PINCTRL_PIN(ELITE_PIN_SPI1MOSI_GPIO7, "SPI1MOSI_GPIO7"),
	PINCTRL_PIN(ELITE_PIN_SPI1SS0N_GPIO8, "SPI1SS0N_GPIO8"),
	PINCTRL_PIN(ELITE_PIN_SPI0SS1N_GPIO9, "SPI0SS1N_GPIO9"),
	PINCTRL_PIN(ELITE_PIN_SPI0SS2N_GPIO10, "SPI0SS2N_GPIO10"),
	PINCTRL_PIN(ELITE_PIN_SPI0SS3N_GPIO11, "SPI0SS3N_GPIO11"),
	PINCTRL_PIN(ELITE_PIN_WAKEUP_GPIO16, "WAKEUP_GPIO16"),
	PINCTRL_PIN(ELITE_PIN_WAKEUP_GPIO17, "WAKEUP_GPIO17"),
	PINCTRL_PIN(ELITE_PIN_WAKEUP_GPIO18, "WAKEUP_GPIO18"),
	PINCTRL_PIN(ELITE_PIN_WAKEUP_GPIO19, "WAKEUP_GPIO19"),
	PINCTRL_PIN(ELITE_PIN_SUS_GPIO21, "SUS_GPIO21"),
	PINCTRL_PIN(ELITE_PIN_SUS_GPIO22, "SUS_GPIO22"),
	PINCTRL_PIN(ELITE_PIN_SUS_GPIO23, "SUS_GPIO23"),
	PINCTRL_PIN(ELITE_PIN_SD0_CD_GPIO28, "SD0_CD_GPIO28"),
	PINCTRL_PIN(ELITE_PIN_SD0_PWRSW_GPIO32, "SD0_PWRSW_GPIO32"),
	PINCTRL_PIN(ELITE_PIN_SD0_CLK_GPIO33, "SD0_CLK_GPIO33"),
	PINCTRL_PIN(ELITE_PIN_SD0_CMD_GPIO34, "SD0_CMD_GPIO34"),
	PINCTRL_PIN(ELITE_PIN_SD0_WP_GPIO35, "SD0_WP_GPIO35"),
	PINCTRL_PIN(ELITE_PIN_SD0_DATA0_GPIO36, "SD0_DATA0_GPIO36"),
	PINCTRL_PIN(ELITE_PIN_SD0_DATA1_GPIO37, "SD0_DATA1_GPIO37"),
	PINCTRL_PIN(ELITE_PIN_SD0_DATA2_GPIO38, "SD0_DATA2_GPIO38"),
	PINCTRL_PIN(ELITE_PIN_SD0_DATA3_GPIO39, "SD0_DATA3_GPIO39"),
	PINCTRL_PIN(ELITE_PIN_PCIE_RST0Z_GPIO40, "PCIE_RST0Z_GPIO40"),
	PINCTRL_PIN(ELITE_PIN_PCIE_RST1Z_GPIO41, "PCIE_RST1Z_GPIO41"),
	PINCTRL_PIN(ELITE_PIN_PCIE_CLK_REQ0Z_GPIO42, "PCIE_CLK_REQ0Z_GPIO42"),
	PINCTRL_PIN(ELITE_PIN_NAND_ALE_SD2CLK_GPIO43, "NAND_ALE_SD2CLK_GPIO43"),
	PINCTRL_PIN(ELITE_PIN_NAND_CLE_SD2CMD_GPIO44, "NAND_CLE_SD2CMD_GPIO44"),
	PINCTRL_PIN(ELITE_PIN_NAND_WEN_GPIO45, "NAND_WEN_GPIO45"),
	PINCTRL_PIN(ELITE_PIN_NAND_REN_GPIO46, "NAND_REN_GPIO46"),
	PINCTRL_PIN(ELITE_PIN_NAND_DQS_GPIO47, "NAND_DQS_GPIO47"),
	PINCTRL_PIN(ELITE_PIN_NAND_WPN_SD2PWRSW_GPIO48, "NAND_WPN_SD2PWRSW_GPIO48"),
	PINCTRL_PIN(ELITE_PIN_NAND_WPDN_SD2WP_GPIO49, "NAND_WPDN_SD2WP_GPIO49"),
	PINCTRL_PIN(ELITE_PIN_NAND_RB0N_SD2CD_GPIO50, "NAND_RB0N_SD2CD_GPIO50"),
	PINCTRL_PIN(ELITE_PIN_NAND_RB1N_GPIO51, "NAND_RB1N_GPIO51"),
	PINCTRL_PIN(ELITE_PIN_NAND_CE0N_GPIO52, "NAND_CE0N_GPIO52"),
	PINCTRL_PIN(ELITE_PIN_NAND_CE1N_GPIO53, "NAND_CE1N_GPIO53"),
	PINCTRL_PIN(ELITE_PIN_NAND_IO0_SD2DATA0_GPIO56, "NAND_IO0_SD2DATA0_GPIO56"),
	PINCTRL_PIN(ELITE_PIN_NAND_IO1_SD2DATA1_GPIO57, "NAND_IO1_SD2DATA1_GPIO57"),
	PINCTRL_PIN(ELITE_PIN_NAND_IO2_SD2DATA2_GPIO58, "NAND_IO2_SD2DATA2_GPIO58"),
	PINCTRL_PIN(ELITE_PIN_NAND_IO3_SD2DATA3_GPIO59, "NAND_IO3_SD2DATA3_GPIO59"),
	PINCTRL_PIN(ELITE_PIN_NAND_IO4_SD2DATA4_GPIO60, "NAND_IO4_SD2DATA4_GPIO60"),
	PINCTRL_PIN(ELITE_PIN_NAND_IO5_SD2DATA5_GPIO61, "NAND_IO5_SD2DATA5_GPIO61"),
	PINCTRL_PIN(ELITE_PIN_NAND_IO6_SD2DATA6_GPIO62, "NAND_IO6_SD2DATA6_GPIO62"),
	PINCTRL_PIN(ELITE_PIN_NAND_IO7_SD2DATA7_GPIO63, "NAND_IO7_SD2DATA7_GPIO63"),
	PINCTRL_PIN(ELITE_PIN_SD1_PWRSW_GPIO64, "SD1_PWRSW_GPIO64"),
	PINCTRL_PIN(ELITE_PIN_SD1_CLK_GPIO65, "SD1_CLK_GPIO65"),
	PINCTRL_PIN(ELITE_PIN_SD1_CMD_GPIO66, "SD1_CMD_GPIO66"),
	PINCTRL_PIN(ELITE_PIN_SD1_WP_GPIO67, "SD1_WP_GPIO67"),
	PINCTRL_PIN(ELITE_PIN_SD1_DATA0_GPIO68, "SD1_DATA0_GPIO68"),
	PINCTRL_PIN(ELITE_PIN_SD1_DATA1_GPIO69, "SD1_DATA1_GPIO69"),
	PINCTRL_PIN(ELITE_PIN_SD1_DATA2_GPIO70, "SD1_DATA2_GPIO70"),
	PINCTRL_PIN(ELITE_PIN_SD1_DATA3_GPIO71, "SD1_DATA3_GPIO71"),
	PINCTRL_PIN(ELITE_PIN_SD1_CD_GPIO72, "SD1_CD_GPIO72"),
	PINCTRL_PIN(ELITE_PIN_SF_DI_GPIO73, "SF_DI_GPIO73"),
	PINCTRL_PIN(ELITE_PIN_SF_DO_GPIO74, "SF_DO_GPIO74"),
	PINCTRL_PIN(ELITE_PIN_SF_CS0N_GPIO75, "SF_CS0N_GPIO75"),
	PINCTRL_PIN(ELITE_PIN_SF_CS1N_GPIO76, "SF_CS1N_GPIO76"),
	PINCTRL_PIN(ELITE_PIN_SF_CLK_GPIO77, "SF_CLK_GPIO77"),
	PINCTRL_PIN(ELITE_PIN_KPAD_ROW0_GPIO80, "KPAD_ROW0_GPIO80"),
	PINCTRL_PIN(ELITE_PIN_KPAD_ROW1_GPIO81, "KPAD_ROW1_GPIO81"),
	PINCTRL_PIN(ELITE_PIN_KPAD_ROW2_GPIO82, "KPAD_ROW2_GPIO82"),
	PINCTRL_PIN(ELITE_PIN_KPAD_ROW3_GPIO83, "KPAD_ROW3_GPIO83"),
	PINCTRL_PIN(ELITE_PIN_KPAD_COL0_GPIO84, "KPAD_COL0_GPIO84"),
	PINCTRL_PIN(ELITE_PIN_KPAD_COL1_GPIO85, "KPAD_COL1_GPIO85"),
	PINCTRL_PIN(ELITE_PIN_KPAD_COL2_GPIO86, "KPAD_COL2_GPIO86"),
	PINCTRL_PIN(ELITE_PIN_KPAD_COL3_GPIO87, "KPAD_COL3_GPIO87"),
	PINCTRL_PIN(ELITE_PIN_SM_IOUC_GPIO88, "SM_IOUC_GPIO88"),
	PINCTRL_PIN(ELITE_PIN_SM_STROBE_GPIO89, "SM_STROBE_GPIO89"),
	PINCTRL_PIN(ELITE_PIN_SM_CMDVCC_GPIO90, "SM_CMDVCC_GPIO90"),
	PINCTRL_PIN(ELITE_PIN_SM_RSTIN_GPIO91, "SM_RSTIN_GPIO91"),
	PINCTRL_PIN(ELITE_PIN_SM_OFF_GPIO92, "SM_OFF_GPIO92"),
	PINCTRL_PIN(ELITE_PIN_UART1_TXD_GPIO93, "UART1_TXD_GPIO93"),
	PINCTRL_PIN(ELITE_PIN_UART1_RXD_GPIO94, "UART1_RXD_GPIO94"),
	PINCTRL_PIN(ELITE_PIN_UART0_RTS_UART2TXD_SM1IOUC_GPIO95, "UART0_RTS_UART2TXD_SM1IOUC_GPIO95"),
	PINCTRL_PIN(ELITE_PIN_UART0_CTS_UART2RXD_SM1STROBE_GPIO96, "UART0_CTS_UART2RXD_SM1STROBE_GPIO96"),
	PINCTRL_PIN(ELITE_PIN_UART0_RXD_SM1CMDVCC_GPIO97, "UART0_RXD_SM1CMDVCC_GPIO97"),
	PINCTRL_PIN(ELITE_PIN_UART0_TXD_SM1RSTIN_GPIO98, "UART0_TXD_SM1RSTIN_GPIO98"),
	PINCTRL_PIN(ELITE_PIN_SPI0_CLK_GPIO99, "SPI0_CLK_GPIO99"),
	PINCTRL_PIN(ELITE_PIN_SPI0_MOSI_GPIO100, "SPI0_MOSI_GPIO100"),
	PINCTRL_PIN(ELITE_PIN_SPI0_MISO_GPIO101, "SPI0_MISO_GPIO101"),
	PINCTRL_PIN(ELITE_PIN_SPI0_SS0N_GPIO102, "SPI0_SS0N_GPIO102"),
	PINCTRL_PIN(ELITE_PIN_I2S_DACMCLK_GPIO104, "I2S_DACMCLK_GPIO104"),
	PINCTRL_PIN(ELITE_PIN_I2S_DACBCLK_GPIO105, "I2S_DACBCLK_GPIO105"),
	PINCTRL_PIN(ELITE_PIN_I2S_DACLRC_GPIO106, "I2S_DACLRC_GPIO106"),
	PINCTRL_PIN(ELITE_PIN_I2S_DACDAT0_GPIO107, "I2S_DACDAT0_GPIO107"),
	PINCTRL_PIN(ELITE_PIN_I2S_DACDAT1_I2SADCBCLK_PCMSYNC_GPIO108, "I2S_DACDAT1_I2SADCBCLK_PCMSYNC_GPIO108"),
	PINCTRL_PIN(ELITE_PIN_I2S_DACDAT2_I2SADCLRC_PCMCLK_GPIO109, "I2S_DACDAT2_I2SADCLRC_PCMCLK_GPIO109"),
	PINCTRL_PIN(ELITE_PIN_I2S_DACDAT3_PCMIN_GPIO110, "I2S_DACDAT3_PCMIN_GPIO110"),
	PINCTRL_PIN(ELITE_PIN_I2S_ADCDAT0_GPIO111, "I2S_ADCDAT0_GPIO111"),
	PINCTRL_PIN(ELITE_PIN_I2S_ADCMCLK_I2SADCDAT1_PCMOUT_GPIO112, "I2S_ADCMCLK_I2SADCDAT1_PCMOUT_GPIO112"),
	PINCTRL_PIN(ELITE_PIN_PCM_MCLK_I2SADCDAT2_GPIO113, "PCM_MCLK_I2SADCDAT2_GPIO113"),
	PINCTRL_PIN(ELITE_PIN_SPDIFI_GPIO114, "SPDIFI_GPIO114"),
	PINCTRL_PIN(ELITE_PIN_SPDIFO_I2SADCDAT3_GPIO115, "SPDIFO_I2SADCDAT3_GPIO115"),
	PINCTRL_PIN(ELITE_PIN_I2C0_SCL_GPIO116, "I2C0_SCL_GPIO116"),
	PINCTRL_PIN(ELITE_PIN_I2C0_SDA_GPIO117, "I2C0_SDA_GPIO117"),
	PINCTRL_PIN(ELITE_PIN_I2C1_SCL_GPIO118, "I2C1_SCL_GPIO118"),
	PINCTRL_PIN(ELITE_PIN_I2C1_SDA_GPIO119, "I2C1_SDA_GPIO119"),
	//PINCTRL_PIN(ELITE_PIN_DISP_CLKO_GPIO120, "DISP_CLKO_GPIO120"),
	//PINCTRL_PIN(ELITE_PIN_DISP_CLKI_GPIO121, "DISP_CLKI_GPIO121"),
	//PINCTRL_PIN(ELITE_PIN_GFX_GPIO3_GPIO122, "GFX_GPIO3_GPIO122"),
	//PINCTRL_PIN(ELITE_PIN_GFX_GPIO0_GPIO123, "GFX_GPIO0_GPIO123"),
	//PINCTRL_PIN(ELITE_PIN_DIU_I2C1_DAT_GPIO124, "DIU_I2C1_DAT_GPIO124"),
	//PINCTRL_PIN(ELITE_PIN_DIU_I2C1_CLK_GPIO125, "DIU_I2C1_CLK_GPIO125"),
	//PINCTRL_PIN(ELITE_PIN_DIU_I2C0_DAT_GPIO126, "DIU_I2C0_DAT_GPIO126"),
	//PINCTRL_PIN(ELITE_PIN_DIU_I2C0_CLK_GPIO127, "DIU_I2C0_CLK_GPIO127"),
	PINCTRL_PIN(ELITE_PIN_TS_DIN0_GPIO128, "TS_DIN0_GPIO128"),
	PINCTRL_PIN(ELITE_PIN_TS_DIN1_TS2DAT_GPIO129, "TS_DIN1_TS2DAT_GPIO129"),
	PINCTRL_PIN(ELITE_PIN_TS_DIN2_TS2CLK_GPIO130, "TS_DIN2_TS2CLK_GPIO130"),
	PINCTRL_PIN(ELITE_PIN_TS_DIN3_TS2SYNC_GPIO131, "TS_DIN3_TS2SYNC_GPIO131"),
	PINCTRL_PIN(ELITE_PIN_TS_DIN4_TS2VALID_GPIO132, "TS_DIN4_TS2VALID_GPIO132"),
	PINCTRL_PIN(ELITE_PIN_TS_DIN5_TS3CLK_GPIO133, "TS_DIN5_TS3CLK_GPIO133"),
	PINCTRL_PIN(ELITE_PIN_TS_DIN6_TS3SYNC_GPIO134, "TS_DIN6_TS3SYNC_GPIO134"),
	PINCTRL_PIN(ELITE_PIN_TS_DIN7_TS3VALID_GPIO135, "TS_DIN7_TS3VALID_GPIO135"),
	PINCTRL_PIN(ELITE_PIN_TS_SYNC_GPIO136, "TS_SYNC_GPIO136"),
	PINCTRL_PIN(ELITE_PIN_TS_VALID_GPIO137, "TS_VALID_GPIO137"),
	PINCTRL_PIN(ELITE_PIN_TS_CLK_GPIO138, "TS_CLK_GPIO138"),
	PINCTRL_PIN(ELITE_PIN_TS3_DAT_GPIO139, "TS3_DAT_GPIO139"),
	PINCTRL_PIN(ELITE_PIN_TS1_DAT_GPIO140, "TS1_DAT_GPIO140"),
	PINCTRL_PIN(ELITE_PIN_TS1_CLk_GPIO141, "TS1_CLk_GPIO141"),
	PINCTRL_PIN(ELITE_PIN_TS1_SYNC_GPIO142, "TS1_SYNC_GPIO142"),
	PINCTRL_PIN(ELITE_PIN_TS1_VALID_GPIO143, "TS1_VALID_GPIO143"),
	PINCTRL_PIN(ELITE_PIN_AGC_OUT0_GPIO144, "AGC_OUT0_GPIO144"),
	PINCTRL_PIN(ELITE_PIN_AGC_OUT1_GPIO145, "AGC_OUT1_GPIO145"),
	PINCTRL_PIN(ELITE_PIN_USB_SW0_GPIO152, "USB_SW0_GPIO152"),
	PINCTRL_PIN(ELITE_PIN_USB_ATTA0_GPIO153, "USB_ATTA0_GPIO153"),
	PINCTRL_PIN(ELITE_PIN_USB_OC0_GPIO154, "USB_OC0_GPIO154"),
	PINCTRL_PIN(ELITE_PIN_USB_OC1_GPIO155, "USB_OC1_GPIO155"),
	PINCTRL_PIN(ELITE_PIN_USB_OC2_GPIO156, "USB_OC2_GPIO156"),
	PINCTRL_PIN(ELITE_PIN_USB_OC3_GPIO157, "USB_OC3_GPIO157"),	
};

static const unsigned mmc0_pins[] = {
	ELITE_PIN_SD0_CD_GPIO28,
//	ELITE_PIN_SD0_PWRSW_GPIO32,
	ELITE_PIN_SD0_CLK_GPIO33,
	ELITE_PIN_SD0_CMD_GPIO34,
	ELITE_PIN_SD0_WP_GPIO35,
	ELITE_PIN_SD0_DATA0_GPIO36,
	ELITE_PIN_SD0_DATA1_GPIO37,
};
static const unsigned pcie_pins[] = {
	ELITE_PIN_PCIE_RST0Z_GPIO40,
	ELITE_PIN_PCIE_RST1Z_GPIO41,
	ELITE_PIN_PCIE_CLK_REQ0Z_GPIO42,
};
static const unsigned sata_led_pins[] = {
	ELITE_PIN_SATA_LED0_GPIO4,
};
static const unsigned nand_0_pins[] = {
	ELITE_PIN_NAND_WEN_GPIO45,
	ELITE_PIN_NAND_REN_GPIO46,
	ELITE_PIN_NAND_DQS_GPIO47,
	ELITE_PIN_NAND_RB1N_GPIO51,
	ELITE_PIN_NAND_CE0N_GPIO52,
	ELITE_PIN_NAND_CE1N_GPIO53,
};
static const unsigned mmc2_nand_1_pins[] = {
	ELITE_PIN_NAND_ALE_SD2CLK_GPIO43,
	ELITE_PIN_NAND_CLE_SD2CMD_GPIO44,
	ELITE_PIN_NAND_WPN_SD2PWRSW_GPIO48,
	ELITE_PIN_NAND_WPDN_SD2WP_GPIO49,
	ELITE_PIN_NAND_RB0N_SD2CD_GPIO50,
	ELITE_PIN_NAND_IO0_SD2DATA0_GPIO56,
	ELITE_PIN_NAND_IO1_SD2DATA1_GPIO57,
	ELITE_PIN_NAND_IO2_SD2DATA2_GPIO58,
	ELITE_PIN_NAND_IO3_SD2DATA3_GPIO59,
	ELITE_PIN_NAND_IO4_SD2DATA4_GPIO60,
	ELITE_PIN_NAND_IO5_SD2DATA5_GPIO61,
	ELITE_PIN_NAND_IO6_SD2DATA6_GPIO62,
	ELITE_PIN_NAND_IO7_SD2DATA7_GPIO63,
};
static const unsigned mmc1_pins[] = {
//	ELITE_PIN_SD1_PWRSW_GPIO64,
	ELITE_PIN_SD1_CLK_GPIO65,
	ELITE_PIN_SD1_CMD_GPIO66,
	ELITE_PIN_SD1_WP_GPIO67,
	ELITE_PIN_SD1_DATA0_GPIO68,
	ELITE_PIN_SD1_DATA1_GPIO69,
	ELITE_PIN_SD1_DATA2_GPIO70,
	ELITE_PIN_SD1_DATA3_GPIO71,
	ELITE_PIN_SD1_CD_GPIO72,
};
//serial flash
static const unsigned sf_pins[] = {
	ELITE_PIN_SF_DI_GPIO73,
	ELITE_PIN_SF_DO_GPIO74,
	ELITE_PIN_SF_CS0N_GPIO75,
	ELITE_PIN_SF_CS1N_GPIO76,
	ELITE_PIN_SF_CLK_GPIO77,
};
static const unsigned kpad_pins[] = {
	ELITE_PIN_KPAD_ROW0_GPIO80,
	ELITE_PIN_KPAD_ROW1_GPIO81,
	ELITE_PIN_KPAD_ROW2_GPIO82,
	ELITE_PIN_KPAD_ROW3_GPIO83,
	ELITE_PIN_KPAD_COL0_GPIO84,
	ELITE_PIN_KPAD_COL1_GPIO85,
	ELITE_PIN_KPAD_COL2_GPIO86,
	ELITE_PIN_KPAD_COL3_GPIO87,
};
//smart card
static const unsigned sm0_pins[] = {
	ELITE_PIN_SM_IOUC_GPIO88,
	ELITE_PIN_SM_STROBE_GPIO89,
	ELITE_PIN_SM_CMDVCC_GPIO90,
	ELITE_PIN_SM_RSTIN_GPIO91,
	ELITE_PIN_SM_OFF_GPIO92,
};
static const unsigned sm1_0_pins[] = {
	ELITE_PIN_SM1OFF_GPIO3,
};
static const unsigned uart0_0_sm1_1_pins[] = {
	ELITE_PIN_UART0_RXD_SM1CMDVCC_GPIO97,
	ELITE_PIN_UART0_TXD_SM1RSTIN_GPIO98,
};
static const unsigned uart0_1_uart2_sm1_2_pins[] = {
	ELITE_PIN_UART0_RTS_UART2TXD_SM1IOUC_GPIO95,
	ELITE_PIN_UART0_CTS_UART2RXD_SM1STROBE_GPIO96,
};
static const unsigned uart1_pins[] = {
	ELITE_PIN_UART1_TXD_GPIO93,
	ELITE_PIN_UART1_RXD_GPIO94,
};
static const unsigned pwm1_pins[] = {
	ELITE_PIN_PWMOUT1_GPIO0,
};
static const unsigned pwm2_pins[] = {
	ELITE_PIN_PWMOUT2_GPIO1,
};
static const unsigned pwm3_pins[] = {
	ELITE_PIN_PWMOUT3_GPIO2,
};
static const unsigned spi0_pins[] = {
	ELITE_PIN_SPI0SS1N_GPIO9,
	ELITE_PIN_SPI0SS2N_GPIO10,
	ELITE_PIN_SPI0SS3N_GPIO11,
	ELITE_PIN_SPI0_CLK_GPIO99,
	ELITE_PIN_SPI0_MOSI_GPIO100,
	ELITE_PIN_SPI0_MISO_GPIO101,
	ELITE_PIN_SPI0_SS0N_GPIO102,
};
static const unsigned spi1_pins[] = {
	ELITE_PIN_SPI1CLK_GPIO5,
	ELITE_PIN_SPI1MISO_GPIO6,
	ELITE_PIN_SPI1MOSI_GPIO7,
	ELITE_PIN_SPI1SS0N_GPIO8,
};
static const unsigned i2s_dac_0_pins[] = {
	ELITE_PIN_I2S_DACMCLK_GPIO104,
	ELITE_PIN_I2S_DACBCLK_GPIO105,
	ELITE_PIN_I2S_DACLRC_GPIO106,
	ELITE_PIN_I2S_DACDAT0_GPIO107,
};
static const unsigned i2s_adc_0_pins[] = {
	ELITE_PIN_I2S_ADCDAT0_GPIO111,
};
static const unsigned i2s_dac_1_i2s_adc_1_pcm_0_pins[] = {
	ELITE_PIN_I2S_DACDAT1_I2SADCBCLK_PCMSYNC_GPIO108,
	ELITE_PIN_I2S_DACDAT2_I2SADCLRC_PCMCLK_GPIO109,
};
static const unsigned i2s_adc_2_pcm_1_pins[] = {
	ELITE_PIN_I2S_ADCMCLK_I2SADCDAT1_PCMOUT_GPIO112,
};
static const unsigned i2s_dac_2_pcm_2_pins[] = {
	ELITE_PIN_I2S_DACDAT3_PCMIN_GPIO110,
};
static const unsigned i2s_adc_3_pcm_3_pins[] = {
	ELITE_PIN_PCM_MCLK_I2SADCDAT2_GPIO113,
};
static const unsigned i2s_adc_4_spdif_0_pins[] = {
	ELITE_PIN_SPDIFO_I2SADCDAT3_GPIO115,
};
static const unsigned spdif_1_pins[] = {
	ELITE_PIN_SPDIFI_GPIO114,
};
static const unsigned i2c0_pins[] = {
	ELITE_PIN_I2C0_SCL_GPIO116,
	ELITE_PIN_I2C0_SDA_GPIO117,
};
static const unsigned i2c1_pins[] = {
	ELITE_PIN_I2C1_SCL_GPIO118,
	ELITE_PIN_I2C1_SDA_GPIO119,
};
//transport stream
static const unsigned ts0_0_pins[] = {
	ELITE_PIN_TS_DIN0_GPIO128,
	ELITE_PIN_TS_SYNC_GPIO136,
	ELITE_PIN_TS_VALID_GPIO137,
	ELITE_PIN_TS_CLK_GPIO138,
};
static const unsigned ts0_1_ts2_pins[] = {
	ELITE_PIN_TS_DIN1_TS2DAT_GPIO129,
	ELITE_PIN_TS_DIN2_TS2CLK_GPIO130,
	ELITE_PIN_TS_DIN3_TS2SYNC_GPIO131,
	ELITE_PIN_TS_DIN4_TS2VALID_GPIO132,
};
static const unsigned ts0_2_ts3_0_pins[] = {
	ELITE_PIN_TS_DIN5_TS3CLK_GPIO133,
	ELITE_PIN_TS_DIN6_TS3SYNC_GPIO134,
	ELITE_PIN_TS_DIN7_TS3VALID_GPIO135,
};
static const unsigned ts1_pins[] = {
	ELITE_PIN_TS1_DAT_GPIO140,
	ELITE_PIN_TS1_CLk_GPIO141,
	ELITE_PIN_TS1_SYNC_GPIO142,
	ELITE_PIN_TS1_VALID_GPIO143,
};
static const unsigned ts3_1_pins[] = {
	ELITE_PIN_TS3_DAT_GPIO139,
};
//automatic gain control
static const unsigned agc_pins[] = {
	ELITE_PIN_AGC_OUT0_GPIO144,
	ELITE_PIN_AGC_OUT1_GPIO145,
};

static const unsigned cir_pins[] = {
	ELITE_PIN_SUS_GPIO21,
};


static const unsigned usb_pins[] = {
	// ELITE_PIN_USB_SW0_GPIO152,
	ELITE_PIN_USB_ATTA0_GPIO153,
	ELITE_PIN_USB_OC0_GPIO154,
	ELITE_PIN_USB_OC1_GPIO155,
	ELITE_PIN_USB_OC2_GPIO156,
	ELITE_PIN_USB_OC3_GPIO157,
};

#define GROUP(fname)					\
	{						\
		.name = #fname"_grp",				\
		.pins = fname##_pins,		\
		.num_pins = ARRAY_SIZE(fname##_pins),	\
	}

static const struct elite_group elite_groups[] = {
	GROUP(mmc0),
	GROUP(usb),
	GROUP(pcie),
	GROUP(cir),
	GROUP(sata_led),
	GROUP(nand_0),
	GROUP(mmc2_nand_1),
	GROUP(mmc1),
	GROUP(sf),
	GROUP(kpad),
	GROUP(sm0),
	GROUP(sm1_0),
	GROUP(uart0_0_sm1_1),
	GROUP(uart1),
	GROUP(uart0_1_uart2_sm1_2),
	GROUP(pwm1),
	GROUP(pwm2),
	GROUP(pwm3),
	GROUP(spi0),
	GROUP(spi1),
	GROUP(i2s_dac_0),
	GROUP(i2s_adc_0),
	GROUP(i2s_dac_1_i2s_adc_1_pcm_0),
	GROUP(i2s_adc_2_pcm_1),
	GROUP(i2s_dac_2_pcm_2),
	GROUP(i2s_adc_3_pcm_3),
	GROUP(i2s_adc_4_spdif_0),
	GROUP(spdif_1),
	GROUP(i2c0),
	GROUP(i2c1),
	GROUP(ts0_0),
	GROUP(ts0_1_ts2),
	GROUP(ts0_2_ts3_0),
	GROUP(ts1),
	GROUP(ts3_1),
	GROUP(agc),
};

static int elite_pinctrl_list_groups(struct pinctrl_dev *pctldev, unsigned selector)
{
	if (selector >= ARRAY_SIZE(elite_groups))
		return -EINVAL;
	
	return 0;
}

static const char *elite_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
				       unsigned selector)
{
	if (selector >= ARRAY_SIZE(elite_groups))
		return NULL;
	return elite_groups[selector].name;
}

static int elite_pinctrl_get_group_pins(struct pinctrl_dev *pctldev, unsigned selector,
			       const unsigned ** pins,
			       unsigned * num_pins)
{
	if (selector >= ARRAY_SIZE(elite_groups))
		return -EINVAL;

	*pins = (unsigned *) elite_groups[selector].pins;
	*num_pins = elite_groups[selector].num_pins;
	
	return 0;
}

static void elite_pinctrl_pin_dbg_show(struct pinctrl_dev *pctldev,
				       struct seq_file *s,
				       unsigned offset)
{
	seq_printf(s, " " DRIVER_NAME);
}

static struct pinctrl_ops elite_pctrl_ops = {
	.list_groups = elite_pinctrl_list_groups,
	.get_group_name = elite_pinctrl_get_group_name,
	.get_group_pins = elite_pinctrl_get_group_pins,
	.pin_dbg_show = elite_pinctrl_pin_dbg_show,
};
static const char * const cir_groups[] = { 
	"cir_grp" 
};

static const char * const mmc0_groups[] = { 
	"mmc0_grp" 
};
static const char * const pcie_groups[] = { 
	"pcie_grp" 
};
static const char * const sata_led_groups[] = {
	"sata_led_grp",
};
static const char * const nand_groups[] = {
	"nand_0_grp", 
	"mmc2_nand_1_grp" 
};
static const char * const mmc2_groups[] = {
	"mmc2_nand_1_grp"
};
static const char * const mmc1_groups[] = {
	"mmc1_grp"
};
static const char * const sf_groups[] = {
	"sf_grp" 
};
static const char * const kpad_groups[] = {
	"kpad_grp" 
};
static const char * const sm0_groups[] = { 
	"sm0_grp" 
};
static const char * const sm1_groups[] = { 
	"sm1_0_grp" ,
	"uart0_0_sm1_1_grp",
	"uart0_1_uart2_sm1_2"
};
static const char * const uart0_groups[] = { 
	"uart0_0_sm1_1_grp" ,
	"uart0_1_uart2_sm1_2_grp"
};
static const char * const uart1_groups[] = { 
	"uart1_grp" 
};
static const char * const uart2_groups[] = { 
	"uart0_1_uart2_sm1_2_grp" 
};
static const char * const pwm1_groups[] = { 
	"pwm1_grp" 
};
static const char * const pwm2_groups[] = { 
	"pwm2_grp" 
};
static const char * const pwm3_groups[] = { 
	"pwm3_grp" 
};
static const char * const spi0_groups[] = { 
	"spi0_grp" 
};
static const char * const spi1_groups[] = { 
	"spi1_grp" 
};
static const char * const i2s_dac_groups[] = { 
	"i2s_dac_0_grp",
	"i2s_dac_1_i2s_adc_1_pcm_0_grp",
	"i2s_dac_2_pcm_2_grp",
	
	"i2s_adc_0_grp",
	"i2s_adc_2_pcm_1_grp",
	"i2s_adc_3_pcm_3_grp",
	"i2s_adc_4_spdif_0_grp",

	"spdif_1_grp",
};
static const char * const i2s_adc_groups[] = { 
	"i2s_dac_0_grp",
	"i2s_dac_1_i2s_adc_1_pcm_0_grp",
	"i2s_dac_2_pcm_2_grp",
	
	"i2s_adc_0_grp",
	"i2s_adc_2_pcm_1_grp",
	"i2s_adc_3_pcm_3_grp",
	"i2s_adc_4_spdif_0_grp",

	"spdif_1_grp",
};
static const char * const pcm_groups[] = { 
	"i2s_dac_1_i2s_adc_1_pcm_0_grp",
	"i2s_adc_2_pcm_1_grp",
	"i2s_dac_2_pcm_2_grp",
	"i2s_adc_3_pcm_3_grp"
};
static const char * const spdif_groups[] = { 
	"i2s_adc_4_spdif_0_grp",
	"spdif_1_grp"
};
static const char * const i2c0_groups[] = { 
	"i2c0_grp" 
};
static const char * const i2c1_groups[] = { 
	"i2c1_grp" 
};
static const char * const ts0_groups[] = { 
	"ts0_0_grp",
	"ts0_1_ts2_grp",
	"ts0_2_ts3_0_grp"
};
static const char * const ts1_groups[] = { 
	"ts1_grp" 
};
static const char * const ts2_groups[] = { 
	"ts0_1_ts2_grp" 
};
static const char * const ts3_groups[] = { 
	"ts0_2_ts3_0_grp",
	"ts3_1_grp"
};
static const char * const agc_groups[] = { 
	"agc_grp"
};
static const char * const usb_groups[] = { 
	"usb_grp" 
};

#define FUNCTION(gname, _regop, _val)					\
	{						\
		.name = #gname,				\
		.groups = gname ## _groups,		\
		.num_groups = ARRAY_SIZE(gname ## _groups),	\
		.regop = BITWISE_ ## _regop,		\
		.val = _val,		\
	}

static const struct elite_pmx_func elite_functions[] = {
	FUNCTION(mmc0, NONE, 0),
	FUNCTION(usb, NONE, 0),
	FUNCTION(pcie, NONE, 0),
	FUNCTION(nand, AND, PMX_NAND),
	FUNCTION(mmc2, OR, PMX_SD2),
	FUNCTION(mmc1, NONE, 0),
	FUNCTION(sf, NONE, 0),
	FUNCTION(kpad, AND, PMX_KPAD_EN),
	FUNCTION(sm0, NONE, 0),
	FUNCTION(sm1, OR, PMX_SM1),
	FUNCTION(uart0, AND, PMX_UART0),
	FUNCTION(uart1, NONE, 0),
	FUNCTION(uart2, OR, PMX_UART2),
	FUNCTION(pwm1, OR, PMX_PWM_EN),
	FUNCTION(pwm2, OR, PMX_PWM_EN),
	FUNCTION(pwm3, OR, PMX_PWM_EN),
	FUNCTION(spi0, OR, PMX_SPI_EN),
	FUNCTION(spi1, OR, PMX_SPI_EN),
	FUNCTION(i2s_adc, COMP, PMX_I2S_ADC),
	FUNCTION(i2s_dac, COMP, PMX_I2S_DAC),
	FUNCTION(pcm, COMP, PMX_PCM),
	FUNCTION(spdif, NONE, 0),
	FUNCTION(i2c0, NONE, 0),
	FUNCTION(i2c1, NONE, 0),
	FUNCTION(ts0, NONE, 0),
	FUNCTION(cir, NONE, 0),
	FUNCTION(ts1, AND, PMX_TS1),
	FUNCTION(ts2, OR, PMX_TS23),
	FUNCTION(ts3, OR, PMX_TS23),
	FUNCTION(agc, NONE, 0),
};

int elite_pinmux_list_funcs(struct pinctrl_dev *pctldev, unsigned selector)
{
	if (selector >= ARRAY_SIZE(elite_functions))
		return -EINVAL;
	
	return 0;
}

const char *elite_pinmux_get_fname(struct pinctrl_dev *pctldev, unsigned selector)
{
	return elite_functions[selector].name;
}

static int elite_pinmux_get_groups(struct pinctrl_dev *pctldev, unsigned selector,
			  const char * const **groups,
			  unsigned * const num_groups)
{
	*groups = elite_functions[selector].groups;
	*num_groups = elite_functions[selector].num_groups;
	
	return 0;
}

int elite_pinmux_enable(struct pinctrl_dev *pctldev, unsigned selector,
		unsigned group)
{
	struct elite_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	u32 val = elite_pinctrl_pinmux_get(pctrl);
	int i;

	switch(elite_functions[selector].regop) {
	case BITWISE_NONE:
		return 0;
	case BITWISE_OR:
		val |= elite_functions[selector].val;
		break;
	case BITWISE_AND:
		val &= elite_functions[selector].val;
		break;
	case BITWISE_COMP:
		val &= ~PMX_I2S_PCM_MASK;
		val |= (elite_functions[selector].val & PMX_I2S_PCM_MASK) | PMX_I2SMCLK_EN;
		break;
	default:
		return -EINVAL;
	}

	elite_pinctrl_pinmux_set(pctrl, val);

	dev_dbg(pctrl->dev, "Enable group: %s, Shared reg val: 0x%08x\n", 
		elite_groups[group].name, val);
	
	for (i = 0; i < elite_groups[group].num_pins; i++)
		elite_pinctrl_func_en(pctrl, *(elite_groups[group].pins+i), 0);

	return 0;
}

void elite_pinmux_disable(struct pinctrl_dev *pctldev, unsigned selector,
		unsigned group)
{
	struct elite_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	u32 val = elite_pinctrl_pinmux_get(pctrl);
	int i;

	switch(elite_functions[selector].val) {
	case PMX_I2SMCLK_EN:
	case PMX_SATALED_EN:
	case PMX_KPAD_EN:
	case PMX_PWM_EN:
	case PMX_SPI_EN:
		val &= elite_functions[selector].val;
		break;
	default:
		goto out;
	}

	elite_pinctrl_pinmux_set(pctrl, val);
out:	
	dev_dbg(pctrl->dev, "Disable group: %s, Shared reg val: 0x%08x\n", 
		elite_groups[group].name, val); 

	for (i = 0; i < elite_groups[group].num_pins; i++)
		elite_pinctrl_func_en(pctrl, *(elite_groups[group].pins+i), 0);
}

static int elite_pinmux_request_gpio(struct pinctrl_dev *pctldev,
	struct pinctrl_gpio_range *range, unsigned offset)
{
	struct elite_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(pctrl->dev, "Requested GPIO: %u\n", offset); 
	elite_pinctrl_func_en(pctrl, offset, 1);
	return 0;
}

static void elite_pinmux_free_gpio(struct pinctrl_dev *pctldev,
	struct pinctrl_gpio_range *range, unsigned offset)
{
	struct elite_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	elite_pinctrl_func_en(pctrl, offset, 0);
}

struct pinmux_ops elite_pmx_ops = {
	.list_functions = elite_pinmux_list_funcs,
	.get_function_name = elite_pinmux_get_fname,
	.get_function_groups = elite_pinmux_get_groups,
	.enable = elite_pinmux_enable,
	.disable = elite_pinmux_disable,
	.gpio_request_enable = elite_pinmux_request_gpio,
	.gpio_disable_free = elite_pinmux_free_gpio,
};

static int elite_pin_config_get(struct pinctrl_dev *pctldev,
		    unsigned offset,
		    unsigned long *config)
{
	struct elite_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	enum elite_pinconf_param param = ELITE_PINCONF_UNPACK_PARAM(*config);
	u16 arg;
	
	dev_dbg(pctrl->dev, "Get Config PIN NUM: %u\n", offset);

	if (ELITE_PINCONF_PARAM_PULL != param)
		return -ENOTSUPP;
	/* Find setting for pin @ offset */
	arg = elite_pinctrl_pullupdown_ctrl0_get(pctrl, offset);

	*config = ELITE_PINCONF_PACK(param, arg);

	return 0;
}

static int elite_pin_config_set(struct pinctrl_dev *pctldev,
		    unsigned offset,
		    unsigned long config)
{
	struct elite_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	enum elite_pinconf_param param = ELITE_PINCONF_UNPACK_PARAM(config);
	u16 arg = ELITE_PINCONF_UNPACK_ARG(config);
	enum elite_pinconf_pull pull = (enum elite_pinconf_pull)arg;

	dev_dbg(pctrl->dev, "Set Config PIN NUM: %u\n", offset);
  
	if (ELITE_PINCONF_PARAM_PULL != param)
		return -ENOTSUPP;
			
	elite_pinctrl_pullupdown_en(pctrl, offset, 1);
	if (pull != ELITE_PINCONF_PULL_NONE)
		elite_pinctrl_pullupdown_ctrl0_set(pctrl, offset, pull);
	
	return 0;
}

static int elite_pin_config_group_get (struct pinctrl_dev *pctldev,
		    unsigned selector,
		    unsigned long *config)
{
	struct elite_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	enum elite_pinconf_param param = ELITE_PINCONF_UNPACK_PARAM(*config);
	u16 arg;

	if(strcmp(elite_functions[selector].name, "mmc0") &&
		strcmp(elite_functions[selector].name, "usb"))
		return -ENOTSUPP;

	switch (param) {
	case ELITE_PINCONF_PARAM_DRV:
		arg = elite_pinctrl_sd0_drive_strength_get(pctrl);
		break;
	case ELITE_PINCONF_PARAM_VOLT:
		arg = elite_pinctrl_sd0_signal_voltage_get(pctrl);
		break;
	case ELITE_PINCONF_PARAM_USB:
		arg = (u16)elite_pinctrl_usb_mode_get(pctrl);
		break;
	default:
		return -EINVAL;
	}
	*config = ELITE_PINCONF_PACK(param, arg);
	
	return 0;
}

static int elite_pin_config_group_set (struct pinctrl_dev *pctldev,
		    unsigned selector,
		    unsigned long config)
{
	struct elite_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	enum elite_pinconf_param param = ELITE_PINCONF_UNPACK_PARAM(config);
	u16 arg = ELITE_PINCONF_UNPACK_ARG(config);

	if(strcmp(elite_functions[selector].name, "mmc0") &&
		strcmp(elite_functions[selector].name, "usb"))
		return -ENOTSUPP;
	
	switch (param) {
	case ELITE_PINCONF_PARAM_DRV:
		dev_dbg(pctrl->dev, "Config PIN group: drive strength\n"); 
		elite_pinctrl_sd0_drive_strength_set(pctrl, (enum elite_pinconf_drvstrength)arg);
		break;
	case ELITE_PINCONF_PARAM_VOLT:
		dev_dbg(pctrl->dev, "Config PIN group: signal voltage\n"); 
		elite_pinctrl_sd0_signal_voltage_set(pctrl, (enum elite_pinconf_voltage)arg);
		break;
	case ELITE_PINCONF_PARAM_USB:
		dev_dbg(pctrl->dev, "Config PIN group: USB\n"); 
		elite_pinctrl_usb_mode_set(pctrl, (enum elite_pinconf_usb_mode)arg);
              break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void elite_pin_config_dbg_show(struct pinctrl_dev *pctldev,
				   struct seq_file *s, unsigned offset)
{
}

static void elite_pin_config_group_dbg_show(struct pinctrl_dev *pctldev,
					 struct seq_file *s, unsigned selector)
{
}

static struct pinconf_ops elite_pconf_ops = {
	.pin_config_get = elite_pin_config_get,
	.pin_config_set = elite_pin_config_set,
	.pin_config_group_get = elite_pin_config_group_get,
	.pin_config_group_set = elite_pin_config_group_set,
	.pin_config_dbg_show = elite_pin_config_dbg_show,
	.pin_config_group_dbg_show = elite_pin_config_group_dbg_show,
};


static struct pinctrl_gpio_range elite_pinctrl_gpio_range = {
	.name = "Elite GPIOs",
	.id = 0,
	.base = 0,
	.pin_base = 0,
	.npins = ARCH_NR_GPIOS,
};

static struct pinctrl_desc elite_pinctrl_desc = {
	.name = DRIVER_NAME,
	.pins = elite_pins,
	.npins = ARRAY_SIZE(elite_pins), //NUM_GPIOS
	.pctlops = &elite_pctrl_ops,
	.pmxops = &elite_pmx_ops,
	.confops = &elite_pconf_ops,
	.owner = THIS_MODULE,
};

static struct of_device_id elite_pinctrl_of_match[] __devinitdata = {
	{ .compatible = "s3graphics,elite1000-pinctrl", .data = NULL },
	{ },
};
MODULE_DEVICE_TABLE(of, elite_pinctrl_of_match);

static int __devinit elite_pinctrl_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct elite_pinctrl *pctrl;
	struct resource *res;
	int ret = 0;

	match = of_match_device(elite_pinctrl_of_match, &pdev->dev);

	if (!match) {
		dev_err(&pdev->dev, "Can't match the device\n");
	}

	pctrl = devm_kzalloc(&pdev->dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl) {
		dev_err(&pdev->dev, "Failed to alloc elite_pctrl\n");
		return -ENOMEM;
	}
	pctrl->dev = &pdev->dev;

	platform_set_drvdata(pdev, pctrl);
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "MEM resource is missed\n");
		ret = -ENODEV;
		goto err;
	}
#if 0 //share the same memory space with GPIO lib driver	
	if (!devm_request_mem_region(&pdev->dev, res->start,
					resource_size(res),
					dev_name(&pdev->dev))) {
		dev_err(&pdev->dev,
			"Failed to request MEM resource\n");
		ret = -ENODEV;
		goto err;
	}
#endif	
	pctrl->virt_base = devm_ioremap(&pdev->dev, res->start,
					resource_size(res));
	if (!pctrl->virt_base) {
		dev_err(&pdev->dev, "Failed to ioremap regs\n");
		ret = -ENODEV;
		goto err;
	}

	
	pctrl->pctrl = pinctrl_register(&elite_pinctrl_desc, &pdev->dev, pctrl);
	if (IS_ERR(pctrl->pctrl)) {
		dev_err(&pdev->dev, "Failed to register pinctrl driver\n");
		ret = PTR_ERR(pctrl->pctrl);
		goto err;
	}

	pinctrl_add_gpio_range(pctrl->pctrl, &elite_pinctrl_gpio_range);

	dev_info(&pdev->dev, "Probed elite pinctrl driver: %s\n", 
		pinctrl_dev_get_name(pctrl->pctrl));

	return ret;
err:
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, pctrl);
	return ret;
}

static int __devexit elite_pinctrl_remove(struct platform_device *pdev)
{
	struct elite_pinctrl *pctrl = platform_get_drvdata(pdev);

	pinctrl_remove_gpio_range(pctrl->pctrl, &elite_pinctrl_gpio_range);
	pinctrl_unregister(pctrl->pctrl);

	return 0;
}

static struct platform_device_id elite_pinctrl_driver_ids[] = {
    {
        .name  = "elite-pinctrl",
		.driver_data	= (kernel_ulong_t)NULL,
    }, 
	{ /* sentinel */ },
};

static struct platform_driver elite_pinctrl_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = elite_pinctrl_of_match,
	},
	.id_table  = elite_pinctrl_driver_ids,
	.probe = elite_pinctrl_probe,
	.remove = __devexit_p(elite_pinctrl_remove),
};

static int __init elite_pinctrl_init(void)
{
	return platform_driver_register(&elite_pinctrl_driver);
}
//arch_initcall(elite_pinctrl_init);
core_initcall(elite_pinctrl_init);

static void __exit elite_pinctrl_exit(void)
{
	platform_driver_unregister(&elite_pinctrl_driver);
}
module_exit(elite_pinctrl_exit);


MODULE_AUTHOR("S3 Graphics Linux-OpenGL-Platform");
MODULE_DESCRIPTION("elite SoC pinctrl driver");
MODULE_LICENSE("GPL v2");
