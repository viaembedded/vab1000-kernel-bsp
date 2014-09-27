/*
 * linux/arch/arm/mach-elite/generic.c

 * elite generic architecture level codes

 * This program is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/cpufreq.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#include <linux/regulator/machine.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-generic.h>

#include <linux/mfd/wm831x/core.h>
#include <linux/mfd/wm831x/pdata.h>
#include <linux/mfd/wm831x/irq.h>
#include <linux/mfd/wm831x/gpio.h>

#include <asm/mach-types.h>
#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>
#include <asm/irq.h>
#include <asm/sizes.h>
#include <asm/io.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <asm/hardware/gic.h>
#include <asm/pgalloc.h>

#include <mach/iomap.h>
#include <mach/pinconf-elite.h>
#include <mach/mmc.h>
#include <mach/ehci.h>

#include "board.h"
#include "cpu-elite.h"
#include "elite-pg.h"

unsigned long elite_pgd_phys;
static pgd_t *elite_pgd;
extern void elite_pmu_enable(void);
extern void elite_init_wakeupcfg(void);
extern void elite_pmu_reset(char mode, const char *cmd);
extern void elite_pmu_power_off(char mode, const char *cmd);
void *elite_context_area = NULL;
extern unsigned int system_rev;
/* define hog settings that are always done on boot */
#define ELITE_MUX_HOG(group,func) \
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("elite-pinctrl.0",group,func)
#define ELITE_PIN_GROUP_HOG(group,conf) \
	PIN_MAP_CONFIGS_GROUP_HOG_DEFAULT("elite-pinctrl.0",group,conf)
#define ELITE_PIN_HOG(pin,conf) \
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("elite-pinctrl.0",pin,conf)
#define ELITE_PIN_IDLE(pin,conf,dev) \
	PIN_MAP_CONFIGS_PIN(dev, PINCTRL_STATE_IDLE, "elite-pinctrl.0", \
	pin, conf)

/* These default states associated with device and that changed at runtime */
#define ELITE_MUX(group,func,dev) \
	PIN_MAP_MUX_GROUP_DEFAULT(dev,"elite-pinctrl.0",group,func)
#define ELITE_PIN_GROUP(group,conf,dev) \
	PIN_MAP_CONFIGS_GROUP_DEFAULT(dev,"elite-pinctrl.0",group,conf)
#define ELITE_PIN(pin,conf,dev) \
	PIN_MAP_CONFIGS_PIN_DEFAULT(dev,"elite-pinctrl.0",pin,conf)
/* SD/MMC specific state */
#define ELITE_PIN_GROUP_SD(group,conf,dev) \
	PIN_MAP_CONFIGS_GROUP(dev, PINCTRL_STATE_SD_PULLDOWN, "elite-pinctrl.0", group, conf)
#define ELITE_PIN_SD(pin,conf,dev) \
	PIN_MAP_CONFIGS_PIN(dev, PINCTRL_STATE_SD_PULLDOWN, "elite-pinctrl.0", pin, conf)

static unsigned long pin_pullup_conf[] = {
	ELITE_PINCONF_PACK(ELITE_PINCONF_PARAM_PULL, ELITE_PINCONF_PULL_UP),
};

static unsigned long pin_pulldown_conf[] = {
	ELITE_PINCONF_PACK(ELITE_PINCONF_PARAM_PULL, ELITE_PINCONF_PULL_DOWN),
};

static unsigned long group_usbmode_conf[] = {
	ELITE_PINCONF_PACK(ELITE_PINCONF_PARAM_USB, ELITE_PINCONF_HOST),
};

/* SD 3.0 drive strength control */
#if 0
static unsigned long group_drvstrength_type_a_conf[] = {
	ELITE_PINCONF_PACK(ELITE_PINCONF_PARAM_DRV, ELITE_PINCONF_PAD_DRV_TYPE_A),
};
#endif
static unsigned long group_drvstrength_type_b_conf[] = {
	ELITE_PINCONF_PACK(ELITE_PINCONF_PARAM_DRV, ELITE_PINCONF_PAD_DRV_TYPE_B),
};
#if 0
static unsigned long group_drvstrength_type_c_conf[] = {
	ELITE_PINCONF_PACK(ELITE_PINCONF_PARAM_DRV, ELITE_PINCONF_PAD_DRV_TYPE_C),
};

static unsigned long group_drvstrength_type_d_conf[] = {
	ELITE_PINCONF_PACK(ELITE_PINCONF_PARAM_DRV, ELITE_PINCONF_PAD_DRV_TYPE_D),
};

/* SD 3.0 signal voltage control */
static unsigned long group_voltage_1_8_conf[] = {
	ELITE_PINCONF_PACK(ELITE_PINCONF_PARAM_VOLT, ELITE_PINCONF_PAD_1_8),
};
#endif
static unsigned long group_voltage_3_3_conf[] = {
	ELITE_PINCONF_PACK(ELITE_PINCONF_PARAM_VOLT, ELITE_PINCONF_PAD_3_3),
};

/* Pin control settings */
static struct pinctrl_map __initdata elite_pinmux_map[] = {
#if 0
	/**
	 * booting up time stuff
	 * anonymous maps for PCIe, etc.
	 */
	/* EFuse*/
	ELITE_PIN_HOG("SPI0SS3N_GPIO11",pin_pullup_conf),//EN_EFUSE_SOC
	/* GPU */
	ELITE_PIN_HOG("SF_CS1N_GPIO76",pin_pulldown_conf), //Select DAC to output HDTV(0) or CRT(1)
	/* PowerManagment */
	ELITE_PIN_HOG("WAKEUP_GPIO18",pin_pulldown_conf),//wakeup input 2
		/* SATA LED */
	ELITE_MUX_HOG("sata_led_grp", "sata_led"),
	ELITE_PIN_HOG("SATA_LED0_GPIO4", pin_pullup_conf),

	/**
	 * Runtime stuff
	 * per-device maps for MMC/SD, SPI and UART, etc.
	 */
#endif

#if 0
	/* PWM */
	ELITE_MUX_HOG("pwm1_grp", "pwm1"),
	ELITE_MUX_HOG("pwm2_grp", "pwm2"),
	ELITE_MUX_HOG("pwm3_grp", "pwm3"),
	ELITE_PIN_HOG("PWMOUT1_GPIO0",pin_pulldown_conf),
	ELITE_PIN_HOG("PWMOUT2_GPIO1",pin_pulldown_conf),
	ELITE_PIN_HOG("PWMOUT3_GPIO2",pin_pulldown_conf),
#endif
	
	/* cir  */
	ELITE_MUX_HOG("cir_grp", "cir"),

	/* SPI */
	ELITE_MUX_HOG("spi0_grp", "spi0"),
	ELITE_MUX_HOG("spi1_grp", "spi1"),
	ELITE_PIN_HOG("SPI0_CLK_GPIO99",pin_pulldown_conf),
	ELITE_PIN_HOG("SPI0_MOSI_GPIO100",pin_pulldown_conf),
	ELITE_PIN_HOG("SPI0_MISO_GPIO101",pin_pulldown_conf),
	ELITE_PIN_HOG("SPI0_SS0N_GPIO102",pin_pullup_conf),	
	ELITE_PIN_HOG("SPI0SS1N_GPIO9",pin_pullup_conf),
	ELITE_PIN_HOG("SPI0SS2N_GPIO10",pin_pullup_conf),
	ELITE_PIN_HOG("SPI0SS3N_GPIO11",pin_pullup_conf),
	ELITE_PIN_HOG("SPI1CLK_GPIO5",pin_pulldown_conf),
	ELITE_PIN_HOG("SPI1MISO_GPIO6",pin_pulldown_conf),
	ELITE_PIN_HOG("SPI1MOSI_GPIO7",pin_pulldown_conf),
	ELITE_PIN_HOG("SPI1SS0N_GPIO8",pin_pullup_conf),



	/* PCIe */
	ELITE_MUX_HOG("pcie_grp","pcie"),
	ELITE_PIN_HOG("WAKEUP_GPIO16",pin_pullup_conf),//WAKEUP2_SUS
	ELITE_PIN_HOG("WAKEUP_GPIO17",pin_pullup_conf),//wakeup input 0 from PPCIE port0
	ELITE_PIN_HOG("PCIE_RST0Z_GPIO40",pin_pulldown_conf),//PCIE Host 0 Reset output, active-low
	ELITE_PIN_HOG("PCIE_RST1Z_GPIO41",pin_pulldown_conf),//PCIE Host 1 Reset output, active-low
	ELITE_PIN_HOG("PCIE_CLK_REQ0Z_GPIO42",pin_pullup_conf),//PCIE clock request input, active-low	

	/* NAND */
	ELITE_MUX_HOG("mmc2_nand_1_grp", "nand"),
	ELITE_PIN_HOG("NAND_IO0_SD2DATA0_GPIO56",pin_pulldown_conf),
	ELITE_PIN_HOG("NAND_IO1_SD2DATA1_GPIO57",pin_pulldown_conf),
	ELITE_PIN_HOG("NAND_IO2_SD2DATA2_GPIO58",pin_pulldown_conf),
	ELITE_PIN_HOG("NAND_IO3_SD2DATA3_GPIO59",pin_pulldown_conf),
	ELITE_PIN_HOG("NAND_IO4_SD2DATA4_GPIO60",pin_pulldown_conf),
	ELITE_PIN_HOG("NAND_IO5_SD2DATA5_GPIO61",pin_pulldown_conf),
	ELITE_PIN_HOG("NAND_IO6_SD2DATA6_GPIO62",pin_pulldown_conf),
	ELITE_PIN_HOG("NAND_IO7_SD2DATA7_GPIO63",pin_pulldown_conf),
	ELITE_PIN_HOG("NAND_ALE_SD2CLK_GPIO43",pin_pullup_conf),
	ELITE_PIN_HOG("NAND_CLE_SD2CMD_GPIO44",pin_pulldown_conf),
	ELITE_PIN_HOG("NAND_WEN_GPIO45",pin_pullup_conf),
	ELITE_PIN_HOG("NAND_REN_GPIO46",pin_pullup_conf),
	ELITE_PIN_HOG("NAND_DQS_GPIO47",pin_pulldown_conf),
	ELITE_PIN_HOG("NAND_WPN_SD2PWRSW_GPIO48",pin_pullup_conf),
	ELITE_PIN_HOG("NAND_WPDN_SD2WP_GPIO49",pin_pullup_conf),
	ELITE_PIN_HOG("NAND_RB0N_SD2CD_GPIO50",pin_pullup_conf),
	ELITE_PIN_HOG("NAND_RB1N_GPIO51",pin_pullup_conf),
	ELITE_PIN_HOG("NAND_CE0N_GPIO52",pin_pullup_conf),
	ELITE_PIN_HOG("NAND_CE1N_GPIO53",pin_pullup_conf),

	/* i2c0 */
	ELITE_MUX_HOG("i2c0_grp", "i2c0"),
	ELITE_PIN_HOG("I2C0_SCL_GPIO116",pin_pullup_conf),
	ELITE_PIN_HOG("I2C0_SDA_GPIO117",pin_pullup_conf),
	/* i2c1 */
	ELITE_MUX_HOG("i2c1_grp", "i2c1"),
	ELITE_PIN_HOG("I2C1_SCL_GPIO118",pin_pullup_conf),
	ELITE_PIN_HOG("I2C1_SDA_GPIO119",pin_pullup_conf),

	/* USB */
	ELITE_MUX_HOG("usb_grp", "usb"),
	ELITE_PIN_GROUP_HOG("usb_grp",group_usbmode_conf),
	ELITE_PIN_HOG("USB_SW0_GPIO152",pin_pullup_conf),
	ELITE_PIN_HOG("USB_ATTA0_GPIO153",pin_pulldown_conf),
	ELITE_PIN_HOG("USB_OC0_GPIO154",pin_pulldown_conf),
	ELITE_PIN_HOG("USB_OC1_GPIO155",pin_pulldown_conf),
	ELITE_PIN_HOG("USB_OC2_GPIO156",pin_pulldown_conf),
	ELITE_PIN_HOG("USB_OC3_GPIO157",pin_pulldown_conf),
	ELITE_PIN_HOG("SUS_GPIO22",pin_pullup_conf), //USB 1 2 3 PowerSwitch

	/* SD0 */
	ELITE_MUX("mmc0_grp", "mmc0","elite-mci.0"),
	ELITE_PIN_GROUP("mmc0_grp", group_drvstrength_type_b_conf, "elite-mci.0"),
	ELITE_PIN_GROUP("mmc0_grp", group_voltage_3_3_conf, "elite-mci.0"),
	ELITE_PIN("SD0_CD_GPIO28",pin_pullup_conf,"elite-mci.0"),
	ELITE_PIN("SD0_CLK_GPIO33",pin_pullup_conf,"elite-mci.0"),
	ELITE_PIN("SD0_CMD_GPIO34",pin_pullup_conf,"elite-mci.0"),
	ELITE_PIN("SD0_WP_GPIO35",pin_pullup_conf,"elite-mci.0"),
	ELITE_PIN("SD0_DATA0_GPIO36",pin_pullup_conf,"elite-mci.0"),
	ELITE_PIN("SD0_DATA1_GPIO37",pin_pullup_conf,"elite-mci.0"),
	ELITE_PIN("SD0_DATA2_GPIO38",pin_pullup_conf,"elite-mci.0"),
	ELITE_PIN("SD0_DATA3_GPIO39",pin_pullup_conf,"elite-mci.0"),
	ELITE_PIN("SD0_PWRSW_GPIO32",pin_pullup_conf,"elite-mci.0"),
	// used to workaroud GPIO & SD/MMC bugs
	ELITE_PIN_SD("SD0_CD_GPIO28",pin_pulldown_conf,"elite-mci.0"),
	ELITE_PIN_SD("SD0_CLK_GPIO33",pin_pulldown_conf,"elite-mci.0"),
	ELITE_PIN_SD("SD0_CMD_GPIO34",pin_pulldown_conf,"elite-mci.0"),
	ELITE_PIN_SD("SD0_WP_GPIO35",pin_pulldown_conf,"elite-mci.0"),
	ELITE_PIN_SD("SD0_DATA0_GPIO36",pin_pulldown_conf,"elite-mci.0"),
	ELITE_PIN_SD("SD0_DATA1_GPIO37",pin_pulldown_conf,"elite-mci.0"),
	ELITE_PIN_SD("SD0_DATA2_GPIO38",pin_pulldown_conf,"elite-mci.0"),
	ELITE_PIN_SD("SD0_DATA3_GPIO39",pin_pulldown_conf,"elite-mci.0"),
	ELITE_PIN_SD("SD0_PWRSW_GPIO32",pin_pulldown_conf,"elite-mci.0"),
	/* SD1 */
	ELITE_MUX("mmc1_grp", "mmc1","elite-mci.1"),
	ELITE_PIN("SD1_CD_GPIO72",pin_pullup_conf,"elite-mci.1"),
	ELITE_PIN("SD1_CLK_GPIO65",pin_pullup_conf,"elite-mci.1"),
	ELITE_PIN("SD1_CMD_GPIO66",pin_pullup_conf,"elite-mci.1"),
	ELITE_PIN("SD1_WP_GPIO67",pin_pullup_conf,"elite-mci.1"),
	ELITE_PIN("SD1_DATA0_GPIO68",pin_pullup_conf,"elite-mci.1"),
	ELITE_PIN("SD1_DATA1_GPIO69",pin_pullup_conf,"elite-mci.1"),
	ELITE_PIN("SD1_DATA2_GPIO70",pin_pullup_conf,"elite-mci.1"),
	ELITE_PIN("SD1_DATA3_GPIO71",pin_pullup_conf,"elite-mci.1"),
	ELITE_PIN("SD1_PWRSW_GPIO64",pin_pullup_conf,"elite-mci.1"),
	// used to workaroud GPIO & SD/MMC bugs	
	ELITE_PIN_SD("SD1_CD_GPIO72",pin_pulldown_conf,"elite-mci.1"),
	ELITE_PIN_SD("SD1_CLK_GPIO65",pin_pulldown_conf,"elite-mci.1"),
	ELITE_PIN_SD("SD1_CMD_GPIO66",pin_pulldown_conf,"elite-mci.1"),
	ELITE_PIN_SD("SD1_WP_GPIO67",pin_pulldown_conf,"elite-mci.1"),
	ELITE_PIN_SD("SD1_DATA0_GPIO68",pin_pulldown_conf,"elite-mci.1"),
	ELITE_PIN_SD("SD1_DATA1_GPIO69",pin_pulldown_conf,"elite-mci.1"),
	ELITE_PIN_SD("SD1_DATA2_GPIO70",pin_pulldown_conf,"elite-mci.1"),
	ELITE_PIN_SD("SD1_DATA3_GPIO71",pin_pulldown_conf,"elite-mci.1"),
	ELITE_PIN_SD("SD1_PWRSW_GPIO64",pin_pulldown_conf,"elite-mci.1"),

	//ELITE_MUX("mmc2_nand_1_grp", "mmc2","elite-mci.2"),
#if 0   //comment out temporarily
	ELITE_MUX_HOG("mmc2_nand_1_grp", "mmc2"),
#endif
	ELITE_PIN("NAND_CLE_SD2CMD_GPIO44",pin_pullup_conf,"elite-mci.2"),
	ELITE_PIN("NAND_WPN_SD2PWRSW_GPIO48",pin_pullup_conf,"elite-mci.2"),
	ELITE_PIN("NAND_WPDN_SD2WP_GPIO49",pin_pullup_conf,"elite-mci.2"),
	ELITE_PIN("NAND_RB0N_SD2CD_GPIO50",pin_pullup_conf,"elite-mci.2"),
	ELITE_PIN("NAND_IO0_SD2DATA0_GPIO56",pin_pullup_conf,"elite-mci.2"),
	ELITE_PIN("NAND_IO1_SD2DATA1_GPIO57",pin_pullup_conf,"elite-mci.2"),
	ELITE_PIN("NAND_IO2_SD2DATA2_GPIO58",pin_pullup_conf,"elite-mci.2"),
	ELITE_PIN("NAND_IO3_SD2DATA3_GPIO59",pin_pullup_conf,"elite-mci.2"),
	ELITE_PIN("NAND_IO4_SD2DATA4_GPIO60",pin_pullup_conf,"elite-mci.2"),
	ELITE_PIN("NAND_IO5_SD2DATA5_GPIO61",pin_pullup_conf,"elite-mci.2"),
	ELITE_PIN("NAND_IO6_SD2DATA6_GPIO62",pin_pullup_conf,"elite-mci.2"),
	ELITE_PIN("NAND_IO7_SD2DATA7_GPIO63",pin_pullup_conf,"elite-mci.2"),
	// used to workaroud GPIO & SD/MMC bugs	
	ELITE_PIN_SD("NAND_CLE_SD2CMD_GPIO44",pin_pulldown_conf,"elite-mci.2"),
	ELITE_PIN_SD("NAND_WPN_SD2PWRSW_GPIO48",pin_pulldown_conf,"elite-mci.2"),
	ELITE_PIN_SD("NAND_WPDN_SD2WP_GPIO49",pin_pulldown_conf,"elite-mci.2"),
	ELITE_PIN_SD("NAND_RB0N_SD2CD_GPIO50",pin_pulldown_conf,"elite-mci.2"),
	ELITE_PIN_SD("NAND_IO0_SD2DATA0_GPIO56",pin_pulldown_conf,"elite-mci.2"),
	ELITE_PIN_SD("NAND_IO1_SD2DATA1_GPIO57",pin_pulldown_conf,"elite-mci.2"),
	ELITE_PIN_SD("NAND_IO2_SD2DATA2_GPIO58",pin_pulldown_conf,"elite-mci.2"),
	ELITE_PIN_SD("NAND_IO3_SD2DATA3_GPIO59",pin_pulldown_conf,"elite-mci.2"),
	ELITE_PIN_SD("NAND_IO4_SD2DATA4_GPIO60",pin_pulldown_conf,"elite-mci.2"),
	ELITE_PIN_SD("NAND_IO5_SD2DATA5_GPIO61",pin_pulldown_conf,"elite-mci.2"),
	ELITE_PIN_SD("NAND_IO6_SD2DATA6_GPIO62",pin_pulldown_conf,"elite-mci.2"),
	ELITE_PIN_SD("NAND_IO7_SD2DATA7_GPIO63",pin_pulldown_conf,"elite-mci.2"),

	/* Key Pad */
	ELITE_MUX_HOG("kpad_grp", "kpad"),
	ELITE_PIN_HOG("KPAD_ROW0_GPIO80",pin_pulldown_conf),
	ELITE_PIN_HOG("KPAD_ROW1_GPIO81",pin_pulldown_conf),
	ELITE_PIN_HOG("KPAD_ROW2_GPIO82",pin_pulldown_conf),
	ELITE_PIN_HOG("KPAD_ROW3_GPIO83",pin_pulldown_conf),
	ELITE_PIN_HOG("KPAD_COL0_GPIO84",pin_pulldown_conf),
	ELITE_PIN_HOG("KPAD_COL1_GPIO85",pin_pulldown_conf),
	ELITE_PIN_HOG("KPAD_COL2_GPIO86",pin_pulldown_conf),
	ELITE_PIN_HOG("KPAD_COL3_GPIO87",pin_pulldown_conf),
	
	/* I2S */
	ELITE_MUX_HOG("i2s_dac_0_grp", "i2s_dac"),
	ELITE_MUX_HOG("i2s_dac_1_i2s_adc_1_pcm_0_grp", "i2s_dac"),
	ELITE_MUX_HOG("i2s_dac_2_pcm_2_grp", "i2s_dac"),

	ELITE_MUX_HOG("i2s_adc_0_grp", "i2s_dac"),
	ELITE_MUX_HOG("i2s_adc_2_pcm_1_grp", "i2s_dac"),
	ELITE_MUX_HOG("i2s_adc_3_pcm_3_grp", "i2s_dac"),
	ELITE_MUX_HOG("i2s_adc_4_spdif_0_grp", "i2s_dac"),
	
	ELITE_MUX_HOG("spdif_1_grp", "i2s_dac"),
	
	ELITE_PIN_HOG("I2S_DACMCLK_GPIO104",pin_pulldown_conf),
	ELITE_PIN_HOG("I2S_DACBCLK_GPIO105",pin_pulldown_conf),
	ELITE_PIN_HOG("I2S_DACLRC_GPIO106",pin_pulldown_conf),
	ELITE_PIN_HOG("I2S_DACDAT0_GPIO107",pin_pulldown_conf),
	ELITE_PIN_HOG("I2S_DACDAT1_I2SADCBCLK_PCMSYNC_GPIO108",pin_pulldown_conf),
	ELITE_PIN_HOG("I2S_DACDAT2_I2SADCLRC_PCMCLK_GPIO109",pin_pulldown_conf),
	ELITE_PIN_HOG("I2S_DACDAT3_PCMIN_GPIO110",pin_pulldown_conf),
	ELITE_PIN_HOG("I2S_ADCDAT0_GPIO111",pin_pulldown_conf),
	ELITE_PIN_HOG("I2S_ADCMCLK_I2SADCDAT1_PCMOUT_GPIO112",pin_pulldown_conf),
	ELITE_PIN_HOG("PCM_MCLK_I2SADCDAT2_GPIO113",pin_pulldown_conf),
	ELITE_PIN_HOG("SPDIFO_I2SADCDAT3_GPIO115",pin_pulldown_conf),
	ELITE_PIN_HOG("SPDIFI_GPIO114",pin_pulldown_conf),

	/* SmartCard0 */
	ELITE_MUX_HOG("sm0_grp", "sm0"),
	ELITE_PIN_HOG("SM_IOUC_GPIO88",pin_pullup_conf),
	ELITE_PIN_HOG("SM_OFF_GPIO92",pin_pullup_conf),
	
	/* UART1 (debug) */
	ELITE_MUX_HOG("uart1_grp", "uart1"),
	ELITE_PIN_HOG("UART1_TXD_GPIO93",pin_pulldown_conf),
	ELITE_PIN_HOG("UART1_RXD_GPIO94",pin_pulldown_conf),


	/* sf */
	ELITE_MUX_HOG("sf_grp", "sf"),
	ELITE_PIN_HOG("SF_DI_GPIO73",pin_pulldown_conf),
	ELITE_PIN_HOG("SF_DO_GPIO74",pin_pulldown_conf),
	ELITE_PIN_HOG("SF_CS0N_GPIO75",pin_pullup_conf),
	ELITE_PIN_HOG("SF_CS1N_GPIO76",pin_pullup_conf),
	ELITE_PIN_HOG("SF_CLK_GPIO77",pin_pulldown_conf),

	/* TS */
	ELITE_MUX_HOG("ts0_0_grp", "ts0"),
	ELITE_MUX("ts0_1_ts2_grp", "ts0","elite-ts.0"),
	ELITE_MUX("ts0_2_ts3_0_grp", "ts0","elite-ts.0"),
	ELITE_MUX_HOG("ts1_grp", "ts1"),
	ELITE_MUX_HOG("ts3_1_grp", "ts3"),
	ELITE_PIN_HOG("TS_DIN0_GPIO128",pin_pulldown_conf),
	ELITE_PIN_HOG("TS_DIN1_TS2DAT_GPIO129",pin_pulldown_conf),
	ELITE_PIN_HOG("TS_DIN2_TS2CLK_GPIO130",pin_pulldown_conf),
	ELITE_PIN_HOG("TS_DIN3_TS2SYNC_GPIO131",pin_pulldown_conf),
	ELITE_PIN_HOG("TS_DIN4_TS2VALID_GPIO132",pin_pulldown_conf),
	ELITE_PIN_HOG("TS_DIN5_TS3CLK_GPIO133",pin_pulldown_conf),
	ELITE_PIN_HOG("TS_DIN6_TS3SYNC_GPIO134",pin_pulldown_conf),
	ELITE_PIN_HOG("TS_DIN7_TS3VALID_GPIO135",pin_pulldown_conf),
	ELITE_PIN_HOG("TS_SYNC_GPIO136",pin_pulldown_conf),
	ELITE_PIN_HOG("TS_VALID_GPIO137",pin_pulldown_conf),
	ELITE_PIN_HOG("TS_CLK_GPIO138",pin_pulldown_conf),
	ELITE_PIN_HOG("TS1_DAT_GPIO140",pin_pulldown_conf),
	ELITE_PIN_HOG("TS1_CLk_GPIO141",pin_pulldown_conf),
	ELITE_PIN_HOG("TS1_VALID_GPIO143",pin_pulldown_conf),
	ELITE_PIN_HOG("TS1_SYNC_GPIO142",pin_pulldown_conf),
	ELITE_PIN_HOG("TS3_DAT_GPIO139",pin_pulldown_conf),
	
#if 0
	/* SmartCard1 */
	ELITE_MUX("sm1_0_grp", "sm1","elite-sm.1"),
	ELITE_MUX("uart0_0_sm1_1_grp", "sm1","elite-sm.1"),
	ELITE_MUX("uart0_1_uart2_sm1_2_grp", "sm1","elite-sm.1"),
	ELITE_PIN("UART0_RTS_UART2TXD_SM1IOUC_GPIO95",pin_pullup_conf,"elite-sm.1"),
	ELITE_PIN("SM1OFF_GPIO3",pin_pullup_conf,"elite-sm.1"),
#endif
};


/* uhci host controller */
static struct resource elite_uhci1_resources[] = {
    [0] = {
        .start  = (ELITE_USB20_HOST_CFG_BASE + 0x800 + 0x200),
        .end    = (ELITE_USB20_HOST_CFG_BASE + 0x800 + 0x400 -1),
        .flags  = IORESOURCE_MEM,
    },
    [1] = {
        .start = IRQ_UHDC,
        .end    = IRQ_UHDC,
        .flags = IORESOURCE_IRQ,
    },
};

static struct resource elite_uhci2_resources[] = {
    [0] = {
        .start  = (ELITE_USB20_HOST_CFG_BASE + 0x800 + 0x1400),
        .end    = (ELITE_USB20_HOST_CFG_BASE + 0x800 + 0x1600 -1),
        .flags  = IORESOURCE_MEM,
    },
    [1] = {
        .start = IRQ_UHDC,
        .end    = IRQ_UHDC,
        .flags = IORESOURCE_IRQ,
    },

};

static u64 elite_uhci_dma_mask = 0xFFFFF000;

static struct platform_device elite_uhci1_device = {
    .name           = "elite-uhci",
    .id             = 0,
    .dev            = {
        .dma_mask = &elite_uhci_dma_mask,
        .coherent_dma_mask = ~0,
    },
    .num_resources  = ARRAY_SIZE(elite_uhci1_resources),
    .resource       = elite_uhci1_resources,
};

static struct platform_device elite_uhci2_device = {
    .name           = "elite-uhci",
    .id             = 1,
    .dev            = {
        .dma_mask = &elite_uhci_dma_mask,
        .coherent_dma_mask = ~0,
    },
    .num_resources  = ARRAY_SIZE(elite_uhci2_resources),
    .resource       = elite_uhci2_resources,
};

/* ehci host controller */
static struct elite_usb_platform_data usb_pdata = {
    .pw0_gpio_pin = 480,
    .pwother_gpio_pin = 22,
    .success_pin = 0,
};

static struct resource elite_ehci_resources[] = {
    [0] = {
        .start  = (ELITE_USB20_HOST_CFG_BASE + 0x800),          // include pci config
        .end    = (ELITE_USB20_HOST_CFG_BASE + 0x800 + 0x200 -1),
        .flags  = IORESOURCE_MEM,
    },
    [1] = {
        .start = IRQ_UHDC,      //0x1a
        .end    = IRQ_UHDC,
        .flags = IORESOURCE_IRQ,
    },
};

static u64 elite_ehci_dma_mask = 0xFFFFF000;

static struct platform_device elite_ehci_device = {
    .name           = "elite-ehci",
    .id             = 0,
    .dev            = {
        .platform_data = &usb_pdata,
        .dma_mask = &elite_ehci_dma_mask,
        .coherent_dma_mask = ~0,
    },
    .num_resources  = ARRAY_SIZE(elite_ehci_resources),
    .resource       = elite_ehci_resources,
};



/* SATA controller */
static struct resource elite_sata_resource[] = {
	{
		.start  = ELITE_SATA_CTRL_CFG_BASE,
		.end    = ELITE_SATA_CTRL_CFG_BASE + SZ_2K - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = IRQ_SATA,
		.end    = IRQ_SATA,
		.flags  = IORESOURCE_IRQ,
	},
};

static u64 elite_sata_dma_mask = 0xffffffffUL;

static struct platform_device elite_sata_device = {
	.name       = "elite-sata",
	.id         = 0,
	.dev = {
		.dma_mask = &elite_sata_dma_mask,
		.coherent_dma_mask = ~0,
		},
	.num_resources = ARRAY_SIZE(elite_sata_resource),
	.resource	= elite_sata_resource,
};

static struct resource elite_uart0_resources[] = {
	[0] = {
		.start  = ELITE_UART1_BASE,
		.end    = ELITE_UART1_BASE + SZ_64K - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_UART0,
		.end	= IRQ_UART0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource elite_uart1_resources[] = {
	[0] = {
		.start  = ELITE_UART2_BASE,
		.end    = ELITE_UART2_BASE + SZ_64K - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_UART1,
		.end	= IRQ_UART1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource elite_uart2_resources[] = {
	[0] = {
		.start  = ELITE_UART3_BASE,
		.end    = ELITE_UART3_BASE + SZ_64K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device elite_uart0_device = {
    .name           = "elite-uart",
    .id             = 0,
    .num_resources  = ARRAY_SIZE(elite_uart0_resources),
    .resource       = elite_uart0_resources,
};

static struct platform_device elite_uart1_device = {
    .name           = "elite-uart",
    .id             = 1,
    .num_resources  = ARRAY_SIZE(elite_uart1_resources),
    .resource       = elite_uart1_resources,
};

static struct platform_device elite_uart2_device = {
    .name           = "elite-uart",
    .id             = 2,
    .num_resources  = ARRAY_SIZE(elite_uart2_resources),
    .resource       = elite_uart2_resources,
};

static struct resource elite_sf_resources[] = {
	[0] = {
		.start  = ELITE_SF_MEM_CTRL_CFG_BASE,
		.end    = ELITE_SF_MEM_CTRL_CFG_BASE + SZ_1K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device elite_sf_device = {
    .name           = "sf",
    .id             = 0,
    .num_resources  = ARRAY_SIZE(elite_sf_resources),
    .resource       = elite_sf_resources,
};

static struct resource elite_nor_resources[] = {
	[0] = {
		.start  = ELITE_NORF_CTRL_CFG_BASE,
		.end    = ELITE_NORF_CTRL_CFG_BASE + SZ_1K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device elite_nor_device = {
    .name           = "nor",
    .id             = 0,
    .num_resources  = ARRAY_SIZE(elite_nor_resources),
    .resource       = elite_nor_resources,
};

static struct resource elite_sdmmc0_resources[] = {
	[0] = {
		.start	= ELITE_SDMMC0_BASE,
		.end	= ELITE_SDMMC0_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SDC0,
		.end	= IRQ_SDC0,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start  = IRQ_SDC0_DMA,
		.end    = IRQ_SDC0_DMA,
		.flags  = IORESOURCE_IRQ,
	},
	 [3] = {
	     .start    = IRQ_PMC_WAKEUP,
	     .end      = IRQ_PMC_WAKEUP,
	     .flags    = IORESOURCE_IRQ,
	 },
};

static struct resource elite_sdmmc1_resources[] = {
	[0] = {
		.start	= ELITE_SDMMC1_BASE,
		.end	= ELITE_SDMMC1_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SDC1,
		.end	= IRQ_SDC1,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start  = IRQ_SDC1_DMA,
		.end    = IRQ_SDC1_DMA,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct resource elite_sdmmc2_resources[] = {
	[0] = {
		.start	= ELITE_SDMMC2_BASE,
		.end	= ELITE_SDMMC2_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SDC2,
		.end	= IRQ_SDC2,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start  = IRQ_SDC2_DMA,
		.end    = IRQ_SDC2_DMA,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct elite_mmc_platform_data mmc0_pdata = {
	.max_freq = 200000000,
	.pwrsw_pin = 32,
	.nonremovable = false, 
};

static struct elite_mmc_platform_data mmc1_pdata = {
	.max_freq = 50000000,
	.pwrsw_pin = 64,
	.nonremovable = false, 
};

static struct elite_mmc_platform_data mmc2_pdata = {
	.max_freq = 50000000,
	.pwrsw_pin = 48,
	.nonremovable = false, 
};

static u64 elite_sdmmc_dma_mask = 0xffffffffUL;

static struct platform_device elite_sdmmc0_device = {
	.name = "elite-mci",
	.id = 0,
	.dev = {
		.platform_data = &mmc0_pdata,
		.dma_mask = &elite_sdmmc_dma_mask,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(elite_sdmmc0_resources),
	.resource = elite_sdmmc0_resources,
};

struct platform_device elite_sdmmc1_device = {
	.name = "elite-mci",
	.id = 1,
	.dev = {
		.platform_data = &mmc1_pdata,
		.dma_mask = &elite_sdmmc_dma_mask,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(elite_sdmmc1_resources),
	.resource = elite_sdmmc1_resources,
};
/* for broadcom wifi */
EXPORT_SYMBOL(elite_sdmmc1_device);

static struct platform_device elite_sdmmc2_device = {
	.name = "elite-mci",
	.id = 2,
	.dev = {
		.platform_data = &mmc2_pdata,
		.dma_mask = &elite_sdmmc_dma_mask,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(elite_sdmmc2_resources),
	.resource = elite_sdmmc2_resources,
};

static struct resource elite_nand_resources[] = {
	[0] = {
		.start  = ELITE_NF_CTRL_CFG_BASE,
		.end    = ELITE_NF_CTRL_CFG_BASE + SZ_1K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static u64 elite_nand_dma_mask = 0xffffffffUL;

static struct platform_device elite_nand_device = {
	.name = "elite-nand",
	.id = 0,
	.dev = {
		.dma_mask = &elite_nand_dma_mask,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(elite_nand_resources),
	.resource = elite_nand_resources,
};

static struct resource elite_i2s_resources[] = {
    [0] = {
		.start  = ELITE_I2S_BASE,
		.end    = ELITE_I2S_BASE + SZ_64K - 1,
		.flags  = IORESOURCE_MEM,
    },
};

static u64 elite_i2s_dma_mask = 0xffffffffUL;

static struct platform_device elite_i2s_device = {
	.name = "i2s",
	.id = 0,
	.dev = {
		.dma_mask = &elite_i2s_dma_mask,
		.coherent_dma_mask = ~0,
	},
	.num_resources = ARRAY_SIZE(elite_i2s_resources),
	.resource = elite_i2s_resources,
};

static struct resource elite_pcm_resources[] = {
    [0] = {
		.start  = ELITE_PCM_BASE,
		.end    = ELITE_PCM_BASE + SZ_64K - 1,
		.flags  = IORESOURCE_MEM,
    },
};

static u64 elite_pcm_dma_mask = 0xffffffffUL;

static struct platform_device elite_pcm_device = {
	.name = "pcm",
	.id = 0,
	.dev = {
		.dma_mask = &elite_pcm_dma_mask,
		.coherent_dma_mask = ~0,
		},
	.num_resources = ARRAY_SIZE(elite_pcm_resources),
	.resource = elite_pcm_resources,
};

static struct resource elite_dsp_resources[] = {
        [0] = {
                .start = IRQ_DSP2ARM,
                .end   = IRQ_DSP2ARM,
                .flags = IORESOURCE_IRQ,
        },
        [1] = {
                .start = ELITE_DSP_CTRL_BASE,
                .end   = ELITE_DSP_CTRL_BASE + SZ_1K -1,
                .flags = IORESOURCE_MEM,
        },
};

static struct platform_device elite_dsp_device = {
        .name = "elite-xtensa",
        .id = 0,
        .num_resources = ARRAY_SIZE(elite_dsp_resources),
        .resource = elite_dsp_resources,
};

static struct resource elite_i2c0_resources[] = {
    [0] = {
        .start = ELITE_I2C0_BASE,
        .end   = ELITE_I2C0_BASE + SZ_4K - 1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = IRQ_I2C0,
        .end   = IRQ_I2C0,
        .flags = IORESOURCE_IRQ,
    },
};

static struct platform_device elite_i2c_device0 = {
    .name  = "elite-i2c",
    .id    = 0,
    .num_resources = ARRAY_SIZE(elite_i2c0_resources),
    .resource = elite_i2c0_resources,
};

static struct resource elite_i2c1_resources[] = {
    [0] = {
        .start = ELITE_I2C1_BASE,
        .end   = ELITE_I2C1_BASE + SZ_4K - 1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = IRQ_I2C1,
        .end   = IRQ_I2C1,
        .flags = IORESOURCE_IRQ,
    },
};

static struct platform_device elite_i2c_device1 = {
    .name = "elite-i2c",
    .id   = 1,
    .num_resources = ARRAY_SIZE(elite_i2c1_resources),
    .resource = elite_i2c1_resources,
};

static struct resource elite_kpad_resources[] = {
	[0] = {
		.start  = ELITE_KPAD_BASE,
		.end    = ELITE_KPAD_BASE + SZ_64K - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_KPAD,
		.end    = IRQ_KPAD,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device elite_kpad_device = {
	.name			= "elite-kpad",
	.id			= 0,
	.num_resources  = ARRAY_SIZE(elite_kpad_resources),
	.resource		= elite_kpad_resources,
};

static struct resource elite_cir_resources[] = {
	[0] = {
		.start  = ELITE_CIR_BASE,
		.end    = ELITE_CIR_BASE + SZ_64K - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_CIR,
		.end    = IRQ_CIR ,
		.flags  = IORESOURCE_IRQ,
	},	
};

static struct platform_device elite_cir_device = {
	.name			= "elite-cir",
	.id			= 0,
	.num_resources  = ARRAY_SIZE(elite_cir_resources),
	.resource		= elite_cir_resources,
};


static struct platform_device e_i2s_device = {
	.name			= "elite-i2s",
	.id			= -1,
};

static struct platform_device elite_i2s_pcm_device = {
	.name			= "elite-i2s-pcm-audio",
	.id			= -1,
};

static struct platform_device dummy_codec_device = {
	.name			= "dummy-codec-wm",
	.id			= -1,
};

static struct platform_device elite_facki2c_device = {
	.name	= "elite-fake-i2c",
	.id	= 0,
};
static struct resource elite_rtc_resources[] = {
    [0] = {
        .start = ELITE_RTC_BASE,
        .end   = ELITE_RTC_BASE + SZ_4K - 1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = IRQ_RTC0,
        .end   = IRQ_RTC0,
        .flags = IORESOURCE_IRQ,
    },
};

static struct platform_device elite_rtc_device = {
    .name  = "elite-rtc",
    .id    = 0,
    .num_resources = ARRAY_SIZE(elite_rtc_resources),
    .resource = elite_rtc_resources,
};
#define PMP_WDT_BASE  0xD839E040
static struct resource elite_wdt_resources[] = {
    [0] = {
        .start = PMP_WDT_BASE,
        .end   = PMP_WDT_BASE+SZ_16 - 1,
        .flags = IORESOURCE_MEM,
    },  
};

static struct platform_device elite_wdt_device = {
    .name  = "elite-wdt",
    .id    = 0,
    .num_resources = ARRAY_SIZE(elite_wdt_resources),
    .resource = elite_wdt_resources,
};

static struct resource elite_gpio_resources[] = {
    [0] = {
        .start = ELITE_GPIO_BASE,
        .end   = ELITE_GPIO_BASE + SZ_4K - 1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = IRQ_GPIO,
        .end   = IRQ_GPIO,
        .flags = IORESOURCE_IRQ,
    },
};

static struct platform_device elite_gpio_device = {
    .name  = "elite-gpio",
    .id    = 0,
    .num_resources = ARRAY_SIZE(elite_gpio_resources),
    .resource = elite_gpio_resources,
};

static struct resource elite_pinctrl_resources[] = {
    [0] = {
        .start = ELITE_GPIO_BASE,
        .end   = ELITE_GPIO_BASE + SZ_4K - 1,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device elite_pinctrl_device = {
    .name  = "elite-pinctrl",
    .id    = 0,
    .num_resources = ARRAY_SIZE(elite_pinctrl_resources),
    .resource = elite_pinctrl_resources,
};

static struct resource elite_crypto_resources[] = {
        [0] = {
                .start = ELITE_SECURITY_ENGINE_CFG_BASE,
                .end = ELITE_SECURITY_ENGINE_CFG_BASE + SZ_4K,
                .flags = IORESOURCE_MEM,
        },   
        [1] = {
                .start  = IRQ_SE,
                .end    = IRQ_SE,
                .flags  = IORESOURCE_IRQ,
        },   
};

static u64 elite_crypto_dma_mask = 0xffffffffUL;

static struct platform_device elite_crypto_device = {
        .name = "elite-se",
        .id = 0, 
        .dev = {
                .dma_mask = &elite_crypto_dma_mask, /* dma_set_mask */
                .coherent_dma_mask = ~0,
        },   
        .num_resources = ARRAY_SIZE(elite_crypto_resources),
        .resource = elite_crypto_resources,
};


/* 
 * Here give a example for wrting gfx driver
 */
static struct resource elite_gfx_resources[] = {
	[0] = {
		.start = ELITE_GFX_MMIO_BASE,
		.end   = ELITE_GFX_MMIO_BASE + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_GFX_INTA,
		.end   = IRQ_GFX_INTA,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_GFX_INTB,
		.end   = IRQ_GFX_INTB,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device elite_gfx_device = {
	.name			= "elite-gfx",
	.id			= 0,
	.num_resources  = ARRAY_SIZE(elite_gfx_resources),
	.resource		= elite_gfx_resources,
};


static struct platform_device elite_snd_device = {
	.name  = "elite1000-audio",
	.id   = -1,
};

static struct platform_device *elite_devices[] __initdata = {
	&elite_uhci1_device,
	&elite_uhci2_device,
	&elite_ehci_device,
	&elite_sata_device,
	//&elite_uart0_device,
	&elite_uart1_device,
	//&elite_uart2_device,
	&elite_sdmmc0_device,
	&elite_sdmmc1_device,
#if 0  //comment out temporarily
	&elite_sdmmc2_device,
#endif
	&elite_sf_device,
	&elite_nor_device,
	&elite_nand_device,
	&elite_i2s_device,
	&elite_pcm_device,
	&elite_dsp_device,
	&elite_i2c_device0,
	&elite_i2c_device1,
	&elite_gfx_device,
	&elite_facki2c_device,
	&elite_kpad_device,
	&elite_cir_device,
	&dummy_codec_device, 
	&e_i2s_device,
	&elite_i2s_pcm_device,
	&elite_rtc_device,
	&elite_gpio_device,
	&elite_pinctrl_device,
	&elite_crypto_device,
	&elite_wdt_device,
	&elite_snd_device,
};

/* peripheral client attached with i2c bus */
static struct i2c_board_info i2c_devs[] __initdata = {
    { I2C_BOARD_INFO("wm8900", 0x1B), }, /* i2c-wired audio codec WM8900 */
};

static struct i2c_board_info fake_i2c_devs[] __initdata = {
	{I2C_BOARD_INFO("ilitek", 0x41), },
};

static struct i2c_board_info tuner_i2c_devs[] __initdata = {
	{I2C_BOARD_INFO("tuner", 0x1C), },
};

static struct i2c_board_info frontpad_i2c_devs[] __initdata = {
	{I2C_BOARD_INFO("frontpad", 0x10), },
};

static struct i2c_board_info wm8960_i2c_devs[] __initdata = {
	{I2C_BOARD_INFO("wm8960", 0x1a), },
};


#ifdef CONFIG_REGULATOR
static struct regulator_consumer_supply wm8326_dcdc1_consumers[] __initdata = {
	{ .supply = "vdd_cpu", },  
};

static struct regulator_consumer_supply vddmmc_consumers[] __initdata = {
	REGULATOR_SUPPLY("vmmc", "elite-mci.0"), //LDO2 for sd0
};

static struct regulator_init_data wm8326_vddcpu __initdata = {
	.constraints = {
		.name = "PVDD_CPU",
		.min_uV = 600000,
		.max_uV = 1800000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.consumer_supplies = wm8326_dcdc1_consumers,
	.num_consumer_supplies = ARRAY_SIZE(wm8326_dcdc1_consumers),
};

static struct regulator_init_data vddmmc __initdata = {
	.constraints = {
		.name = "VDDMMC",
		.min_uV = 1700000,
		.max_uV = 3600000,
		.always_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies = ARRAY_SIZE(vddmmc_consumers),
	.consumer_supplies = vddmmc_consumers,
};

static struct wm831x_pdata reference_wm8326_pdata __initdata = {
	.wm831x_num = 1,
	.dcdc = {
		 &wm8326_vddcpu,        /* DCDC1 */
	},
	.ldo = {
		NULL,      /* LDO1 */
		&vddmmc,   /* LDO2 */
	},
};

static struct i2c_board_info pmic_devs[] __initdata = {
	{
		I2C_BOARD_INFO("wm8326", 0x34),
		.platform_data = &reference_wm8326_pdata,
        },
};
#endif //CONFIG_REGULATOR


static int elite_create_pg_pgtable(void)
{
	int i;
	pmd_t *pmd;

	unsigned long addr_v[] = {
		(unsigned long)elite_context_area,
		(unsigned long)virt_to_phys(elite_pg_startup),
		(unsigned long)__cortex_a9_restore,
	};
	unsigned long addr_p[] = {
		(unsigned long)virt_to_phys(elite_context_area),
		(unsigned long)virt_to_phys(elite_pg_startup),
		(unsigned long)virt_to_phys(__cortex_a9_restore),
	};
	unsigned int flags = PMD_TYPE_SECT | PMD_SECT_AP_WRITE |
		PMD_SECT_WBWA | PMD_SECT_S;

	elite_pgd = pgd_alloc(&init_mm);
	if (!elite_pgd)
		return -ENOMEM;

	for (i=0; i<ARRAY_SIZE(addr_p); i++) {
		unsigned long v = addr_v[i];
		pmd = pmd_offset(elite_pgd + pgd_index(v), v);
		*pmd = __pmd((addr_p[i] & PGDIR_MASK) | flags);
		flush_pmd_entry(pmd);
		outer_clean_range(__pa(pmd), __pa(pmd + 1));
	}

	elite_pgd_phys = virt_to_phys(elite_pgd);
	__cpuc_flush_dcache_area(&elite_pgd_phys,
		sizeof(elite_pgd_phys));
	outer_clean_range(__pa(&elite_pgd_phys),
		__pa(&elite_pgd_phys+1));

	__cpuc_flush_dcache_area(&elite_context_area,
		sizeof(elite_context_area));
	outer_clean_range(__pa(&elite_context_area),
		__pa(&elite_context_area+1));

	return 0;
}


static int elite_get_chip_version()
{ 
    int iVersion = 0;
    volatile unsigned int * pChipVersionNumber = (unsigned int *)ioremap(0xD80A8500, 4);

    if(pChipVersionNumber)
    {
        iVersion = (*pChipVersionNumber)&0x0F;

        iounmap(pChipVersionNumber);
    }

    return iVersion;
}

static void __init elite1000_init(void)
{
	/* Initialize pinmuxing */
	pinctrl_register_mappings(elite_pinmux_map,
                    ARRAY_SIZE(elite_pinmux_map));

	i2c_register_board_info(1, fake_i2c_devs, ARRAY_SIZE(fake_i2c_devs));

	printk("elite1000_init system_rev:0x%x, chipversion:%d\n", system_rev, elite_get_chip_version());
	if ((system_rev&0xff00) == 0x7100) //for s3 STB
	{	
		if(system_rev == 0x71a1) //a1 use i2c0
		{
			i2c_register_board_info(0, tuner_i2c_devs, ARRAY_SIZE(tuner_i2c_devs));
		}
		else if(system_rev == 0x71a2) //a2 use i2c1
		{
			i2c_register_board_info(1, tuner_i2c_devs, ARRAY_SIZE(tuner_i2c_devs));
		}
		else //default use i2c1
		{
			i2c_register_board_info(1, tuner_i2c_devs, ARRAY_SIZE(tuner_i2c_devs));
		}
	} 
	else if((system_rev&0xff00) == 0x7300) //for socket board
	{
		if(system_rev == 0x73a2) //a2 use i2c0
		{
			i2c_register_board_info(0, tuner_i2c_devs, ARRAY_SIZE(tuner_i2c_devs));
		}
		else //default use i2c0
		{
			i2c_register_board_info(0, tuner_i2c_devs, ARRAY_SIZE(tuner_i2c_devs));
		}
	}
	else if((system_rev&0xff00) == 0x9000) //for LJ STB
	{
		if(system_rev == 0x90a1) //a1 use i2c1
		{
			i2c_register_board_info(1, tuner_i2c_devs, ARRAY_SIZE(tuner_i2c_devs));
    	}
		else if(system_rev == 0x90a2) //a2 use i2c1
		{
			i2c_register_board_info(1, tuner_i2c_devs, ARRAY_SIZE(tuner_i2c_devs));
		}
		else //default use i2c1
		{
			i2c_register_board_info(1, tuner_i2c_devs, ARRAY_SIZE(tuner_i2c_devs));
    	}
	}
	else if((system_rev&0xff00) == 0xa000) //for SVA
	{
		if(system_rev == 0xa0a2) //a2 use i2c1
		{
			i2c_register_board_info(1, tuner_i2c_devs, ARRAY_SIZE(tuner_i2c_devs));
		}
		else //default use i2c1
		{
			i2c_register_board_info(1, tuner_i2c_devs, ARRAY_SIZE(tuner_i2c_devs));
		}
	}
	else //default use i2c1
	{
		i2c_register_board_info(1, tuner_i2c_devs, ARRAY_SIZE(tuner_i2c_devs));
	}

	
	i2c_register_board_info(0, frontpad_i2c_devs, ARRAY_SIZE(frontpad_i2c_devs));

	if (system_rev == 0x73a2) {
		i2c_register_board_info(1, wm8960_i2c_devs, ARRAY_SIZE(wm8960_i2c_devs));
	}

	//i2c_register_board_info(0, i2c_devs, ARRAY_SIZE(fake_i2c_devs));

#ifdef CONFIG_REGULATOR
	i2c_register_board_info(1, pmic_devs, ARRAY_SIZE(pmic_devs));
#endif

	platform_add_devices(elite_devices, ARRAY_SIZE(elite_devices));

	elite_context_area = kzalloc(CONTEXT_SIZE_BYTES * MAXCPUS, GFP_KERNEL);

	elite_create_pg_pgtable();

	elite_pmu_enable();
	
	elite_init_wakeupcfg();

	arm_pm_restart = elite_pmu_reset;
	
	pm_power_off = elite_pmu_power_off;

	/* for broadcom WL330 WIFI */
	gpio_request(0, "power-Broadcom-WIFI");
	gpio_direction_output(0, 1);
	gpio_request(1, "IRQ-wake-Broadcom-WIFI");
}

static void elite1000_fixup(struct tag *tags,
			   char **cmdline, struct meminfo *mi)
{
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].size = SZ_1M*760 - PHYS_OFFSET;
	mi->bank[1].start = SZ_1M*760;
	mi->bank[1].size = SZ_1G - SZ_1M*768;

	mi->nr_banks = 2;
}

MACHINE_START(ELITE_1000, "elite1000")
	.atag_offset	= 0x02000100,
	.fixup		= elite1000_fixup,
	.map_io		= elite_map_io,
	.init_early	= elite_init_early,
	.init_irq	= elite_init_irq,
	.handle_irq	= gic_handle_irq,
	.timer		= &elite_timer,
	.init_machine	= elite1000_init,
MACHINE_END

