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
#include <linux/of_platform.h>
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
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("elite-pinctrl",group,func)
#define ELITE_PIN_GROUP_HOG(group,conf) \
	PIN_MAP_CONFIGS_GROUP_HOG_DEFAULT("elite-pinctrl",group,conf)
#define ELITE_PIN_HOG(pin,conf) \
	PIN_MAP_CONFIGS_PIN_HOG_DEFAULT("elite-pinctrl",pin,conf)
#define ELITE_PIN_IDLE(pin,conf,dev) \
	PIN_MAP_CONFIGS_PIN(dev, PINCTRL_STATE_IDLE, "elite-pinctrl", \
	pin, conf)

/* These default states associated with device and that changed at runtime */
#define ELITE_MUX(group,func,dev) \
	PIN_MAP_MUX_GROUP_DEFAULT(dev,"elite-pinctrl",group,func)
#define ELITE_PIN_GROUP(group,conf,dev) \
	PIN_MAP_CONFIGS_GROUP_DEFAULT(dev,"elite-pinctrl",group,conf)
#define ELITE_PIN(pin,conf,dev) \
	PIN_MAP_CONFIGS_PIN_DEFAULT(dev,"elite-pinctrl",pin,conf)
/* SD/MMC specific state */
#define ELITE_PIN_GROUP_SD(group,conf,dev) \
	PIN_MAP_CONFIGS_GROUP(dev, PINCTRL_STATE_SD_PULLDOWN, "elite-pinctrl", group, conf)
#define ELITE_PIN_SD(pin,conf,dev) \
	PIN_MAP_CONFIGS_PIN(dev, PINCTRL_STATE_SD_PULLDOWN, "elite-pinctrl", pin, conf)

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
	ELITE_MUX_HOG("mmc2_nand_1_grp", "mmc2"),
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

//elite1k-520016c-JSS-01 ++S
        /* UART0 */
        ELITE_MUX_HOG("uart0_0_sm1_1_grp", "uart0"),
        ELITE_MUX_HOG("uart0_1_uart2_sm1_2_grp", "uart0"),
        ELITE_PIN_HOG("UART0_RXD_SM1CMDVCC_GPIO97", pin_pulldown_conf),
        ELITE_PIN_HOG("UART0_TXD_SM1RSTIN_GPIO98", pin_pulldown_conf),

        /* UART2 */
        ELITE_MUX_HOG("uart0_1_uart2_sm1_2_grp", "uart2"),
        ELITE_PIN_HOG("UART0_RTS_UART2TXD_SM1IOUC_GPIO95", pin_pulldown_conf),
        ELITE_PIN_HOG("UART0_CTS_UART2RXD_SM1STROBE_GPIO96", pin_pulldown_conf),
//elite1k-520016c-JSS-01 ++E

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

/*
 * The following lookup table is used to override device names when devices
 * are registered from device tree.
 *
 * For drivers that require platform data to be provided from the machine
 * file, a platform data pointer can also be supplied along with the
 * devices names. Usually, the platform data elements that cannot be parsed
 * from the device tree by the drivers (example: function pointers) are
 * supplied. But it should be noted that this is a temporary mechanism and
 * at some point, the drivers should be capable of parsing all the platform
 * data from the device tree.
 */
static const struct of_dev_auxdata elite1000_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("s3graphics,elite1000-pinctrl", ELITE_GPIO_BASE, 
				"elite-pinctrl", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-gpio", ELITE_GPIO_BASE, 
				"elite-gpio", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-uart", ELITE_UART1_BASE,
				"elite-uart.0", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-uart", ELITE_UART2_BASE,
				"elite-uart.1", NULL),
//elite1k-520016c-JSS-01 ++S
	OF_DEV_AUXDATA("s3graphics,elite1000-uart", ELITE_UART3_BASE,
				"elite-uart.2", NULL),
//elite1k-520016c-JSS-01 ++E
	OF_DEV_AUXDATA("s3graphics,elite1000-sdmmc", ELITE_SDMMC0_BASE,
				"elite-mci.0", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-sdmmc", ELITE_SDMMC1_BASE,
				"elite-mci.1", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-sdmmc", ELITE_SDMMC2_BASE,
				"elite-mci.2", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-i2c", ELITE_I2C0_BASE,
				"elite-i2c.0", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-i2c", ELITE_I2C1_BASE,
				"elite-i2c.1", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-nand", ELITE_NF_CTRL_CFG_BASE,
				"elite-nand.0", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-gfx", ELITE_GFX_MMIO_BASE,
				"elite-gfx.0", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-sata", ELITE_SATA_CTRL_CFG_BASE,
				"elite-sata.0", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-i2s", ELITE_I2S_BASE,
				"elite-i2s.0", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-pcm", ELITE_PCM_BASE,
				"elite-pcm.0", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-xtensa", ELITE_DSP_CTRL_BASE,
				"elite-xtensa.0", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-cir", ELITE_CIR_BASE,
				"elite-cir.0", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-uhci", 0xd8007a00,
				"elite-uhci.0", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-uhci", 0xd8008c00,
				"elite-uhci.1", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-ehci", 0xd8007800,
				"elite-ehci.0", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-se", ELITE_SECURITY_ENGINE_CFG_BASE,
				"elite-se.0", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-cir", ELITE_CIR_BASE,
				"elite-cir.0", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-kpad", ELITE_KPAD_BASE,
				"elite-kpad.0", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-rtc", ELITE_RTC_BASE,
				"elite-rtc.0", NULL),
	OF_DEV_AUXDATA("s3graphics,elite1000-audio-wm8960", 0,
				"elite1000-snd-wm8960", NULL),
	{},
};

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

#ifdef CONFIG_REGULATOR
	i2c_register_board_info(1, pmic_devs, ARRAY_SIZE(pmic_devs));
#endif

	of_platform_populate(NULL, of_default_bus_match_table,
				elite1000_auxdata_lookup, NULL);

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

static const char *elite1000_dt_compat[] __initdata = {
	"s3graphics,elite1000",
	NULL
};

//DT_MACHINE_START(ELITE1000_DT, "S3GRAPHICS ELITE1000 (Flattened Device Tree)")
DT_MACHINE_START(ELITE1000_DT, "elite1000")
	.init_irq	= elite_init_irq,
	.map_io		= elite_map_io,
	.init_early	= elite_init_early,
	.handle_irq	= gic_handle_irq,
	.init_machine	= elite1000_init,
	.timer		= &elite_timer,
	.dt_compat	= elite1000_dt_compat,
	.restart    = elite_pmu_reset,
MACHINE_END

