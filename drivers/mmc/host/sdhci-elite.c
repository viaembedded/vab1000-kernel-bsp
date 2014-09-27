/*
 * sdhci-elite.c Support for SDHCI on S3Graphics's Elite SoC
 *
 * Author: S3Graphics OpenGL Platform Team
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mmc/host.h>
#include <linux/of.h>
#include "sdhci-pltfm.h"


static struct sdhci_pltfm_data sdhci_elite_pdata = {
	.ops	= NULL,
	.quirks	= SDHCI_QUIRK_NO_BUSY_IRQ | SDHCI_QUIRK_FORCE_DMA,
};

static int __devinit sdhci_elite_probe(struct platform_device *pdev)
{
	return sdhci_pltfm_register(pdev, NULL);
}

static int __devexit sdhci_elite_remove(struct platform_device *pdev)
{
	return sdhci_pltfm_unregister(pdev);
}

static const struct of_device_id sdhci_elite_of_match[] = {
	{
		.compatible = "s3graphics,elite2000-sdhci",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sdhci_elite_of_match);

static struct platform_device_id sdhci_elite_driver_ids[] = {
	{
		.name = "sdhci-elite.0",
	}, {
		.name = "sdhci-elite.1",
	},
	{},
};
MODULE_DEVICE_TABLE(platform, sdhci_elite_driver_ids);

static struct platform_driver sdhci_elite_driver = {
	.driver		= {
		.name	= "s3graphics,elite2000,sdhci-elite",
		.owner	= THIS_MODULE,
		.pm	= SDHCI_PLTFM_PMOPS,
		.of_match_table = of_match_ptr(sdhci_elite_of_match),
	},
	.id_table	= sdhci_elite_driver_ids,
	.probe		= sdhci_elite_probe,
	.remove		= __devexit_p(sdhci_elite_remove),
};

module_platform_driver(sdhci_elite_driver);

MODULE_DESCRIPTION("SDHCI driver for Elite2000");
MODULE_AUTHOR("S3Graphics");
MODULE_LICENSE("GPL v2");
