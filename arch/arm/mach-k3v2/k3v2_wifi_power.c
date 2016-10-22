/*
 * Copyright (C) 2009 Texas	Instruments
 *
 * Author: Pradeep Gurumath	<pradeepgurumath@ti.com>
 * This	program	is free	software; you can redistribute it and/or modify
 * it under	the	terms of the GNU General Public	License	version	2 as
 * published by	the	Free Software Foundation.
 */

/* linux/arch/arm/mach-k3v2/k3v2_wifi_power.c
 */

/*=========================================================================
 *
 * histoty
 *
 *=========================================================================
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>
#include <linux/random.h>
#include <linux/skbuff.h>
#include <generated/mach-types.h>
#include <linux/wifi_tiwlan.h>
#include <asm/mach-types.h>
#include <linux/mux.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/clk.h>
#include <linux/mtd/nve_interface.h>
#include <linux/ctype.h>
#include <linux/wl12xx.h>


#include "k3v2_wifi_power.h"

int k3v2_wifi_power(bool power_on);

static struct wl12xx_platform_data k3v2_wlan_data __initdata = {
	.irq = 0,
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = WL12XX_TCXOCLOCK_26,
	//z00186406 add to enable outband irq begin
	.platform_quirks  = WL12XX_PLATFORM_QUIRK_EDGE_IRQ,
	//z00186406 add to enable outband irq end
};

struct wifi_host_s {
        struct regulator *vdd;
        struct clk *clk;
        struct iomux_block *block;
        struct block_config *config;
};

struct wifi_host_s *wifi_host;

static int __init k3v2_wifi_init(void)
{
	int ret = 0;

	printk("TI WL18XX: %s \n", __func__);

        wifi_host = kzalloc(sizeof(struct wifi_host_s), GFP_KERNEL);
        wifi_host->clk = clk_get(NULL, "clk_pmu32kb");
        //delete the regulator LDO14 that wifi not used
        wifi_host->block = iomux_get_block("block_wifi");
        wifi_host->config = iomux_get_blockconfig("block_wifi");
        blockmux_set(wifi_host->block, wifi_host->config, LOWPOWER);

	/* set power gpio */
	ret = gpio_request(K3V2_WIFI_POWER_GPIO, NULL);
	if (ret < 0) {
		pr_err("%s: gpio_request failed, ret:%d.\n", __func__,
			K3V2_WIFI_POWER_GPIO);
		goto err_power_gpio_request;
	}
	gpio_direction_output(K3V2_WIFI_POWER_GPIO, 0);

	//z00186406 add to request out band irq, and convert gpio number to irq number begin
	ret = gpio_request(K3V2_WIFI_IRQ_GPIO, NULL);
	if(ret < 0) {
		pr_err( "%s: gpio_request failed, ret:%d.\n", __func__,
			K3V2_WIFI_IRQ_GPIO );
		goto err_irq_gpio_request;
	}
	gpio_direction_input(K3V2_WIFI_IRQ_GPIO);
	k3v2_wlan_data.irq = gpio_to_irq(K3V2_WIFI_IRQ_GPIO);
	//z00186406 add to request out band irq, and convert gpio number to irq number end

	wl12xx_set_platform_data(&k3v2_wlan_data);

	return 0;

//z00186406 add enable out band irq begin
err_irq_gpio_request:
       gpio_free(K3V2_WIFI_IRQ_GPIO);
//z00186406 add enable out band irq end

err_power_gpio_request:
	gpio_free(K3V2_WIFI_POWER_GPIO);
	return ret;
}
device_initcall(k3v2_wifi_init);

int k3v2_wifi_power(bool power_on)
{
        printk("TI WL18XX: Powering %s WiFi\n", (power_on ? "on" : "off"));

        if (power_on) {
                clk_enable(wifi_host->clk);
                gpio_set_value(K3V2_WIFI_POWER_GPIO, 1);
                mdelay(15);
                gpio_set_value(K3V2_WIFI_POWER_GPIO, 0);
                mdelay(1);
                gpio_set_value(K3V2_WIFI_POWER_GPIO, 1);
                mdelay(70);
        } else {
                gpio_set_value(K3V2_WIFI_POWER_GPIO, 0);
                clk_disable(wifi_host->clk);
        }

        return 0;
}
EXPORT_SYMBOL(k3v2_wifi_power);
