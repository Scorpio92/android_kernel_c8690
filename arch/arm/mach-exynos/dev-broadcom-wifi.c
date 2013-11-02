/*=========================================================
Additions and modifications made by Cellon Communications
==========================================================*/
/* linux/arch/arm/mach-msm/dev-broadcom-wifi.c
*/
/*
 * Bluetooth wifi
 *
 *  Copyright (C) 2011 Samsung Electronics Co., Ltd.
 *  Copyright (C) 2011 Google, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>

#include <plat/devs.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio.h>
#include <mach/regs-gpio.h>
#include <plat/sdhci.h>

//Cellon add start, Eagle.Yin, 2013/1/23, add debug log in eng version
#ifdef CONFIG_CELLON_ENG_LOG
#define BCMDHD_DEBUG
#endif
//Cellon add end, Eagle.Yin, 2013/1/23, add debug log in eng version 

#define WLAN_OOB_GPIO  EXYNOS4_GPX1(2) //wlan_wake_host , the GPIO number of platform, modified
#define WL_REG_ON	EXYNOS4_GPZ(5) //WL_REG_ON , the GPIO number of exynos platform, modified 


// Cellon add start, Ted Shi, 2012/08/06, for porting bcm4330 bt 
#ifdef BCMDHD_DEBUG
#define BCM_WIFI_DBG(fmt, ...) printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)
#else
#define BCM_WIFI_DBG(fmt, ...)
#endif
// Cellon add end, Ted Shi, 2012/08/06

#ifdef CONFIG_BCM_WIFI_PREMEM
#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	17

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *brcm_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

#define DHD_SKB_HDRSIZE			336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)


int __init brcm_init_wifi_mem(void)
{
	int i;
	int j;

	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;
	
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			goto err_mem_alloc;
	}

	BCM_WIFI_DBG("%s: WIFI MEM Allocated\n", __FUNCTION__);
	return 0;
	
err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		kfree(wifi_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

 err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}
#endif

static unsigned int wlan_on_gpio_table[][4] = {
        {WL_REG_ON , S3C_GPIO_SFN(1), GPIO_LEVEL_HIGH, S3C_GPIO_PULL_NONE},
        {WLAN_OOB_GPIO, S3C_GPIO_SFN(0xF), GPIO_LEVEL_NONE, S3C_GPIO_PULL_DOWN},
};

static unsigned int wlan_off_gpio_table[][4] = {
        {WL_REG_ON , S3C_GPIO_SFN(1), GPIO_LEVEL_LOW, S3C_GPIO_PULL_NONE},
        {WLAN_OOB_GPIO, 0 , GPIO_LEVEL_NONE, S3C_GPIO_PULL_DOWN},
};

static unsigned int wlan_sdio_on_table[][4] = {
        {EXYNOS4_GPK3(0), S3C_GPIO_SFN(2), GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE}, //mmc3clk
        {EXYNOS4_GPK3(1), S3C_GPIO_SFN(2), GPIO_LEVEL_NONE, S3C_GPIO_PULL_UP}, //mmc3cmd
        {EXYNOS4_GPK3(3), S3C_GPIO_SFN(2), GPIO_LEVEL_NONE, S3C_GPIO_PULL_UP}, //mmc3data0
        {EXYNOS4_GPK3(4), S3C_GPIO_SFN(2), GPIO_LEVEL_NONE, S3C_GPIO_PULL_UP}, //mmc3data1
        {EXYNOS4_GPK3(5), S3C_GPIO_SFN(2), GPIO_LEVEL_NONE, S3C_GPIO_PULL_UP}, //mmc3data2
        {EXYNOS4_GPK3(6), S3C_GPIO_SFN(2), GPIO_LEVEL_NONE, S3C_GPIO_PULL_UP}, //mmc3data3
};

static unsigned int wlan_sdio_off_table[][4] = {
        {EXYNOS4_GPK3(0), 1, GPIO_LEVEL_LOW, S3C_GPIO_PULL_NONE},
        {EXYNOS4_GPK3(1), 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
        {EXYNOS4_GPK3(3), 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
        {EXYNOS4_GPK3(4), 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
        {EXYNOS4_GPK3(5), 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
        {EXYNOS4_GPK3(6), 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
};

static void s3c_config_gpio_alive_table(int array_size, unsigned int (*gpio_table)[4])
{
	u32 i, gpio;
	for (i = 0; i < array_size; i++) {
		gpio = gpio_table[i][0];
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(gpio_table[i][1]));
		s3c_gpio_setpull(gpio, gpio_table[i][3]);
		if (gpio_table[i][2] != GPIO_LEVEL_NONE)
		    gpio_set_value(gpio, gpio_table[i][2]);
	}
}

static int brcm_wlan_power(int on)
{
	BCM_WIFI_DBG("%s: power  %s \n", __func__, on ? "on" : "off");
	if (on) {
		s3c_config_gpio_alive_table(ARRAY_SIZE(wlan_on_gpio_table), wlan_on_gpio_table);
		udelay(200);
		gpio_set_value(WL_REG_ON, GPIO_LEVEL_LOW);
		mdelay(200);
		gpio_set_value(WL_REG_ON, GPIO_LEVEL_HIGH);
		mdelay(100);
		BCM_WIFI_DBG("WLAN: WL_REG_ON = %d \n", gpio_get_value(WL_REG_ON));
	} 
	else {
		gpio_set_value(WL_REG_ON, GPIO_LEVEL_LOW);
		s3c_config_gpio_alive_table(ARRAY_SIZE(wlan_off_gpio_table), wlan_off_gpio_table);
		BCM_WIFI_DBG("WLAN: WL_REG_ON = %d \n", gpio_get_value(WL_REG_ON));
	}

	return 0;
}

static int brcm_wlan_reset(int on)
{
	BCM_WIFI_DBG("%s: power  %s \n", __func__, on ? "on" : "off");
	gpio_set_value(WL_REG_ON,
			on ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
	return 0;
}

static int sdhci_s3c_sdio_card_detect(int on)
{
	BCM_WIFI_DBG("%s: on = %s \n", __func__, on ? "on" : "off");
	if (on)
		{
		s3c_config_gpio_alive_table(ARRAY_SIZE(wlan_sdio_on_table), wlan_sdio_on_table);
		mmc_force_presence_change(&s3c_device_hsmmc3);
		}
	else
		s3c_config_gpio_alive_table(ARRAY_SIZE(wlan_sdio_off_table), wlan_sdio_off_table);

	udelay(200);

	//mmc_force_presence_change(&s3c_device_hsmmc3);
	/* msleep(500);  wait for carddetect */
	return 0;
}


static struct resource brcm_wifi_resources[] = {
	[0] = {
		.name		= "bcmdhd_wlan_irq",
		.start		= IRQ_EINT(10),
		.end		= IRQ_EINT(10),
		//.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL
	},
};

static struct wifi_platform_data brcm_wifi_control = {
	.set_power      = brcm_wlan_power,
	.set_reset      = brcm_wlan_reset,
	.set_carddetect = sdhci_s3c_sdio_card_detect,
#ifdef CONFIG_BCM_WIFI_PREMEM
	.mem_prealloc   = brcm_wifi_mem_prealloc,
#endif
};

static struct platform_device brcm_wifi_device = {
        .name           = "bcmdhd_wlan",
        .id             = 1,
        .num_resources  = ARRAY_SIZE(brcm_wifi_resources),
        .resource       = brcm_wifi_resources,
        .dev            = {
                .platform_data = &brcm_wifi_control,
        },
};

int __init broadcom_wifi_init(void)
{
	int ret;

	BCM_WIFI_DBG("%s: start \n", __func__);
#ifdef CONFIG_BCM_WIFI_PREMEM
	brcm_init_wifi_mem();
#endif
	ret = platform_device_register(&brcm_wifi_device);
	return ret;
}
late_initcall(broadcom_wifi_init);
