/*=========================================================
Additions and modifications made by Cellon Communications
==========================================================*/
/* linux/arch/arm/mach-msm/dev-broadcom-bt.c
*/
/*
 * Bluetooth Broadcom GPIO
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/wakelock.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include <asm/mach-types.h>

#include <mach/gpio.h>
#include <plat/gpio-cfg.h>

#define BT_UART_CFG
//#define BT_LPM_ENABLE

//Cellon add start, Eagle.Yin, 2013/1/23, add debug log in eng version 
#ifdef CONFIG_CELLON_ENG_LOG
#define BT_DEBUG
#endif
//Cellon add end, Eagle.Yin, 2013/1/23, add debug log in eng version 

#define BT_WAKE_HOST EXYNOS4_GPX1(5)
#define HOST_WAKE_BT EXYNOS4_GPC1(1)
#define BT_REG_EN EXYNOS4_GPZ(6)
#define IRQ_BT_WAKE_HOST IRQ_EINT(13)

#ifdef BT_DEBUG
#define BCM_BT_DBG(fmt, ...) printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)
#else
#define BCM_BT_DBG(fmt, ...)
#endif


static struct rfkill *bt_rfkill;

struct bcm_bt_lpm {
	int host_wake;

	struct hrtimer enter_lpm_timer;
	ktime_t enter_lpm_delay;

	struct hci_dev *hdev;

	struct wake_lock host_wake_lock;
	struct wake_lock bt_wake_lock;
	char wake_lock_name[100];
} bt_lpm;

#ifdef BT_UART_CFG
int bt_is_running;
EXPORT_SYMBOL(bt_is_running);

extern int s3c_gpio_slp_cfgpin(unsigned int pin, unsigned int config);
extern int s3c_gpio_slp_setpull_updown(unsigned int pin, unsigned int config);

static unsigned int bt_uart_on_table[][4] = {
	{EXYNOS4_GPA0(0), 2, 2, S3C_GPIO_PULL_NONE},
	{EXYNOS4_GPA0(1), 2, 2, S3C_GPIO_PULL_NONE},
	{EXYNOS4_GPA0(2), 2, 2, S3C_GPIO_PULL_NONE},
	{EXYNOS4_GPA0(3), 2, 2, S3C_GPIO_PULL_NONE},
};

void bt_config_gpio_table(int array_size, unsigned int (*gpio_table)[4])
{
	u32 i, gpio;

	for (i = 0; i < array_size; i++) {
		gpio = gpio_table[i][0];
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(gpio_table[i][1]));
		s3c_gpio_setpull(gpio, gpio_table[i][3]);
		if (gpio_table[i][2] != 2)
			gpio_set_value(gpio, gpio_table[i][2]);
	}
}

void bt_uart_rts_ctrl(int flag)
{
	if (!gpio_get_value(BT_REG_EN))
		return;
	if (flag) {
		/* BT RTS Set to HIGH */
		s3c_gpio_cfgpin(EXYNOS4_GPA0(3), S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(EXYNOS4_GPA0(3), S3C_GPIO_PULL_NONE);
		gpio_set_value(EXYNOS4_GPA0(3), 1);
		s3c_gpio_slp_cfgpin(EXYNOS4_GPA0(3), S3C_GPIO_SLP_OUT0);
		s3c_gpio_slp_setpull_updown(EXYNOS4_GPA0(3), S3C_GPIO_PULL_NONE);
	} else {
		/* BT RTS Set to LOW */
		s3c_gpio_cfgpin(EXYNOS4_GPA0(3), S3C_GPIO_OUTPUT);
		gpio_set_value(EXYNOS4_GPA0(3), 0);
		s3c_gpio_cfgpin(EXYNOS4_GPA0(3), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(EXYNOS4_GPA0(3), S3C_GPIO_PULL_NONE);
	}
}
EXPORT_SYMBOL(bt_uart_rts_ctrl);
#endif

static int bcm4330_bt_rfkill_set_power(void *data, bool blocked)
{
	/* rfkill_ops callback. Turn transmitter on when blocked is false */
	if (!blocked) {
		BCM_BT_DBG("[BT] Bluetooth Power On.\n");
#ifdef BT_UART_CFG
		bt_config_gpio_table(ARRAY_SIZE(bt_uart_on_table),
					bt_uart_on_table);
#endif
		gpio_set_value(BT_REG_EN, 0);
		msleep(500);//modify by justy 20121016 for bt open fail Probability
		gpio_set_value(BT_REG_EN, 1);
		msleep(100);//modify by justy 20121016 for bt open fail Probability
	} else {
		BCM_BT_DBG("[BT] Bluetooth Power Off.\n");
		bt_is_running = 0;
		gpio_set_value(BT_REG_EN, 0);
	}
	return 0;
}

static const struct rfkill_ops bcm4330_bt_rfkill_ops = {
	.set_block = bcm4330_bt_rfkill_set_power,
};

#ifdef BT_LPM_ENABLE
static void set_wake_locked(int wake)
{
	if (wake)
		wake_lock(&bt_lpm.bt_wake_lock);

	gpio_set_value(HOST_WAKE_BT, wake);
}

static enum hrtimer_restart enter_lpm(struct hrtimer *timer)
{
	if (bt_lpm.hdev != NULL)
		set_wake_locked(0);

	bt_is_running = 0;
	wake_lock_timeout(&bt_lpm.bt_wake_lock, HZ/2);

	return HRTIMER_NORESTART;
}

static void bcm_bt_lpm_exit_lpm_locked(struct hci_dev *hdev)
{
	bt_lpm.hdev = hdev;

	hrtimer_try_to_cancel(&bt_lpm.enter_lpm_timer);
	bt_is_running = 1;
	set_wake_locked(1);

	BCM_BT_DBG("[BT] bcm_bt_lpm_exit_lpm_locked\n");
	hrtimer_start(&bt_lpm.enter_lpm_timer, bt_lpm.enter_lpm_delay,
		HRTIMER_MODE_REL);
}

static void update_host_wake_locked(int host_wake)
{
	if (host_wake == bt_lpm.host_wake)
		return;

	bt_lpm.host_wake = host_wake;

	bt_is_running = 1;

	if (host_wake) {
		wake_lock(&bt_lpm.host_wake_lock);
	} else  {
		/* Take a timed wakelock, so that upper layers can take it.
		 * The chipset deasserts the hostwake lock, when there is no
		 * more data to send.
		 */
		wake_lock_timeout(&bt_lpm.host_wake_lock, HZ/2);
	}
}

static irqreturn_t host_wake_isr(int irq, void *dev)
{
	int host_wake;

	host_wake = gpio_get_value(BT_WAKE_HOST);
	irq_set_irq_type(irq, host_wake ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);

	if (!bt_lpm.hdev) {
		bt_lpm.host_wake = host_wake;
		return IRQ_HANDLED;
	}

	update_host_wake_locked(host_wake);

	return IRQ_HANDLED;
}

static int bcm_bt_lpm_init(struct platform_device *pdev)
{
	int irq;
	int ret;

	hrtimer_init(&bt_lpm.enter_lpm_timer, CLOCK_MONOTONIC,
			HRTIMER_MODE_REL);
	bt_lpm.enter_lpm_delay = ktime_set(4, 0);  /* 1 sec */ /*1->3*//*3->4*/
	bt_lpm.enter_lpm_timer.function = enter_lpm;

	bt_lpm.host_wake = 0;
	bt_is_running = 0;

	snprintf(bt_lpm.wake_lock_name, sizeof(bt_lpm.wake_lock_name),
			"BT_host_wake");
	wake_lock_init(&bt_lpm.host_wake_lock, WAKE_LOCK_SUSPEND,
			 bt_lpm.wake_lock_name);

	snprintf(bt_lpm.wake_lock_name, sizeof(bt_lpm.wake_lock_name),
			"BT_bt_wake");
	wake_lock_init(&bt_lpm.bt_wake_lock, WAKE_LOCK_SUSPEND,
			 bt_lpm.wake_lock_name);

	irq = IRQ_BT_WAKE_HOST;
	ret = request_irq(irq, host_wake_isr, IRQF_TRIGGER_HIGH,
		"bt_wake_host", NULL);
	if (ret) {
		pr_err("[BT] Request_host wake irq failed.\n");
		return ret;
	}

	ret = irq_set_irq_wake(irq, 1);
	if (ret) {
		pr_err("[BT] Set_irq_wake failed.\n");
		return ret;
	}

	return 0;
}

static int bcm_hci_wake_peer(struct notifier_block *this, unsigned long event, void *ptr)
{
	struct hci_dev *hdev = (struct hci_dev *) ptr;

	if (event == HCI_DEV_REG) {
		if (hdev != NULL) {
			hdev->wake_peer = bcm_bt_lpm_exit_lpm_locked;
			BCM_BT_DBG("[BT] wake_peer is registered.\n");
		}
	} else if (event == HCI_DEV_UNREG) {
		pr_info("[BT] %s: handle HCI_DEV_UNREG noti\n", __func__);
		if (hdev != NULL && bt_lpm.hdev == hdev) {
			bt_lpm.hdev = NULL;
			BCM_BT_DBG("[BT] bt_lpm.hdev set to NULL\n");
		}
	}

	return NOTIFY_DONE;
}

static struct notifier_block bcm_bt_nblock = {
	.notifier_call = bcm_hci_wake_peer
};
#endif

static int bcm4330_bluetooth_probe(struct platform_device *pdev)
{
	int rc = 0;
	int ret;

	rc = gpio_request(BT_REG_EN, "bcm4330_bten_gpio");
	if (unlikely(rc)) {
		pr_err("[BT] BT_REG_EN request failed.\n");
		return rc;
	}
// Cellon delete start, Ted Shi, 2012/09/24, for porting bcm4330 bt 
/*
	rc = gpio_request(HOST_WAKE_BT, "bcm4330_hostwakebt_gpio");
	if (unlikely(rc)) {
		pr_err("[BT] HOST_WAKE_BT request failed.\n");
		gpio_free(BT_REG_EN);
		return rc;
	}
	rc = gpio_request(BT_WAKE_HOST, "bcm4330_bthostwake_gpio");
	if (unlikely(rc)) {
		pr_err("[BT] BT_WAKE_HOST request failed.\n");
		gpio_free(HOST_WAKE_BT);
		gpio_free(BT_REG_EN);
		return rc;
	}
	gpio_direction_input(BT_WAKE_HOST);
	gpio_direction_output(HOST_WAKE_BT, 0);
*/
// Cellon delete end, Ted Shi, 2012/09/24
	gpio_direction_output(BT_REG_EN, 0);

	bt_rfkill = rfkill_alloc("bcm4330 Bluetooth", &pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &bcm4330_bt_rfkill_ops,
				NULL);

	if (unlikely(!bt_rfkill)) {
		pr_err("[BT] bt_rfkill alloc failed.\n");
// Cellon delete start, Ted Shi, 2012/09/24, for porting bcm4330 bt 
/*
		gpio_free(BT_WAKE_HOST);
		gpio_free(HOST_WAKE_BT);
*/
// Cellon delete end, Ted Shi, 2012/09/24
		gpio_free(BT_REG_EN);
		return -ENOMEM;
	}

	rfkill_init_sw_state(bt_rfkill, 0);

	rc = rfkill_register(bt_rfkill);

	if (unlikely(rc)) {
		pr_err("[BT] bt_rfkill register failed.\n");
		rfkill_destroy(bt_rfkill);
// Cellon delete start, Ted Shi, 2012/09/24, for porting bcm4330 bt 
/*
		gpio_free(BT_WAKE_HOST);
		gpio_free(HOST_WAKE_BT);
*/
// Cellon delete end, Ted Shi, 2012/09/24
		gpio_free(BT_REG_EN);
		return -1;
	}

	rfkill_set_sw_state(bt_rfkill, true);

#ifdef BT_LPM_ENABLE
	ret = bcm_bt_lpm_init(pdev);
	if (ret) {
		rfkill_unregister(bt_rfkill);
		rfkill_destroy(bt_rfkill);
// Cellon delete start, Ted Shi, 2012/09/24, for porting bcm4330 bt 
/*
		gpio_free(BT_WAKE_HOST);
		gpio_free(HOST_WAKE_BT);
*/
// Cellon delete end, Ted Shi, 2012/09/24
		gpio_free(BT_REG_EN);
	}

	hci_register_notifier(&bcm_bt_nblock);
#endif
	return rc;
}

static int bcm4330_bluetooth_remove(struct platform_device *pdev)
{
	rfkill_unregister(bt_rfkill);
	rfkill_destroy(bt_rfkill);

	gpio_free(BT_REG_EN);
// Cellon delete start, Ted Shi, 2012/09/24, for porting bcm4330 bt 
/*
	gpio_free(HOST_WAKE_BT);
	gpio_free(BT_WAKE_HOST);
*/
// Cellon delete end, Ted Shi, 2012/09/24
	wake_lock_destroy(&bt_lpm.host_wake_lock);
	wake_lock_destroy(&bt_lpm.bt_wake_lock);

#ifdef BT_LPM_ENABLE
	hci_unregister_notifier(&bcm_bt_nblock);
#endif
	return 0;
}

static struct platform_driver bcm4330_bluetooth_platform_driver = {
	.probe = bcm4330_bluetooth_probe,
	.remove = bcm4330_bluetooth_remove,
	.driver = {
		   .name = "bcm4330_bluetooth",
		   .owner = THIS_MODULE,
		   },
};

static int __init bcm4330_bluetooth_init(void)
{
	BCM_BT_DBG("[BT] bcm4330_bluetooth_init called \n");
	return platform_driver_register(&bcm4330_bluetooth_platform_driver);
}

static void __exit bcm4330_bluetooth_exit(void)
{
	platform_driver_unregister(&bcm4330_bluetooth_platform_driver);
}


module_init(bcm4330_bluetooth_init);
module_exit(bcm4330_bluetooth_exit);

MODULE_ALIAS("platform:bcm4330");
MODULE_DESCRIPTION("bcm4330_bluetooth");
MODULE_LICENSE("GPL");
