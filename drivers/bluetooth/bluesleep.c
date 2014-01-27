/*=========================================================
Additions and modifications made by Cellon Communications
==========================================================*/

#include <linux/module.h>	/* kernel module definitions */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h> /* event notifications */
#include "hci_uart.h"

#include <mach/regs-gpio.h>
#include <linux/gpio.h>
#include <plat/map-base.h>
#include <plat/gpio-cfg.h>
#include <linux/tty.h>
#include <linux/serial_core.h>
#include <plat/regs-serial.h>
#include "../tty/serial/samsung.h"
#include <linux/wakelock.h>
#include <linux/delay.h>

#define BT_SLEEP_DBG
#ifndef BT_SLEEP_DBG
#ifdef BT_DBG(fmt, arg...)
#undef BT_DBG(fmt, arg...)
#define BT_DBG(fmt, arg...) //do {printk(KERN_ALERT "%s: " fmt "\n" , __FUNCTION__ , ## arg);} while(0)
#endif
#endif
//#define BT_DBG(fmt, arg...) do {printk(KERN_ALERT "%s: " fmt "\n" , __FUNCTION__ , ## arg);} while(0)
/*
 * Defines
 */

#define VERSION		"1.1"
#define PROC_DIR	"bluetooth/sleep"
// Cellon add start, Ted Shi, 2012/10/06, for bt sleep problem
extern int bt_is_running;
// Cellon add end, Ted Shi, 2012/10/06

struct bluesleep_info {
	unsigned bt_wake_host;
	unsigned host_wake_bt;
	unsigned wake_host_irq;
	struct uart_port *uport;
	struct wake_lock wake_lock;
};

/* work function */
static void bluesleep_sleep_work(struct work_struct *work);

/* work queue */
DECLARE_DELAYED_WORK(sleep_workqueue, bluesleep_sleep_work);

/* Macros for handling sleep work */
#define bluesleep_rx_busy()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_tx_busy()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_rx_idle()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_tx_idle()     schedule_delayed_work(&sleep_workqueue, 0)

#define UART0_RTS EXYNOS4_GPA0(3)
//#define UART0_CTS EXYNOS4_GPA0(3)

/* 1 second timeout */
#define TX_TIMER_INTERVAL	1

/* state variable names and bit positions */
#define BT_PROTO	  0x01
#define BT_TXDATA	0x02
#define BT_ASLEEP	0x04

/* global pointer to a single hci device. */
static struct hci_dev *bluesleep_hdev;
//Cellon add start, Eagle.Yin, 2012/12/10, for BT can't open issue
static volatile  int  g_is_reg=0;
//Cellon add end, Eagle.Yin, 2012/12/10, for BT can't open issue

static struct bluesleep_info *bsi;
//static int wakeup_system_flag=1;
/* module usage */
static atomic_t open_count = ATOMIC_INIT(1);

/*
 * Local function prototypes
 */

static int bluesleep_hci_event(struct notifier_block *this,
			    unsigned long event, void *data);

/*
 * Global variables
 */

/** Global state flags */
static unsigned long flags;

/** Tasklet to respond to change in btwakehost line */
static struct tasklet_struct btwakehost_task;

/** Transmission timer */
static struct timer_list tx_timer;

/** Lock for state transitions */
static spinlock_t rw_lock;

static  struct wake_lock wake_lock_isr;

/** Notifier block for HCI events */
struct notifier_block hci_event_nblock = {
	.notifier_call = bluesleep_hci_event,
};

struct proc_dir_entry *bluetooth_dir, *sleep_dir;

/*
 * Local functions
 */
extern struct uart_port * s3c24xx_serial_get_port(int index);
extern unsigned int s3c24xx_serial_tx_empty(struct uart_port *port);

 void uart_power(int on)
{
	struct uart_port * port;
	unsigned int umstat ;
	unsigned int ufcon;
//	printk("%s: enter, on = %d  \n",__func__,on);
	port = s3c24xx_serial_get_port(0);
	ufcon = rd_regl(port, S3C2410_UFCON);
	
//	ufcon = rd_regl(port, S3C2410_UFCON);
	umstat=rd_regb(port, S3C2410_UMSTAT);
//	printk(KERN_INFO "%s dev = ttySAC%d ufcon = %x --umstat=%x\n",__FUNCTION__,port->line,ufcon,umstat);

	if (on) { //open uart fifo and change rts from gpio to  UART_0_RTSn 
	  if(!wake_lock_active((&bsi->wake_lock)))
	  {
	           wake_lock(&bsi->wake_lock);
//	           printk(KERN_INFO "wake_lock is called in %s \n",__func__);
	  }
		s3c_gpio_cfgpin(UART0_RTS, S3C_GPIO_SFN(2)); //gpio to UART_0_RTSn
		s3c_gpio_setpull(UART0_RTS,S3C_GPIO_PULL_DOWN);
//mask by justy 20121017 for bluetooth open fail sometimes start 
		//ufcon |= S3C2410_UFCON_FIFOMODE;
                //wr_regl(port, S3C2410_UFCON, ufcon);
//mask by justy 20121017 for bluetooth open fail sometimes end 
		ufcon = rd_regl(port, S3C2410_UFCON);
		umstat=rd_regb(port, S3C2410_UMSTAT);
		
//		printk(KERN_INFO "%s on dev = ttySAC%d ufcon = %x --umstat=%x\n",__FUNCTION__,port->line,ufcon,umstat);
	} else { //close uart fifo and change rts from UART_0_RTSn to gpio
                       
			s3c_gpio_cfgpin(UART0_RTS, S3C_GPIO_OUTPUT);
			s3c_gpio_setpull(UART0_RTS, S3C_GPIO_PULL_NONE);
//mask by justy 20121017 for bluetooth open fail sometimes start
			//ufcon &= ~S3C2410_UFCON_FIFOMODE;
			//wr_regl(port, S3C2410_UFCON, ufcon);
//mask by justy 20121017 for bluetooth open fail sometimes end
			s3c_gpio_cfgpin(UART0_RTS, S3C_GPIO_SFN(1)); //UART_0_RTSn to gpio
			gpio_set_value(UART0_RTS, 1);
			
			ufcon = rd_regl(port, S3C2410_UFCON);
//			printk(KERN_INFO "%s off dev = ttySAC%d ufcon = %x \n",__FUNCTION__,port->line,ufcon);
			if(wake_lock_active(&bsi->wake_lock))
      	{
      		wake_unlock(&bsi->wake_lock);
      	}
	}
}



/**
 * @brief@  main sleep work handling function which update the flags
 * and activate and deactivate UART ,check FIFO.
 */
static void bluesleep_sleep_work(struct work_struct *work)
{
}

void bluesleep_start_tx_timer(void)
{
//		printk("bluesleep_start_tx_timer\n");
		mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL * HZ));
}

/**
 * A tasklet function that runs in tasklet context and reads the value
 * of the bt_wake_host GPIO pin and further defer the work.
 * @param data Not used.
 */
 extern int irq_set_irq_type(unsigned int irq, unsigned int type);
 /*
 * Cellon modify start, Eagle.Yin, 2012/12/10, for BT can't open issue
 * @modify the function of bluesleep_btwakehost_task , bluesleep_tx_timer_expire
 *  for BT can't open when the droid reboot, enable BT again.
 */
static void bluesleep_btwakehost_task(unsigned long data)
{
	unsigned long irq, irq_flags;
	int bt_wake_host = 0;
	int host_wake_bt = 0;

	irq = data;
	
	spin_lock_irqsave(&rw_lock, irq_flags);
 	if(!wake_lock_active((&bsi->wake_lock)))
		{
			wake_lock(&bsi->wake_lock);
//			printk("wake_lock is called in %s \n",__func__);
		}

	if(!g_is_reg || (bsi->uport == NULL))
	{
		goto task_end;
	}
	
	bt_wake_host = gpio_get_value(bsi->bt_wake_host);
//	irq_set_irq_type(irq, bt_wake_host ? IRQF_TRIGGER_LOW:IRQF_TRIGGER_HIGH );
	host_wake_bt = gpio_get_value(bsi->host_wake_bt);
//	printk(" bt_wake_host : %d , host_wake_bt : %d , g_is_reg: %d \n",bt_wake_host, host_wake_bt, g_is_reg);
// Cellon add start, Ted Shi, 2012/10/06, for bt sleep problem
	bt_is_running = 1;
// Cellon add end, Ted Shi, 2012/10/06

	if (bt_wake_host)
	{	
			if (host_wake_bt)
			{
				if (!test_bit(BT_ASLEEP, &flags))
				{
					if(s3c24xx_serial_tx_empty(bsi->uport))
					{
					 	set_bit(BT_ASLEEP, &flags);
//						printk("bluesleep_btwakehost_task, sleep\n");
						// Cellon add start, Ted Shi, 2012/10/06, for bt sleep problem
						bt_is_running = 0;
						// Cellon add end, Ted Shi, 2012/10/06
						/* close uart power*/ 
						uart_power(0);
					}
					else
					{
						bluesleep_start_tx_timer();
					}
				}
			}
			else
			{
					//printk("Host can't sleep in %s \n",__func__);
					bluesleep_start_tx_timer();
			}
	}
	else
	{
		if (test_bit(BT_ASLEEP, &flags))
		{
			clear_bit(BT_ASLEEP, &flags);
//			printk("bluesleep_btwakehost_task, wakeup\n");
			/* open uart power*/
			uart_power(1);
			bluesleep_start_tx_timer();
		}
		else
		{
			printk("wakeup early\n");
		}
	}
task_end:	
	spin_unlock_irqrestore(&rw_lock, irq_flags);
}

/**
 * Handles proper timer action when outgoing data is delivered to the
 * HCI line discipline. Sets BT_TXDATA.
 */
static void bluesleep_outgoing_data(void)
{
	unsigned long irq_flags;
	int host_wake_bt = 0;

	spin_lock_irqsave(&rw_lock, irq_flags);

// Cellon add start, Ted Shi, 2012/10/06, for bt sleep problem
	bt_is_running = 1;
// Cellon add end, Ted Shi, 2012/10/06
	
//	printk("%s: enter \n",__func__);
	/* log data passing by */
	set_bit(BT_TXDATA, &flags);
	
	host_wake_bt = gpio_get_value(bsi->host_wake_bt);
//	printk("host_wake_bt : %d \n", host_wake_bt);	
	/* if the tx side is sleeping... */
	if (host_wake_bt) 
	{
			gpio_set_value(bsi->host_wake_bt, 0);
			if (test_bit(BT_ASLEEP, &flags))
        	{
    			clear_bit(BT_ASLEEP, &flags);
				//printk("bluesleep_outgoing_data, wakeup ... \n");
				/* open uart power*/
				uart_power(1);
			}
	}

	spin_unlock_irqrestore(&rw_lock, irq_flags);
}

/**
 * Handles HCI device events.
 * @param this Not used.
 * @param event The event that occurred.
 * @param data The HCI device associated with the event.
 * @return <code>NOTIFY_DONE</code>.
 */
static int bluesleep_hci_event(struct notifier_block *this,
				unsigned long event, void *data)
{
	struct hci_dev *hdev = (struct hci_dev *) data;
	struct hci_uart *hu;
	struct uart_state *state;

	if (!hdev)
		return NOTIFY_DONE;
//	printk("%s: event = %lx \n",__func__,event);
	switch (event) {
	case HCI_DEV_REG:
		if (!bluesleep_hdev) {
			bluesleep_hdev = hdev;
			hu  = (struct hci_uart *) hdev->driver_data;
			state = (struct uart_state *) hu->tty->driver_data;
			bsi->uport = state->uart_port;
			g_is_reg=1;
		}
		break;
	case HCI_DEV_UNREG:
		bluesleep_hdev = NULL;
		bsi->uport = NULL;
		g_is_reg=0;
		break;
	case HCI_DEV_WRITE:
//		printk("%s: ready call bluesleep_outgoing_data func \n",__func__);
		bluesleep_outgoing_data();
		break;
	}

	return NOTIFY_DONE;
}


/**
 * Handles transmission timer expiration.
 * @param data Not used.
 */
static void bluesleep_tx_timer_expire(unsigned long data)
{
	unsigned long irq_flags;
	int bt_wake_host = 0;
	int host_wake_bt = 0;
	
//	printk("%s: enter \n",__func__);
	
	spin_lock_irqsave(&rw_lock, irq_flags);

	bt_wake_host = gpio_get_value(bsi->bt_wake_host);
	host_wake_bt = gpio_get_value(bsi->host_wake_bt);
//	printk(" bt_wake_host : %d , host_wake_bt : %d , g_is_reg: %d \n",bt_wake_host, host_wake_bt, g_is_reg);
	
// Cellon add start, Ted Shi, 2012/10/06, for bt sleep problem
//	bt_is_running = 0;
// Cellon add end, Ted Shi, 2012/10/06	

//	BT_DBG("Tx timer expired");
	if(!host_wake_bt)
	{
			/* clear the incoming data flag */
			gpio_set_value(bsi->host_wake_bt, 1);
			clear_bit(BT_TXDATA, &flags);
	}

#if 1
	if (bt_wake_host)
	{	
		if (!test_bit(BT_ASLEEP, &flags))
		{
			if(!g_is_reg ||(bsi->uport == NULL))
			{
				set_bit(BT_ASLEEP, &flags);
//				printk("bluetooth g_is_reg = 0, go to sleep\n");
				bt_is_running = 0;
				/* close uart power*/ 
				uart_power(0);
			}
			else if(s3c24xx_serial_tx_empty(bsi->uport))
			{
				set_bit(BT_ASLEEP, &flags);
//				printk("bluetooth go to sleep\n");
				bt_is_running = 0;
				/* close uart power*/ 
				uart_power(0);
			}
			else
			{
//				printk("bluetooth uart tx not empty\n");
				bluesleep_start_tx_timer();
			}
		}
	}
	else
	{
			bluesleep_start_tx_timer();
//			printk("bluetooth can't sleep!\n");
	}
#endif	

	spin_unlock_irqrestore(&rw_lock, irq_flags);
}
/* Cellon modify end, Eagle.Yin, 2012/12/10, for BT can't open issue */

/**
 * Schedules a tasklet to run when receiving an interrupt on the
 * <code>HOST_WAKE</code> GPIO pin.
 * @param irq Not used.
 * @param dev_id Not used.
 */
static irqreturn_t bluesleep_btwakehost_isr(int irq, void *dev_id)
{
//	int bt_wake_host;
	
	/* schedule a tasklet to handle the change in the host wake line */
//	printk("bluesleep_btwakehost_isr , bt_wake_host : %d \n",gpio_get_value(bsi->bt_wake_host));
	wake_lock_timeout(&wake_lock_isr, msecs_to_jiffies(1000));
	
//	bt_wake_host = gpio_get_value(bsi->bt_wake_host);
//	irq_set_irq_type(irq, bt_wake_host ? IRQF_TRIGGER_FALLING:IRQF_TRIGGER_RISING);
	tasklet_schedule(&btwakehost_task);
//	bluesleep_btwakehost_task(irq);
	return IRQ_HANDLED;
}

/**
 * Starts the Sleep-Mode Protocol on the Host.
 * @return On success, 0. On error, -1, and <code>errno</code> is set
 * appropriately.
 */
static int bluesleep_start(void)
{
	int retval;
	unsigned long irq_flags;

	spin_lock_irqsave(&rw_lock, irq_flags);

	if (test_bit(BT_PROTO, &flags)) {
		spin_unlock_irqrestore(&rw_lock, irq_flags);
//		printk("proto = 1 already \n");
		return 0;
	}

	spin_unlock_irqrestore(&rw_lock, irq_flags);

	if (!atomic_dec_and_test(&open_count)) {
		atomic_inc(&open_count);
//		printk("open count return EBUSY \n");
		return -EBUSY;
	}

	/* start the timer */
	bluesleep_start_tx_timer();
	/* assert HOST_WAKE_BT */
	gpio_set_value(bsi->host_wake_bt, 0);
	//set_irq_wake(bsi->wake_host_irq,1);
//	printk("bsi->wake_host_irq is %x \n",bsi->wake_host_irq);
	irq_set_irq_wake(bsi->wake_host_irq,1);
	//retval = request_irq(bsi->wake_host_irq, bluesleep_btwakehost_isr,
				//IRQF_TRIGGER_RISING |IRQF_TRIGGER_FALLING,
				//IRQF_TRIGGER_RISING,
				//"bluetooth btwakehost", NULL);
	retval = request_threaded_irq(bsi->wake_host_irq,NULL,bluesleep_btwakehost_isr,
	//retval = request_threaded_irq(IRQ_EINT6,NULL, bluesleep_btwakehost_isr,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING ,
				 //IRQF_TRIGGER_FALLING,
	//			 IRQF_TRIGGER_RISING,
				"bluetooth btwakehost", NULL);
	if (retval  < 0) {
		BT_ERR("Couldn't acquire BT_HOST_WAKE IRQ");
		goto fail;
	}
/*
	retval = enable_irq_wake(bsi->wake_host_irq);
	if (retval < 0) {
		BT_ERR("Couldn't enable BT_HOST_WAKE as wakeup interrupt");
		free_irq(bsi->wake_host_irq, NULL);
		goto fail;
	}
*/
	set_bit(BT_PROTO, &flags);
	return 0;
fail:
	del_timer(&tx_timer);
	atomic_inc(&open_count);

	return retval;
}

/**
 * Stops the Sleep-Mode Protocol on the Host.
 */
static void bluesleep_stop(void)
{
	unsigned long irq_flags;
	spin_lock_irqsave(&rw_lock, irq_flags);

	if (!test_bit(BT_PROTO, &flags)) {
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		printk("bluesleep stop already \n");
		return;
	}

	/* assert BT_WAKE */
	gpio_set_value(bsi->host_wake_bt, 0);
	del_timer(&tx_timer);
	clear_bit(BT_PROTO, &flags);

	if (test_bit(BT_ASLEEP, &flags)) {
		clear_bit(BT_ASLEEP, &flags);
		uart_power(1);
	}
	atomic_inc(&open_count);

	spin_unlock_irqrestore(&rw_lock, irq_flags);
	if (disable_irq_wake(bsi->wake_host_irq))
		BT_ERR("Couldn't disable btwakehost IRQ wakeup mode \n");
	free_irq(bsi->wake_host_irq, NULL);
	if(wake_lock_active(&bsi->wake_lock))
	{
		wake_unlock(&bsi->wake_lock);
//		printk("wake_unlock is called in %s \n",__func__);
	}
}
/**
 * Read the <code>BT_WAKE</code> GPIO pin value via the proc interface.
 * When this function returns, <code>page</code> will contain a 1 if the
 * pin is high, 0 otherwise.
 * @param page Buffer for writing data.
 * @param start Not used.
 * @param offset Not used.
 * @param count Not used.
 * @param eof Whether or not there is more data to be read.
 * @param data Not used.
 * @return The number of bytes written.
 */
static int bluepower_read_proc_hostwakebt(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	return sprintf(page, "hostwakebt:%u \n", gpio_get_value(bsi->host_wake_bt));
}

/**
 * Write the <code>BT_WAKE</code> GPIO pin value via the proc interface.
 * @param file Not used.
 * @param buffer The buffer to read from.
 * @param count The number of bytes to be written.
 * @param data Not used.
 * @return On success, the number of bytes written. On error, -1, and
 * <code>errno</code> is set appropriately.
 */
static int bluepower_write_proc_hostwakebt(struct file *file, const char *buffer,
					unsigned long count, void *data)
{
	char *buf;
	if (count < 1)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}
		 //s3c_gpio_cfgpin(UART0_RTS, S3C_GPIO_OUTPUT);
                //s3c_gpio_setpull(UART0_RTS, S3C_GPIO_PULL_NONE);
	if (buf[0] == '0') {
//		printk("bluepower_write_proc_hostwakebt,buf[0] ==0 \n");
		//gpio_set_value(UART0_RTS, 0);
		gpio_set_value(bsi->host_wake_bt, 0);
	} else if (buf[0] == '1') {
//		printk("bluepower_write_proc_hostwakebt,buf[0] ==1 \n");
		gpio_set_value(bsi->host_wake_bt, 1);
		//gpio_set_value(UART0_RTS, 1);
	} else {
		kfree(buf);
		return -EINVAL;
	}

	kfree(buf);
	return count;
}

/**
 * Read the <code>BT_WAKE_HOST</code> GPIO pin value via the proc interface.
 * When this function returns, <code>page</code> will contain a 1 if the pin
 * is high, 0 otherwise.
 * @param page Buffer for writing data.
 * @param start Not used.
 * @param offset Not used.
 * @param count Not used.
 * @param eof Whether or not there is more data to be read.
 * @param data Not used.
 * @return The number of bytes written.
 */
static int bluepower_read_proc_btwakehost(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	return sprintf(page, "btwakehost: %u \n", gpio_get_value(bsi->bt_wake_host));
}


/**
 * Read the low-power status of the Host via the proc interface.
 * When this function returns, <code>page</code> contains a 1 if the Host
 * is asleep, 0 otherwise.
 * @param page Buffer for writing data.
 * @param start Not used.
 * @param offset Not used.
 * @param count Not used.
 * @param eof Whether or not there is more data to be read.
 * @param data Not used.
 * @return The number of bytes written.
 */
static int bluesleep_read_proc_asleep(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	unsigned int asleep;

	asleep = test_bit(BT_ASLEEP, &flags) ? 1 : 0;
	*eof = 1;
	return sprintf(page, "asleep: %u \n", asleep);
}

/**
 * Read the low-power protocol being used by the Host via the proc interface.
 * When this function returns, <code>page</code> will contain a 1 if the Host
 * is using the Sleep Mode Protocol, 0 otherwise.
 * @param page Buffer for writing data.
 * @param start Not used.
 * @param offset Not used.
 * @param count Not used.
 * @param eof Whether or not there is more data to be read.
 * @param data Not used.
 * @return The number of bytes written.
 */
static int bluesleep_read_proc_proto(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	unsigned int proto;

	proto = test_bit(BT_PROTO, &flags) ? 1 : 0;
	*eof = 1;
	return sprintf(page, "proto: %u \n", proto);
}

/**
 * Modify the low-power protocol used by the Host via the proc interface.
 * @param file Not used.
 * @param buffer The buffer to read from.
 * @param count The number of bytes to be written.
 * @param data Not used.
 * @return On success, the number of bytes written. On error, -1, and
 * <code>errno</code> is set appropriately.
 */
static int bluesleep_write_proc_proto(struct file *file, const char *buffer,
					unsigned long count, void *data)
{
	char proto;

	if (count < 1)
		return -EINVAL;

	if (copy_from_user(&proto, buffer, 1))
		return -EFAULT;

	if (proto == '0'){
//		printk("proto == 0,will call bluesleep_stop \n");
		bluesleep_stop();
	}
	else{
//		printk("proto == 1,will call bluesleep_start \n");
		bluesleep_start();
	}

	/* claim that we wrote everything */
	return count;
}

static int __init bluesleep_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;
	bsi = kzalloc(sizeof(struct bluesleep_info), GFP_KERNEL);
	if (!bsi)
		return -ENOMEM;

// Cellon add start, Ted Shi, 2012/10/06, for bt sleep problem
	bt_is_running = 0;
// Cellon add end, Ted Shi, 2012/10/06

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
				"bt_wake_host");
	if (!res) {
		BT_ERR("couldn't find bt_wake_host gpio \n");
		ret = -ENODEV;
		goto free_bsi;
	}
	bsi->bt_wake_host = res->start;

	ret = gpio_request(bsi->bt_wake_host, "bt_wake_host");
	if (ret){
                printk("gpio request bluetooth bt_wake_host error : %d \n", ret);
		goto free_bsi;
        }else {
                s3c_gpio_cfgpin(bsi->bt_wake_host, S3C_GPIO_SFN(0xf)); //input
                s3c_gpio_setpull(bsi->bt_wake_host, S3C_GPIO_PULL_UP); //pull high because  wakeup active is low
        }
	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
				"host_wake_bt");
	if (!res) {
		BT_ERR("couldn't find host_wake_bt gpio \n");
		ret = -ENODEV;
		goto free_bt_wake_host;
	}
	bsi->host_wake_bt = res->start;

	ret = gpio_request(bsi->host_wake_bt, "host_wake_bt");
	if (ret){
		printk("gpio request bluetooth bt_wakeup error : %d \n", ret);
		goto free_bt_wake_host;
	} else {
                s3c_gpio_cfgpin(bsi->host_wake_bt, S3C_GPIO_OUTPUT);
                s3c_gpio_setpull(bsi->host_wake_bt, S3C_GPIO_PULL_NONE);
        }	

	/* assert bt wake */
	gpio_set_value(bsi->host_wake_bt, 0);

	bsi->wake_host_irq = platform_get_irq_byname(pdev, "wake_host_irq");
	if (bsi->wake_host_irq < 0) {
		BT_ERR("couldn't find bt_wake_host irq \n");
		ret = -ENODEV;
		goto free_host_wake_bt;
	}
       wake_lock_init(&bsi->wake_lock, WAKE_LOCK_SUSPEND, "bluesleep");

       wake_lock_init(&wake_lock_isr, WAKE_LOCK_SUSPEND, "bluesleep_isr");
	   
	return 0;

free_host_wake_bt:
	gpio_free(bsi->host_wake_bt);
free_bt_wake_host:
	gpio_free(bsi->bt_wake_host);
free_bsi:
	kfree(bsi);
	return ret;
}

static int bluesleep_remove(struct platform_device *pdev)
{
	/* assert bt wake */
	gpio_set_value(bsi->host_wake_bt, 0);
	if (test_bit(BT_PROTO, &flags)) {
		if (disable_irq_wake(bsi->wake_host_irq))
			BT_ERR("Couldn't disable btwakehost IRQ wakeup mode \n");
		free_irq(bsi->wake_host_irq, NULL);
		del_timer(&tx_timer);
		if (test_bit(BT_ASLEEP, &flags))
			uart_power(1);
	}

	gpio_free(bsi->bt_wake_host);
	gpio_free(bsi->host_wake_bt);
	kfree(bsi);
	return 0;
}
//add by linda
extern void s3c24xx_serial_restar_up(struct uart_port *port);
/*
static int bcm4330_bluetooth_resume(struct device *dev)
{
	uart_power(1);
       //s3c24xx_serial_restar_up( s3c24xx_serial_get_port(0));
	printk("in bcm4330_bluetooth_resume,come to bluesleep_btwakehost_isr???????--%d\n",gpio_get_value(bsi->bt_wake_host));
	//enable_irq_wake(bsi->wake_host_irq);
	//tasklet_schedule(&btwakehost_task);        
	return 0;
}
*/
#ifdef CONFIG_PM
static const struct dev_pm_ops bcm4330_bluetooth_pm_ops = {	
	//.resume		= bcm4330_bluetooth_resume,
};
#endif
//add end
static struct platform_driver bluesleep_driver = {
	.probe = bluesleep_probe,
	.remove = bluesleep_remove,
	.driver = {
		.name = "bluesleep",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &bcm4330_bluetooth_pm_ops, //add by linda
#endif
	},
};
/**
 * Initializes the module.
 * @return On success, 0. On error, -1, and <code>errno</code> is set
 * appropriately.
 */
static int __devinit bluesleep_init(void)
{
	int retval;
	struct proc_dir_entry *ent;

//	BT_INFO(" Sleep Mode Driver Ver %s", VERSION);

	retval = platform_driver_register(&bluesleep_driver);
	if (retval)
		return retval;

	bluesleep_hdev = NULL;

	bluetooth_dir = proc_mkdir("bluetooth", NULL);
	if (bluetooth_dir == NULL) {
		BT_ERR("Unable to create /proc/bluetooth directory");
		return -ENOMEM;
	}

	sleep_dir = proc_mkdir("sleep", bluetooth_dir);
	if (sleep_dir == NULL) {
		BT_ERR("Unable to create /proc/%s directory", PROC_DIR);
		return -ENOMEM;
	}

	/* Creating read/write "hostwakebt" entry */
	ent = create_proc_entry("hostwakebt", 0, sleep_dir);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/hostwakebt entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}
	ent->read_proc = bluepower_read_proc_hostwakebt;
	ent->write_proc = bluepower_write_proc_hostwakebt;

	/* read only proc entries */
	if (create_proc_read_entry("btwakehost", 0, sleep_dir,
				bluepower_read_proc_btwakehost, NULL) == NULL) {
		BT_ERR("Unable to create /proc/%s/btwakehost entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	/* read/write proc entries */
	ent = create_proc_entry("proto", 0, sleep_dir);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/proto entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}
	ent->read_proc = bluesleep_read_proc_proto;
	ent->write_proc = bluesleep_write_proc_proto;

	/* read only proc entries */
	if (create_proc_read_entry("asleep", 0,
			sleep_dir, bluesleep_read_proc_asleep, NULL) == NULL) {
		BT_ERR("Unable to create /proc/%s/asleep entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	flags = 0; /* clear all status bits */

	/* Initialize spinlock. */
	spin_lock_init(&rw_lock);

	/* Initialize timer */
	init_timer(&tx_timer);
	tx_timer.function = bluesleep_tx_timer_expire;
	tx_timer.data = 0;

	/* initialize host wake tasklet */
	tasklet_init(&btwakehost_task, bluesleep_btwakehost_task, 0);

	hci_register_notifier(&hci_event_nblock);

	return 0;

fail:
	remove_proc_entry("asleep", sleep_dir);
	remove_proc_entry("proto", sleep_dir);
	remove_proc_entry("btwakehost", sleep_dir);
	remove_proc_entry("hostwakebt", sleep_dir);
	remove_proc_entry("sleep", bluetooth_dir);
	remove_proc_entry("bluetooth", 0);
	return retval;
}

/**
 * Cleans up the module.
 */
static void __devexit bluesleep_exit(void)
{
	hci_unregister_notifier(&hci_event_nblock);
	platform_driver_unregister(&bluesleep_driver);

	remove_proc_entry("asleep", sleep_dir);
	remove_proc_entry("proto", sleep_dir);
	remove_proc_entry("btwakehost", sleep_dir);
	remove_proc_entry("hostwakebt", sleep_dir);
	remove_proc_entry("sleep", bluetooth_dir);
	remove_proc_entry("bluetooth", 0);
}

module_init(bluesleep_init);
module_exit(bluesleep_exit);

MODULE_DESCRIPTION("Bluetooth Sleep Mode Driver ver %s " VERSION);
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
