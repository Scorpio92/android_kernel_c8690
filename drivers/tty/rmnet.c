/*
 * xmd_rmnet.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * Copyright (C) 2011 Intel Mobile Communications GmbH.
 * Author: Srinivas M <srinivasaraox.mandadapu@intel.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * RMNET Driver modified by Intel from Google msm_rmnet.c
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/tty.h>
#include <linux/netdevice.h>
#include <linux/poll.h>
#include <linux/crc-ccitt.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <asm/uaccess.h>
#include <asm/string.h>
#include <linux/types.h>
#include <linux/compiler.h>
#include <linux/signal.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/etherdevice.h>
#include <linux/ipv6.h>
#include <linux/ip.h>
#include <asm/byteorder.h>
#include <asm/atomic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/********************CONFIGURABLE OPTIONS********************/
/* Following flag has to be enabled if data logs are needed to be 
 * captured */
// Cellon delete start, Ted Shi, 2012/11/30, for remover rmnet0 log
//#define PRINT_DATA_LOG
// Cellon delete end, Ted Shi, 2012/11/30

#if defined (PRINT_DATA_LOG)
#define pr_data_info pr_info
#else
#define pr_data_info(...)
#endif

/* Should be defined only if streaming is enabled on the modem side for 
 * the used IPC */
#define MODEM_STREAMING

/* Should be commented out when compiled for Android environment. Going forward,
 * the following flag needs to be made configurable via BUILD command */
//#define LINUX_HOST 
/************************************************************/

#ifndef LINUX_HOST
#include <linux/wakelock.h>
#endif


MODULE_LICENSE("GPL");

/**************************DEFINES***************************/
#define RMNET_ETH_HDR_SIZE  14
#define RMNET_MTU_SIZE      1514
#define RMNET_IPV4_VER      0x4
#define RMNET_IPV6_VER      0x6
#define IPV6_HEADER_SZ      40
#define IPV6_PROTO_TYPE     0x08DD
#define MAX_NET_IF          2   /* Should be configured based on number of RMNET
                                   Net interfaces needed in the system */
#define N_RMNET             25  /* The N_RMNET value should not be used by any 
                                   other line disc driver in the system. Refer 
                                   to the list of line disciplines listed in 
                                   tty.h file. */

/************************************************************/
struct rmnet_private
/************************STRUCTURES**************************/
{  
  struct tty_struct       *tty;
  struct net_device       *dev;
  struct sk_buff          *skb;
  unsigned short          ip_data_in_skb;
  unsigned short          len;        /* IP datagram length */
  void                    *ptr;
  atomic_t                drv_status;
  struct net_device_stats stats;
#ifndef LINUX_HOST
  struct wake_lock        rmnet_wake_lock;
#endif
};
/************************************************************/

/**********************GLOBAL VARIABLES**********************/
struct rmnet_private *g_rmnet_private[MAX_NET_IF] = {NULL,};
/************************************************************/

/*******************FUNCTION DECLARATION*********************/
static int  rmnet_init(void);
static void rmnet_netif_rx_cb(struct net_device *ptr_dev, \
                            const unsigned char *rx_addr, int sz);
/************************************************************/

/*******************FUNCTION DEFINITIONS********************/

/********************* Line disc driver operations starts *********************/
static int rmnet_asynctty_open(struct tty_struct *tty)
{    
  unsigned short index = 0, dev_num = 0xFFFF;  
  pr_info("%s++\n", __FUNCTION__);

  if (NULL == tty->ops->write) {
    pr_err("%s:tty->ops->write == NULL\n", __FUNCTION__);
    return -EOPNOTSUPP;  
  }    
  for (index = 0; index < MAX_NET_IF; index++) {        
    if((NULL != g_rmnet_private[index]) && 
       (0 == atomic_read(&(g_rmnet_private[index]->drv_status)))) {   
      atomic_set(&(g_rmnet_private[index]->drv_status) , 1);
      dev_num = index;
      pr_info("%s:dev_num = %d\n", __FUNCTION__, dev_num);
      break;
    }
  }
  if(0xFFFF == dev_num) {
    pr_err("%s: Error: Unknown device\n", __FUNCTION__);    
    return -EOPNOTSUPP;
  }

  g_rmnet_private[dev_num]->tty = tty;
  tty->disc_data = g_rmnet_private[dev_num];
  pr_info("g_rmnet_data[%d]->tty is initialized with %p \n", dev_num, tty);    
  return 0;
}


static void rmnet_asynctty_receive(struct tty_struct *tty, 
                                   const unsigned char *buf,
                                   char *cflags, int count)
{  
  struct rmnet_private *p;  
  pr_data_info("%s++\n", __FUNCTION__);  
  p = tty->disc_data;
  if(!p)
  {
    pr_err("%s:Error tty ldisc not opened\n", __FUNCTION__);
    return;
  }
  if(0 == atomic_read(&(p->drv_status ))) {
    pr_err("%s:Error Driver is not ACTIVE \n", __FUNCTION__);
    return;
  }
  if(p->dev) {
    rmnet_netif_rx_cb(p->dev, buf, count);
  }
  else {
    pr_err("%s:Error Net Device is not initialized \n", __FUNCTION__);
  }
  pr_data_info("%s-- \n\n", __FUNCTION__);
}


static void rmnet_asynctty_close(struct tty_struct *tty)
{
  struct rmnet_private *p;
  pr_info("%s++\n", __FUNCTION__);

  p = tty->disc_data;
  if(!p){
    pr_err("%s:Error tty ldisc not opened\n", __FUNCTION__);
    return;  
  }
  atomic_set(&(p->drv_status), 0);  
  p->tty = NULL;
  tty->disc_data = NULL;    
}


static int rmnet_asynctty_hangup(struct tty_struct *tty)
{
  pr_info("%s++\n", __FUNCTION__);
  
  // Behaviour in case of HSIC not yet verified
  //rmnet_asynctty_close(tty);
  
  return 0;
}


/*Line disc info*/
static struct tty_ldisc_ops rmnet_ldisc = {
  .owner        = THIS_MODULE,
  .magic        = TTY_LDISC_MAGIC,
  .name         = "ip_ldisc",
  .open         = rmnet_asynctty_open,
  .close        = rmnet_asynctty_close,
  .hangup       = rmnet_asynctty_hangup,
  .receive_buf  = rmnet_asynctty_receive,
};
/************************ Line disc driver operations ends *******************/


/**************************** Net driver operations starts *******************/
/* Receive data from TTY and forward it to TCP/IP stack 
   The implementation also handles partial packets.
   Ethernet hdr is appended to IP datagram before passing it to TCPIP stack.
*/
void rmnet_netif_rx_cb(struct net_device *dev, const unsigned char *buf, int sz)
{
  struct rmnet_private *p = NULL;   
  int ver = 0;
  unsigned short tempval = 0;
  pr_data_info("%s++\n", __FUNCTION__);

  //pr_data_info("<<<<<=================\n%s++\n",__FUNCTION__);
  //pr_data_info("dev->name = %s\n sz = %d\n", dev->name, sz);
  p = netdev_priv(dev);  
  if(!p){
    pr_err("%s:Error tty ldisc not opened\n", __FUNCTION__);
    return;  
  }
  while(sz) {
    if (0 == p->len) {
#if defined(__BIG_ENDIAN_BITFIELD)
      ver = buf[0] & 0x0F;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
      ver = (buf[0] & 0xF0) >> 4;
#endif
      if (ver == RMNET_IPV4_VER) {
        p->len = (buf[2]<<8)| buf[3];
        //pr_data_info("transport proto type = %d\n", buf[9]);
      } else if (ver == RMNET_IPV6_VER) {
        p->len = IPV6_HEADER_SZ + ((buf[4]<<8)| buf[5]);
      } else {
        pr_err("%s:!!!!!!!!!!!!Wrong Version: sz = %d\n\n", __FUNCTION__, sz);
        return;
      }
      //pr_info("sz = %d, len = %d\n", sz, p->len);
      if (p->len + RMNET_ETH_HDR_SIZE > RMNET_MTU_SIZE) {
        p->ptr = NULL;
        sz -= p->len;
        buf += p->len;
        p->len = 0;
        continue;
      } else {
        p->len += RMNET_ETH_HDR_SIZE;
        p->skb = dev_alloc_skb(p->len + NET_IP_ALIGN);
        if (p->skb == NULL) {
          /* TODO: We need to handle this case later */
          pr_err("%s:!!!!!!!!!!!!!!!!!!skbuf alloc failed!!!!!!!!!!!!!!\n\n", __FUNCTION__);
          return;
        }
        p->skb->dev = dev;
        skb_reserve(p->skb, NET_IP_ALIGN);
        p->ptr = skb_put(p->skb, p->len);

        /* adding ethernet header */
        {
          char temp[] = {0xB6,0x91,0x24,0xa8,0x14,0x72,0xb6,0x91,0x24,
                         0xa8,0x14,0x72,0x08,0x0};
          struct ethhdr *eth_hdr = (struct ethhdr *) temp;

          if (ver == RMNET_IPV6_VER) {
            eth_hdr->h_proto = htons(IPV6_PROTO_TYPE);
          }
          memcpy((void *)eth_hdr->h_dest,
                 (void*)dev->dev_addr,
                 sizeof(eth_hdr->h_dest));
          memcpy((void *)(p->ptr),
                 (void *)eth_hdr,
                 sizeof(struct ethhdr));
        }
      }
    }

    tempval = (sz < (p->len - RMNET_ETH_HDR_SIZE - p->ip_data_in_skb))? 
               sz:(p->len - RMNET_ETH_HDR_SIZE - p->ip_data_in_skb);
    memcpy((p->ptr) + RMNET_ETH_HDR_SIZE + p->ip_data_in_skb, buf, tempval);
    p->ip_data_in_skb += tempval;      
    sz -= tempval;
    buf += tempval;
    if (p->ip_data_in_skb < (p->len - RMNET_ETH_HDR_SIZE)) {
      continue;
    }
	
#ifndef LINUX_HOST
    wake_lock_timeout(&p->rmnet_wake_lock, HZ / 2);
#endif

    p->skb->protocol = eth_type_trans(p->skb, dev);
    p->stats.rx_packets++;
    p->stats.rx_bytes += p->skb->len;
    netif_rx(p->skb);
    p->len = 0;
    p->ip_data_in_skb = 0;
  }
  //pr_data_info("p->stats.rx_packets: %d\n", p->stats.rx_packets);
}


static int rmnet_open(struct net_device *dev)
{
  pr_info("%s++\n", __FUNCTION__);
  netif_start_queue(dev);
  return 0;
}


static int rmnet_stop(struct net_device *dev)
{
  pr_info("%s++\n", __FUNCTION__);  
  netif_stop_queue(dev);
  return 0;
}


/* Receive data from TCP/IP stack and forward it to TTY 
   Ethernet header is removed before sending IP datagram to the modem.
   IP Datagram length is appended before the IP Hdr as per modem requirement.
*/
static int rmnet_xmit(struct sk_buff *skb, struct net_device *dev)
{  
  struct rmnet_private *p = NULL;  
  pr_data_info("%s++\n", __FUNCTION__);
  
  if(NULL == skb) {
    pr_err("%s:(skb==NULL)\n", __FUNCTION__);  
    return 0;
  }
  if (NULL == dev) {
    pr_err("%s:(dev==NULL)\n", __FUNCTION__);
    goto FREE_SKB;
  }
  p = netdev_priv(dev);
  if(!p)
  {
    pr_err("%s:Error tty ldisc not opened\n", __FUNCTION__);
    goto FREE_SKB;
  } 
  if(0 == atomic_read(&(p->drv_status ))) {
    pr_err("%s: Driver status is not initialised\n", __FUNCTION__);
    goto FREE_SKB;
  }
  if(p->tty) {
    /* pr_data_info("\n=================>>>\n");
    pr_data_info("p->tty== %p\n",p->tty);
    pr_data_info("Dest MAC Addr = %x %x %x %x %x %x\n", skb->data[0], \
          skb->data[1], skb->data[2], skb->data[3], skb->data[4], skb->data[5]);        
    pr_data_info("Src MAC Addr = %x %x %x %x %x %x\n", skb->data[6], \
        skb->data[7], skb->data[8], skb->data[9], skb->data[10], skb->data[11]);
    pr_data_info("Eth Protocol type %x %x \nIP datagram start %x %x %x %x %x \
              %x\n", skb->data[12],skb->data[13],skb->data[14],skb->data[15],\
                  skb->data[16],skb->data[17],skb->data[18],skb->data[19]); */
	
#if defined (MODEM_STREAMING)
    set_bit(TTY_DO_WRITE_WAKEUP, &(p->tty->flags));  
    p->tty->ops->write(p->tty, (&skb->data[RMNET_ETH_HDR_SIZE]),
                       (skb->len - RMNET_ETH_HDR_SIZE));
#else
    /* IMPORTANT: IP Datagram length is appended in Network byte order, before 
     * the IP packet as per modem requirement.
     * This is not required if modem supports streaming data for USB HSIC */
    skb->data[12] = (skb->len - RMNET_ETH_HDR_SIZE) >> 8;
    skb->data[13] = (skb->len - RMNET_ETH_HDR_SIZE) & 0xFF ;  

    /* pr_data_info("dev_name = %s \n length MSB = %x LSB = %x \nIP datagram \
       start %x %x %x %x %x %x\n", dev->name, skb->data[12],skb->data[13], \
       skb->data[14],skb->data[15],skb->data[16],skb->data[17],skb->data[18], \
       skb->data[19]); */
    set_bit(TTY_DO_WRITE_WAKEUP, &(p->tty->flags));  
    p->tty->ops->write(p->tty, (&skb->data[RMNET_ETH_HDR_SIZE - 2]),
                       ((skb->len+2) - RMNET_ETH_HDR_SIZE));
#endif

    p->stats.tx_packets++;
    p->stats.tx_bytes += skb->len;    
  }
  
FREE_SKB:  
  dev_kfree_skb_irq(skb);  
  return NETDEV_TX_OK;
}

static struct net_device_stats *rmnet_get_stats(struct net_device *dev)
{  
  struct rmnet_private *p ;  
 // pr_info("%s++\n", __FUNCTION__);

  p = netdev_priv(dev);
  if(NULL != p)
  {
    return &p->stats;
  }
  else
  {
    return NULL;
  }
}


static void rmnet_tx_timeout(struct net_device *dev)
{
  pr_info("%s++\n", __FUNCTION__);
  /* Not implemented. 
     Presently kept to check if this condition is happening in dev phase..
  */
}


/* Net device info */
static struct net_device_ops rmnet_ops = {
  .ndo_open       = rmnet_open,
  .ndo_stop       = rmnet_stop,
  .ndo_start_xmit = rmnet_xmit,
  .ndo_get_stats  = rmnet_get_stats,
  .ndo_tx_timeout = rmnet_tx_timeout,
};


static void __init rmnet_setup(struct net_device *dev)
{
  pr_info("%s++\n", __FUNCTION__);
  
  dev->netdev_ops     = &rmnet_ops;
  dev->watchdog_timeo = 20; /* Might need fine tuning !! */
  dev->mtu            = RMNET_MTU_SIZE;  
  dev->flags         &= ~IFF_MULTICAST;
  dev->flags         &= ~IFF_BROADCAST;  
  ether_setup(dev);  
  random_ether_addr(dev->dev_addr);
}


#ifndef LINUX_HOST
static const char *ch_name[2] = {
  "TTY_DATA0",
  "TTY_DATA1",
};
#endif


static int __init rmnet_init(void)
{    
  int ret = -1;
  unsigned int n = 0;
  struct net_device *dev  = NULL;
  struct rmnet_private *p = NULL;
  pr_info("%s++\n", __FUNCTION__);

  for (n = 0; n < MAX_NET_IF; n++) {
    dev = alloc_netdev(sizeof(struct rmnet_private),
                       "rmnet%d", rmnet_setup);
    if (!dev) {
      pr_err("%s : alloc_netdev failed \n", __FUNCTION__);
      return -ENOMEM;
    }

    p = netdev_priv(dev);    
    p->dev = dev;
    p->ptr = NULL;
    p->len = 0;
    p->skb = NULL;
    p->ip_data_in_skb = 0;  
    atomic_set(&(p->drv_status), 0);  
    g_rmnet_private[n] = p;
#ifndef LINUX_HOST
    wake_lock_init(&p->rmnet_wake_lock, WAKE_LOCK_SUSPEND, ch_name[n]);
#endif
    ret = register_netdev(dev);    
    if (ret < 0) {
      pr_err("%s : register_netdev failed \n", __FUNCTION__);  
      free_netdev(dev);
      p   = NULL;
      dev = NULL;
      return ret;  /* Should we return here or retry after sometime or try for
                      the next device ?? */
    }
    p   = NULL;
    dev = NULL;
  }
  return 0;
}
/************************ Net driver operations ends *************************/

/************************ rmnet driver entry/exit ****************************/
static int __init rmnet_async_init(void)
{  
  int err = -EFAULT;
  pr_info("%s++\n", __FUNCTION__);

  /*
  The N_RMNET value should not be used by any other line disc driver in the 
  system. Refer to the list of line disciplines listed in tty.h file.
  The same value (as N_RMNET) should also be used by application when setting up
  the line disc association:
    int fdrmnet = open("/dev/ttyACM1",  O_RDWR | O_NOCTTY | O_NONBLOCK);		
    int ldisc = N_RMNET;		ioctl (fdrmnet, TIOCSETD, &ldisc);	    
  In some older Linux kernels, dynamically adding new line disc driver is not 
  allowed. In such cases,either an existing line disc number (but not used in 
  the system) can be used, or a new one needs to be added in tty.h and the 
  kernel needs to be rebuild.
  */
  err = tty_register_ldisc(N_RMNET, &rmnet_ldisc);
  if (err != 0) {
    pr_err("%s : error(%d) Registering Line Disc.\n", __FUNCTION__, err);
    return err;
  }
  err = rmnet_init();
  if (err != 0) {
    pr_err("%s : error(%d) Initializing RMNET failed \n", __FUNCTION__, err);
    return err;
  }
  pr_info("RMNET: Registering Line Disc/ RMNET Initialization successful.\n");
  return err;
}


static void __exit rmnet_async_cleanup(void)
{  
  unsigned int n = 0;
  pr_info("%s++\n", __FUNCTION__);

  if (tty_unregister_ldisc(N_RMNET) != 0) {
    pr_err(KERN_ERR "rmnet_async_cleanup: Failed to unregister RMNET line discipline\n");
  }
  for (n = 0; n < MAX_NET_IF; n++) {
    unregister_netdev(g_rmnet_private[n]->dev);
	
#ifndef LINUX_HOST
    wake_lock_destroy(&(g_rmnet_private[n]->rmnet_wake_lock));
#endif

    free_netdev(g_rmnet_private[n]->dev);  
    g_rmnet_private[n] = NULL;
  }
}


module_init(rmnet_async_init);
module_exit(rmnet_async_cleanup);
/******************************************************************************/
