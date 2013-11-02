/* linux/arch/arm/mach-exynos/include/mach/pm-core.h
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Based on arch/arm/mach-s3c2410/include/mach/pm-core.h,
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * EXYNOS4210 - PM core support for arch/arm/plat-s5p/pm.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <mach/regs-pmu.h>
#include <mach/regs-gpio.h>	//yulu

static inline void s3c_pm_debug_init_uart(void)
{
	/* nothing here yet */
}

static inline void s3c_pm_arch_prepare_irqs(void)
{
	unsigned int tmp;
#if 0
	tmp = __raw_readl(S5P_WAKEUP_MASK);
	tmp &= ~(1 << 31);
	__raw_writel(tmp, S5P_WAKEUP_MASK);
#else

	s3c_irqwake_intmask &= ~(0xFFF << 20);
	s3c_irqwake_intmask |= (0x3F << 20);//Robin, Mask IRQ2/IRQ3/FIQ2/FIQ3 of external GIC
	s3c_irqwake_eintmask = __raw_readl(S5P_EINT_WAKEUP_MASK);
	printk("S5P_EINT_WAKEUP_MASK:0x%x\n", s3c_irqwake_eintmask);
	//s3c_irqwake_eintmask &=~(0x1 << 2);	//yulu
	/***************************************/
	// eint bit ... 7654 3210
	// binary:      1110 1011
	// hex:            e    b
//	s3c_irqwake_eintmask = 0xffffffff;
//	s3c_irqwake_eintmask &= ~(0x01<<2); // 2 for ONO
//	s3c_irqwake_eintmask &= ~(0x01<<4); // 4 for CHG_IN
//	s3c_irqwake_eintmask &= ~(0x01);     // FG_IRQ_AP
//modify start by D.Z cellon 2012/12/03
//	s3c_irqwake_eintmask = 0x40100000;
		s3c_irqwake_eintmask = 0x40000000;
//mofify end by  D.Z cellon 2012/12/03
//zkx del
   //     s3c_irqwake_intmask |= (0x1 << 1);  //ylk: mask RTC_ALARM
        
#endif
	__raw_writel(s3c_irqwake_intmask, S5P_WAKEUP_MASK);
	__raw_writel(s3c_irqwake_eintmask, S5P_EINT_WAKEUP_MASK);
}

static inline void s3c_pm_arch_stop_clocks(void)
{
	/* nothing here yet */
}

/* Cellon add start, Eagle.Yin, 2012/11/17, for screen on patch */
#if 0
static inline void s3c_pm_arch_show_resume_irqs(void)
{
	//yulu
	pr_info("WAKEUP_STAT: 0x%x\n", __raw_readl(S5P_WAKEUP_STAT));
	pr_info("WAKUP_INT0_PEND: 0x%x\n", __raw_readl(S5P_EINT_PEND(0)));
	pr_info("WAKUP_INT1_PEND: 0x%x\n", __raw_readl(S5P_EINT_PEND(1)));
	pr_info("WAKUP_INT2_PEND: 0x%x\n", __raw_readl(S5P_EINT_PEND(2)));
	pr_info("WAKUP_INT3_PEND: 0x%x\n", __raw_readl(S5P_EINT_PEND(3)));
	//Robin Wang, Clear Pendings...
	//__raw_writel(__raw_readl(S5P_EINT_PEND(0)),S5P_EINT_PEND(0));
	//__raw_writel(__raw_readl(S5P_EINT_PEND(1)),S5P_EINT_PEND(1));
	//__raw_writel(__raw_readl(S5P_EINT_PEND(2)),S5P_EINT_PEND(2));
	//__raw_writel(__raw_readl(S5P_EINT_PEND(3)),S5P_EINT_PEND(3));
}
#else
extern unsigned int pm_wakeup_stat, pm_eint_pend0,pm_eint_pend1,pm_eint_pend2,pm_eint_pend3;
static inline void s3c_pm_arch_show_resume_irqs(void)
{

	pm_wakeup_stat=__raw_readl(S5P_WAKEUP_STAT);

	pm_eint_pend0=__raw_readl(S5P_EINT_PEND(0));
	pm_eint_pend1=__raw_readl(S5P_EINT_PEND(1));
	pm_eint_pend2=__raw_readl(S5P_EINT_PEND(2));
	pm_eint_pend3=__raw_readl(S5P_EINT_PEND(3));
 
	printk("WAKEUP_STAT: 0x%x\n",pm_wakeup_stat );
	printk("WAKUP_INT0_PEND: 0x%x\n", pm_eint_pend0);
	printk("WAKUP_INT1_PEND: 0x%x\n", pm_eint_pend1);
	printk("WAKUP_INT2_PEND: 0x%x\n",  pm_eint_pend2);
	printk("WAKUP_INT3_PEND: 0x%x\n",  pm_eint_pend3);

	__raw_writel((0x1<<2),S5P_EINT_PEND(0));
	//__raw_writel(pm_eint_pend1 |(0x1<<7),S5P_EINT_PEND(1));
}
#endif
/* Cellon add end, Eagle.Yin, 2012/11/17, for screen on patch */

static inline void s3c_pm_arch_update_uart(void __iomem *regs,
					   struct pm_uart_save *save)
{
	/* nothing here yet */
}

static inline void s3c_pm_restored_gpios(void)
{
	/* nothing here yet */
}

static inline void s3c_pm_saved_gpios(void)
{
	/* nothing here yet */
}
