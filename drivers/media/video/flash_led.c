

#include <linux/device.h>
#include <linux/gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/err.h>


struct class *flash_led_class;


static ssize_t flash_led_show(struct class *class, struct class_attribute *attr, 
	char *buf)
{
	return 0;
}

static ssize_t flash_led_store(struct class *class, struct class_attribute *attr, 
		const char *buf, size_t count)
{

	if(!strncmp(buf,"on",2)){

		printk("%s : Flash Led On.....\n",__func__);
		if (gpio_request(EXYNOS4212_GPM3(7), "GPM3_7") < 0)			//torch CAM_GPIO9
			printk("failed gpio_request(GPIO_CAM_TORCH_3H7) for camera control\n");
		s3c_gpio_setpull(EXYNOS4212_GPM3(7), S3C_GPIO_PULL_UP);
		gpio_direction_output(EXYNOS4212_GPM3(7), 1);  
		gpio_free(EXYNOS4212_GPM3(7)); 

		return count;

	}

	if(!strncmp(buf,"off",3)){

		printk("%s : Flash Led Off.....\n",__func__);
		
		if (gpio_request(EXYNOS4212_GPM3(7), "GPM3_7") < 0)			//torch CAM_GPIO9
			printk("failed gpio_request(GPIO_CAM_TORCH_3H7) for camera control\n");
		s3c_gpio_setpull(EXYNOS4212_GPM3(7), S3C_GPIO_PULL_DOWN);
		gpio_direction_output(EXYNOS4212_GPM3(7), 0);  
		gpio_free(EXYNOS4212_GPM3(7)); 

		return count;

	}
		
	return count;
}


static ssize_t flash_led_K11_store(struct class *class, struct class_attribute *attr, 
		const char *buf, size_t count)
{

	if(!strncmp(buf,"on",2)){

		printk("%s : Flash Led On.....\n",__func__);
		if (gpio_request(EXYNOS4212_GPM3(6), "GPM3_7") < 0)			//torch CAM_GPIO9
			printk("failed gpio_request(GPIO_CAM_TORCH_3H7) for camera control\n");
		s3c_gpio_setpull(EXYNOS4212_GPM3(6), S3C_GPIO_PULL_UP);
		gpio_direction_output(EXYNOS4212_GPM3(6), 1);  
		gpio_free(EXYNOS4212_GPM3(6)); 

		return count;

	}

	if(!strncmp(buf,"off",3)){

		printk("%s : Flash Led Off.....\n",__func__);
		
		if (gpio_request(EXYNOS4212_GPM3(6), "GPM3_6") < 0)			//torch CAM_GPIO9
			printk("failed gpio_request(GPIO_CAM_TORCH_3H7) for camera control\n");
		s3c_gpio_setpull(EXYNOS4212_GPM3(6), S3C_GPIO_PULL_DOWN);
		gpio_direction_output(EXYNOS4212_GPM3(6), 0);  
		gpio_free(EXYNOS4212_GPM3(6)); 

		return count;

	}
		
	return count;
}


static ssize_t flash_led_flash_store(struct class *class, struct class_attribute *attr, 
		const char *buf, size_t count)
{

	if(!strncmp(buf,"on",2)){

		printk("%s : Flash Led On.....\n",__func__);
		if (gpio_request(EXYNOS4_GPK1(1), "GPM3_7") < 0)			//torch CAM_GPIO9
			printk("failed gpio_request(GPIO_CAM_TORCH_3H7) for camera control\n");
		s3c_gpio_setpull(EXYNOS4_GPK1(1), S3C_GPIO_PULL_UP);
		gpio_direction_output(EXYNOS4_GPK1(1), 1);  
		gpio_free(EXYNOS4_GPK1(1)); 

		return count;

	}

	if(!strncmp(buf,"off",3)){

		printk("%s : Flash Led Off.....\n",__func__);
		
		if (gpio_request(EXYNOS4_GPK1(1), "GPM3_7") < 0)			//torch CAM_GPIO9
			printk("failed gpio_request(GPIO_CAM_TORCH_3H7) for camera control\n");
		s3c_gpio_setpull(EXYNOS4_GPK1(1), S3C_GPIO_PULL_DOWN);
		gpio_direction_output(EXYNOS4_GPK1(1), 0);  
		gpio_free(EXYNOS4_GPK1(1)); 

		return count;

	}
		
	return count;
}



static CLASS_ATTR(flashledtest, S_IRUGO|S_IWUSR|S_IWGRP,
		flash_led_show, flash_led_store);

static CLASS_ATTR(led_torch_k11, S_IRUGO|S_IWUSR|S_IWGRP,
		flash_led_show, flash_led_K11_store);

static CLASS_ATTR(led_flash, S_IRUGO|S_IWUSR|S_IWGRP,
		flash_led_show, flash_led_flash_store);


static int __init flash_led_init(void)
{
	printk("%s: Entering......\n",__func__);

	flash_led_class = class_create(THIS_MODULE, "flash_led_test");
	if (IS_ERR(flash_led_class)) {
		printk("Unable to create flash_led class; errno = %ld\n", PTR_ERR(flash_led_class));
	}
	
	if (class_create_file(flash_led_class, &class_attr_flashledtest) < 0){
		printk("Failed to create range info file");
	}
	if (class_create_file(flash_led_class, &class_attr_led_torch_k11) < 0){
		printk("Failed to create range info file");
	}
	if (class_create_file(flash_led_class, &class_attr_led_flash) < 0){
		printk("Failed to create range info file");
	}
	return 0;
}

static void __exit flash_led_exit(void)
{
	class_destroy(flash_led_class);
	return ;
}
module_init(flash_led_init);
module_exit(flash_led_exit);


MODULE_AUTHOR("Goeun Lee <charles.hu@cellon.com>");
MODULE_DESCRIPTION("Driver for Flash Led Test");
MODULE_LICENSE("GPL");

