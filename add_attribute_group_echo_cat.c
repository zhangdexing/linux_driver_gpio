/*
 * Input driver for resistor ladder connected on ADC
 *
 * Copyright (c) 2016 Alexandre Belloni
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>

#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/slab.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>


#include <linux/sysfs.h>

struct keys_button {
	u32 voltage;
	u32 keycode;
};

struct keys_state {
	struct iio_channel *channel;
	u32 num_keys;
	u32 last_key;
	u32 keyup_voltage;
	const struct keys_button *map;
};

struct chr_dev
{
 struct cdev io_cdev;
 dev_t dev_nr;
 struct class *io_class;
 struct device *io_device;
};


#define TAG	 "LED_IOCTL:"

#define DEVICE_NAME "My_Module"
#define DEVICE_COUNT 1

struct xgpio_device_t{
	int gpio;
	struct device *dev;
	struct input_dev *input;
};
struct xgpio_device_t *xgpio;

#define OPEN_LED  _IO('S',0)
#define CLOSE_LED  _IO('S',1)
#define OPEN_EVENT  _IO('S',2)


//--------------------zdx

static int my_open (struct inode *inode, struct file *filefp)
{
	
	//struct xgpio_device_t * xdev = dev_get_drvdata(dev);

	printk(KERN_ERR"open the char device. gpio=%d\n",xgpio->gpio);
	return 0;//success
}

static int my_release(struct inode *inode, struct file *filefp)
{	
	printk(KERN_ERR"close the char device\n");
	return 0;
}

static ssize_t my_read (struct file *filefp, char __user *buf, size_t count, loff_t *off)
{
	/*
	int ret;
	int msg[2];


	ret = copy_to_user(buf, msg, count);
	if(ret != 0)
	{
		printk(KERN_ERR"copy_to_user ERR\n");
		return -EFAULT;
	}
	*/
	
	return 0;
}

static long my_ioctl(struct file*filefp,unsigned int cmd,unsigned long arg)
 {
//	struct xgpio_device_t * xdev = dev_get_drvdata(dev);
	
	printk(KERN_ERR" io contrl  device. gpio=%d\n",xgpio->gpio);
	

	if(cmd == OPEN_LED){//open led
	    	printk("%s========================open led \n",TAG);
		gpio_direction_output(xgpio->gpio,0);

		
	}
	else if(cmd == CLOSE_LED){
	    	printk("%s========================close led \n",TAG);
		gpio_direction_output(xgpio->gpio,1);
	
	}
	else if(cmd == OPEN_EVENT){
	    	printk("%s========================OPEN_EVENT \n",TAG);

		input_report_key(xgpio->input, KEY_MEDIA, 1);

		input_sync(xgpio->input);

		input_report_key(xgpio->input, KEY_MEDIA, 0);

		input_sync(xgpio->input);
	
	}

	return 0;
 }


static struct file_operations io_flops = {
	.owner = THIS_MODULE,
	.open = my_open,
	.release = my_release,
	.read = my_read,
	.unlocked_ioctl = my_ioctl,
};


 int io_init(void )
{
	int res;
	struct chr_dev *chr_devp;
	chr_devp = (struct chr_dev *)kmalloc(sizeof(struct chr_dev ), GFP_KERNEL);

	res = alloc_chrdev_region(&chr_devp->dev_nr, 0, 1, "io_chrdev");
	if(res){
    	printk(KERN_ERR"==>alloc chrdev region failed!\n");
        goto chrdev_err;
	} 

	cdev_init(&chr_devp->io_cdev, &io_flops);

	res = cdev_add(&chr_devp->io_cdev, chr_devp->dev_nr, DEVICE_COUNT);
	if(res){
		printk(KERN_ERR"==>cdev add failed!\n");
        goto cdev_err;
	}


	
	chr_devp->io_class = class_create(THIS_MODULE,"io_class");
	if(IS_ERR(&chr_devp->io_class)){
	 	 res =  PTR_ERR(chr_devp->io_class);
    goto class_err;
	}
	
	chr_devp->io_device = device_create(chr_devp->io_class,NULL, chr_devp->dev_nr, NULL,"io_device");
	if(IS_ERR(&chr_devp->io_device)){
   	   	res = PTR_ERR(&chr_devp->io_device);
       	goto device_err;
    }
	
	printk(KERN_ERR " initialized finish.\n");
	
	return 0;

device_err:
	device_destroy(chr_devp->io_class, chr_devp->dev_nr);
	class_destroy(chr_devp->io_class);

class_err:
	cdev_del(&chr_devp->io_cdev); 

cdev_err:
	unregister_chrdev_region(chr_devp->dev_nr, DEVICE_COUNT);

chrdev_err:
	kfree(chr_devp);
	return res;

}


static int keys_load_keymap(struct device *dev, struct keys_state *st)
{
	struct keys_button *map;
	struct fwnode_handle *child;
	int i;

	st->num_keys = device_get_child_node_count(dev);
	if (st->num_keys == 0) {
		dev_err(dev, "keymap is missing\n");
		return -EINVAL;
	}

	map = devm_kmalloc_array(dev, st->num_keys, sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	i = 0;
	device_for_each_child_node(dev, child) {

		if (fwnode_property_read_u32(child, "linux,code",
					     &map[i].keycode)) {
			dev_err(dev, "Key with invalid or missing linux,code\n");
			fwnode_handle_put(child);
			return -EINVAL;
		}

		i++;
	}

	st->map = map;
	return 0;
}
static ssize_t xgpio_state_store(struct device *dev,struct device_attribute *attr,const char *buf,size_t count){

	printk("-----------enter %s  \n",__func__);
	return count;
}

static ssize_t xgpio_state_show(struct device *dev,struct device_attribute *attr,char *buf){

	printk("-----------enter %s  \n",__func__);
	return 0;
}

/*   创建/sys/devices/platform/test-leds/state节点
	 cat state ---> xgpio_state_show
	 echo data > state   ---->  xgpio_state_store
*/
static DEVICE_ATTR(state,S_IWUSR|S_IRUSR,xgpio_state_show,xgpio_state_store);

static struct attribute *xgpio_attrs[] = {
	&dev_attr_state.attr,
	NULL
};

static const struct attribute_group xgpio_group = {
	.attrs = xgpio_attrs,
};


static int keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct keys_state *st;

	int i;
	int error;
	struct device_node* np = pdev->dev.of_node;
	int gpio;
	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;


	error = keys_load_keymap(dev, st);
	if (error)
		return error;

	platform_set_drvdata(pdev, st);

	xgpio = kmalloc(sizeof(struct xgpio_device_t),GFP_KERNEL);
	if(!xgpio)
		return -ENOMEM;

	xgpio->input = devm_input_allocate_device(dev);
	if (!xgpio->input) {
		dev_err(dev, "failed to allocate xgpio->input device\n");
		return -ENOMEM;
	}

	

	xgpio->input->name = pdev->name;
	xgpio->input->phys = "test-leds/input0";

	xgpio->input->id.bustype = BUS_HOST;
	xgpio->input->id.vendor = 0x0001;
	xgpio->input->id.product = 0x0001;
	xgpio->input->id.version = 0x0100;

	__set_bit(EV_KEY, xgpio->input->evbit);
	for (i = 0; i < st->num_keys; i++)
		__set_bit(st->map[i].keycode, xgpio->input->keybit);

	if (device_property_read_bool(dev, "autorepeat"))
		__set_bit(EV_REP, xgpio->input->evbit);

	error = input_register_device(xgpio->input);
	if (error) {
		dev_err(dev, "Unable to register xgpio->input device: %d\n", error);
		return error;
	}
	//---------------
	gpio = of_get_named_gpio(np,"gpio",0);
	printk("========================request gpio %d  \n",gpio);
	if(!gpio_is_valid(gpio))
	{
	printk("========================get dts gpio failed .  gpio %d  \n",gpio);
	return -EINVAL;
	}

	if(devm_gpio_request(&pdev->dev,gpio,"test-led") != 0)
	{
	printk("========================request gpio failed! invalid gpio %d  \n",gpio);
	return -EINVAL;
	}
	
	xgpio->gpio = gpio;

	xgpio->dev = &pdev->dev;
	// init cdev
	io_init();
	
	error = sysfs_create_group(&pdev->dev.kobj,&xgpio_group);
	if(error){
		dev_err(dev, "===================Unable to create group file  %d\n", error);
		return -ENODEV;
	}
	
	return 0;
}

static int keys_remove(struct platform_device *dev)
{
	printk(KERN_NOTICE "remove...\n");
	
	kfree(xgpio);
	
	sysfs_remove_group(&dev->dev.kobj,&xgpio_group);
	//devm_gpio_free(&dev->dev,xdev->gpio);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id keys_of_match[] = {
	{ .compatible = "test-leds", },
	{ }
};
MODULE_DEVICE_TABLE(of, keys_of_match);
#endif

static struct platform_driver __refdata keys_driver = {
	.driver = {
		.name = "test-leds",
		.of_match_table = of_match_ptr(keys_of_match),
	},
	.probe = keys_probe,
	.remove = keys_remove,
	
};
module_platform_driver(keys_driver);

MODULE_AUTHOR("Alexandre Belloni <alexandre.belloni@free-electrons.com>");
MODULE_DESCRIPTION("Input driver for resistor ladder connected on ADC");
MODULE_LICENSE("GPL v2");