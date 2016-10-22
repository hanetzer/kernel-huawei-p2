/*
 * Torchclass driver 
 *
 *  Author: 	Zhoujie (zhou.jie1981@163.com)
 *  Date:  	2013/01/16
 *  Version:	1.0
 *  History:	2013/01/16      Frist add driver for Torchclass,this is virtual device to manage torch 
 *  
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>


#define LOG_TAG "K3_TORCH"

//#define DEBUG_DEBUG 1
#include "../isp/cam_log.h"

typedef  struct  _torch_class {
	struct class *classptr;
	struct device *pDevice;
}torch_class;

torch_class torch;

static int brightness_level = 0;
dev_t devnum;

#define TORCH_LED    "torch_led"

static int __init torch_module_init(void)
{
	int ret;
       ret = alloc_chrdev_region(&devnum, 0, 1, TORCH_LED);
	if(ret)
	{
		printk("error %s fail to alloc a dev_t!!!\n",__func__);
		return -1;
	}

	torch.classptr= class_create(THIS_MODULE, "torch");
	if (IS_ERR(torch.classptr)) {
		pr_info("class_create failed %d\n", ret);
		ret = PTR_ERR(torch.classptr);
		return -1;
	}
	return 0;
}

int register_torch_led(struct device_attribute *attr)
{
	int ret;
	torch.classptr->dev_attrs = attr;
	torch.pDevice  = device_create(torch.classptr, NULL, devnum,NULL,"%s",TORCH_LED);

	if (IS_ERR(torch.pDevice)) {
		pr_info("class_device_create failed %s \n", TORCH_LED);
		ret = PTR_ERR(torch.pDevice);
		return -1;
	}
	return 0;
}

EXPORT_SYMBOL(register_torch_led);

static void __exit torch_module_deinit(void)
{
	device_destroy(torch.classptr, devnum);
	class_destroy(torch.classptr);
	unregister_chrdev_region(devnum, 1);
}

module_init(torch_module_init);
module_exit(torch_module_deinit);
MODULE_AUTHOR("Jiezhou");
MODULE_DESCRIPTION("Torch led virtul device");
MODULE_LICENSE("GPL");
