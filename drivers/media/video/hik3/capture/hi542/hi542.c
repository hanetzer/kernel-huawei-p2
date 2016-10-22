/*
 *  hi542 camera driver source file
 *
 *  CopyRight (C) Hisilicon Co., Ltd.
 *	Author :
 *  Version:  1.2
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/videodev2.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <asm/div64.h>
#include <mach/hisi_mem.h>
#include "mach/hardware.h"
#include <mach/boardid.h>
#include <mach/gpio.h>
#include "../isp/sensor_common.h"
#include "hi542.h"
/*#include "../isp/k3_isp_io.h"*/
#include <asm/bug.h>
#include <linux/device.h>

#define LOG_TAG "HI542"
/* #define DEBUG_DEBUG 1 */
#include "../isp/cam_log.h"
#include <../isp/cam_util.h>
#include <hsad/config_interface.h>

#define HI542_SLAVE_ADDRESS 0x40
#define HI542_CHIP_ID       (0xB1)

#define HI542_CAM_MODULE_SKIPFRAME     4

#define HI542_FLIP		0x0011
#define HI542_PIXEL_ORDER_CHANGED	(1)

#define HI542_GAIN_REG_H	0x0589
#define HI542_GAIN_REG_L	0x058A
#define HI542_EXPOSURE_REG_0	0x0111
#define HI542_EXPOSURE_REG_1	0x0112
#define HI542_EXPOSURE_REG_2	0x0113

//#define HI542_FOXCONN_AP_WRITEAE_MODE
/* camera sensor override parameters, define in binning preview mode */
#define HI542_MAX_ISO			800
#define HI542_MIN_ISO			100

const struct isp_reg_t isp_init_regs_hi542[] = {

};

static framesize_s hi542_framesizes[] = {
	{0, 0, 1280, 960, 2791, 988, 30, 30, 0x1bc, 0x172, 0x100, VIEW_FULL, RESOLUTION_4_3, false, {hi542_framesize_pre_30fps, ARRAY_SIZE(hi542_framesize_pre_30fps)} },
	//{0, 0, 2608, 1952, 2791, 1980, 15, 15, 0x1bc, 0x172, 0x100, VIEW_FULL, RESOLUTION_4_3, false, {hi542_framesize_full_15fps, ARRAY_SIZE(hi542_framesize_full_15fps)} },
};

static camera_sensor hi542_sensor;
static void hi542_set_default(void);
#ifdef CONFIG_DEBUG_FS
extern u32 get_main_sensor_id(void);
#endif
/*
 **************************************************************************
 * FunctionName: hi542_read_reg;
 * Description : read hi542 reg by i2c;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi542_read_reg(u16 reg, u8 *val)
{
	return k3_ispio_read_reg(hi542_sensor.i2c_config.index,
				 hi542_sensor.i2c_config.addr, reg, (u16 *)val, hi542_sensor.i2c_config.val_bits);
}

/*
 **************************************************************************
 * FunctionName: hi542_write_reg;
 * Description : write hi542 reg by i2c;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi542_write_reg(u16 reg, u8 val, u8 mask)
{
	return k3_ispio_write_reg(hi542_sensor.i2c_config.index,
			hi542_sensor.i2c_config.addr, reg, val, hi542_sensor.i2c_config.val_bits, mask);
}

/*
 **************************************************************************
 * FunctionName: hi542_write_seq;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int hi542_write_seq(const struct _sensor_reg_t *seq, u32 size, u8 mask)
{
	print_debug("Enter %s, seq[%#x], size=%d", __func__, (int)seq, size);
	return k3_ispio_write_seq(hi542_sensor.i2c_config.index,
			hi542_sensor.i2c_config.addr, seq, size, hi542_sensor.i2c_config.val_bits, mask);
}

/*
 **************************************************************************
 * FunctionName: hi542_write_isp_seq;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void hi542_write_isp_seq(const struct isp_reg_t *seq, u32 size)
{
	print_debug("Enter %s, seq[%#x], size=%d", __func__, (int)seq, size);
	k3_ispio_write_isp_seq(seq, size);
}


/*
 **************************************************************************
 * FunctionName: hi542_enum_frame_intervals;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi542_enum_frame_intervals(struct v4l2_frmivalenum *fi)
{
	assert(fi);

	print_debug("enter %s", __func__);
	if (fi->index >= CAMERA_MAX_FRAMERATE)
		return -EINVAL;

	fi->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fi->discrete.numerator = 1;
	fi->discrete.denominator = (fi->index+1);
	return 0;
}


/*
 **************************************************************************
 * FunctionName: hi542_get_format;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi542_get_format(struct v4l2_fmtdesc *fmt)
{
	if (fmt->type == V4L2_BUF_TYPE_VIDEO_OVERLAY)
		fmt->pixelformat = hi542_sensor.fmt[STATE_PREVIEW];
	else
		fmt->pixelformat = hi542_sensor.fmt[STATE_CAPTURE];

	return 0;
}

/*
 **************************************************************************
 * FunctionName: hi542_enum_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi542_enum_framesizes(struct v4l2_frmsizeenum *framesizes)
{
	u32 max_index = ARRAY_SIZE(camera_framesizes) - 1;
	u32 this_max_index = ARRAY_SIZE(hi542_framesizes) - 1;

	assert(framesizes);

	print_debug("enter %s; ", __func__);

	if (framesizes->index > max_index) {
		print_error("framesizes->index = %d error", framesizes->index);
		return -EINVAL;
	}

	if ((camera_framesizes[framesizes->index].width > hi542_framesizes[this_max_index].width)
		|| (camera_framesizes[framesizes->index].height > hi542_framesizes[this_max_index].height)) {
		print_error("framesizes->index = %d error", framesizes->index);
		return -EINVAL;
	}

	framesizes->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	framesizes->discrete.width = hi542_framesizes[this_max_index].width;
	framesizes->discrete.height = hi542_framesizes[this_max_index].height;

	return 0;
}

/*
 **************************************************************************
 * FunctionName: hi542_try_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi542_try_framesizes(struct v4l2_frmsizeenum *framesizes)
{
	int max_index = ARRAY_SIZE(hi542_framesizes) - 1;

	assert(framesizes);

	print_debug("Enter Function:%s  ", __func__);


	if ((framesizes->discrete.width <= hi542_framesizes[max_index].width)
	    && (framesizes->discrete.height <= hi542_framesizes[max_index].height)) {
		print_debug("===========width = %d", framesizes->discrete.width);
		print_debug("===========height = %d", framesizes->discrete.height);
		return 0;
	}

	print_error("frame size too large, [%d,%d]",
		    framesizes->discrete.width, framesizes->discrete.height);
	return -EINVAL;
}

/*
 **************************************************************************
 * FunctionName: hi542_set_framesizes;
 * Description : NA;
 * Input       : flag: if 1, set framesize to sensor,
 *					   if 0, only store framesize to camera_interface;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi542_set_framesizes(camera_state state,
				 struct v4l2_frmsize_discrete *fs, int flag, camera_setting_view_type view_type)
{
	int i = 0;
	bool match = false;

	assert(fs);

	print_info("Enter Function:%s State(%d), flag=%d, width=%d, height=%d",
		    __func__, state, flag, fs->width, fs->height);

	if (VIEW_FULL == view_type) {
		for (i = 0; i < ARRAY_SIZE(hi542_framesizes); i++) {
			if ((hi542_framesizes[i].width >= fs->width)
			    && (hi542_framesizes[i].height >= fs->height)
			    && (VIEW_FULL == hi542_framesizes[i].view_type)
			    && (camera_get_resolution_type(fs->width, fs->height)
			    <= hi542_framesizes[i].resolution_type)) {
				fs->width = hi542_framesizes[i].width;
				fs->height = hi542_framesizes[i].height;
				match = true;
				break;
			}
		}
	}

	if (false == match) {
		for (i = 0; i < ARRAY_SIZE(hi542_framesizes); i++) {
			if ((hi542_framesizes[i].width >= fs->width)
			    && (hi542_framesizes[i].height >= fs->height)
			    && (camera_get_resolution_type(fs->width, fs->height)
			    <= hi542_framesizes[i].resolution_type)) {
				fs->width = hi542_framesizes[i].width;
				fs->height = hi542_framesizes[i].height;
				break;
			}
		}
	}

	if (i >= ARRAY_SIZE(hi542_framesizes)) {
		print_error("request resolution larger than sensor's max resolution");
		return -EINVAL;
	}

	if (state == STATE_PREVIEW)
		hi542_sensor.preview_frmsize_index = i;
	else
		hi542_sensor.capture_frmsize_index = i;

	return 0;
}

/*
 **************************************************************************
 * FunctionName: hi542_get_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi542_get_framesizes(camera_state state,
				 struct v4l2_frmsize_discrete *fs)
{
	int frmsize_index;

	assert(fs);

	if (state == STATE_PREVIEW)
		frmsize_index = hi542_sensor.preview_frmsize_index;
	else if (state == STATE_CAPTURE)
		frmsize_index = hi542_sensor.capture_frmsize_index;
	else
		return -EINVAL;

	fs->width = hi542_framesizes[frmsize_index].width;
	fs->height = hi542_framesizes[frmsize_index].height;

	return 0;
}

/*
 **************************************************************************
 * FunctionName: hi542_init_reg;
 * Description : download initial seq for sensor init;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi542_init_reg(void)
{
	int size = 0;

	print_debug("Enter Function:%s  , initsize=%d",
		    __func__, sizeof(hi542_init_regs));

	size = ARRAY_SIZE(isp_init_regs_hi542);
	hi542_write_isp_seq(isp_init_regs_hi542, size);

	if (0 != k3_ispio_init_csi(hi542_sensor.mipi_index,
			hi542_sensor.mipi_lane_count, hi542_sensor.lane_clk)) {
		return -EFAULT;
	}

	size = ARRAY_SIZE(hi542_init_regs);
	if (0 != hi542_write_seq(hi542_init_regs, size, 0x00)) {
		print_error("line %d, fail to init hi542 sensor", __LINE__);
		return -EFAULT;
	}

	return 0;
}

static int hi542_set_hflip(int flip)
{
	print_debug("enter %s flip=%d", __func__, flip);
	hi542_sensor.hflip = flip;
	return 0;
}
static int hi542_get_hflip(void)
{
	print_debug("enter %s", __func__);

	return hi542_sensor.hflip;
}
static int hi542_set_vflip(int flip)
{
	print_debug("enter %s flip=%d", __func__, flip);

	hi542_sensor.vflip = flip;

	return 0;
}
static int hi542_get_vflip(void)
{
	print_debug("enter %s", __func__);
	return hi542_sensor.vflip;
}

static int hi542_update_flip(u16 width, u16 height)
{
	u8 new_flip = ((hi542_sensor.vflip << 1) | hi542_sensor.hflip);
	print_debug("Enter %s  ", __func__);
	if (hi542_sensor.old_flip != new_flip) {
		k3_ispio_update_flip((hi542_sensor.old_flip ^ new_flip) & 0x03, width, height, HI542_PIXEL_ORDER_CHANGED);

		hi542_sensor.old_flip = new_flip;
		hi542_write_reg(HI542_FLIP, hi542_sensor.vflip ? 0x02 : 0x00, ~0x02);
		hi542_write_reg(HI542_FLIP, hi542_sensor.hflip ? 0x01 : 0x00, ~0x01);
	}
	msleep(200);
	return 0;
}

/*
 **************************************************************************
 * FunctionName: hi542_framesize_switch;
 * Description : switch frame size, used by preview and capture
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi542_framesize_switch(camera_state state)
{
	u8 next_frmsize_index;

	if (state == STATE_PREVIEW)
		next_frmsize_index = hi542_sensor.preview_frmsize_index;
	else
		next_frmsize_index = hi542_sensor.capture_frmsize_index;

	print_debug("Enter Function:%s frm index=%d", __func__, next_frmsize_index);

	if (next_frmsize_index >= ARRAY_SIZE(hi542_framesizes)){
		print_error("Unsupport sensor setting index: %d",next_frmsize_index);
		return -ETIME;
	}

	if (0 != hi542_write_seq(hi542_sensor.frmsize_list[next_frmsize_index].sensor_setting.setting
		,hi542_sensor.frmsize_list[next_frmsize_index].sensor_setting.seq_size, 0x00)) {
		print_error("fail to init sonyimx134 sensor");
		return -ETIME;
	}

	return 0;
}


/*
 **************************************************************************
 * FunctionName: hi542_stream_on;
 * Description : download preview seq for sensor preview;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi542_stream_on(camera_state state)
{
	print_debug("Enter Function:%s ", __func__);
	return hi542_framesize_switch(state);
}


/*  **************************************************************************
* FunctionName: hi542_check_sensor;
* Description : NA;
* Input       : NA;
* Output      : NA;
* ReturnValue : NA;
* Other       : NA;
***************************************************************************/
static int hi542_check_sensor(void)
{
	u8 idl = 0x1;
	u8 idh = 0x1;
	u16 id = 0;

	int ret = -1;
	u8 val = 1;

	//ret = hi542_read_reg(0x0000, &idh);
	//ret = hi542_read_reg(0x0001, &idl);
#if 0
	ret = hi542_read_reg(0x0002, &val);
	print_info("%s: read 0x0002, ret=%d, val=%d.", __func__, ret, val);

	ret = hi542_read_reg(0x0004, &val);
	print_info("%s: read 0x0004, ret=%d, val=%d.", __func__, ret, val);

	hi542_write_reg(HI542_FLIP, 0x3,  0);
	ret = hi542_read_reg(HI542_FLIP, &val);
	print_info("%s: read 0x0101, ret=%d, val=%d.", __func__, ret, val);
#endif

	ret = hi542_read_reg(0x0004, &idl);
	//id = ((idh << 8) | idl);
	id = idl;
	print_info("hi542 product id:0x%x", id);
#ifdef CONFIG_DEBUG_FS
	if (HI542_CHIP_ID != id) {
		id = (u16)get_main_sensor_id();
		print_info("hi542 debugfs product id:0x%x", id);
	}
#endif
	if (HI542_CHIP_ID != id) {
		print_error("Invalid product id ,Could not load sensor hi542");
		return -ENODEV;
	}

	return 0;
}

/****************************************************************************
* FunctionName: hi542_power;
* Description : NA;
* Input       : NA;
* Output      : NA;
* ReturnValue : NA;
* Other       : NA;
***************************************************************************/
int hi542_power(camera_power_state power)
{
	int ret = 0;

	if (power == POWER_ON) {
		k3_ispldo_power_sensor(power, "pri-cameralog-vcc");
		ret = camera_power_core_ldo(power);
		udelay(200);
		k3_ispldo_power_sensor(power, "camera-vcc");
		udelay(1);
		k3_ispldo_power_sensor(power, "sec-cameralog-vcc");
		k3_isp_io_enable_mclk(MCLK_ENABLE, hi542_sensor.sensor_index);
		udelay(20);
		ret = k3_ispgpio_power_sensor(&hi542_sensor, power);
		ret = k3_ispio_ioconfig(&hi542_sensor, power);
		k3_ispgpio_reset_sensor(hi542_sensor.sensor_index, power,
					hi542_sensor.power_conf.reset_valid);
		udelay(500);
	} else {
		k3_ispio_deinit_csi(hi542_sensor.mipi_index);
		ret = k3_ispio_ioconfig(&hi542_sensor, power);
		ret = k3_ispgpio_power_sensor(&hi542_sensor, power);
		k3_ispldo_power_sensor(power, "sec-cameralog-vcc");
		camera_power_core_ldo(power);
		udelay(200);
		k3_ispldo_power_sensor(power, "pri-cameralog-vcc");
		k3_ispldo_power_sensor(power, "camera-vcc");
		k3_ispgpio_reset_sensor(hi542_sensor.sensor_index, power,
					hi542_sensor.power_conf.reset_valid);
		k3_isp_io_enable_mclk(MCLK_DISABLE, hi542_sensor.sensor_index);
	}
	return ret;
}

void hi542_set_gain(u32 gain)
{
	if (gain == 0)
		return;
	hi542_write_reg(HI542_GAIN_REG_H, (gain >> 8) & 0xff, 0x00);
	hi542_write_reg(HI542_GAIN_REG_L, gain & 0xff, 0x00);
}

void hi542_set_exposure(u32 exposure)
{
	hi542_write_reg(HI542_EXPOSURE_REG_0, (exposure >> 16) & 0x0f, 0x00);
	hi542_write_reg(HI542_EXPOSURE_REG_1, (exposure >> 8) & 0xff, 0x00);
	hi542_write_reg(HI542_EXPOSURE_REG_2, exposure & 0xf0, 0x00);	/*fraction part not used */
}

static u32 hi542_get_override_param(camera_override_type_t type)
{
	u32 ret_val = sensor_override_params[type];

	switch (type) {
	case OVERRIDE_ISO_HIGH:
		ret_val = HI542_MAX_ISO;
		break;

	case OVERRIDE_ISO_LOW:
		ret_val = HI542_MIN_ISO;
		break;

	default:
		print_error("%s:not override or invalid type %d, use default",__func__, type);
	}

	return ret_val;
}

/*
 **************************************************************************
 * FunctionName: hi542_reset;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/

static int hi542_reset(camera_power_state power_state)
{
	print_debug("%s  ", __func__);
/*
	if (POWER_ON == power_state) {
		k3_isp_io_enable_mclk(MCLK_ENABLE, hi542_sensor.sensor_index);
		k3_ispgpio_reset_sensor(hi542_sensor.sensor_index, power_state,
			      hi542_sensor.power_conf.reset_valid);
		udelay(500);
	} else {
		k3_ispgpio_reset_sensor(hi542_sensor.sensor_index, power_state,
			      hi542_sensor.power_conf.reset_valid);
		udelay(10);
		k3_isp_io_enable_mclk(MCLK_DISABLE, hi542_sensor.sensor_index);
	}
*/
	return 0;
}

/*
 **************************************************************************
 * FunctionName: hi542_init;
 * Description : hi542 init function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : Error code indicating success or failure;
 * Other       : NA;
 **************************************************************************
*/
static int hi542_init(void)
{
	static bool hi542_check = false;
	print_debug("%s  ", __func__);
/*
	if (false == hi542_check)
	{
		if (check_suspensory_camera("HI542") != 1)
		{
			return -ENODEV;
		}
		hi542_check = true;
	}	
*/	
#if 0
	if (!camera_timing_is_match(0)) {
		print_error("%s: sensor timing don't match.\n", __func__);
		return -ENODEV;
	}
#endif

	if (hi542_sensor.owner && !try_module_get(hi542_sensor.owner)) {
		print_error("%s: try_module_get fail", __func__);
		return -ENOENT;
	}

	k3_ispio_power_init("pri-cameralog-vcc", LDO_VOLTAGE_28V, LDO_VOLTAGE_28V);	/*analog 2.85V*/
	k3_ispio_power_init("camera-vcc", LDO_VOLTAGE_18V, LDO_VOLTAGE_18V);	/*IO 1.8V*/
	//k3_ispio_power_init("cameravcm-vcc", LDO_VOLTAGE_28V, LDO_VOLTAGE_28V);	/*AF 2.85V*/
	k3_ispio_power_init("sec-cameralog-vcc", LDO_VOLTAGE_28V, LDO_VOLTAGE_28V);	/*analog 2.85V*/

	return 0;
}

/*
 **************************************************************************
 * FunctionName: hi542_exit;
 * Description : hi542 exit function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void hi542_exit(void)
{
	print_debug("enter %s", __func__);

	k3_ispio_power_deinit();

	if (hi542_sensor.owner)
		module_put(hi542_sensor.owner);

	print_debug("exit %s", __func__);
}

/*
 **************************************************************************
 * FunctionName: hi542_shut_down;
 * Description : hi542 shut down function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void hi542_shut_down(void)
{
	print_debug("enter %s", __func__);
	k3_ispgpio_power_sensor(&hi542_sensor, POWER_OFF);
}

/*
 **************************************************************************
 * FunctionName: hi542_set_default;
 * Description : init hi542_sensor;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void hi542_set_default(void)
{
	unsigned int chip_id;
	int i;

	hi542_sensor.init = hi542_init;
	hi542_sensor.exit = hi542_exit;
	hi542_sensor.shut_down = hi542_shut_down;
	hi542_sensor.reset = hi542_reset;
	hi542_sensor.check_sensor = hi542_check_sensor;
	hi542_sensor.power = hi542_power;
	hi542_sensor.init_reg = hi542_init_reg;
	hi542_sensor.stream_on = hi542_stream_on;

	hi542_sensor.get_format = hi542_get_format;
	hi542_sensor.set_flash = NULL;
	hi542_sensor.get_flash = NULL;
	hi542_sensor.set_scene = NULL;
	hi542_sensor.get_scene = NULL;

	hi542_sensor.enum_framesizes = hi542_enum_framesizes;
	hi542_sensor.try_framesizes = hi542_try_framesizes;
	hi542_sensor.set_framesizes = hi542_set_framesizes;
	hi542_sensor.get_framesizes = hi542_get_framesizes;

	hi542_sensor.enum_frame_intervals = hi542_enum_frame_intervals;
	hi542_sensor.try_frame_intervals = NULL;
	hi542_sensor.set_frame_intervals = NULL;
	hi542_sensor.get_frame_intervals = NULL;

	hi542_sensor.get_capability = NULL;

	hi542_sensor.set_hflip = hi542_set_hflip;
	hi542_sensor.get_hflip = hi542_get_hflip;
	hi542_sensor.set_vflip = hi542_set_vflip;
	hi542_sensor.get_vflip = hi542_get_vflip;
	hi542_sensor.update_flip = hi542_update_flip;

	strcpy(hi542_sensor.info.name, "hi542_sunny");
	hi542_sensor.interface_type = MIPI2;
	hi542_sensor.mipi_lane_count = CSI_LINES_2;
	hi542_sensor.mipi_index = CSI_INDEX_1;
	hi542_sensor.sensor_index = CAMERA_SENSOR_SECONDARY;
	hi542_sensor.skip_frames = 0;

	hi542_sensor.power_conf.pd_valid = LOW_VALID;
	hi542_sensor.power_conf.reset_valid = LOW_VALID;
	hi542_sensor.power_conf.vcmpd_valid = LOW_VALID;

	hi542_sensor.i2c_config.index = I2C_PRIMARY;
	hi542_sensor.i2c_config.speed = I2C_SPEED_400;
	hi542_sensor.i2c_config.addr = HI542_SLAVE_ADDRESS;
	hi542_sensor.i2c_config.addr_bits = 16;
	hi542_sensor.i2c_config.val_bits = I2C_8BIT;

	hi542_sensor.preview_frmsize_index = 0;
	hi542_sensor.capture_frmsize_index = 0;
	hi542_sensor.frmsize_list = hi542_framesizes;
	hi542_sensor.fmt[STATE_PREVIEW] = V4L2_PIX_FMT_RAW10;
	hi542_sensor.fmt[STATE_CAPTURE] = V4L2_PIX_FMT_RAW10;
#ifdef HI542_FOXCONN_AP_WRITEAE_MODE
	hi542_sensor.aec_addr[0] = 0; /*high byte*/
	hi542_sensor.aec_addr[1] = 0;
	hi542_sensor.aec_addr[2] = 0;
	hi542_sensor.agc_addr[0] = 0; /*high byte*/
	hi542_sensor.agc_addr[1] = 0;
#else
	hi542_sensor.aec_addr[0] = 0x0111; /*high byte*/
	hi542_sensor.aec_addr[1] = 0x0112;
	hi542_sensor.aec_addr[2] = 0x0113;
	hi542_sensor.agc_addr[0] = 0x0589; /*high byte*/
	hi542_sensor.agc_addr[1] = 0x058A;
#endif

	hi542_sensor.sensor_type = 0;

	hi542_sensor.set_gain = hi542_set_gain;
	hi542_sensor.set_exposure = hi542_set_exposure;
	hi542_sensor.get_override_param = hi542_get_override_param;
	
	hi542_sensor.set_vts = NULL;
	hi542_sensor.get_vts_reg_addr = NULL;

	hi542_sensor.sensor_gain_to_iso = NULL;
	hi542_sensor.sensor_iso_to_gain = NULL;

	hi542_sensor.set_effect = NULL;

	hi542_sensor.isp_location = CAMERA_USE_K3ISP;
	hi542_sensor.sensor_tune_ops = NULL;

	hi542_sensor.af_enable = 0;

	hi542_sensor.image_setting.lensc_param =NULL; 
	hi542_sensor.image_setting.ccm_param =NULL; 
	hi542_sensor.image_setting.awb_param = NULL; 

	hi542_sensor.fps_max = 30;
	hi542_sensor.fps_min = 15;
	hi542_sensor.fps = 25;

	hi542_sensor.owner = THIS_MODULE;

	hi542_sensor.info.facing = CAMERA_FACING_BACK;
	hi542_sensor.info.orientation = 90;   /*270;*/
	hi542_sensor.info.focal_length = 439; /* 4.39mm*/
	hi542_sensor.info.h_view_angle = 66; /*  66.1 degree */
	hi542_sensor.info.v_view_angle = 50;
	hi542_sensor.lane_clk = CLK_400M;
	hi542_sensor.hflip = 0;
	hi542_sensor.vflip = 0;
	hi542_sensor.old_flip = 0;

}

/*
 **************************************************************************
 * FunctionName: hi542_module_init;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static __init int hi542_module_init(void)
{
	hi542_set_default();
	return register_camera_sensor(hi542_sensor.sensor_index, &hi542_sensor);
}

/*
 **************************************************************************
 * FunctionName: hi542_module_exit;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void __exit hi542_module_exit(void)
{
	unregister_camera_sensor(hi542_sensor.sensor_index, &hi542_sensor);
}

MODULE_AUTHOR("Hisilicon");
module_init(hi542_module_init);
module_exit(hi542_module_exit);
MODULE_LICENSE("GPL");

/********************************** END **********************************************/
