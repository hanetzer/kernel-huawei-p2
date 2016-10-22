/*
 * sonyimx134 sensor driver 
 *
 *  Author: 	Zhoujie (zhou.jie1981@163.com)
 *  Date:  	2013/01/05
 *  Version:	1.0
 *  History:	2013/01/05      Frist add driver for sonyimx134 
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
#include "sonyimx134.h"
/*#include "../isp/k3_isp_io.h"*/
#include <asm/bug.h>
#include <linux/device.h>

#define LOG_TAG "SONYIMX134"
//#define DEBUG_DEBUG 1 
#include "../isp/cam_log.h"
#include <../isp/cam_util.h>
#include "imx134_tune.h"
//#define SONYIMX135_AP_WRITEAE_MODE
#define  SONYIMX134_MAX_ISO 			1550


#define SONYIMX134_MIN_ISO                      100

#define SONYIMX134_AUTO_FPS_MAX_GAIN 0x60

#define SONYIMX134_AUTO_FPS_MIN_GAIN 0x24

#define SONYIMX134_MAX_FRAMERATE         30

#define SONYIMX134_MIN_FRAMERATE          10

#define SONYIMX134_MIN_CAP_FRAMERATE  8

#define SONYIMX134_FLASH_TRIGGER_GAIN 0xff

#define SONYIMX134_SHARPNESS_PREVIEW  0x30

#define SONYIMX134_SHARPNESS_CAPTURE  0x08

#define SONYIMX134_SLAVE_ADDRESS 0x20
#define SONYIMX134_CHIP_ID       (0x0134)

#define SONYIMX134_CAM_MODULE_SKIPFRAME     4

#define SONYIMX134_FLIP		0x0101

#define SONYIMX134_EXPOSURE_REG_H	0x0202
#define SONYIMX134_EXPOSURE_REG_L	0x0203
#define SONYIMX134_GAIN_REG_H		0x0204
#define SONYIMX134_GAIN_REG_L		0x0205

#define SONYIMX134_VTS_REG_H		0x0340
#define SONYIMX134_VTS_REG_L		0x0341

#define SONYIMX134_APERTURE_FACTOR  200 //F2.0
#define SONYIMX134_EQUIVALENT_FOCUS	0
enum sensor_module_type 
{
	MODULE_LITEON,
	MODULE_SUNNY,
	MODULE_UNSUPPORT
};

static u8 sensor_module;
#ifdef IMX134_OTP
#define OTP_VCM  		0xa6
#define OTP_VCM_REG 	0x40
#define OTP_ID_AWB  	0xa4
#define OTP_ID_REG 		0x00
#define OTP_AWB_REG 	0x05
#define OTP_LSC_1  		0xa4
#define OTP_LSC_2  		0xa6
#define OTP_LSC_1_REG 	0x0b
#define OTP_LSC_2_REG	0x00	

#define SONYIMX134_OTP_ID_READ				(1 << 0)
#define SONYIMX134_OTP_VCM_READ				(1 << 1)
#define SONYIMX134_OTP_LSC_READ				(1 << 2)
#define SONYIMX134_OTP_LSC_WRITED			(1 << 3)
#define SONYIMX134_OTP_LSC_FILE_ERR         		(1 << 4)
#define SONYIMX134_OTP_AWB_READ				(1 << 5)


#define SONYIMX134_SENSOR_LSC_MODE			0x0700
#define SONYIMX134_SENSOR_LSC_EN				0x4500
#define SONYIMX134_SENSOR_RAM_SEL			0x3A63

#define SONYIMX134_SENSOR_LSC_RAM			0x4800
//7*5*4*2
#define SONYIMX134_OTP_LSC_SIZE  280
static u8 sonyimx134_otp_flag = 0;
/* VCM start and end values */
static u16 sonyimx134_vcm_start = 0;
static u16 sonyimx134_vcm_end = 0;

static u8 sonyimx134_otp_lsc_param[SONYIMX134_OTP_LSC_SIZE] ;
extern int ispv1_read_sensor_byte_addr8(i2c_index_t index, u8 i2c_addr, u16 reg, u16 *val, i2c_length length);//add by zhoujie
static void sonyimx134_get_otp_from_sensor(void);
static void sonyimx134_otp_get_vcm(u16 *vcm_start, u16 *vcm_end);
static bool sonyimx134_otp_enable_lsc(bool enable);
static bool  sonyimx134_otp_read_vcm(void);
static bool sonyimx134_otp_set_lsc(void);
int sonyimx134_read_otp(u8 i2c_addr,u16 reg,u8 *buf,u16 count);
extern void _sonyimx134_otp_get_vcm(u16 *vcm_start, u16 *vcm_end,vcm_info_s *vcm,u16 otp_vcm_start, u16 otp_vcm_end);
#endif

extern int sonyimx134_get_af_param(camera_af_param_t type);

static camera_capability sonyimx134_cap[] = {
	{V4L2_CID_FLASH_MODE, THIS_FLASH},
	{V4L2_CID_FOCUS_MODE, THIS_FOCUS_MODE},
};
/*
 * should be calibrated, three lights, from 0x1c264
 * here is long exposure
 */
char sonyimx134_lensc_param[86*3] = {
};

/* should be calibrated, 6 groups 3x3, from 0x1c1d8 */
short sonyimx134_ccm_param[54] = {
};

char sonyimx134_awb_param[] = {
};

static framesize_s sonyimx134_framesizes[] = {
	/* 1600x1200, just close with quarter size */
	{0, 2, 1600, 1200, 3600, 1480, 30, 30, 0x1BC, 0x172, 0x100, VIEW_FULL, RESOLUTION_4_3, true, {sonyimx134_framesize_1600x1200, ARRAY_SIZE(sonyimx134_framesize_1600x1200)} }, 

	/* 5M wide(2560x1440), 1080P and 1080P EIS use this size */
	//{0, 2, 2560, 1440, 3600, 1480, 30, 30, 0x1bc, 0x172, 0x200, VIEW_CROP, RESOLUTION_16_9, false, {sonyimx134_framesize_5M_wide, ARRAY_SIZE(sonyimx134_framesize_5M_wide)} },

	/* 8M wide 3264x1856 */
	//{0, 2, 3280, 1856, 3600, 1878, 24, 24, 0x190, 0x14d, 0x1ff, VIEW_FULL, RESOLUTION_16_9, false, {sonyimx134_framesize_8M_wide, ARRAY_SIZE(sonyimx134_framesize_8M_wide)} },

	/* full size 15fps cs, es will revise a little(not correct 13fps) */
	{0, 2, 3264, 2448, 3600, 2962, 15, 15, 0x1BC, 0x172, 0x200, VIEW_FULL, RESOLUTION_4_3, false, {sonyimx134_framesize_full, ARRAY_SIZE(sonyimx134_framesize_full)} },
};

static camera_sensor sonyimx134_sensor;
static void sonyimx134_set_default(void);

/*
 **************************************************************************
 * FunctionName: sonyimx134_read_reg;
 * Description : read sonyimx134 reg by i2c;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int sonyimx134_read_reg(u16 reg, u8 *val)
{
	return k3_ispio_read_reg(sonyimx134_sensor.i2c_config.index,
				 sonyimx134_sensor.i2c_config.addr, reg, (u16*)val, sonyimx134_sensor.i2c_config.val_bits);
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_write_reg;
 * Description : write sonyimx134 reg by i2c;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int sonyimx134_write_reg(u16 reg, u8 val, u8 mask)
{
	return k3_ispio_write_reg(sonyimx134_sensor.i2c_config.index,
			sonyimx134_sensor.i2c_config.addr, reg, val, sonyimx134_sensor.i2c_config.val_bits, mask);
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_write_seq;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int sonyimx134_write_seq(const struct _sensor_reg_t *seq, u32 size, u8 mask)
{
	print_debug("Enter %s, seq[%#x], size=%d", __func__, (int)seq, size);
	return k3_ispio_write_seq(sonyimx134_sensor.i2c_config.index,
			sonyimx134_sensor.i2c_config.addr, seq, size, sonyimx134_sensor.i2c_config.val_bits, mask);
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_write_isp_seq;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void sonyimx134_write_isp_seq(const struct isp_reg_t *seq, u32 size)
{
	print_debug("Enter %s, seq[%#x], size=%d", __func__, (int)seq, size);
	k3_ispio_write_isp_seq(seq, size);
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_enum_frame_intervals;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int sonyimx134_enum_frame_intervals(struct v4l2_frmivalenum *fi)
{
	assert(fi);

	print_debug("enter %s", __func__);
	if (fi->index >= CAMERA_MAX_FRAMERATE) {
		return -EINVAL;
	}

	fi->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fi->discrete.numerator = 1;
	fi->discrete.denominator = (fi->index + 1);
	return 0;
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_get_format;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int sonyimx134_get_format(struct v4l2_fmtdesc *fmt)
{
	if (fmt->type == V4L2_BUF_TYPE_VIDEO_OVERLAY) {
		fmt->pixelformat = sonyimx134_sensor.fmt[STATE_PREVIEW];
	} else {
		fmt->pixelformat = sonyimx134_sensor.fmt[STATE_CAPTURE];
	}
	return 0;
}

static int sonyimx134_get_capability(u32 id, u32 *value)
{
	int i;
	for (i = 0; i < sizeof(sonyimx134_cap) / sizeof(sonyimx134_cap[0]); ++i) {
		if (id == sonyimx134_cap[i].id) {
			*value = sonyimx134_cap[i].value;
			break;
		}
	}
	return 0;
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_enum_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int sonyimx134_enum_framesizes(struct v4l2_frmsizeenum *framesizes)
{
	u32 max_index = ARRAY_SIZE(camera_framesizes) - 1;
	u32 this_max_index = ARRAY_SIZE(sonyimx134_framesizes) - 1;

	assert(framesizes);

	print_debug("enter %s; ", __func__);

	if (framesizes->index > max_index) {
		print_error("framesizes->index = %d error", framesizes->index);
		return -EINVAL;
	}

	if ((camera_framesizes[framesizes->index].width > sonyimx134_framesizes[this_max_index].width)
		|| (camera_framesizes[framesizes->index].height > sonyimx134_framesizes[this_max_index].height)) {
		print_error("framesizes->index = %d error", framesizes->index);
		return -EINVAL;
	}

	framesizes->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	framesizes->discrete.width = sonyimx134_framesizes[this_max_index].width;
	framesizes->discrete.height = sonyimx134_framesizes[this_max_index].height;

	return 0;
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_try_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int sonyimx134_try_framesizes(struct v4l2_frmsizeenum *framesizes)
{
	int max_index = ARRAY_SIZE(sonyimx134_framesizes) - 1;

	assert(framesizes);

	print_debug("Enter Function:%s  ", __func__);


	if ((framesizes->discrete.width <= sonyimx134_framesizes[max_index].width)
	    && (framesizes->discrete.height <= sonyimx134_framesizes[max_index].height)) {
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
 * FunctionName: sonyimx134_set_framesizes;
 * Description : NA;
 * Input       : flag: if 1, set framesize to sensor,
 *					   if 0, only store framesize to camera_interface;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int sonyimx134_set_framesizes(camera_state state,
				 struct v4l2_frmsize_discrete *fs, int flag, camera_setting_view_type view_type)
{
	int i = 0;
	bool match = false;

	assert(fs);

	print_info("Enter Function:%s State(%d), flag=%d, width=%d, height=%d",
		   __func__, state, flag, fs->width, fs->height);

	if (VIEW_FULL == view_type) {
		for (i = 0; i < ARRAY_SIZE(sonyimx134_framesizes); i++) {
			if ((sonyimx134_framesizes[i].width >= fs->width)
			    && (sonyimx134_framesizes[i].height >= fs->height)
			    && (VIEW_FULL == sonyimx134_framesizes[i].view_type)
			    && (camera_get_resolution_type(fs->width, fs->height)
			    <= sonyimx134_framesizes[i].resolution_type)) {
				fs->width = sonyimx134_framesizes[i].width;
				fs->height = sonyimx134_framesizes[i].height;
				match = true;
				break;
			}
		}
	}

	if (false == match) {
		for (i = 0; i < ARRAY_SIZE(sonyimx134_framesizes); i++) {
			if ((sonyimx134_framesizes[i].width >= fs->width)
			    && (sonyimx134_framesizes[i].height >= fs->height)
			    && (camera_get_resolution_type(fs->width, fs->height)
			    <= sonyimx134_framesizes[i].resolution_type)) {
				fs->width = sonyimx134_framesizes[i].width;
				fs->height = sonyimx134_framesizes[i].height;
				break;
			}
		}
	}

	if (i >= ARRAY_SIZE(sonyimx134_framesizes)) {
		print_error("request resolution larger than sensor's max resolution");
		return -EINVAL;
	}

	if (state == STATE_PREVIEW) {
		sonyimx134_sensor.preview_frmsize_index = i;
	} else {
		sonyimx134_sensor.capture_frmsize_index = i;
	}

	return 0;
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_get_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int sonyimx134_get_framesizes(camera_state state,
				     struct v4l2_frmsize_discrete *fs)
{
	int frmsize_index;

	assert(fs);

	if (state == STATE_PREVIEW) {
		frmsize_index = sonyimx134_sensor.preview_frmsize_index;
	} else if (state == STATE_CAPTURE) {
		frmsize_index = sonyimx134_sensor.capture_frmsize_index;
	} else {
		return -EINVAL;
	}
	fs->width = sonyimx134_framesizes[frmsize_index].width;
	fs->height = sonyimx134_framesizes[frmsize_index].height;

	return 0;
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_init_reg;
 * Description : download initial seq for sensor init;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int sonyimx134_init_reg(void)
{
	int size = 0;

	print_debug("Enter Function:%s  , initsize=%d",
		    __func__, sizeof(sonyimx134_init_regs));


	if(sensor_module ==  MODULE_SUNNY)
	{
		size = ARRAY_SIZE(isp_init_regs_sonyimx134_Sunny);
		sonyimx134_write_isp_seq(isp_init_regs_sonyimx134_Sunny, size);
	}
	else
	{
		size = ARRAY_SIZE(isp_init_regs_sonyimx134_Liteon);
		sonyimx134_write_isp_seq(isp_init_regs_sonyimx134_Liteon, size);
	}

	if (0 != k3_ispio_init_csi(sonyimx134_sensor.mipi_index,
			sonyimx134_sensor.mipi_lane_count, sonyimx134_sensor.lane_clk)) {
		return -EFAULT;
	}

	size = ARRAY_SIZE(sonyimx134_init_regs);
	if (0 != sonyimx134_write_seq(sonyimx134_init_regs, size, 0x00)) {
		print_error("line %d, fail to init sonyimx134 sensor",__LINE__);
		return -EFAULT;
	}

#ifdef IMX134_OTP
	if((sonyimx134_otp_flag & SONYIMX134_OTP_LSC_READ) ==SONYIMX134_OTP_LSC_READ)
	{
		sonyimx134_otp_enable_lsc(false);
		sonyimx134_otp_set_lsc();
		sonyimx134_otp_enable_lsc(true);
	}
#endif
	return 0;
}

static int sonyimx134_set_hflip(int flip)
{
	print_debug("enter %s flip=%d", __func__, flip);
	sonyimx134_sensor.hflip = flip;
	return 0;
}
static int sonyimx134_get_hflip(void)
{
	print_debug("enter %s", __func__);

	return sonyimx134_sensor.hflip;
}
static int sonyimx134_set_vflip(int flip)
{
	print_debug("enter %s flip=%d", __func__, flip);

	sonyimx134_sensor.vflip = flip;

	return 0;
}
static int sonyimx134_get_vflip(void)
{
	print_debug("enter %s", __func__);
	return sonyimx134_sensor.vflip;
}

static int sonyimx134_update_flip(u16 width, u16 height)
{
	u8 new_flip = ((sonyimx134_sensor.vflip << 1) | sonyimx134_sensor.hflip);
    print_debug("Enter %s  ", __func__);
	if(sonyimx134_sensor.old_flip != new_flip) {
		k3_ispio_update_flip((sonyimx134_sensor.old_flip ^ new_flip) & 0x03, width, height, PIXEL_ORDER_CHANGED);

		sonyimx134_sensor.old_flip = new_flip;
		sonyimx134_write_reg(SONYIMX134_FLIP, sonyimx134_sensor.vflip ? 0x02 : 0x00, ~0x02);
		sonyimx134_write_reg(SONYIMX134_FLIP, sonyimx134_sensor.hflip ? 0x01 : 0x00, ~0x01);
	}
	msleep(200);
	return 0;
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_framesize_switch;
 * Description : switch frame size, used by preview and capture
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int sonyimx134_framesize_switch(camera_state state)
{
	u8 next_frmsize_index;

	if (state == STATE_PREVIEW)
		next_frmsize_index = sonyimx134_sensor.preview_frmsize_index;
	else
		next_frmsize_index = sonyimx134_sensor.capture_frmsize_index;

	print_debug("Enter Function:%s frm index=%d", __func__, next_frmsize_index);

	if (next_frmsize_index >= ARRAY_SIZE(sonyimx134_framesizes)){
		print_error("Unsupport sensor setting index: %d",next_frmsize_index);
		return -ETIME;
	}

	if (0 != sonyimx134_write_seq(sonyimx134_sensor.frmsize_list[next_frmsize_index].sensor_setting.setting
		,sonyimx134_sensor.frmsize_list[next_frmsize_index].sensor_setting.seq_size, 0x00)) {
		print_error("fail to init sonyimx134 sensor");
		return -ETIME;
	}

	return 0;
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_stream_on;
 * Description : download preview seq for sensor preview;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int sonyimx134_stream_on(camera_state state)
{
	print_debug("Enter Function:%s ", __func__);
	return sonyimx134_framesize_switch(state);
}

/*  **************************************************************************
* FunctionName: sonyimx134_check_sensor;
* Description : NA;
* Input       : NA;
* Output      : NA;
* ReturnValue : NA;
* Other       : NA;
***************************************************************************/
static int sonyimx134_check_sensor(void)
{
	u8 idl = 0;
	u8 idh = 0;
	u16 id = 0;
	int pin_id = 0;

	sonyimx134_read_reg(0x0016, &idh);
	sonyimx134_read_reg(0x0017, &idl);

	id = ((idh << 8) | idl);
	print_info("sonyimx134 product id:0x%x", id);
	if (SONYIMX134_CHIP_ID != id) {
		print_error("Invalid product id ,Could not load sensor sonyimx134\n");
		return -ENODEV;
	}

	pin_id = gpio_get_value(GPIO_13_7);
	if(pin_id < 0)
	{
		pin_id = 0;
		print_error("sonyimx134_check_sensor fail to get gpio value!!! set pin_id to 0 by default MODULE_LITEON !\n");
	}
	sensor_module = pin_id>0 ? MODULE_SUNNY:MODULE_LITEON;
	
	if(sensor_module ==  MODULE_SUNNY){
		sonyimx134_sensor.vcm = &vcm_dw9714_Sunny;
		snprintf(sonyimx134_sensor.info.name, sizeof(sonyimx134_sensor.info.name),"sonyimx134_sunny");
	}else{
		sonyimx134_sensor.vcm = &vcm_dw9714_Liteon;
		snprintf(sonyimx134_sensor.info.name, sizeof(sonyimx134_sensor.info.name),"sonyimx134_liteon");
	}
	
#ifdef IMX134_OTP
	sonyimx134_sensor.vcm->get_vcm_otp = sonyimx134_otp_get_vcm;
	sonyimx134_get_otp_from_sensor();
#endif
	return 0;
}

/****************************************************************************
* FunctionName: sonyimx134_check_sensor;
* Description : NA;
* Input       : NA;
* Output      : NA;
* ReturnValue : NA;
* Other       : NA;
***************************************************************************/
int sonyimx134_power(camera_power_state power)
{
	int ret = 0;

	print_debug("Enter Function:%s\n ", __func__);

	if (power == POWER_ON) {
		k3_ispldo_power_sensor(power, "pri-cameralog-vcc");
		ret = camera_power_core_ldo(power);
		udelay(200);
		k3_ispldo_power_sensor(power, "camera-vcc");
		k3_ispldo_power_sensor(power, "cameravcm-vcc");
		udelay(1);
		k3_ispldo_power_sensor(power, "sec-cameralog-vcc");

		k3_ispgpio_power_sensor(&sonyimx134_sensor, power);
		k3_ispio_ioconfig(&sonyimx134_sensor, power);
		msleep(3);
	} else {
		k3_ispio_deinit_csi(sonyimx134_sensor.mipi_index);
		k3_ispio_ioconfig(&sonyimx134_sensor, power);
		k3_ispgpio_power_sensor(&sonyimx134_sensor, power);

		k3_ispldo_power_sensor(power, "sec-cameralog-vcc");
		k3_ispldo_power_sensor(power, "cameravcm-vcc");
		camera_power_core_ldo(power);
		udelay(200);
		k3_ispldo_power_sensor(power, "pri-cameralog-vcc");
		k3_ispldo_power_sensor(power, "camera-vcc");
	}
	return ret;
}

static int sonyimx134_get_sensor_aperture()
{
	return SONYIMX134_APERTURE_FACTOR;
}

static int sonyimx134_get_equivalent_focus()
{
	return SONYIMX134_EQUIVALENT_FOCUS;
}


/*
 * Here gain is in unit 1/16 of sensor gain,
 * y36721 todo, temporarily if sensor gain=0x10, ISO is 100
 * in fact we need calibrate an ISO-ET-gain table.
 */
u32 sonyimx134_gain_to_iso(int gain)
{
	return (gain * 100) / 0x10;
}

u32 sonyimx134_iso_to_gain(int iso)
{
	return (iso * 0x10) / 100;
}

void sonyimx134_set_gain(u32 gain)
{
	if (gain == 0)
		return;
	gain = 256 - (256 * 16) / gain;
	//sonyimx134_write_reg(SONYIMX134_GAIN_REG_H, (gain >> 8) & 0xff, 0x00);
	sonyimx134_write_reg(SONYIMX134_GAIN_REG_L, gain & 0xff, 0x00);
}

void sonyimx134_set_exposure(u32 exposure)
{
	exposure >>= 4;
	sonyimx134_write_reg(SONYIMX134_EXPOSURE_REG_H, (exposure >> 8) & 0xff, 0x00);
	sonyimx134_write_reg(SONYIMX134_EXPOSURE_REG_L, exposure & 0xff, 0x00);
}

void sonyimx134_set_vts(u16 vts)
{
	print_debug("Enter %s  ", __func__);
	sonyimx134_write_reg(SONYIMX134_VTS_REG_H, (vts >> 8) & 0xff, 0x00);
	sonyimx134_write_reg(SONYIMX134_VTS_REG_L, vts & 0xff, 0x00);
}

u32 sonyimx134_get_vts_reg_addr(void)
{
	return SONYIMX134_VTS_REG_H;
}


static u32 sonyimx134_get_override_param(camera_override_type_t type)
{
	u32 ret_val = sensor_override_params[type];

	switch (type) {
	case OVERRIDE_ISO_HIGH:
		ret_val = SONYIMX134_MAX_ISO;
		break;

	case OVERRIDE_ISO_LOW:
		ret_val = SONYIMX134_MIN_ISO;
		break;

	case OVERRIDE_AUTO_FPS_GAIN_HIGH:
		ret_val = SONYIMX134_AUTO_FPS_MAX_GAIN;
		break;

	case OVERRIDE_AUTO_FPS_GAIN_LOW:
		ret_val = SONYIMX134_AUTO_FPS_MIN_GAIN;
		break;

	case OVERRIDE_FPS_MAX:
		ret_val = SONYIMX134_MAX_FRAMERATE;
		break;

	case OVERRIDE_FPS_MIN:
		ret_val = SONYIMX134_MIN_FRAMERATE;
		break;

	case OVERRIDE_CAP_FPS_MIN:
		ret_val = SONYIMX134_MIN_CAP_FRAMERATE;
		break;

	case OVERRIDE_FLASH_TRIGGER_GAIN:
		ret_val = SONYIMX134_FLASH_TRIGGER_GAIN;
		break;

	case OVERRIDE_SHARPNESS_PREVIEW:
		ret_val = SONYIMX134_SHARPNESS_PREVIEW;
		break;

	case OVERRIDE_SHARPNESS_CAPTURE:
		ret_val = SONYIMX134_SHARPNESS_CAPTURE;
		break;

	default:
		print_error("%s:not override or invalid type %d, use default",__func__, type);
	}

	return ret_val;
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_reset;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int sonyimx134_reset(camera_power_state power_state)
{
	print_debug("%s  \n", __func__);

	if (POWER_ON == power_state) {
		k3_isp_io_enable_mclk(MCLK_ENABLE, sonyimx134_sensor.sensor_index);
		udelay(100);
		k3_ispgpio_reset_sensor(sonyimx134_sensor.sensor_index, power_state,
			      sonyimx134_sensor.power_conf.reset_valid);
		udelay(500);
	} else {
		k3_ispgpio_reset_sensor(sonyimx134_sensor.sensor_index, power_state,
			      sonyimx134_sensor.power_conf.reset_valid);
		udelay(10);
		k3_isp_io_enable_mclk(MCLK_DISABLE, sonyimx134_sensor.sensor_index);
	}

	return 0;
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_init;
 * Description : sonyimx134 init function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : Error code indicating success or failure;
 * Other       : NA;
 **************************************************************************
*/
static int sonyimx134_init(void)
{
	print_debug("%s  ", __func__);

	if (!camera_timing_is_match(0)) {
		print_error("%s: sensor timing don't match.\n", __func__);
		return -ENODEV;
	}

	if (sonyimx134_sensor.owner && !try_module_get(sonyimx134_sensor.owner)) {
		print_error("%s: try_module_get fail", __func__);
		return -ENOENT;
	}

	k3_ispio_power_init("pri-cameralog-vcc", LDO_VOLTAGE_28V, LDO_VOLTAGE_28V);	/*analog 2.85V */
	k3_ispio_power_init("camera-vcc", LDO_VOLTAGE_18V, LDO_VOLTAGE_18V);	/*IO 1.8V */
	k3_ispio_power_init("cameravcm-vcc", LDO_VOLTAGE_28V, LDO_VOLTAGE_28V);	/*AF 2.85V */
	k3_ispio_power_init("sec-cameralog-vcc", LDO_VOLTAGE_28V, LDO_VOLTAGE_28V);	/*analog 2.85V */

	return 0;
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_exit;
 * Description : sonyimx134 exit function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void sonyimx134_exit(void)
{
	print_debug("enter %s", __func__);

	k3_ispio_power_deinit();

	if (sonyimx134_sensor.owner) {
		module_put(sonyimx134_sensor.owner);
	}
	print_debug("exit %s", __func__);
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_shut_down;
 * Description : sonyimx134 shut down function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void sonyimx134_shut_down(void)
{
	print_debug("enter %s", __func__);
	k3_ispgpio_power_sensor(&sonyimx134_sensor, POWER_OFF);
}

#ifdef IMX134_OTP
int sonyimx134_read_otp(u8 i2c_addr,u16 reg,u8 *buf,u16 count)
{
	u16 i;
	int ret;
	u16 val = 0;
	for(i=0;i<count;i++)
	{
		ret =ispv1_read_sensor_byte_addr8(sonyimx134_sensor.i2c_config.index,i2c_addr,reg+i,&val,sonyimx134_sensor.i2c_config.val_bits);
		if(ret !=0)
			print_error("sonyimx134_read_otp  %d\n",ret);
		buf[i] = (val&0xff);
	}
	return 0;
}

static bool sonyimx134_otp_read_id(void)
{
	u8 buf[5];	
	u8 vendor_id =0;
	u8 module_type = MODULE_UNSUPPORT;
	
	print_debug("enter %s", __func__);

	if((sonyimx134_otp_flag & SONYIMX134_OTP_ID_READ) ==SONYIMX134_OTP_ID_READ)//we think OTP data is not correct at all
	{
		print_debug("%s OTP ID data is read allread!!!\n",__func__);
		return true;
	}

	sonyimx134_read_otp(OTP_ID_AWB,OTP_ID_REG,buf,5);

	print_debug("module info year 20%02d month %d day %d, SNO. 0x%x  vendor id&version 0x%x\n",
		buf[0],buf[1],buf[2],buf[3],buf[4]);

	vendor_id = buf[4]>>4;
	if(vendor_id ==0x01) //sunny
	{
		module_type = MODULE_SUNNY;
	}

	if(vendor_id == 0x03) //Liteon
	{
		module_type = MODULE_LITEON;
	}

	if(sensor_module == module_type)
	{
		sonyimx134_otp_flag |= SONYIMX134_OTP_ID_READ;
	}
	else
	{
		print_error("%s OTP data is worng!!!\n",__func__);
		return false;
	}	
	return true;	
}


static bool sonyimx134_otp_read_awb(void)
{
	u8 buf[6];
	//u16 i;

	print_debug("enter %s", __func__);
	if((sonyimx134_otp_flag & SONYIMX134_OTP_ID_READ) ==0)//we think OTP data is not correct at all
	{
		print_error("%s OTP data is worng!!!\n",__func__);
		return false;
	}

	if((sonyimx134_otp_flag & SONYIMX134_OTP_AWB_READ) ==SONYIMX134_OTP_AWB_READ)//we think OTP data is not correct at all
	{
		print_debug("%s OTP AWB data is read allread!!!\n",__func__);
		return true;
	}

	sonyimx134_read_otp(OTP_ID_AWB,OTP_AWB_REG,buf,6);
	//for(i=0;i<6;i++)
	//{
	//	print_debug("awb otp data[%d]=%x\n",i,buf[i]);
	//}
	sonyimx134_otp_flag |= SONYIMX134_OTP_AWB_READ;
	return true;
}

static bool sonyimx134_otp_read_lsc(void)
{
	print_debug("enter %s", __func__);
	if((sonyimx134_otp_flag & SONYIMX134_OTP_ID_READ) ==0)//we think OTP data is not correct at all
	{
		print_error("%s OTP data is worng!!!\n",__func__);
		return false;
	}

	if((sonyimx134_otp_flag & SONYIMX134_OTP_LSC_READ) ==SONYIMX134_OTP_LSC_READ)//we think OTP data is not correct at all
	{
		print_debug("%s OTP LSC data is read allread!!!\n",__func__);
		return true;
	}

	memset(sonyimx134_otp_lsc_param, 0, SONYIMX134_OTP_LSC_SIZE);
	//LSC 0xa4:0b--0xff  / 0xa6:00--0x22  total = 280
	sonyimx134_read_otp(OTP_LSC_1,OTP_LSC_1_REG,sonyimx134_otp_lsc_param,0xff-0x0b+1);
	sonyimx134_read_otp(OTP_LSC_2,OTP_LSC_2_REG,&sonyimx134_otp_lsc_param[0xff-0x0b+1],0x22+1);

	print_debug("%s LCS[0]= %x,LSC[247] = %x  LSC[248]=%x,LSC[279]=%d\n",__func__,
		sonyimx134_otp_lsc_param[0],sonyimx134_otp_lsc_param[247],sonyimx134_otp_lsc_param[248],sonyimx134_otp_lsc_param[279]);
	sonyimx134_otp_flag |= SONYIMX134_OTP_LSC_READ;
	return true;
}


/****************************************************************************
* FunctionName: sonyimx134_otp_set_lsc;
* Description : Set lens shading parameters to sensor registers.; cost time is 0.0341s on sunny module
* Input       : NA;
* Output      : NA;
* ReturnValue : bool;
* Other       : NA;
***************************************************************************/
static bool sonyimx134_otp_set_lsc(void)
{
	u8 *pval = NULL;
	int i = 0;
	
	print_debug("enter %s\n", __func__);

	/* Lens shading parameters are burned OK. */
	if((sonyimx134_otp_flag & SONYIMX134_OTP_LSC_READ) ==0)
	{
		print_error("%s OTP data is worng!!!\n",__func__);
		return false;
	}
	pval = sonyimx134_otp_lsc_param;
	/* Write lens shading parameters to sensor registers. */
	for (i=0; i<SONYIMX134_OTP_LSC_SIZE; i++)
	{
		sonyimx134_write_reg(SONYIMX134_SENSOR_LSC_RAM+i, *(pval+i), 0x00);
		print_debug("LSC[%d] = %d  \n",i,*(pval+i));
	}
	print_debug("%s, set OTP LSC to sensor OK.\n", __func__);

	return true;
}


/****************************************************************************
* FunctionName: sonyimx134_otp_enable_lsc;
* Description : Enable LSC correct.;
* Input       : bool;
* Output      : NA;
* ReturnValue : NA;
* Other       : NA;
***************************************************************************/
static bool sonyimx134_otp_enable_lsc(bool enable)
{
	u8 lscMode = 0x00;
	u8 selToggle = 0x00;
	u8 lscEnable = 0x00;
	
	print_debug("enter %s", __func__);
	if((sonyimx134_otp_flag & SONYIMX134_OTP_LSC_READ) ==0)
	{
		print_error("%s OTP data is worng!!!\n",__func__);
		return false;
	}
	/* Open OTP lsc mode */
	if (enable) {
		lscMode = 0x01;
		selToggle = 0x01;
		lscEnable = 0x1F;
		print_debug("%s, OTP LSC enabled!", __func__);
	}
	sonyimx134_write_reg(SONYIMX134_SENSOR_LSC_EN, lscEnable, 0x00);
	sonyimx134_write_reg(SONYIMX134_SENSOR_LSC_MODE, lscMode, 0x00);
	sonyimx134_write_reg(SONYIMX134_SENSOR_RAM_SEL, selToggle, 0x00);

	return true;
}




/****************************************************************************
* FunctionName: sonyimx134_otp_read_vcm;
* Description : Get AF motor parameters from OTP.;
* Input       : NA;
* Output      : NA;
* ReturnValue : bool;
* Other       : NA;
***************************************************************************/
static bool sonyimx134_otp_read_vcm(void)
{
	u8 buf[4];
    u16 start_code,end_code;

	print_debug("enter %s", __func__);
	if((sonyimx134_otp_flag & SONYIMX134_OTP_ID_READ) ==0)//we think OTP data is not correct at all
	{
		print_error("%s OTP data is worng!!!\n",__func__);
		return false;
	}

	if((sonyimx134_otp_flag & SONYIMX134_OTP_VCM_READ) ==SONYIMX134_OTP_VCM_READ)//we think OTP data is not correct at all
	{
		print_debug("%s OTP VCM data is read allread!!!\n",__func__);
		return true;
	}

	sonyimx134_read_otp(OTP_VCM,OTP_VCM_REG,buf,4);
	print_debug("raw buf_vcm[0]= %x, buf_vcm[1] = %x buf_vcm[2] = %x buf_vcm[3] = %x\n",buf[0],buf[1],buf[2],buf[3]);

	sonyimx134_otp_flag |= SONYIMX134_OTP_VCM_READ;
	start_code = buf[0];
	start_code <<= 8;
	start_code +=buf[1];
	end_code = buf[2];
	end_code <<= 8;
	end_code += buf[3];

	if((start_code != end_code) &&(end_code>start_code))
	{
		/* VCM param is read  */
		sonyimx134_otp_flag |= SONYIMX134_OTP_VCM_READ;
		sonyimx134_vcm_start = start_code;
		sonyimx134_vcm_end = end_code;	
		print_debug("sonyimx134_vcm_start= %x, sonyimx134_vcm_end = %x \n",sonyimx134_vcm_start,sonyimx134_vcm_end);
		return true;
	}
	else
	{
		print_error("%s VCM OTP data is worng!!!\n",__func__);
		return false;
	}
}
/****************************************************************************
* FunctionName: sonyimx134_otp_get_vcm;
* Description : Get vcm start and stop parameters read from OTP.;
* Input       : NA;
* Output      : vcm_start vcm_end
* ReturnValue : NA;
* Other       : NA;
***************************************************************************/
static void sonyimx134_otp_get_vcm(u16 *vcm_start, u16 *vcm_end)
{	
	_sonyimx134_otp_get_vcm(vcm_start, vcm_end,sonyimx134_sensor.vcm,sonyimx134_vcm_start,sonyimx134_vcm_end);
	print_info("%s, start: %#x, end: %#x", __func__, *vcm_start, *vcm_end);
}
//this function time cost is about 0.0475s on sunny module
static void sonyimx134_get_otp_from_sensor(void) 
{
	sonyimx134_otp_read_id();
	sonyimx134_otp_read_awb();
	sonyimx134_otp_read_vcm();
	sonyimx134_otp_read_lsc();//cost 0.042996s on sunny module
}
#endif
/*
 **************************************************************************
 * FunctionName: sonyimx134_set_default;
 * Description : init sonyimx134_sensor;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void sonyimx134_set_default(void)
{
	sonyimx134_sensor.init = sonyimx134_init;
	sonyimx134_sensor.exit = sonyimx134_exit;
	sonyimx134_sensor.shut_down = sonyimx134_shut_down;
	sonyimx134_sensor.reset = sonyimx134_reset;
	sonyimx134_sensor.check_sensor = sonyimx134_check_sensor;
	sonyimx134_sensor.power = sonyimx134_power;
	sonyimx134_sensor.init_reg = sonyimx134_init_reg;
	sonyimx134_sensor.stream_on = sonyimx134_stream_on;

	sonyimx134_sensor.get_format = sonyimx134_get_format;
	sonyimx134_sensor.set_flash = NULL;
	sonyimx134_sensor.get_flash = NULL;
	sonyimx134_sensor.set_scene = NULL;
	sonyimx134_sensor.get_scene = NULL;

	sonyimx134_sensor.enum_framesizes = sonyimx134_enum_framesizes;
	sonyimx134_sensor.try_framesizes = sonyimx134_try_framesizes;
	sonyimx134_sensor.set_framesizes = sonyimx134_set_framesizes;
	sonyimx134_sensor.get_framesizes = sonyimx134_get_framesizes;

	sonyimx134_sensor.enum_frame_intervals = sonyimx134_enum_frame_intervals;
	sonyimx134_sensor.try_frame_intervals = NULL;
	sonyimx134_sensor.set_frame_intervals = NULL;
	sonyimx134_sensor.get_frame_intervals = NULL;

	sonyimx134_sensor.get_capability = sonyimx134_get_capability;

	sonyimx134_sensor.set_hflip = sonyimx134_set_hflip;
	sonyimx134_sensor.get_hflip = sonyimx134_get_hflip;
	sonyimx134_sensor.set_vflip = sonyimx134_set_vflip;
	sonyimx134_sensor.get_vflip = sonyimx134_get_vflip;
	sonyimx134_sensor.update_flip = sonyimx134_update_flip;

	strcpy(sonyimx134_sensor.info.name,"sonyimx134");
	sonyimx134_sensor.interface_type = MIPI1;
	sonyimx134_sensor.mipi_lane_count = CSI_LINES_4;
	sonyimx134_sensor.mipi_index = CSI_INDEX_0;
	sonyimx134_sensor.sensor_index = CAMERA_SENSOR_PRIMARY;
	sonyimx134_sensor.skip_frames = 1;

	sonyimx134_sensor.power_conf.pd_valid = LOW_VALID;
	sonyimx134_sensor.power_conf.reset_valid = LOW_VALID;
	sonyimx134_sensor.power_conf.vcmpd_valid = LOW_VALID;

	sonyimx134_sensor.i2c_config.index = I2C_PRIMARY;
	sonyimx134_sensor.i2c_config.speed = I2C_SPEED_400;
	sonyimx134_sensor.i2c_config.addr = SONYIMX134_SLAVE_ADDRESS;
	sonyimx134_sensor.i2c_config.addr_bits = 16;
	sonyimx134_sensor.i2c_config.val_bits = I2C_8BIT;

	sonyimx134_sensor.preview_frmsize_index = 0;
	sonyimx134_sensor.capture_frmsize_index = 0;
	sonyimx134_sensor.frmsize_list = sonyimx134_framesizes;
	sonyimx134_sensor.fmt[STATE_PREVIEW] = V4L2_PIX_FMT_RAW10;
	sonyimx134_sensor.fmt[STATE_CAPTURE] = V4L2_PIX_FMT_RAW10;

#ifdef SONYIMX134_AP_WRITEAE_MODE /* just an example and test case for AP write AE mode */
	sonyimx134_sensor.aec_addr[0] = 0;
	sonyimx134_sensor.aec_addr[1] = 0;
	sonyimx134_sensor.aec_addr[2] = 0;
	sonyimx134_sensor.agc_addr[0] = 0;
	sonyimx134_sensor.agc_addr[1] = 0;
	sonyimx134_sensor.ap_writeAE_delay = 1500; /* 5 expo and gain registers, 1500us is enough */
#else
	sonyimx134_sensor.aec_addr[0] = 0x0000;
	sonyimx134_sensor.aec_addr[1] = 0x0202;
	sonyimx134_sensor.aec_addr[2] = 0x0203;
	sonyimx134_sensor.agc_addr[0] = 0x0000; /*0x0204 high byte not needed*/
	sonyimx134_sensor.agc_addr[1] = 0x0205;
#endif
	sonyimx134_sensor.sensor_type = SENSOR_SONY;
	sonyimx134_sensor.sensor_rgb_type = SENSOR_RGGB;/* changed by y00231328. add bayer order*/

	sonyimx134_sensor.set_gain = sonyimx134_set_gain;
	sonyimx134_sensor.set_exposure = sonyimx134_set_exposure;

	sonyimx134_sensor.set_vts = sonyimx134_set_vts;
	sonyimx134_sensor.get_vts_reg_addr = sonyimx134_get_vts_reg_addr;

	sonyimx134_sensor.get_override_param = sonyimx134_get_override_param;
	sonyimx134_sensor.get_af_param = sonyimx134_get_af_param;
	sonyimx134_sensor.sensor_gain_to_iso = NULL;
	sonyimx134_sensor.sensor_iso_to_gain = NULL;

	sonyimx134_sensor.get_sensor_aperture = sonyimx134_get_sensor_aperture;
	sonyimx134_sensor.get_equivalent_focus = NULL;

	sonyimx134_sensor.set_effect = NULL;

	sonyimx134_sensor.isp_location = CAMERA_USE_K3ISP;
	sonyimx134_sensor.sensor_tune_ops = NULL;

	sonyimx134_sensor.af_enable = 1;
	//sonyimx134_sensor.vcm = &vcm_dw9714;

	sonyimx134_sensor.image_setting.lensc_param = sonyimx134_lensc_param;
	sonyimx134_sensor.image_setting.ccm_param = sonyimx134_ccm_param;
	sonyimx134_sensor.image_setting.awb_param = sonyimx134_awb_param;

	sonyimx134_sensor.fps_max = 30;
	sonyimx134_sensor.fps_min = 16;
	sonyimx134_sensor.fps = 25;

	sonyimx134_sensor.owner = THIS_MODULE;

	sonyimx134_sensor.info.facing = CAMERA_FACING_BACK;
	sonyimx134_sensor.info.orientation = 270;
	sonyimx134_sensor.info.focal_length = 296;	/* 2.96mm */
	sonyimx134_sensor.info.h_view_angle = 75;	/*  66.1 degree */
	sonyimx134_sensor.info.v_view_angle = 75;
	sonyimx134_sensor.lane_clk = CLK_400M;
	sonyimx134_sensor.hflip = 0;
	sonyimx134_sensor.vflip = 0;
	sonyimx134_sensor.old_flip = 0;

}

/*
 **************************************************************************
 * FunctionName: sonyimx134_module_init;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static __init int sonyimx134_module_init(void)
{
	sonyimx134_set_default();
	return register_camera_sensor(sonyimx134_sensor.sensor_index, &sonyimx134_sensor);
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_module_exit;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void __exit sonyimx134_module_exit(void)
{
	unregister_camera_sensor(sonyimx134_sensor.sensor_index, &sonyimx134_sensor);
}

MODULE_AUTHOR("Hisilicon");
module_init(sonyimx134_module_init);
module_exit(sonyimx134_module_exit);
MODULE_LICENSE("GPL");

/********************************** END **********************************************/
