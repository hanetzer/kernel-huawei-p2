/*
 *  s5k3h2yx_foxconn camera driver source file
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
#include <mach/gpio.h>
#include "../isp/sensor_common.h"
#include "s5k3h2yx_foxconn.h"
/*#include "../isp/k3_isp_io.h"*/
#include <asm/bug.h>
#include <linux/device.h>

#define LOG_TAG "S5K3H2YX_FOXCONN"
/* #define DEBUG_DEBUG 1 */
#include "../isp/cam_log.h"

#define S5K3H2YX_FOXCONN_SLAVE_ADDRESS 0x6e
#define S5K3H2YX_FOXCONN_CHIP_ID       (0x2d28)

#define S5K3H2YX_FOXCONN_CAM_MODULE_SKIPFRAME     4

#define S5K3H2YX_FOXCONN_HFLIP    0
#define S5K3H2YX_FOXCONN_VFLIP    0

#define S5K3H2YX_FOXCONN_NO_FLIP	0x00
#define S5K3H2YX_FOXCONN_H_FLIP	0x01
#define S5K3H2YX_FOXCONN_V_FLIP	0x02
#define S5K3H2YX_FOXCONN_HV_FLIP	0x03

#define S5K3H2YX_FOXCONN_EXPOSURE_REG_H	0x0202
#define S5K3H2YX_FOXCONN_EXPOSURE_REG_L	0x0203
#define S5K3H2YX_FOXCONN_GAIN_REG_H		0x0204
#define S5K3H2YX_FOXCONN_GAIN_REG_L		0x0205

#define S5K3H2YX_FOXCONN_VTS_REG_H		0x0340
#define S5K3H2YX_FOXCONN_VTS_REG_L		0x0341

const struct isp_reg_t isp_init_regs_s5k3h2yx_foxconn[] = {
/* BLC */
	{0x1c58b, 0xff}, //avoid false contour Richard@0323
	{0x1c58c, 0xff}, //avoid false contour Richard@0323

/* AEC */
	{0x1c14a, 0x03},
	{0x1c14b, 0x0a},
	{0x1c14c, 0x0a}, //aec fast step//
	{0x1c14e, 0x08}, //slow step//08
	{0x1c140, 0x01}, //banding
	{0x1c13e, 0x02}, //real gain mode for OV8830

	{0x66401, 0x00}, //window weight
	{0x66402, 0x20}, //StatWin_Left
	{0x66403, 0x00},
	{0x66404, 0x20}, //StatWin_Top
	{0x66405, 0x00},
	{0x66406, 0x20}, //StatWin_Right
	{0x66407, 0x00},
	{0x66408, 0x28}, //StatWin_Bottom
	{0x66409, 0x00}, //definiton ofthe center 3x3 window
	{0x6640a, 0xc8}, //nWin_Left
	{0x6640d, 0x00},
	{0x6640e, 0x96}, //nWin_Top
	{0x66411, 0x04},
	{0x66412, 0xb0}, //nWin_Width
	{0x66415, 0x03},
	{0x66416, 0x84}, //nWin_Height
	{0x6642e, 0x01}, //nWin_Weight_0 weight pass
	{0x6642f, 0x01}, //nWin_Weight_1
	{0x66430, 0x01}, //nWin_Weight_2
	{0x66431, 0x01}, //nWin_Weight_3
	{0x66432, 0x02}, //nWin_Weight_4
	{0x66433, 0x02}, //nWin_Weight_5
	{0x66434, 0x02}, //nWin_Weight_6
	{0x66435, 0x02}, //nWin_Weight_7
	{0x66436, 0x04}, //nWin_Weight_8
	{0x66437, 0x02}, //nWin_Weight_9
	{0x66438, 0x02}, //nWin_Weight_10
	{0x66439, 0x02}, //nWin_Weight_11
	{0x6643a, 0x02}, //nWin_Weight_12
	{0x6644e, 0x03}, //nWin_Weight_Shift
	{0x6644f, 0x04}, //black level
	{0x66450, 0xf8}, //saturate level
	{0x6645b, 0x1a}, //black weight1
	{0x6645d, 0x1a}, //black weight2
	{0x66460, 0x04}, //saturate per1
	{0x66464, 0x0a}, //saturate per2
	{0x66467, 0x14}, //saturate weight1
	{0x66469, 0x14}, //saturate weight2
	//auto AE control
	{0x1c590, 0x00},
	{0x1c591, 0x00}, //turn on
	{0x1c592, 0x50}, //high ratio
	{0x1c593, 0x20}, //low ratio
	{0x1c594, 0x01}, //high weight
	{0x1c595, 0x05},
	{0x1c596, 0x09},
	{0x1c597, 0x01}, //low weight
	{0x1c598, 0x02},
	{0x1c599, 0x04},

/* Raw Stretch */
	{0x65020, 0x01}, //RAW Stretch Target
	{0x66500, 0x28},
	{0x66501, 0x00},
	{0x66502, 0xff},
	{0x66503, 0x0f},
	{0x1c1b0, 0xff},
	{0x1c1b1, 0xff},
	{0x1c1b2, 0x01},
	{0x65905, 0x08},
	{0x66301, 0x02}, //high level step
	{0x66302, 0xd0}, //ref bin
	{0x66303, 0x0a}, //PsPer0
	{0x66304, 0x10}, //PsPer1
	{0x1c5a4, 0x01}, //use new high stretch
	{0x1c5a5, 0x20}, //stretch low step
	{0x1c5a6, 0x20}, //stretch high step
	{0x1c5a7, 0x08}, //stretch slow range
	{0x1c5a8, 0x02}, //stretch slow step
	{0x1c1b8, 0x10}, //ratio scale


	{0x1c5a2, 0x04}, //target stable range
	{0x1c5a3, 0x06}, //stretch target slow range

/* De-noise */
	{0x65604, 0x00}, //Richard for new curve 0314
	{0x65605, 0x00}, //Richard for new curve 0314
	{0x65606, 0x00}, //Richard for new curve 0314
	{0x65607, 0x00}, //Richard for new curve 0314

	{0x65510, 0x0f}, //G dns slope change from 0x4 to 0xf Richard 0320
	{0x6551a, 0x02}, //Raw G Dns, Richard 0320
	{0x6551b, 0x03}, //Richard for new curve 0320
	{0x6551c, 0x05}, //Richard for new curve 0320
	{0x6551d, 0x07}, //Richard for new curve 0320
	{0x6551e, 0x09}, //Richard for new curve 0320
	{0x6551f, 0x0b}, //Richard for new curve 0314
	{0x65520, 0x0f}, //Richard for new curve 0314
	{0x65522, 0x00}, //RAW BR De-noise
	{0x65523, 0x06},
	{0x65524, 0x00},
	{0x65525, 0x0c},
	{0x65526, 0x00},
	{0x65527, 0x12},
	{0x65528, 0x00},
	{0x65529, 0x24},
	{0x6552a, 0x00},
	{0x6552b, 0xf0},
	{0x6552c, 0x00},
	{0x6552d, 0xf0},
	{0x6552e, 0x00},
	{0x6552f, 0xf0},

	{0x65c00, 0x03}, //UV De-noise
	{0x65c01, 0x05},
	{0x65c02, 0x08},
	{0x65c03, 0x1f},
	{0x65c04, 0x1f},
	{0x65c05, 0x1f},

/* sharpeness */
	{0x65600, 0x00},
	{0x65601, 0x10}, //0319
	{0x65602, 0x00},
	{0x65603, 0x40}, //0319
	{0x65608, 0x06},
	{0x65609, 0x20},
	{0x6560c, 0x00},
	{0x6560d, 0x08}, //0319
	{0x6560e, 0x10}, //MinSharpenTp
	{0x6560f, 0x60}, //MaxSharpenTp
	{0x65610, 0x10}, //MinSharpenTm
	{0x65611, 0x60}, //MaxSharpenTm
	{0x65613, 0x1f}, //SharpenAlpha
	{0x65615, 0x08}, //HFreq_thre
	{0x65617, 0x06}, //HFreq_coef

/* auto uv saturation */
	{0x1c4e8, 0x01}, //Enable
	{0x1c4e9, 0x40},
	{0x1c4ea, 0x78},
	{0x1c4eb, 0x80}, //keep back for new cmx 0310
	{0x1c4ec, 0x70}, //keep back for new cmx 0310


/* Global Gamma */
	{0x1c49b, 0x01},
	{0x1c49c, 0x02},
	{0x1c49d, 0x01}, //gamma 2.0 0310
	{0x1c49e, 0x02},
	{0x1c49f, 0x01}, //gamma 2.0 0310
	{0x1c4a0, 0x00},
	{0x1c4a1, 0x18},
	{0x1c4a2, 0x00},
	{0x1c4a3, 0x88}, //gamma 2.0 0310 //avoid false contour Richard@0323


/* Tone Mapping */
	//contrast curve 21
	{0x1C4C0, 0x19},
	{0x1C4C1, 0x2c},
	{0x1C4C2, 0x3b},
	{0x1C4C3, 0x47},
	{0x1C4C4, 0x52},
	{0x1C4C5, 0x5C},
	{0x1C4C6, 0x66},
	{0x1C4C7, 0x70},
	{0x1C4C8, 0x7A},
	{0x1C4C9, 0x85},
	{0x1C4CA, 0x91},
	{0x1C4CB, 0xA0},
	{0x1C4CC, 0xB0},
	{0x1C4CD, 0xC5},
	{0x1C4CE, 0xDF},
	{0x1c4d4, 0x20}, //EDR scale
	{0x1c4d5, 0x20}, //EDR scale
	{0x1c4cf, 0x80},
	{0x65a00, 0x1b},
	{0x65a01, 0xc0},

	//dark boost
	{0x1c4b0, 0x02},
	{0x1c4b1, 0x80},

	//curve gain control
	{0x1c1b3, 0x30}, //Gain thre1
	{0x1c1b4, 0x70}, //Gain thre2
	{0x1c1b5, 0x01}, //EDR gain control
	{0x1c1b6, 0x01}, //Curve Gain control
	{0x1c1b7, 0x00}, //after gamma cut ratio

	//Manual UV curve 22
	{0x1C998, 0x00},
	{0x1C999, 0xe2},
	{0x1C99A, 0x01},
	{0x1C99B, 0x11},
	{0x1C99C, 0x01},
	{0x1C99D, 0x2b},
	{0x1C99E, 0x01},
	{0x1C99F, 0x40},
	{0x1C9A0, 0x01},
	{0x1C9A1, 0x40},
	{0x1C9A2, 0x01},
	{0x1C9A3, 0x40},
	{0x1C9A4, 0x01},
	{0x1C9A5, 0x40},
	{0x1C9A6, 0x01},
	{0x1C9A7, 0x40},
	{0x1C9A8, 0x01},
	{0x1C9A9, 0x40},
	{0x1C9AA, 0x01},
	{0x1C9AB, 0x40},
	{0x1C9AC, 0x01},
	{0x1C9AD, 0x40},
	{0x1C9AE, 0x01},
	{0x1C9AF, 0x40},
	{0x1C9B0, 0x01},
	{0x1C9B1, 0x36},
	{0x1C9B2, 0x01},
	{0x1C9B3, 0x18},
	{0x1C9B4, 0x00},
	{0x1C9B5, 0xE6},
	{0x1C9B6, 0x00},
	{0x1C9B7, 0xAA},

/* LENC */
	{0x1c247, 0x00}, //one profile
	{0x1c24c, 0x00},
	{0x1c24d, 0x40},
	{0x1c24e, 0x00},
	{0x1c24f, 0x80},
	{0x1c248, 0x40},
	{0x1c24a, 0x20},
	{0x1c574, 0x00},
	{0x1c575, 0x20},
	{0x1c576, 0x00},
	{0x1c577, 0xf0},
	{0x1c578, 0x40},

	{0x65200, 0x0d},
	{0x65206, 0x3c},
	{0x65207, 0x04},
	{0x65208, 0x3e},
	{0x65209, 0x02},
	{0x6520a, 0x36},
	{0x6520b, 0x0c},
	{0x65214, 0x28},
	{0x65216, 0x20},

	{0x1c264, 0x06},
	{0x1c265, 0x04},
	{0x1c266, 0x04},
	{0x1c267, 0x04},
	{0x1c268, 0x04},
	{0x1c269, 0x05},
	{0x1c26a, 0x03},
	{0x1c26b, 0x02},
	{0x1c26c, 0x01},
	{0x1c26d, 0x01},
	{0x1c26e, 0x02},
	{0x1c26f, 0x03},
	{0x1c270, 0x03},
	{0x1c271, 0x01},
	{0x1c272, 0x00},
	{0x1c273, 0x00},
	{0x1c274, 0x01},
	{0x1c275, 0x03},
	{0x1c276, 0x03},
	{0x1c277, 0x01},
	{0x1c278, 0x00},
	{0x1c279, 0x00},
	{0x1c27a, 0x01},
	{0x1c27b, 0x03},
	{0x1c27c, 0x03},
	{0x1c27d, 0x02},
	{0x1c27e, 0x01},
	{0x1c27f, 0x01},
	{0x1c280, 0x03},
	{0x1c281, 0x04},
	{0x1c282, 0x03},
	{0x1c283, 0x04},
	{0x1c284, 0x04},
	{0x1c285, 0x04},
	{0x1c286, 0x04},
	{0x1c287, 0x03},
	{0x1c288, 0x22},
	{0x1c289, 0x21},
	{0x1c28a, 0x22},
	{0x1c28b, 0x21},
	{0x1c28c, 0x23},
	{0x1c28d, 0x21},
	{0x1c28e, 0x21},
	{0x1c28f, 0x22},
	{0x1c290, 0x22},
	{0x1c291, 0x21},
	{0x1c292, 0x21},
	{0x1c293, 0x21},
	{0x1c294, 0x1f},
	{0x1c295, 0x20},
	{0x1c296, 0x22},
	{0x1c297, 0x1f},
	{0x1c298, 0x20},
	{0x1c299, 0x21},
	{0x1c29a, 0x21},
	{0x1c29b, 0x20},
	{0x1c29c, 0x20},
	{0x1c29d, 0x1d},
	{0x1c29e, 0x1d},
	{0x1c29f, 0x1d},
	{0x1c2a0, 0x22},
	{0x1c2a1, 0x23},
	{0x1c2a2, 0x26},
	{0x1c2a3, 0x27},
	{0x1c2a4, 0x25},
	{0x1c2a5, 0x24},
	{0x1c2a6, 0x24},
	{0x1c2a7, 0x23},
	{0x1c2a8, 0x24},
	{0x1c2a9, 0x24},
	{0x1c2aa, 0x22},
	{0x1c2ab, 0x24},
	{0x1c2ac, 0x21},
	{0x1c2ad, 0x1e},
	{0x1c2ae, 0x21},
	{0x1c2af, 0x24},
	{0x1c2b0, 0x22},
	{0x1c2b1, 0x22},
	{0x1c2b2, 0x22},
	{0x1c2b3, 0x22},
	{0x1c2b4, 0x22},
	{0x1c2b5, 0x25},
	{0x1c2b6, 0x22},
	{0x1c2b7, 0x22},
	{0x1c2b8, 0x22},
	{0x1c2b9, 0x26},


/* AWB */
	{0x66201, 0x52},
	{0x66203, 0x14}, //crop window
	{0x66211, 0xe8}, //awb top limit
	{0x66212, 0x12}, //awb bottom limit

	{0x1c182, 0x08},
	{0x1c183, 0x00}, //MinNum
	{0x1c184, 0x04}, //AWB Step
	{0x1c58d, 0x00}, //LimitAWBAtD65Enable

	{0x1c1be, 0x00}, //AWB offset
	{0x1c1bf, 0x00},
	{0x1c1c0, 0x00},
	{0x1c1c1, 0x00},

	{0x1c1aa, 0x00}, //avgAllEnable
	{0x1c1ad, 0x02}, //weight of A
	{0x1c1ae, 0x10}, //weight of D65
	{0x1c1af, 0x04}, //weight of CWF

	{0x1c5ac, 0x80}, //pre-gain
	{0x1c5ad, 0x80},
	{0x1c5ae, 0x80},

	{0x1ccce, 0x02}, //awb shift
	{0x1cccf, 0x08}, //B gain for A
	{0x1ccd0, 0x20}, //R gain for A
	{0x1c5b8, 0xd8}, //B gain for C outdoor Richard@0517
	{0x1c5b9, 0x58}, //R gain for C outdoor Richard@0517
	{0x1ccd1, 0x08}, //B gain for C indoor Richard@0517
	{0x1ccd2, 0x20}, //R gain for C indoor Richard@0517
	{0x1ccd3, 0x08}, //B gain for D indoor
	{0x1ccd4, 0x20}, //R gain for D indoor
	{0x1cccc, 0xf0}, //B gain for D outdoor
	{0x1cccd, 0x08}, //R gain for D outdoor

	{0x1c5b4, 0x02}, //C indoor/outdoor switch lum 1 Richard@0517
	{0x1c5b5, 0xff}, //C indoor/outdoor switch lum 1 Richard@0517
	{0x1c5b6, 0x04}, //C indoor/outdoor switch lum 2 Richard@0517
	{0x1c5b7, 0xff}, //C indoor/outdoor switch lum 2 Richard@0517

	{0x1ccd5, 0x46}, //CT_A
	{0x1ccd6, 0x76}, //CT_C
	{0x1ccd7, 0xd0}, //CT_D

	{0x1c5cd, 0x01}, //high light awb shift, modified by Jiangtao to avoid blurish when high CT 0310
	{0x1c5ce, 0x00},
	{0x1c5cf, 0x8e},
	{0x1c5d0, 0x00},
	{0x1c5d1, 0xfc},
	{0x1c5d2, 0x03},
	{0x1c5d3, 0x00},
	{0x1c5d4, 0x38}, //Richard 0507
	{0x1c5d5, 0xb0}, //Richard 0507
	{0x1c5d6, 0xa4}, //Richard 0507
	{0x1c5d7, 0xd0}, //Richard 0507
	{0x1c5d8, 0x40}, //?
	{0x1c1c2, 0x00},
	{0x1c1c3, 0x20},


	{0x66206, 0x0c},
	{0x66207, 0x0e},
	{0x66208, 0x0f},
	{0x66209, 0x6f},
	{0x6620a, 0x6a},
	{0x6620b, 0xda},
	{0x6620c, 0xe9},
	{0x6620d, 0x4a},
	{0x6620e, 0x34},
	{0x6620f, 0x63},
	{0x66210, 0x5f},
	{0x66201, 0x52},

	{0x6620f, 0x71}, //D65 split
	{0x66210, 0x4e}, //A split

	{0x1c1c8, 0x01},
	{0x1c1c9, 0x19},
	{0x1c1cc, 0x00},
	{0x1c1cd, 0xa3},
	{0x1c1d0, 0x01},
	{0x1c1d1, 0xd9},
	{0x1c254, 0x00},
	{0x1c255, 0xb7},
	{0x1c256, 0x00},
	{0x1c257, 0xdb},
	{0x1c258, 0x01},
	{0x1c259, 0x41},
	{0x1c25a, 0x01},
	{0x1c25b, 0x82},


/* Color matrix */
	{0x1C1d8, 0x01}, //center matrix, add 10% saturation 0310, remember change uv saturation back
	{0x1C1d9, 0xad},
	{0x1C1da, 0xff},
	{0x1C1db, 0x5d},
	{0x1C1dc, 0xff},
	{0x1C1dd, 0xf6},
	{0x1C1de, 0xff},
	{0x1C1df, 0xa4},
	{0x1C1e0, 0x01},
	{0x1C1e1, 0x8a},
	{0x1C1e2, 0xff},
	{0x1C1e3, 0xd2},
	{0x1C1e4, 0xff},
	{0x1C1e5, 0xc6},
	{0x1C1e6, 0xff},
	{0x1C1e7, 0x8d},
	{0x1C1e8, 0x01},
	{0x1C1e9, 0xad},

	{0x1C1FC, 0x00}, //cmx left delta
	{0x1C1FD, 0x00},
	{0x1C1FE, 0x00},
	{0x1C1FF, 0x00},
	{0x1C200, 0x00},
	{0x1C201, 0x00},
	{0x1C202, 0x00},
	{0x1C203, 0x00},
	{0x1C204, 0x00},
	{0x1C205, 0x00},
	{0x1C206, 0x00},
	{0x1C207, 0x00},
	{0x1C208, 0x00},
	{0x1C209, 0x00},
	{0x1C20A, 0x00},
	{0x1C20B, 0x00},
	{0x1C20C, 0x00},
	{0x1C20D, 0x00},

	{0x1C220, 0x00}, //cmx right delta
	{0x1C221, 0xbd},
	{0x1C222, 0xff},
	{0x1C223, 0x64},
	{0x1C224, 0xff},
	{0x1C225, 0xdf},
	{0x1C226, 0x00},
	{0x1C227, 0x3d},
	{0x1C228, 0xff},
	{0x1C229, 0xe8},
	{0x1C22A, 0xff},
	{0x1C22B, 0xdb},
	{0x1C22C, 0x00},
	{0x1C22D, 0x06},
	{0x1C22E, 0x00},
	{0x1C22F, 0x6f},
	{0x1C230, 0xff},
	{0x1C231, 0x8b},

	/* dpc */
	{0x65409, 0x04},
	{0x6540a, 0x02},
	{0x6540b, 0x01},
	{0x6540c, 0x01},
	{0x6540d, 0x04},
	{0x6540e, 0x02},
	{0x6540f, 0x01},
	{0x65410, 0x01},

};

/*
 * should be calibrated, three lights, from 0x1c264
 * here is long exposure
 */
char s5k3h2yx_foxconn_lensc_param[86*3] = {
};

/* should be calibrated, 6 groups 3x3, from 0x1c1d8 */
short s5k3h2yx_foxconn_ccm_param[54] = {
};

char s5k3h2yx_foxconn_awb_param[] = {
};

static framesize_s s5k3h2yx_foxconn_framesizes[] = {
	/* 1600x1200, just close with quarter size */
	//{1632, 1232, 3283, 2499, {s5k3h2yx_foxconn_framesize_1600x1200, ARRAY_SIZE(s5k3h2yx_foxconn_framesize_1600x1200)}},
	/* full size 10.3 fps */
	{0, 0, 3264, 2448, 0xd8e, 0x9b0, 10, 10, 0, 0, 0x100, VIEW_FULL, RESOLUTION_4_3, false, {s5k3h2yx_foxconn_framesize_full, ARRAY_SIZE(s5k3h2yx_foxconn_framesize_full)} },
};

static camera_sensor s5k3h2yx_foxconn_sensor;
static void s5k3h2yx_foxconn_set_default(void);

/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_read_reg;
 * Description : read s5k3h2yx_foxconn reg by i2c;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int s5k3h2yx_foxconn_read_reg(u16 reg, u8 *val)
{
	return k3_ispio_read_reg(s5k3h2yx_foxconn_sensor.i2c_config.index,
				 s5k3h2yx_foxconn_sensor.i2c_config.addr, reg, (u16 *)val, s5k3h2yx_foxconn_sensor.i2c_config.val_bits);
}

/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_write_reg;
 * Description : write s5k3h2yx_foxconn reg by i2c;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int s5k3h2yx_foxconn_write_reg(u16 reg, u8 val, u8 mask)
{
	return k3_ispio_write_reg(s5k3h2yx_foxconn_sensor.i2c_config.index,
			s5k3h2yx_foxconn_sensor.i2c_config.addr, reg, val, s5k3h2yx_foxconn_sensor.i2c_config.val_bits, mask);
}

/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_write_seq;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int s5k3h2yx_foxconn_write_seq(const struct _sensor_reg_t *seq, u32 size, u8 mask)
{
	print_debug("Enter %s, seq[%#x], size=%d", __func__, (int)seq, size);
	return k3_ispio_write_seq(s5k3h2yx_foxconn_sensor.i2c_config.index,
			s5k3h2yx_foxconn_sensor.i2c_config.addr, seq, size, s5k3h2yx_foxconn_sensor.i2c_config.val_bits, mask);
}

/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_write_isp_seq;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void s5k3h2yx_foxconn_write_isp_seq(const struct isp_reg_t *seq, u32 size)
{
	print_debug("Enter %s, seq[%#x], size=%d", __func__, (int)seq, size);
	k3_ispio_write_isp_seq(seq, size);
}


/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_enum_frame_intervals;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int s5k3h2yx_foxconn_enum_frame_intervals(struct v4l2_frmivalenum *fi)
{
	assert(fi);

	print_debug("enter %s", __func__);
	if (fi->index >= CAMERA_MAX_FRAMERATE) {
		return -EINVAL;
	}

	fi->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fi->discrete.numerator = 1;
	fi->discrete.denominator = (fi->index+1);
	return 0;
}


/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_get_format;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int s5k3h2yx_foxconn_get_format(struct v4l2_fmtdesc *fmt)
{
	if (fmt->type == V4L2_BUF_TYPE_VIDEO_OVERLAY) {
		fmt->pixelformat = s5k3h2yx_foxconn_sensor.fmt[STATE_PREVIEW];
	} else {
		fmt->pixelformat = s5k3h2yx_foxconn_sensor.fmt[STATE_CAPTURE];
	}
	return 0;
}

/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_enum_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int s5k3h2yx_foxconn_enum_framesizes(struct v4l2_frmsizeenum *framesizes)
{
	u32 max_index = ARRAY_SIZE(camera_framesizes) - 1;
	u32 this_max_index = ARRAY_SIZE(s5k3h2yx_foxconn_framesizes) - 1;

	assert(framesizes);

	print_debug("enter %s; ", __func__);

	if (framesizes->index > max_index) {
		print_error("framesizes->index = %d error", framesizes->index);
		return -EINVAL;
	}

	if ((camera_framesizes[framesizes->index].width > s5k3h2yx_foxconn_framesizes[this_max_index].width)
		|| (camera_framesizes[framesizes->index].height > s5k3h2yx_foxconn_framesizes[this_max_index].height)) {
		print_error("framesizes->index = %d error", framesizes->index);
		return -EINVAL;
	}

	framesizes->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	framesizes->discrete.width = s5k3h2yx_foxconn_framesizes[this_max_index].width;
	framesizes->discrete.height = s5k3h2yx_foxconn_framesizes[this_max_index].height;

	return 0;
}

/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_try_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int s5k3h2yx_foxconn_try_framesizes(struct v4l2_frmsizeenum *framesizes)
{
	int max_index = ARRAY_SIZE(s5k3h2yx_foxconn_framesizes) - 1;

	assert(framesizes);

	print_debug("Enter Function:%s  ", __func__);


	if ((framesizes->discrete.width <= s5k3h2yx_foxconn_framesizes[max_index].width)
	    && (framesizes->discrete.height <= s5k3h2yx_foxconn_framesizes[max_index].height)) {
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
 * FunctionName: s5k3h2yx_foxconn_set_framesizes;
 * Description : NA;
 * Input       : flag: if 1, set framesize to sensor,
 *					   if 0, only store framesize to camera_interface;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int s5k3h2yx_foxconn_set_framesizes(camera_state state,
				 struct v4l2_frmsize_discrete *fs, int flag, camera_setting_view_type view_type)
{
	int i = 0;
	bool match = false;

	assert(fs);

	print_debug("Enter Function:%s State(%d), flag=%d, width=%d, height=%d",
		    __func__, state, flag, fs->width, fs->height);

	if (VIEW_FULL == view_type) {
		for (i = 0; i < ARRAY_SIZE(s5k3h2yx_foxconn_framesizes); i++) {
			if ((s5k3h2yx_foxconn_framesizes[i].width >= fs->width)
			    && (s5k3h2yx_foxconn_framesizes[i].height >= fs->height)
			    && (VIEW_FULL == s5k3h2yx_foxconn_framesizes[i].view_type)
			    && (camera_get_resolution_type(fs->width, fs->height)
			    <= s5k3h2yx_foxconn_framesizes[i].resolution_type)) {
				fs->width = s5k3h2yx_foxconn_framesizes[i].width;
				fs->height = s5k3h2yx_foxconn_framesizes[i].height;
				match = true;
				break;
			}
		}
	}

	if (false == match) {
		for (i = 0; i < ARRAY_SIZE(s5k3h2yx_foxconn_framesizes); i++) {
			if ((s5k3h2yx_foxconn_framesizes[i].width >= fs->width)
			    && (s5k3h2yx_foxconn_framesizes[i].height >= fs->height)
			    && (camera_get_resolution_type(fs->width, fs->height)
			    <= s5k3h2yx_foxconn_framesizes[i].resolution_type)) {
				fs->width = s5k3h2yx_foxconn_framesizes[i].width;
				fs->height = s5k3h2yx_foxconn_framesizes[i].height;
				break;
			}
		}
	}

	if (i >= ARRAY_SIZE(s5k3h2yx_foxconn_framesizes)) {
		print_error("request resolution larger than sensor's max resolution");
		return -EINVAL;
	}

	if (state == STATE_PREVIEW) {
		s5k3h2yx_foxconn_sensor.preview_frmsize_index = i;
	} else {
		s5k3h2yx_foxconn_sensor.capture_frmsize_index = i;
	}

	return 0;
}

/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_get_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int s5k3h2yx_foxconn_get_framesizes(camera_state state,
				 struct v4l2_frmsize_discrete *fs)
{
	int frmsize_index;

	assert(fs);

	if (state == STATE_PREVIEW) {
		frmsize_index = s5k3h2yx_foxconn_sensor.preview_frmsize_index;
	} else if (state == STATE_CAPTURE) {
		frmsize_index = s5k3h2yx_foxconn_sensor.capture_frmsize_index;
	} else {
		return -EINVAL;
	}
	fs->width = s5k3h2yx_foxconn_framesizes[frmsize_index].width;
	fs->height = s5k3h2yx_foxconn_framesizes[frmsize_index].height;

	return 0;
}

/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_init_reg;
 * Description : download initial seq for sensor init;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int s5k3h2yx_foxconn_init_reg(void)
{
	int size = 0;
	print_debug("Enter Function:%s  , initsize=%d",
		    __func__, sizeof(s5k3h2yx_foxconn_init_regs));

	size = ARRAY_SIZE(isp_init_regs_s5k3h2yx_foxconn);
	s5k3h2yx_foxconn_write_isp_seq(isp_init_regs_s5k3h2yx_foxconn, size);

	//s5k3h2yx_foxconn_write_reg(0x100, 0x00);
	/* msleep(1); */
#if 1
	if (0 != k3_ispio_init_csi(s5k3h2yx_foxconn_sensor.mipi_index,
				 s5k3h2yx_foxconn_sensor.mipi_lane_count, s5k3h2yx_foxconn_sensor.lane_clk)) {
		return -EFAULT;
	}

	size = ARRAY_SIZE(s5k3h2yx_foxconn_init_regs);
	if (0 != s5k3h2yx_foxconn_write_seq(s5k3h2yx_foxconn_init_regs, size, 0x00)) {
		print_error("fail to init s5k3h2yx_foxconn sensor");
		return -EFAULT;
	}
	//s5k3h2yx_foxconn_write_reg(0x0100, 0x00);
	//s5k3h2yx_foxconn_write_reg(0x3311, 0x00);
	//s5k3h2yx_foxconn_write_reg(0x0100, 0x01);
#else
	if (0 != k3_ispio_init_csi(s5k3h2yx_foxconn_sensor.mipi_index,
				 s5k3h2yx_foxconn_sensor.mipi_lane_count)) {
		return -EFAULT;
	}

	size = ARRAY_SIZE(s5k3h2yx_foxconn_init_regs);
	if (0 != s5k3h2yx_foxconn_write_seq(s5k3h2yx_foxconn_init_regs, size)) {
		print_error("fail to init s5k3h2yx_foxconn sensor");
		return -EFAULT;
	}
#endif

	return 0;
}

/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_framesize_switch;
 * Description : switch frame size, used by preview and capture
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int s5k3h2yx_foxconn_framesize_switch(camera_state state)
{
	u32 size = 0;
	u8 next_frmsize_index;


	if (state == STATE_PREVIEW)
		next_frmsize_index = s5k3h2yx_foxconn_sensor.preview_frmsize_index;
	else
		next_frmsize_index = s5k3h2yx_foxconn_sensor.capture_frmsize_index;

	print_debug("Enter Function:%s frm index=%d", __func__, next_frmsize_index);

	switch (next_frmsize_index) {
	case 0:
		size = ARRAY_SIZE(s5k3h2yx_foxconn_framesize_1600x1200);
		if (0 != s5k3h2yx_foxconn_write_seq(s5k3h2yx_foxconn_framesize_1600x1200, size, 0x00)) {
			print_error("fail to init s5k3h2yx_foxconn sensor");
			return -ETIME;
		}
		break;

	case 1:
		size = ARRAY_SIZE(s5k3h2yx_foxconn_framesize_1080p);
		if (0 != s5k3h2yx_foxconn_write_seq(s5k3h2yx_foxconn_framesize_1080p, size, 0x00)) {
			print_error("fail to init s5k3h2yx_foxconn sensor");
			return -ETIME;
		}
		break;

	case 2:
		size = ARRAY_SIZE(s5k3h2yx_foxconn_framesize_full);
		if (0 != s5k3h2yx_foxconn_write_seq(s5k3h2yx_foxconn_framesize_full, size, 0x00)) {
			print_error("fail to init s5k3h2yx_foxconn sensor");
			return -ETIME;
		}
		break;

	default:
		print_error("fail to init s5k3h2yx_foxconn sensor");
		return -ETIME;
	}


	return 0;
}

/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_stream_on;
 * Description : download preview seq for sensor preview;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int s5k3h2yx_foxconn_stream_on(camera_state state)
{
	print_debug("Enter Function:%s ", __func__);
	return s5k3h2yx_foxconn_framesize_switch(state);
}


/*  **************************************************************************
* FunctionName: s5k3h2yx_foxconn_check_sensor;
* Description : NA;
* Input       : NA;
* Output      : NA;
* ReturnValue : NA;
* Other       : NA;
***************************************************************************/
static int s5k3h2yx_foxconn_check_sensor(void)
{
	u8 idl = 0;
	u8 idh = 0;
	u16 id = 0;

	msleep(1);
	s5k3h2yx_foxconn_read_reg(0x300A, &idh);
	s5k3h2yx_foxconn_read_reg(0x300B, &idl);

	id = ((idh << 8) | idl);
	print_info("s5k3h2yx_foxconn product id:0x%x", id);
	if (S5K3H2YX_FOXCONN_CHIP_ID != id) {
		print_error("Invalid product id ,Could not load sensor s5k3h2yx_foxconn");
		return -ENODEV;
	}

	/*
	 * check s5k3h2 module vendor.
	 * Samsung Electro-Mechanics: MCAMIF_ID = DOVDD
	 * Foxconn: MCAMIF_ID = DGND
	 * Liteon:    MCAMIF_ID = DGND
	 */
	if(gpio_get_value(GPIO_13_7) > 0){
		return -ENODEV;
	}else{
		print_info("s5k3h2yx Liteon/Foxconn module vendor.\n");
	}

	return 0;
}

/****************************************************************************
* FunctionName: s5k3h2yx_foxconn_check_sensor;
* Description : NA;
* Input       : NA;
* Output      : NA;
* ReturnValue : NA;
* Other       : NA;
***************************************************************************/
int s5k3h2yx_foxconn_power(camera_power_state power)
{
	int ret = 0;

	if (power == POWER_ON) {
		k3_ispldo_power_sensor(power,"pri-cameralog-vcc");
		ret = camera_power_core_ldo(power);
		udelay(200);
		k3_ispldo_power_sensor(power,"camera-vcc");
		k3_ispldo_power_sensor(power,"cameravcm-vcc");
		udelay(1);
		k3_ispldo_power_sensor(power,"sec-cameralog-vcc");

		k3_ispgpio_power_sensor(&s5k3h2yx_foxconn_sensor, power);
		k3_ispio_ioconfig(&s5k3h2yx_foxconn_sensor, power);
	} else {
		k3_ispio_deinit_csi(s5k3h2yx_foxconn_sensor.mipi_index);
		k3_ispio_ioconfig(&s5k3h2yx_foxconn_sensor, power);
		k3_ispgpio_power_sensor(&s5k3h2yx_foxconn_sensor, power);

		k3_ispldo_power_sensor(power,"sec-cameralog-vcc");
		k3_ispldo_power_sensor(power,"cameravcm-vcc");
		camera_power_core_ldo(power);
		udelay(200);
		k3_ispldo_power_sensor(power,"pri-cameralog-vcc");
		k3_ispldo_power_sensor(power,"camera-vcc");
	}
	return ret;
}
/*
 * Here gain is in unit 1/16 of sensor gain,
 * y36721 todo, temporarily if sensor gain=0x10, ISO is 100
 * in fact we need calibrate an ISO-ET-gain table.
 */
u32 s5k3h2yx_foxconn_gain_to_iso(int gain)
{
	return (gain * 100) / 0x10;
}

u32 s5k3h2yx_foxconn_iso_to_gain(int iso)
{
	return (iso * 0x10) / 100;
}

void s5k3h2yx_foxconn_set_gain(u32 gain)
{
	gain = gain << 1;
	s5k3h2yx_foxconn_write_reg(S5K3H2YX_FOXCONN_GAIN_REG_H, (gain >> 8) & 0xff, 0x00);
	s5k3h2yx_foxconn_write_reg(S5K3H2YX_FOXCONN_GAIN_REG_L, gain & 0xff, 0x00);
}

void s5k3h2yx_foxconn_set_exposure(u32 exposure)
{
	exposure = exposure >> 4;
	s5k3h2yx_foxconn_write_reg(S5K3H2YX_FOXCONN_EXPOSURE_REG_H, (exposure >> 8) & 0xff, 0x00);
	s5k3h2yx_foxconn_write_reg(S5K3H2YX_FOXCONN_EXPOSURE_REG_L, exposure & 0xff, 0x00);
}

void s5k3h2yx_foxconn_set_vts(u16 vts)
{
	print_debug("Enter %s  ", __func__);
	s5k3h2yx_foxconn_write_reg(S5K3H2YX_FOXCONN_VTS_REG_H, (vts >> 8) & 0xff, 0x00);
	s5k3h2yx_foxconn_write_reg(S5K3H2YX_FOXCONN_VTS_REG_L, vts & 0xff, 0x00);
}

u32 s5k3h2yx_foxconn_get_vts_reg_addr(void)
{
	return S5K3H2YX_FOXCONN_VTS_REG_H;
}

/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_reset;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int s5k3h2yx_foxconn_reset( camera_power_state power_state)
{
	print_debug("%s  ", __func__);

	if (POWER_ON == power_state) {
		k3_isp_io_enable_mclk(MCLK_ENABLE, s5k3h2yx_foxconn_sensor.sensor_index);
		k3_ispgpio_reset_sensor(s5k3h2yx_foxconn_sensor.sensor_index, power_state,
			      s5k3h2yx_foxconn_sensor.power_conf.reset_valid);
		udelay(500);
	} else {
		k3_ispgpio_reset_sensor(s5k3h2yx_foxconn_sensor.sensor_index, power_state,
			      s5k3h2yx_foxconn_sensor.power_conf.reset_valid);
		udelay(10);
		k3_isp_io_enable_mclk(MCLK_DISABLE, s5k3h2yx_foxconn_sensor.sensor_index);
	}

	return 0;
}


/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_init;
 * Description : s5k3h2yx_foxconn init function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : Error code indicating success or failure;
 * Other       : NA;
 **************************************************************************
*/
static int s5k3h2yx_foxconn_init(void)
{
	print_debug("%s  ", __func__);

	if (!camera_timing_is_match(0)){
		print_error("%s: sensor timing don't match.\n", __func__);
		return -ENODEV;
	}

	if (s5k3h2yx_foxconn_sensor.owner && !try_module_get(s5k3h2yx_foxconn_sensor.owner)) {
		print_error("%s: try_module_get fail", __func__);
		return -ENOENT;
	}

	k3_ispio_power_init("pri-cameralog-vcc", LDO_VOLTAGE_28V, LDO_VOLTAGE_28V);	/*analog 2.85V*/
	k3_ispio_power_init("camera-vcc", LDO_VOLTAGE_18V, LDO_VOLTAGE_18V);	/*IO 1.8V*/
	k3_ispio_power_init("cameravcm-vcc", LDO_VOLTAGE_28V, LDO_VOLTAGE_28V);	/*AF 2.85V*/
	k3_ispio_power_init("sec-cameralog-vcc", LDO_VOLTAGE_28V, LDO_VOLTAGE_28V);	/*analog 2.85V*/

	return 0;
}

/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_exit;
 * Description : s5k3h2yx_foxconn exit function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void s5k3h2yx_foxconn_exit(void)
{
	print_debug("enter %s", __func__);

	k3_ispio_power_deinit();

	if (s5k3h2yx_foxconn_sensor.owner) {
		module_put(s5k3h2yx_foxconn_sensor.owner);
	}
	print_debug("exit %s", __func__);
}

/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_shut_down;
 * Description : s5k3h2yx_foxconn shut down function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void s5k3h2yx_foxconn_shut_down(void)
{
	print_debug("enter %s", __func__);
	k3_ispgpio_power_sensor(&s5k3h2yx_foxconn_sensor, POWER_OFF);
}

/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_set_default;
 * Description : init s5k3h2yx_foxconn_sensor;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void s5k3h2yx_foxconn_set_default(void)
{
	s5k3h2yx_foxconn_sensor.init = s5k3h2yx_foxconn_init;
	s5k3h2yx_foxconn_sensor.exit = s5k3h2yx_foxconn_exit;
	s5k3h2yx_foxconn_sensor.shut_down = s5k3h2yx_foxconn_shut_down;
	s5k3h2yx_foxconn_sensor.reset = s5k3h2yx_foxconn_reset;
	s5k3h2yx_foxconn_sensor.check_sensor = s5k3h2yx_foxconn_check_sensor;
	s5k3h2yx_foxconn_sensor.power = s5k3h2yx_foxconn_power;
	s5k3h2yx_foxconn_sensor.init_reg = s5k3h2yx_foxconn_init_reg;
	s5k3h2yx_foxconn_sensor.stream_on = s5k3h2yx_foxconn_stream_on;

	s5k3h2yx_foxconn_sensor.get_format = s5k3h2yx_foxconn_get_format;
	s5k3h2yx_foxconn_sensor.set_flash = NULL;
	s5k3h2yx_foxconn_sensor.get_flash = NULL;
	s5k3h2yx_foxconn_sensor.set_scene = NULL;
	s5k3h2yx_foxconn_sensor.get_scene = NULL;

	s5k3h2yx_foxconn_sensor.enum_framesizes = s5k3h2yx_foxconn_enum_framesizes;
	s5k3h2yx_foxconn_sensor.try_framesizes = s5k3h2yx_foxconn_try_framesizes;
	s5k3h2yx_foxconn_sensor.set_framesizes = s5k3h2yx_foxconn_set_framesizes;
	s5k3h2yx_foxconn_sensor.get_framesizes = s5k3h2yx_foxconn_get_framesizes;

	s5k3h2yx_foxconn_sensor.enum_frame_intervals = s5k3h2yx_foxconn_enum_frame_intervals;
	s5k3h2yx_foxconn_sensor.try_frame_intervals = NULL;
	s5k3h2yx_foxconn_sensor.set_frame_intervals = NULL;
	s5k3h2yx_foxconn_sensor.get_frame_intervals = NULL;

	s5k3h2yx_foxconn_sensor.get_capability = NULL;

	strcpy(s5k3h2yx_foxconn_sensor.info.name,"s5k3h2yx_foxconn");
	s5k3h2yx_foxconn_sensor.interface_type = MIPI1;
	s5k3h2yx_foxconn_sensor.mipi_lane_count = CSI_LINES_2;
	s5k3h2yx_foxconn_sensor.mipi_index = CSI_INDEX_0;
	s5k3h2yx_foxconn_sensor.sensor_index = CAMERA_SENSOR_PRIMARY;
	s5k3h2yx_foxconn_sensor.skip_frames = 0; /* y36721 todo*/

	s5k3h2yx_foxconn_sensor.power_conf.pd_valid = LOW_VALID;
	s5k3h2yx_foxconn_sensor.power_conf.reset_valid = LOW_VALID;
	s5k3h2yx_foxconn_sensor.power_conf.vcmpd_valid = LOW_VALID;

	s5k3h2yx_foxconn_sensor.i2c_config.index = I2C_PRIMARY;
	s5k3h2yx_foxconn_sensor.i2c_config.speed = I2C_SPEED_100;
	s5k3h2yx_foxconn_sensor.i2c_config.addr = S5K3H2YX_FOXCONN_SLAVE_ADDRESS;
	s5k3h2yx_foxconn_sensor.i2c_config.addr_bits = 16;
	s5k3h2yx_foxconn_sensor.i2c_config.val_bits = I2C_8BIT;

	s5k3h2yx_foxconn_sensor.preview_frmsize_index = 0;
	s5k3h2yx_foxconn_sensor.capture_frmsize_index = 0;
	s5k3h2yx_foxconn_sensor.frmsize_list = s5k3h2yx_foxconn_framesizes;
	s5k3h2yx_foxconn_sensor.fmt[STATE_PREVIEW] = V4L2_PIX_FMT_RAW10;
	s5k3h2yx_foxconn_sensor.fmt[STATE_CAPTURE] = V4L2_PIX_FMT_RAW10;

	s5k3h2yx_foxconn_sensor.aec_addr[0] = 0x0000;
	s5k3h2yx_foxconn_sensor.aec_addr[1] = 0x0202;
	s5k3h2yx_foxconn_sensor.aec_addr[2] = 0x0203;
	s5k3h2yx_foxconn_sensor.agc_addr[0] = 0x0204;
	s5k3h2yx_foxconn_sensor.agc_addr[1] = 0x0205;
	s5k3h2yx_foxconn_sensor.sensor_type = SENSOR_SAMSUNG;
	s5k3h2yx_foxconn_sensor.sensor_rgb_type = SENSOR_GRBG;/* changed by y00231328. add bayer order*/

	s5k3h2yx_foxconn_sensor.set_gain = s5k3h2yx_foxconn_set_gain;
	s5k3h2yx_foxconn_sensor.set_exposure = s5k3h2yx_foxconn_set_exposure;

	s5k3h2yx_foxconn_sensor.set_vts = s5k3h2yx_foxconn_set_vts;
	s5k3h2yx_foxconn_sensor.get_vts_reg_addr = s5k3h2yx_foxconn_get_vts_reg_addr;

	s5k3h2yx_foxconn_sensor.sensor_gain_to_iso = s5k3h2yx_foxconn_gain_to_iso;
	s5k3h2yx_foxconn_sensor.sensor_iso_to_gain = s5k3h2yx_foxconn_iso_to_gain;

	s5k3h2yx_foxconn_sensor.get_sensor_aperture = NULL;
	s5k3h2yx_foxconn_sensor.get_equivalent_focus = NULL;

	s5k3h2yx_foxconn_sensor.isp_location = CAMERA_USE_K3ISP;
	s5k3h2yx_foxconn_sensor.sensor_tune_ops = NULL;

	s5k3h2yx_foxconn_sensor.af_enable = 1;
	s5k3h2yx_foxconn_sensor.vcm = &vcm_ad5823;

	s5k3h2yx_foxconn_sensor.image_setting.lensc_param = s5k3h2yx_foxconn_lensc_param;
	s5k3h2yx_foxconn_sensor.image_setting.ccm_param = s5k3h2yx_foxconn_ccm_param;
	s5k3h2yx_foxconn_sensor.image_setting.awb_param = s5k3h2yx_foxconn_awb_param;

	s5k3h2yx_foxconn_sensor.set_effect = NULL;

	s5k3h2yx_foxconn_sensor.fps_max = 30;
	s5k3h2yx_foxconn_sensor.fps_min = 16;
	s5k3h2yx_foxconn_sensor.fps = 25;

	s5k3h2yx_foxconn_sensor.owner = THIS_MODULE;

	s5k3h2yx_foxconn_sensor.info.facing = CAMERA_FACING_BACK;
	s5k3h2yx_foxconn_sensor.info.orientation = 270;
	s5k3h2yx_foxconn_sensor.info.focal_length = 439; /* 4.39mm*/
	s5k3h2yx_foxconn_sensor.info.h_view_angle = 66; /*  66.1 degree */
	s5k3h2yx_foxconn_sensor.info.v_view_angle = 50;
	s5k3h2yx_foxconn_sensor.lane_clk = 0x14;
}

/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_module_init;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static __init int s5k3h2yx_foxconn_module_init(void)
{
	s5k3h2yx_foxconn_set_default();
	return register_camera_sensor(s5k3h2yx_foxconn_sensor.sensor_index, &s5k3h2yx_foxconn_sensor);
}

/*
 **************************************************************************
 * FunctionName: s5k3h2yx_foxconn_module_exit;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void __exit s5k3h2yx_foxconn_module_exit(void)
{
	unregister_camera_sensor(s5k3h2yx_foxconn_sensor.sensor_index, &s5k3h2yx_foxconn_sensor);
}

MODULE_AUTHOR("Hisilicon");
module_init(s5k3h2yx_foxconn_module_init);
module_exit(s5k3h2yx_foxconn_module_exit);
MODULE_LICENSE("GPL");

/********************************** END **********************************************/
