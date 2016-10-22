/* Copyright (c) 2008-2011, Hisilicon Tech. Co., Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *	 * Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *	 * Redistributions in binary form must reproduce the above
 *	   copyright notice, this list of conditions and the following
 *	   disclaimer in the documentation and/or other materials provided
 *	   with the distribution.
 *	 * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *	   contributors may be used to endorse or promote products derived
 *	   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/pwm.h>
#include <mach/platform.h>
#include <mach/gpio.h>

#include "k3_fb.h"
#include "k3_fb_def.h"
#include "mipi_dsi.h"


#define PWM_LEVEL 100


/*----------------Power ON Sequence(power on to Normal mode)----------------------*/

static char powerOnData1[] = {
    0xF0,
    0x5A, 0x5A,
};

static char powerOnData2[] = {
    0xF1,
    0x5A, 0x5A,
};

static char powerOnData3[] = {
    0xFC,
    0x5A, 0x5A,
};

static char powerOnData4[] = {
    0xB1,
    0x01, 0x00, 0x16,
};

static char powerOnData5[] = {
    0xB2,
    0x06, 0x06, 0x06,
};

/* 300 */
static char back_light[] = {
    0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb7, 0xef, 0xd1, 0xca,
    0xd1, 0xd8, 0xda, 0xd8, 0xb5, 0xb8, 0xb0, 0xc5, 0xc8, 0xbf,
    0x00, 0xb9, 0x00, 0x93, 0x00, 0xd9
};

#if 0
static char back_light_700[] = {
    0xfa, 0x02, 0x10, 0x10, 0x10, 0xd1, 0x34, 0xd0, 0xd6, 0xba,
    0xdc, 0xe0, 0xd9, 0xe2, 0xc2, 0xc0, 0xbf, 0xd4, 0xd5, 0xd0,
    0x00, 0x73, 0x00, 0x59, 0x00, 0x82
};
#endif

static char gamma_setting_update[] = {
    0xfa,
    0x03
};

static char powerOnData6[] = {
    0xF8,
    0x28, 0x28, 0x08, 0x08,
    0x40, 0xb0, 0x50, 0x90,
    0x10, 0x30, 0x10, 0x00,
    0x00,
};

static char powerOnData7[] = {
    0xF6,
    0x00, 0x84, 0x09,
};

static char powerOnData8[] = {
    0xb0,
    0x01,
};

static char powerOnData9[] = {
    0xc0,
    0x00,
};

static char powerOnData10[] = {
    0xb0,
    0x09,
};

static char powerOnData11[] = {
    0xd5,
    0x64,
};

static char powerOnData12[] = {
    0xb0,
    0x0b,
};

static char powerOnData13[] = {
    0xd5,
    0xa4,
};

static char powerOnData14[] = {
    0xb0,
    0x0c,
};

static char powerOnData15[] = {
    0xd5,
    0x7e,
};

static char powerOnData16[] = {
    0xb0,
    0x0d,
};

static char powerOnData17[] = {
    0xd5,
    0x20,
};

static char powerOnData18[] = {
    0xb0,
    0x08,
};
	
static char powerOnData19[] = {
    0xfd,
    0xf8,
};

static char powerOnData20[] = {
    0xb0,
    0x04,
};
	
static char powerOnData21[] = {
    0xf2,
    0x4d,
};

static char exit_sleep[] = {
    0x11,
};

static char te_off[] = {
    0x34,
};

static char memory_window_setting1[] = {
    0x2a,
    0x00, 0x1e, 0x02, 0x39,
};

static char memory_window_setting2[] = {
    0x2b,
    0x00, 0x00, 0x03, 0xbf,
};

static char memory_window_setting3[] = {
    0xc0,
    0x01,
};

static char display_on[] = {
    0x29,
};

/*-------------------Power OFF Sequence(Normalto power off)----------------------*/

/* Display Off */
static char display_off[] = {
    0x28,
};

/* Sleep In */
static char enter_sleep[] = {
    0x10,
};
/* WAIT MIN 120ms For Power Down */


static struct dsi_cmd_desc samsung_command_on_cmds[] = {
	{DTYPE_DCS_LWRITE, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData1), powerOnData1},
	{DTYPE_DCS_LWRITE, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData2), powerOnData2},
	{DTYPE_DCS_LWRITE, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData3), powerOnData3},
	{DTYPE_DCS_LWRITE, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData4), powerOnData4},
	{DTYPE_DCS_LWRITE, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData5), powerOnData5},

	{DTYPE_DCS_LWRITE, 0, 1, WAIT_TYPE_MS,
		sizeof(back_light), back_light},
	{DTYPE_DCS_WRITE1, 0, 1, WAIT_TYPE_MS,
		sizeof(gamma_setting_update), gamma_setting_update},

	{DTYPE_DCS_LWRITE, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData6), powerOnData6},
	{DTYPE_DCS_LWRITE, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData7), powerOnData7},

	{DTYPE_DCS_WRITE1, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData8), powerOnData8},
	{DTYPE_DCS_WRITE1, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData9), powerOnData9},
	{DTYPE_DCS_WRITE1, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData10), powerOnData10},
	{DTYPE_DCS_WRITE1, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData11), powerOnData11},
	{DTYPE_DCS_WRITE1, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData12), powerOnData12},
	{DTYPE_DCS_WRITE1, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData13), powerOnData13},
	{DTYPE_DCS_WRITE1, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData14), powerOnData14},
	{DTYPE_DCS_WRITE1, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData15), powerOnData15},
	{DTYPE_DCS_WRITE1, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData16), powerOnData16},
	{DTYPE_DCS_WRITE1, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData17), powerOnData17},
	{DTYPE_DCS_WRITE1, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData18), powerOnData18},
	{DTYPE_DCS_WRITE1, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData19), powerOnData19},
	{DTYPE_DCS_WRITE1, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData20), powerOnData20},
	{DTYPE_DCS_WRITE1, 0, 1, WAIT_TYPE_MS,
		sizeof(powerOnData21), powerOnData21},

	{DTYPE_DCS_WRITE, 0, 120, WAIT_TYPE_MS,
		sizeof(exit_sleep), exit_sleep},

	{DTYPE_DCS_WRITE, 0, 1, WAIT_TYPE_MS,
		sizeof(te_off), te_off},

	{DTYPE_DCS_LWRITE, 0, 1, WAIT_TYPE_MS,
		sizeof(memory_window_setting1), memory_window_setting1},
	{DTYPE_DCS_LWRITE, 0, 1, WAIT_TYPE_MS,
		sizeof(memory_window_setting2), memory_window_setting2},
	{DTYPE_DCS_WRITE1, 0, 1, WAIT_TYPE_MS,
		sizeof(memory_window_setting3), memory_window_setting3},

	{DTYPE_DCS_WRITE, 0, 1, WAIT_TYPE_MS,
		sizeof(display_on), display_on},
};

static struct dsi_cmd_desc samsung_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 0, 1, WAIT_TYPE_MS,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 0, 120, WAIT_TYPE_MS,
		sizeof(enter_sleep), enter_sleep}
};

static struct k3_fb_panel_data samsung_panel_data;

static int samsung_pwm_on(struct k3_fb_data_type *k3fd)
{
	BUG_ON(k3fd == NULL);

	/* backlight on */
	PWM_IOMUX_SET(&(k3fd->panel_info), NORMAL);
	PWM_GPIO_REQUEST(&(k3fd->panel_info));
	gpio_direction_input(k3fd->panel_info.gpio_pwm1);
	mdelay(1);
	pwm_set_backlight(k3fd->bl_level, &(k3fd->panel_info));

	return 0;
}

static int samsung_pwm_off(struct k3_fb_data_type *k3fd)
{
	BUG_ON(k3fd == NULL);

	/* backlight off */
	pwm_set_backlight(0, &(k3fd->panel_info));
	gpio_direction_output(k3fd->panel_info.gpio_pwm0, 0);
	mdelay(1);
	gpio_direction_input(k3fd->panel_info.gpio_pwm1);
	mdelay(1);
	PWM_GPIO_FREE(&(k3fd->panel_info));
	PWM_IOMUX_SET(&(k3fd->panel_info), LOWPOWER);

	return 0;
}

static void samsung_disp_on(struct k3_fb_data_type *k3fd)
{
	u32 edc_base = 0;
	struct k3_panel_info *pinfo = NULL;

	BUG_ON(k3fd == NULL);
	edc_base = k3fd->edc_base;
	pinfo = &(k3fd->panel_info);

	LCD_VCC_ENABLE(pinfo);
	LCD_IOMUX_SET(pinfo, NORMAL);
	LCD_GPIO_REQUEST(pinfo);
	gpio_direction_input(pinfo->gpio_lcd_id0);
	mdelay(1);
	gpio_direction_input(pinfo->gpio_lcd_id1);
	mdelay(1);
	gpio_direction_output(pinfo->gpio_power, 1);
	mdelay(1);
	gpio_direction_output(pinfo->gpio_reset, 1);
	udelay(50);
	gpio_direction_output(pinfo->gpio_reset, 0);
	udelay(25);
	gpio_direction_output(pinfo->gpio_reset, 1);
	mdelay(1);

	mipi_dsi_cmds_tx(samsung_command_on_cmds,
		ARRAY_SIZE(samsung_command_on_cmds), edc_base);
}

static void samsung_disp_off(struct k3_fb_data_type *k3fd)
{
	u32 edc_base = 0;
	struct k3_panel_info *pinfo = NULL;

	BUG_ON(k3fd == NULL);
	edc_base = k3fd->edc_base;
	pinfo = &(k3fd->panel_info);

	mipi_dsi_cmds_tx(samsung_display_off_cmds,
		ARRAY_SIZE(samsung_display_off_cmds), edc_base);

	gpio_direction_input(pinfo->gpio_lcd_id0);
	mdelay(1);
	gpio_direction_input(pinfo->gpio_lcd_id1);
	mdelay(1);
	gpio_direction_output(pinfo->gpio_reset, 0);
	mdelay(1);
	gpio_direction_output(pinfo->gpio_power, 0);
	mdelay(1);
	LCD_GPIO_FREE(pinfo);
	LCD_IOMUX_SET(pinfo, LOWPOWER);
	LCD_VCC_DISABLE(pinfo);
}

static int mipi_samsung_panel_on(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;

	BUG_ON(pdev == NULL);

	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	if (!k3fd->panel_info.display_on) {
		/* lcd display on */
		samsung_disp_on(k3fd);
		k3fd->panel_info.display_on = true;
		if (k3fd->panel_info.bl_set_type & BL_SET_BY_PWM) {
			/* backlight on */
			samsung_pwm_on(k3fd);
		}
	}
	return 0;
}

static int mipi_samsung_panel_off(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;

	BUG_ON(pdev == NULL);

	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	if (k3fd->panel_info.display_on) {
		k3fd->panel_info.display_on = false;
		if (k3fd->panel_info.bl_set_type & BL_SET_BY_PWM) {
			/* backlight off */
			samsung_pwm_off(k3fd);
		}
		/* lcd display off */
		samsung_disp_off(k3fd);
	}

	return 0;
}

static int mipi_samsung_panel_remove(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;

	BUG_ON(pdev == NULL);
	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	k3fb_logi("index=%d, enter!\n", k3fd->index);

	if (k3fd->panel_info.bl_set_type & BL_SET_BY_PWM) {
		PWM_CLK_PUT(&(k3fd->panel_info));
	}
	LCD_VCC_PUT(&(k3fd->panel_info));

	k3fb_logi("index=%d, exit!\n", k3fd->index);

	return 0;
}

static int mipi_samsung_panel_set_backlight(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;

	BUG_ON(pdev == NULL);

	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	if (k3fd->panel_info.bl_set_type & BL_SET_BY_PWM) {
		return pwm_set_backlight(k3fd->bl_level, &(k3fd->panel_info));
	} else {
		return 0;
	}
}

static int mipi_samsung_panel_set_fastboot(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;

	BUG_ON(pdev == NULL);

	k3fd = (struct k3_fb_data_type *)platform_get_drvdata(pdev);
	BUG_ON(k3fd == NULL);

	LCD_VCC_ENABLE(&(k3fd->panel_info));
	LCD_IOMUX_SET(&(k3fd->panel_info), NORMAL);
	LCD_GPIO_REQUEST(&(k3fd->panel_info));

	if (k3fd->panel_info.bl_set_type & BL_SET_BY_PWM) {
		PWM_IOMUX_SET(&(k3fd->panel_info), NORMAL);
		PWM_GPIO_REQUEST(&(k3fd->panel_info));
	}

	k3fd->panel_info.display_on = true;

	return 0;
}

static struct k3_panel_info samsung_panel_info = {0};
static struct k3_fb_panel_data samsung_panel_data = {
	.panel_info = &samsung_panel_info,
	.on = mipi_samsung_panel_on,
	.off = mipi_samsung_panel_off,
	.remove = mipi_samsung_panel_remove,
	.set_backlight = mipi_samsung_panel_set_backlight,
	.set_fastboot = mipi_samsung_panel_set_fastboot,
};

static int __devinit samsung_probe(struct platform_device *pdev)
{
	struct k3_panel_info *pinfo = NULL;
	struct resource *res = NULL;

	pinfo = samsung_panel_data.panel_info;
	/* init lcd panel info */
	pinfo->display_on = false;
	pinfo->xres = 540;
	pinfo->yres = 960;
	pinfo->width = 55;
	pinfo->height = 98;
	pinfo->type = PANEL_MIPI_CMD;
	pinfo->orientation = LCD_PORTRAIT;
	pinfo->bpp = EDC_OUT_RGB_888;
	pinfo->s3d_frm = EDC_FRM_FMT_2D;
	pinfo->bgr_fmt = EDC_RGB;
	pinfo->bl_set_type = BL_SET_BY_PWM;
	pinfo->bl_max = PWM_LEVEL;
	pinfo->bl_min = 1;

	pinfo->frc_enable = 0;
	pinfo->esd_enable = 0;
	pinfo->sbl_enable = 1;

	pinfo->sbl.bl_max = 0xff;
	pinfo->sbl.cal_a = 0x0f;
	pinfo->sbl.cal_b = 0xd8;
	pinfo->sbl.str_limit = 0x40;

	pinfo->ldi.h_back_porch = 4;
	pinfo->ldi.h_front_porch = 18;
	pinfo->ldi.h_pulse_width = 4;
	pinfo->ldi.v_back_porch = 4;
	pinfo->ldi.v_front_porch = 6;
	pinfo->ldi.v_pulse_width = 2;
	pinfo->ldi.hsync_plr = 1;
	pinfo->ldi.vsync_plr = 1;
	pinfo->ldi.pixelclk_plr = 1;
	pinfo->ldi.data_en_plr = 0;
	pinfo->ldi.disp_mode = LDI_DISP_MODE_NOT_3D_FBF;

	/* Note: must init here */
	pinfo->frame_rate = 60;
	pinfo->clk_rate = LCD_GET_CLK_RATE(pinfo);

	pinfo->mipi.lane_nums = DSI_2_LANES;
	pinfo->mipi.color_mode = DSI_24BITS_3;
	pinfo->mipi.vc = 0;
	pinfo->mipi.dsi_bit_clk = 300;

	/* lcd vcc */
	LCD_VCC_GET(pdev, pinfo);
	LCDIO_SET_VOLTAGE(pinfo, 1800000, 1800000);
	/* lcd iomux */
	LCD_IOMUX_GET(pinfo);
	/* lcd resource */
	LCD_RESOURCE(pdev, pinfo, res);

	if (pinfo->bl_set_type & BL_SET_BY_PWM) {
		/* pwm clock*/
		PWM_CLK_GET(pinfo);
		/* pwm iomux */
		PWM_IOMUX_GET(pinfo);
		/* pwm resource */
		PWM_RESOUTCE(pdev, pinfo, res);
	}

	/* alloc panel device data */
	if (platform_device_add_data(pdev, &samsung_panel_data,
		sizeof(struct k3_fb_panel_data))) {
		k3fb_loge("failed to platform_device_add_data!\n");
		platform_device_put(pdev);
		return -ENOMEM;
	}

	k3_fb_add_device(pdev);

	return 0;
}

static int samsung_remove(struct platform_device *pdev)
{
	struct k3_fb_data_type *k3fd = NULL;

	BUG_ON(pdev == NULL);

	k3fd = platform_get_drvdata(pdev);
	/*BUG_ON(k3fd == NULL);*/
	if (!k3fd) {
		return 0;
	}

	if (k3fd->panel_info.bl_set_type & BL_SET_BY_PWM) {
		PWM_CLK_PUT(&(k3fd->panel_info));
	}
	LCD_VCC_PUT(&(k3fd->panel_info));

	return 0;
}

static void samsung_shutdown(struct platform_device *pdev)
{
	int ret = 0;

	ret = samsung_remove(pdev);
	if (ret != 0) {
		k3fb_loge("failed to shutdown, error=%d!\n", ret);
	}
}

static struct platform_driver this_driver = {
	.probe = samsung_probe,
	.remove = samsung_remove,
	.suspend = NULL,
	.resume = NULL,
	.shutdown = samsung_shutdown,
	.driver = {
		.name = "mipi_samsung_S6E39A",
	},
};

static int __init mipi_samsung_panel_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&this_driver);
	if (ret) {
		k3fb_loge("not able to register the driver, error=%d!\n", ret);
		return ret;
	}

	return ret;
}

module_init(mipi_samsung_panel_init);
