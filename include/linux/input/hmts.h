/*
 *
 * FocalTech ft5x06 TouchScreen driver header file.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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
 */
#ifndef __LINUX_HMTS_H__
#define __LINUX_HMTS_H__

#define HMTS_ID		0x88

struct hmts_gesture_platform_data {
	int gesture_enable_to_set;	/* enable/disable gesture */
	int in_pocket;	/* whether in pocket mode or not */
	struct device *dev;
	struct class *gesture_class;
	struct hmts_data *data;
};

struct hmts_platform_data {
	const char *name;
	u32 irqflags;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	u32 group_id;
	u32 num_max_touches;
	bool i2c_pull_up;
	bool gesture_support;
	int (*power_init) (bool);
	int (*power_on) (bool);
	bool mirror_h;
	bool mirror_v;
};

#endif
