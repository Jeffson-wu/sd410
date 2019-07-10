/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/regulator/consumer.h>
#include <linux/input.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/sensors.h>
#include <linux/pm_wakeup.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>

#define LTR329_I2C_NAME			"ltr329"
#define LTR329_LIGHT_INPUT_NAME		"ltr329-light"

#define LTR329_REG_ALS_CTL		0x80
#define LTR329_REG_ALS_MEAS_RATE	0x85
#define LTR329_REG_PART_ID		0x86
#define LTR329_REG_ALS_DATA_CH1_0	0x88
#define LTR329_REG_ALS_DATA_CH1_1	0x89
#define LTR329_REG_ALS_DATA_CH0_0	0x8A
#define LTR329_REG_ALS_DATA_CH0_1	0x8B
#define LTR329_REG_ALS_PS_STATUS	0x8C
#define LTR329_REG_PS_DATA_0		0x8D
#define LTR329_REG_INTERRUPT		0x8F
#define LTR329_REG_MAGIC		0xFF

#define LTR329_PART_ID			0x0A

#define LTR329_ALS_SENSITIVITY		70

#define LTR329_BOOT_TIME_MS		120
#define LTR329_WAKE_TIME_MS		10

#define LTR329_PS_SATURATE_MASK		0x8000
#define LTR329_ALS_INT_MASK		0x08
#define LTR329_PS_INT_MASK		0x02

#define LTR329_ALS_MEASURE_MASK		0x38
#define LTR329_ALS_GAIN_MASK		0x1c

/* default measurement rate is 100 ms */
#define LTR329_ALS_DEFAULT_MEASURE_RATE	0x01
#define LTR329_PS_MEASUREMENT_RATE_10MS	0x08

#define LTR329_CALIBRATE_SAMPLES	15

#define ALS_GAIN_SWITCH_THRESHOLD	60000

#define LTR329_ALS_INVALID(value)	(value & 0x80)

/* LTR329 ALS data is 16 bit */
#define ALS_DATA_MASK			0xffff
#define ALS_LOW_BYTE(data)		((data) & 0xff)
#define ALS_HIGH_BYTE(data)		(((data) >> 8) & 0xff)

/* Calculated by 10% transmittance */
#define LTR329_MAX_LUX			(ALS_DATA_MASK * 10)

/* both als and ps interrupt are enabled */
#define LTR329_INTERRUPT_SETTING	0x03

/* Any proximity distance change will wakeup SoC */
#define LTR329_WAKEUP_ANY_CHANGE	0xff

#define CAL_BUF_LEN			16
enum {
	CMD_WRITE = 0,
	CMD_READ = 1,
};

struct regulator_map {
	struct regulator	*regulator;
	int			min_uv;
	int			max_uv;
	char			*supply;
};

struct pinctrl_config {
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*state[2];
	char			*name[2];
};

struct ltr329_data {
	struct i2c_client	*i2c;
	struct regmap		*regmap;
	struct regulator	*config;
	struct input_dev	*input_light;
	struct input_dev	*input_proximity;
	struct workqueue_struct	*workqueue;

	struct sensors_classdev	als_cdev;
	struct sensors_classdev	ps_cdev;
	struct mutex		ops_lock;
	ktime_t			last_als_ts;
	ktime_t			last_ps_ts;
	struct work_struct	report_work;
	struct work_struct	als_enable_work;
	struct work_struct	als_disable_work;
	struct work_struct	ps_enable_work;
	struct work_struct	ps_disable_work;
	atomic_t		wake_count;

	int			irq_gpio;
	int			irq;
	bool			als_enabled;
	bool			ps_enabled;
	u32			irq_flags;
	int			als_delay;
	int			ps_delay;
	int			als_cal;
	int			ps_cal;
	int			als_gain;
	int			als_persist;
	int			als_integration_time;
	int			als_measure_rate;
	int			ps_led;
	int			ps_pulses;
	int			ps_measure_rate;
	int			als_ps_persist;
	int			ps_wakeup_threshold;

	int			last_als;
	int			last_ps;
	int			flush_count;
	int			power_enabled;

	unsigned int		reg_addr;
	char			calibrate_buf[CAL_BUF_LEN];
	unsigned int		bias;
};

struct als_coeff {
	int ch0_coeff_i;
	int ch1_coeff_i;
	int ch0_coeff_f;
	int ch1_coeff_f;
	int win_fac;
	int sign;
} __attribute__((__packed__));

static struct regulator_map power_config[] = {
	{.supply = "vdd", .min_uv = 2000000, .max_uv = 3300000, },
	{.supply = "vio", .min_uv = 1750000, .max_uv = 1950000, },
};

static struct pinctrl_config pin_config = {
	.name = { "default", "sleep" },
};

static struct als_coeff eqtn_map[] = {
	{
		.ch0_coeff_i = 1,
		.ch1_coeff_i = 1,
		.ch0_coeff_f = 7743,
		.ch1_coeff_f = 1059,
		.win_fac = 44,
		.sign = 1,
	},
	{
		.ch0_coeff_i = 4,
		.ch1_coeff_i = 1,
		.ch0_coeff_f = 2785,
		.ch1_coeff_f = 9548,
		.win_fac = 50,
		.sign = -1,
	},
	{
		.ch0_coeff_i = 0,
		.ch1_coeff_i = 0,
		.ch0_coeff_f = 5926,
		.ch1_coeff_f = 1185,
		.win_fac = 40,
		.sign = 1,
	},
	{
		.ch0_coeff_i = 0,
		.ch1_coeff_i = 0,
		.ch0_coeff_f = 0,
		.ch1_coeff_f = 0,
		.win_fac = 1,
		.sign = 1,
	},
};

/* ALS integration time in 10ms */
static int als_int_fac_table[] = { 10, 5, 20, 40, 15, 25, 30, 35 };
/* ALS gain table, index 4 & 5 are reserved */
static int als_gain_table[] = {1, 2, 4, 8, 1, 1, 48, 96};
/* ALS measurement repeat rate in ms */
static int als_mrr_table[] = {50, 100, 200, 500, 1000, 2000, 2000, 2000};
/* PS measurement repeat rate in ms */
static int ps_mrr_table[] = { 50, 70, 100, 200, 500, 1000, 2000, 10,
				10, 10, 10, 10, 10, 10, 10, 10};

/* Tuned for devices with rubber */
static int ps_distance_table[] =  { 790, 337, 195, 114, 78, 62, 50 };

static int sensitivity_table[] = {150, 150, 100, 100, 0, 0, 100, 1};

static struct sensors_classdev als_cdev = {
	.name = "ltr329-light",
	.vendor = "Lite-On Technology Corp",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "65536",
	.resolution = "1.0",
	.sensor_power = "0.25",
	.min_delay = 50000,
	.max_delay = 2000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = 2,
	.enabled = 0,
	.delay_msec = 50,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};


static int sensor_power_init(struct device *dev, struct regulator_map *map,
		int size)
{
	int rc;
	int i;

	for (i = 0; i < size; i++) {
		map[i].regulator = devm_regulator_get(dev, map[i].supply);
		if (IS_ERR(map[i].regulator)) {
			rc = PTR_ERR(map[i].regulator);
			dev_err(dev, "Regualtor get failed vdd rc=%d\n", rc);
			goto exit;
		}
		if (regulator_count_voltages(map[i].regulator) > 0) {
			rc = regulator_set_voltage(map[i].regulator,
					map[i].min_uv, map[i].max_uv);
			if (rc) {
				dev_err(dev, "Regulator set failed vdd rc=%d\n",
						rc);
				goto exit;
			}
		}
	}

	return 0;

exit:
	/* Regulator not set correctly */
	for (i = i - 1; i >= 0; i--) {
		if (regulator_count_voltages(map[i].regulator))
			regulator_set_voltage(map[i].regulator, 0,
					map[i].max_uv);
	}

	return rc;
}

static int sensor_power_deinit(struct device *dev, struct regulator_map *map,
		int size)
{
	int i;

	for (i = 0; i < size; i++) {
		if (!IS_ERR_OR_NULL(map[i].regulator)) {
			if (regulator_count_voltages(map[i].regulator) > 0)
				regulator_set_voltage(map[i].regulator, 0,
						map[i].max_uv);
		}
	}

	return 0;
}

static int sensor_power_config(struct device *dev, struct regulator_map *map,
		int size, bool enable)
{
	int i;
	int rc = 0;

	if (enable) {
		for (i = 0; i < size; i++) {
			rc = regulator_enable(map[i].regulator);
			if (rc) {
				dev_err(dev, "enable %s failed.\n",
						map[i].supply);
				goto exit_enable;
			}
		}
	} else {
		for (i = 0; i < size; i++) {
			rc = regulator_disable(map[i].regulator);
			if (rc) {
				dev_err(dev, "disable %s failed.\n",
						map[i].supply);
				goto exit_disable;
			}
		}
	}

	return 0;

exit_enable:
	for (i = i - 1; i >= 0; i--)
		regulator_disable(map[i].regulator);

	return rc;

exit_disable:
	for (i = i - 1; i >= 0; i--)
		if (regulator_enable(map[i].regulator))
			dev_err(dev, "enable %s failed\n", map[i].supply);

	return rc;
}

static int sensor_pinctrl_init(struct device *dev,
		struct pinctrl_config *config)
{
	config->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(config->pinctrl)) {
		dev_err(dev, "Failed to get pinctrl\n");
		return PTR_ERR(config->pinctrl);
	}

	config->state[0] =
		pinctrl_lookup_state(config->pinctrl, config->name[0]);
	if (IS_ERR_OR_NULL(config->state[0])) {
		dev_err(dev, "Failed to look up %s\n", config->name[0]);
		return PTR_ERR(config->state[0]);
	}

	config->state[1] =
		pinctrl_lookup_state(config->pinctrl, config->name[1]);
	if (IS_ERR_OR_NULL(config->state[1])) {
		dev_err(dev, "Failed to look up %s\n", config->name[1]);
		return PTR_ERR(config->state[1]);
	}

	return 0;
}

static int ltr329_parse_dt(struct device *dev, struct ltr329_data *ltr)
{
	struct device_node *dp = dev->of_node;
	u32 value;
	int rc;
	int i;

	rc = of_get_named_gpio_flags(dp, "liteon,irq-gpio", 0,
			&ltr->irq_flags);
	if (rc < 0) {
		dev_err(dev, "unable to read irq gpio\n");
		return rc;
	}
	ltr->irq_gpio = rc;

	/* als ps persist */
	rc = of_property_read_u32(dp, "liteon,als-ps-persist", &value);
	if (rc) {
		dev_err(dev, "read liteon,als-ps-persist failed\n");
		return rc;
	}
	ltr->als_ps_persist = value;

	/* ps led */
	rc = of_property_read_u32(dp, "liteon,ps-led", &value);
	if (rc) {
		dev_err(dev, "read liteon,ps-led failed\n");
		return rc;
	}
	ltr->ps_led = value;

	/* ps pulses */
	rc = of_property_read_u32(dp, "liteon,ps-pulses", &value);
	if (rc) {
		dev_err(dev, "read liteon,ps-pulses failed\n");
		return rc;
	}
	if (value > 0x7) {
		dev_err(dev, "liteon,ps-pulses out of range\n");
		return -EINVAL;
	}
	ltr->ps_pulses = value;

	/* als integration time */
	rc = of_property_read_u32(dp, "liteon,als-integration-time", &value);
	if (rc) {
		dev_err(dev, "read liteon,als-integration-time failed\n");
		return rc;
	}
	if (value > 0x7) {
		dev_err(dev, "liteon,als-integration-time out of range\n");
		return -EINVAL;
	}
	ltr->als_integration_time = value;

	/* ps wakeup threshold */
	rc = of_property_read_u32(dp, "liteon,wakeup-threshold", &value);
	if (rc) {
		dev_err(dev, "liteon,wakeup-threshold incorrect, drop to default\n");
		value = LTR329_WAKEUP_ANY_CHANGE;
	}
	if ((value >= ARRAY_SIZE(ps_distance_table)) &&
			(value != LTR329_WAKEUP_ANY_CHANGE)) {
		dev_err(dev, "wakeup threshold too big\n");
		return -EINVAL;
	}
	ltr->ps_wakeup_threshold = value;

	/* ps distance table */
	rc = of_property_read_u32_array(dp, "liteon,ps-distance-table",
			ps_distance_table, ARRAY_SIZE(ps_distance_table));
	if ((rc == -ENODATA) || (rc == -EOVERFLOW)) {
		dev_warn(dev, "liteon,ps-distance-table not correctly set\n");
		return rc;
	}

	for (i = 1; i < ARRAY_SIZE(ps_distance_table); i++) {
		if (ps_distance_table[i - 1] < ps_distance_table[i]) {
			dev_err(dev, "ps distance table should in descend order\n");
			return -EINVAL;
		}
	}

	if (ps_distance_table[0] > PS_DATA_MASK) {
		dev_err(dev, "distance table out of range\n");
		return -EINVAL;
	}

	/* als gain */
	rc = of_property_read_u32(dp, "liteon,als-gain", &value);
	if (rc) {
		dev_err(dev, "read liteon,als-gain failed. Drop to default\n");
		value = 0;
	}
	/* 4 & 5 are reserved */
	if ((value > 0x7) || (value == 0x4) || (value == 0x5)) {
		dev_err(dev, "liteon,als-gain invalid\n");
		return -EINVAL;
	}
	ltr->als_gain = value;

	/* als sensitivity */
	rc = of_property_read_u32_array(dp, "liteon,als-sensitivity",
			sensitivity_table, ARRAY_SIZE(sensitivity_table));
	if (rc)
		dev_info(dev, "read liteon,als-sensitivity failed. Drop to default\n");

	/* als equation map */
	rc = of_property_read_u32_array(dp, "liteon,als-equation-0",
			&eqtn_map[0].ch0_coeff_i, 6);
	if (rc)
		dev_warn(dev, "read liteon,als-equation-0 failed. Drop to default\n");

	rc = of_property_read_u32_array(dp, "liteon,als-equation-0",
			&eqtn_map[1].ch0_coeff_i, 6);
	if (rc)
		dev_warn(dev, "read liteon,als-equation-1 failed. Drop to default\n");

	rc = of_property_read_u32_array(dp, "liteon,als-equation-0",
			&eqtn_map[2].ch0_coeff_i, 6);
	if (rc)
		dev_warn(dev, "read liteon,als-equation-2 failed. Drop to default\n");

	rc = of_property_read_u32_array(dp, "liteon,als-equation-3",
			&eqtn_map[3].ch0_coeff_i, 6);
	if (rc)
		dev_warn(dev, "read liteon,als-equation-3 failed. Drop to default\n");

	return 0;
}

static int ltr329_check_device(struct ltr329_data *ltr)
{
	unsigned int part_id;
	int rc;

	rc = regmap_read(ltr->regmap, LTR329_REG_PART_ID, &part_id);
	if (rc) {
		dev_err(&ltr->i2c->dev, "read reg %d failed.(%d)\n",
				LTR329_REG_PART_ID, rc);
		return rc;
	}

	if (part_id != LTR329_PART_ID)
		return -ENODEV;

	return 0;
}

static int ltr329_init_input(struct ltr329_data *ltr)
{
	struct input_dev *input;
	int status;

	input = devm_input_allocate_device(&ltr->i2c->dev);
	if (!input) {
		dev_err(&ltr->i2c->dev, "allocate light input device failed\n");
		return -ENOMEM;
	}

	input->name = LTR329_LIGHT_INPUT_NAME;
	input->phys = "ltr329/input0";
	input->id.bustype = BUS_I2C;

	input_set_capability(input, EV_ABS, ABS_MISC);
	input_set_abs_params(input, ABS_MISC, 0, LTR329_MAX_LUX, 0, 0);

	status = input_register_device(input);
	if (status) {
		dev_err(&ltr->i2c->dev, "register light input device failed.\n");
		return status;
	}

	ltr->input_light = input;

	return 0;
}

static int ltr329_init_device(struct ltr329_data *ltr)
{
	int rc;
	unsigned int tmp;

	rc = regmap_write(ltr->regmap, LTR329_REG_ALS_MEAS_RATE,
		(ltr->als_integration_time << 3) | (ltr->als_measure_rate));
	if (rc) {
		dev_err(&ltr->i2c->dev, "write %d failed\n",
				LTR329_REG_ALS_MEAS_RATE);
		return rc;
	}

	/* set up als gain */
	rc = regmap_read(ltr->regmap, LTR329_REG_ALS_CTL, &tmp);
	if (rc) {
		dev_err(&ltr->i2c->dev, "read %d register failed\n",
				LTR329_REG_ALS_CTL);
		return rc;
	}
	rc = regmap_write(ltr->regmap, LTR329_REG_ALS_CTL,
			(tmp & (~0x1c)) | (ltr->als_gain << 2));
	if (rc) {
		dev_err(&ltr->i2c->dev, "write %d register failed\n",
				LTR329_REG_ALS_CTL);
		return rc;
	}

	return 0;
}

/* Calculate the lux value based on ADC data */
static int ltr329_calc_lux(int ch0data, int ch1data, int gain, int als_int_fac)
{
	int ratio;
	int lux_i;
	int lux_f;
	int lux;
	struct als_coeff *eqtn;

	/* avoid divided by 0 */
	if ((ch0data == 0) && (ch1data == 0))
		return 0;

	ratio = ch1data * 100 / (ch0data + ch1data);
	if (ratio < 45)
		eqtn = &eqtn_map[0];
	else if ((ratio >= 45) && (ratio < 68))
		eqtn = &eqtn_map[1];
	else if ((ratio >= 68) && (ratio < 99))
		eqtn = &eqtn_map[2];
	else
		eqtn = &eqtn_map[3];

	lux_i = (ch0data * eqtn->ch0_coeff_i + ch1data * eqtn->ch1_coeff_i *
			eqtn->sign) * eqtn->win_fac;
	lux_f = (ch0data * eqtn->ch0_coeff_f + ch1data * eqtn->ch1_coeff_f *
			eqtn->sign) / 100 * eqtn->win_fac;

	lux = (lux_i + abs(lux_f) / 100) / (gain * als_int_fac);

	return lux;
}

/* Calculate adc value based on lux. Return value is positive */
static int ltr329_calc_adc(int ratio, int lux, int gain, int als_int_fac)
{
	int divisor_i;
	int divisor_f;
	int dividend;
	struct als_coeff *eqtn;
	int result;

	/* avoid devided by 0 */
	if (ratio == 0)
		return 0;

	if (ratio < 45)
		eqtn = &eqtn_map[0];
	else if ((ratio >= 45) && (ratio < 68))
		eqtn = &eqtn_map[1];
	else if ((ratio >= 68) && (ratio < 99))
		eqtn = &eqtn_map[2];
	else
		eqtn = &eqtn_map[3];

	dividend = lux * gain * als_int_fac;
	divisor_i = ((100 - ratio) * eqtn->ch0_coeff_i / ratio +
			eqtn->ch1_coeff_i * eqtn->sign) * eqtn->win_fac;
	divisor_f = abs((100 - ratio) * eqtn->ch0_coeff_f / ratio +
			eqtn->ch1_coeff_f * eqtn->sign) * eqtn->win_fac / 10000;

	/* avoid divided by 0 */
	if ((divisor_i + divisor_f) == 0)
		return 0;

	result = dividend / (divisor_i + divisor_f);

	return result <= 0 ? 1 : result;
}

/* update als gain and threshold */
static int ltr329_als_update_setting(struct ltr329_data *ltr,
		int ch0data, int ch1data, int als_int_fac)
{
	int gain_index;
	unsigned int config;
	unsigned int ratio;
	unsigned int adc_base;
	int rc;
	int adc;
	int i;
	u8 als_data[4];

	for (i = ARRAY_SIZE(als_gain_table) - 1; i >= 0; i--) {
		if ((i == 4) || (i == 5))
			continue;

		if ((ch0data + ch1data) * als_gain_table[i] /
				als_gain_table[ltr->als_gain] <
				ALS_GAIN_SWITCH_THRESHOLD)
			break;
	}

	gain_index = i < 0 ? 0 : i;

	/*
	 * Disable als and enable it again to avoid incorrect value.
	 * Updating als gain during als measurement cycle will cause
	 * incorrect light sensor adc value. The logic here is to handle
	 * this scenario.
	 */

	if (ltr->als_gain != gain_index) {
		rc = regmap_read(ltr->regmap, LTR329_REG_ALS_CTL, &config);
		if (rc) {
			dev_err(&ltr->i2c->dev, "read %d failed.(%d)\n",
					LTR329_REG_ALS_CTL, rc);
			return rc;
		}

		/* disable als sensor */
		rc = regmap_write(ltr->regmap, LTR329_REG_ALS_CTL,
				config & (~0x1));
		if (rc) {
			dev_err(&ltr->i2c->dev, "write %d failed.(%d)\n",
					LTR329_REG_ALS_CTL, rc);
			return rc;
		}

		/* write new als gain */
		rc = regmap_write(ltr->regmap, LTR329_REG_ALS_CTL,
				(config & (~0x1c)) | (gain_index << 2));
		if (rc) {
			dev_err(&ltr->i2c->dev, "write %d register failed\n",
					LTR329_REG_ALS_CTL);
			return rc;
		}
	}

	if ((ch0data == 0) && (ch1data == 0)) {
		adc = 1;
	} else {
		ratio = ch1data * 100 / (ch0data + ch1data);
		dev_dbg(&ltr->i2c->dev, "ratio:%d\n", ratio);
		adc = ltr329_calc_adc(ratio, sensitivity_table[gain_index],
				als_gain_table[gain_index], als_int_fac);
	}

	dev_dbg(&ltr->i2c->dev, "adc:%d\n", adc);

	/* catch'ya! */
	adc_base = ch0data * als_gain_table[gain_index] /
		als_gain_table[ltr->als_gain];

	/* upper threshold */
	if (adc_base + adc > ALS_DATA_MASK) {
		als_data[0] = 0xff;
		als_data[1] = 0xff;
	} else {
		als_data[0] = ALS_LOW_BYTE(adc_base + adc);
		als_data[1] = ALS_HIGH_BYTE(adc_base + adc);
	}

	/* lower threshold */
	if (adc_base < adc) {
		als_data[2] = 0x0;
		als_data[3] = 0x0;
	} else {
		als_data[2] = ALS_LOW_BYTE(adc_base - adc);
		als_data[3] = ALS_HIGH_BYTE(adc_base - adc);
	}

	if (ltr->als_gain != gain_index) {
		/* enable als_sensor */
		rc = regmap_write(ltr->regmap, LTR329_REG_ALS_CTL,
				(config & (~0x1c)) | (gain_index << 2) | 0x1);
		if (rc) {
			dev_err(&ltr->i2c->dev, "write %d failed.(%d)\n",
					LTR329_REG_ALS_CTL, rc);
			return rc;
		}

		ltr->als_gain = gain_index;
	}

	return 0;
}

static int ltr329_process_data(struct ltr329_data *ltr, int als_ps)
{
	int als_int_fac;
	ktime_t	timestamp;
	int rc = 0;

	unsigned int tmp;
	u8 als_data[4];
	int lux;
	int ch0data;
	int ch1data;

	u8 ps_data[4];
	int i;
	int distance;

	timestamp = ktime_get_boottime();

	if (als_ps) { /* process als data */
		/* Read data */
		rc = regmap_bulk_read(ltr->regmap, LTR329_REG_ALS_DATA_CH1_0,
				als_data, 4);
		if (rc) {
			dev_err(&ltr->i2c->dev, "read %d failed.(%d)\n",
					LTR329_REG_ALS_DATA_CH1_0, rc);
			goto exit;
		}
		ch0data = als_data[2] | (als_data[3] << 8);
		ch1data = als_data[0] | (als_data[1] << 8);

		rc = regmap_read(ltr->regmap, LTR329_REG_ALS_MEAS_RATE, &tmp);
		if (rc) {
			dev_err(&ltr->i2c->dev, "read %d failed.(%d)\n",
					LTR329_REG_ALS_MEAS_RATE, rc);
			goto exit;
		}

		tmp = (tmp & LTR329_ALS_MEASURE_MASK) >> 3;
		als_int_fac = als_int_fac_table[tmp];
		lux = ltr329_calc_lux(ch0data, ch1data,
				als_gain_table[ltr->als_gain], als_int_fac);

		dev_dbg(&ltr->i2c->dev, "lux:%d als_data:0x%x-0x%x-0x%x-0x%x\n",
				lux, als_data[0], als_data[1],
				als_data[2], als_data[3]);

		rc = regmap_read(ltr->regmap, LTR329_REG_ALS_PS_STATUS, &tmp);
		if (rc) {
			dev_err(&ltr->i2c->dev, "read %d failed.(%d)\n",
					LTR329_REG_ALS_PS_STATUS, rc);
			goto exit;
		}


		if ((lux != ltr->last_als) && (!LTR329_ALS_INVALID(tmp))) {
			input_report_abs(ltr->input_light, ABS_MISC, lux);
			input_event(ltr->input_light, EV_SYN, SYN_TIME_SEC,
					ktime_to_timespec(timestamp).tv_sec);
			input_event(ltr->input_light, EV_SYN, SYN_TIME_NSEC,
					ktime_to_timespec(timestamp).tv_nsec);
			input_sync(ltr->input_light);

			ltr->last_als_ts = timestamp;
		}

		ltr->last_als = lux;

		dev_dbg(&ltr->i2c->dev, "previous als_gain:%d\n",
				ltr->als_gain);

		rc = ltr329_als_update_setting(ltr, ch0data, ch1data,
				als_int_fac);
		if (rc) {
			dev_err(&ltr->i2c->dev, "update setting failed\n");
			goto exit;
		}

		dev_dbg(&ltr->i2c->dev, "new als_gain:%d\n",
				ltr->als_gain);


	} else { /* process ps value */
	}
exit:
	return rc;
}

static irqreturn_t ltr329_irq_handler(int irq, void *data)
{
	struct ltr329_data *ltr = data;
	bool rc;

	rc = queue_work(ltr->workqueue, &ltr->report_work);
	/* wake up event should hold a wake lock until reported */
	if (rc && (atomic_inc_return(&ltr->wake_count) == 1))
		pm_stay_awake(&ltr->i2c->dev);


	return IRQ_HANDLED;
}

static void ltr329_report_work(struct work_struct *work)
{
	struct ltr329_data *ltr = container_of(work, struct ltr329_data,
			report_work);
	int rc;
	unsigned int status;
	u8 buf[7];

	mutex_lock(&ltr->ops_lock);

	/* avoid fake interrupt */
	if (!ltr->power_enabled) {
		dev_dbg(&ltr->i2c->dev, "fake interrupt triggered\n");
		goto exit;
	}

	/* read status */
	rc = regmap_read(ltr->regmap, LTR329_REG_ALS_PS_STATUS, &status);
	if (rc) {
		dev_err(&ltr->i2c->dev, "read %d failed.(%d)\n",
				LTR329_REG_ALS_PS_STATUS, rc);
		status |= LTR329_PS_INT_MASK;
		goto exit;
	}

	dev_dbg(&ltr->i2c->dev, "interrupt issued status=0x%x.\n", status);

	/* als interrupt issueed */
	if ((status & LTR329_ALS_INT_MASK) && (ltr->als_enabled)) {
		rc = ltr329_process_data(ltr, 1);
		if (rc)
			goto exit;
		dev_dbg(&ltr->i2c->dev, "process als done!\n");
	}

	if ((status & LTR329_PS_INT_MASK) && (ltr->ps_enabled)) {
		rc = ltr329_process_data(ltr, 0);
		if (rc)
			goto exit;
		dev_dbg(&ltr->i2c->dev, "process ps data done!\n");
		pm_wakeup_event(&ltr->input_proximity->dev, 200);
	}

exit:
	if (atomic_dec_and_test(&ltr->wake_count)) {
		pm_relax(&ltr->i2c->dev);
		dev_dbg(&ltr->i2c->dev, "wake lock released\n");
	}

	/* clear interrupt */
	if (regmap_bulk_read(ltr->regmap, LTR329_REG_ALS_DATA_CH1_0,
			buf, ARRAY_SIZE(buf)))
		dev_err(&ltr->i2c->dev, "clear interrupt failed\n");

	mutex_unlock(&ltr->ops_lock);
}


static int ltr329_enable_als(struct ltr329_data *ltr, int enable)
{
	int rc = 0;
	unsigned int config;
	unsigned int tmp;
	u8 buf[7];

	rc = regmap_read(ltr->regmap, LTR329_REG_ALS_CTL, &config);
	if (rc) {
		dev_err(&ltr->i2c->dev, "read %d failed.(%d)\n",
				LTR329_REG_ALS_CTL, rc);
		goto exit;
	}

	if (enable) {
		/* enable als_sensor */
		rc = regmap_write(ltr->regmap, LTR329_REG_ALS_CTL,
				config | 0x1);
		if (rc) {
			dev_err(&ltr->i2c->dev, "write %d failed.(%d)\n",
					LTR329_REG_ALS_CTL, rc);
			goto exit;
		}

		rc = regmap_read(ltr->regmap, LTR329_REG_ALS_MEAS_RATE, &tmp);
		if (rc) {
			dev_err(&ltr->i2c->dev, "write %d failed.(%d)\n",
					LTR329_REG_ALS_MEAS_RATE, rc);
			goto exit;
		}

		/* Wait for data ready */
		msleep(als_mrr_table[tmp & 0x7] + LTR329_WAKE_TIME_MS);

		/* Clear last value and report even not change. */
		ltr->last_als = -1;

		rc = ltr329_process_data(ltr, 1);
		if (rc) {
			dev_err(&ltr->i2c->dev, "process als data failed\n");
			goto exit;
		}

		/* clear interrupt */
		rc = regmap_bulk_read(ltr->regmap, LTR329_REG_ALS_DATA_CH1_0,
				buf, ARRAY_SIZE(buf));
		if (rc) {
			dev_err(&ltr->i2c->dev, "clear interrupt failed\n");
			goto exit;
		}

		ltr->als_enabled = true;
	} else {
		/* disable als sensor */
		rc = regmap_write(ltr->regmap, LTR329_REG_ALS_CTL,
				config & (~0x1));
		if (rc) {
			dev_err(&ltr->i2c->dev, "write %d failed.(%d)\n",
					LTR329_REG_ALS_CTL, rc);
			goto exit;
		}

		ltr->als_enabled = false;
	}

exit:
	return rc;
}

static int ltr329_als_sync_delay(struct ltr329_data *ltr,
		unsigned int als_delay)
{
	int index = 0;
	int i;
	unsigned int val;
	int rc = 0;
	int min;

	if (!ltr->power_enabled) {
		dev_dbg(&ltr->i2c->dev, "power is not enabled\n");
		return 0;
	}

	min = abs(als_delay - als_mrr_table[0]);
	for (i = 0; i < ARRAY_SIZE(als_mrr_table); i++) {
		if (als_mrr_table[i] >= 10 *
				als_int_fac_table[ltr->als_integration_time]) {
			if (als_delay == als_mrr_table[i]) {
				index = i;
				break;
			}
			if (min > abs(als_delay - als_mrr_table[i])) {
				index = i;
				min = abs(als_delay - als_mrr_table[i]);
			}
		}
	}

	dev_dbg(&ltr->i2c->dev, "als delay %d ms\n", als_mrr_table[index]);

	rc = regmap_read(ltr->regmap, LTR329_REG_ALS_MEAS_RATE, &val);
	if (rc) {
		dev_err(&ltr->i2c->dev, "read %d failed\n",
				LTR329_REG_ALS_MEAS_RATE);
		goto exit;
	}
	val &= ~0x7;

	ltr->als_measure_rate = index;
	rc = regmap_write(ltr->regmap, LTR329_REG_ALS_MEAS_RATE, val | index);
	if (rc) {
		dev_err(&ltr->i2c->dev, "write %d failed\n",
				LTR329_REG_ALS_MEAS_RATE);
		goto exit;
	}

exit:
	return rc;
}

static void ltr329_als_enable_work(struct work_struct *work)
{
	struct ltr329_data *ltr = container_of(work, struct ltr329_data,
			als_enable_work);

	mutex_lock(&ltr->ops_lock);
	if (!ltr->power_enabled) { /* new HAL? */
		if (sensor_power_config(&ltr->i2c->dev, power_config,
					ARRAY_SIZE(power_config), true)) {
			dev_err(&ltr->i2c->dev, "power up sensor failed.\n");
			goto exit;
		}

		msleep(LTR329_BOOT_TIME_MS);
		ltr->power_enabled = true;
		if (ltr329_init_device(ltr)) {
			dev_err(&ltr->i2c->dev, "init device failed\n");
			goto exit_power_off;
		}

		ltr329_als_sync_delay(ltr, ltr->als_delay);
	}

	if (ltr329_enable_als(ltr, 1)) {
		dev_err(&ltr->i2c->dev, "enable als failed\n");
		goto exit_power_off;
	}

exit_power_off:
	if ((!ltr->als_enabled) && (!ltr->ps_enabled) &&
			ltr->power_enabled) {
		if (sensor_power_config(&ltr->i2c->dev, power_config,
					ARRAY_SIZE(power_config), false)) {
			dev_err(&ltr->i2c->dev, "power up sensor failed.\n");
			goto exit;
		}
		ltr->power_enabled = false;
	}
exit:
	mutex_unlock(&ltr->ops_lock);
}


static void ltr329_als_disable_work(struct work_struct *work)
{
	struct ltr329_data *ltr = container_of(work, struct ltr329_data,
			als_disable_work);

	mutex_lock(&ltr->ops_lock);

	if (ltr329_enable_als(ltr, 0)) {
		dev_err(&ltr->i2c->dev, "disable als failed\n");
		goto exit;
	}

	if ((!ltr->als_enabled) && (!ltr->ps_enabled) && ltr->power_enabled) {
		if (sensor_power_config(&ltr->i2c->dev, power_config,
					ARRAY_SIZE(power_config), false)) {
			dev_err(&ltr->i2c->dev, "power up sensor failed.\n");
			goto exit;
		}

		ltr->power_enabled = false;
	}

exit:
	mutex_unlock(&ltr->ops_lock);
}

static struct regmap_config ltr329_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int ltr329_cdev_enable_als(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct ltr329_data *ltr = container_of(sensors_cdev,
			struct ltr329_data, als_cdev);

	mutex_lock(&ltr->ops_lock);

	if (enable)
		queue_work(ltr->workqueue, &ltr->als_enable_work);
	else
		queue_work(ltr->workqueue, &ltr->als_disable_work);

	mutex_unlock(&ltr->ops_lock);

	return 0;
}

static int ltr329_cdev_set_als_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct ltr329_data *ltr = container_of(sensors_cdev,
			struct ltr329_data, als_cdev);
	int rc;

	mutex_lock(&ltr->ops_lock);

	ltr->als_delay = delay_msec;
	rc = ltr329_als_sync_delay(ltr, delay_msec);

	mutex_unlock(&ltr->ops_lock);

	return rc;
}

static int ltr329_cdev_als_flush(struct sensors_classdev *sensors_cdev)
{
	struct ltr329_data *ltr = container_of(sensors_cdev,
			struct ltr329_data, als_cdev);

	input_event(ltr->input_light, EV_SYN, SYN_CONFIG, ltr->flush_count++);
	input_sync(ltr->input_light);

	return 0;
}

static ssize_t ltr329_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ltr329_data *ltr = dev_get_drvdata(dev);
	unsigned int val;
	int rc;
	ssize_t count = 0;
	int i;

	if (ltr->reg_addr == LTR329_REG_MAGIC) {
		for (i = 0; i <= 0x1f; i++) {
			rc = regmap_read(ltr->regmap, LTR329_REG_ALS_CTL + i,
					&val);
			if (rc) {
				dev_err(&ltr->i2c->dev, "read %d failed\n",
						LTR329_REG_ALS_CTL + i);
				break;
			}
			count += snprintf(&buf[count], PAGE_SIZE,
					"0x%x: 0x%x\n", LTR329_REG_ALS_CTL + i,
					val);
		}
	} else {
		rc = regmap_read(ltr->regmap, ltr->reg_addr, &val);
		if (rc) {
			dev_err(&ltr->i2c->dev, "read %d failed\n",
					ltr->reg_addr);
			return rc;
		}
		count += snprintf(&buf[count], PAGE_SIZE, "0x%x:0x%x\n",
				ltr->reg_addr, val);
	}

	return count;
}

static ssize_t ltr329_register_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct ltr329_data *ltr = dev_get_drvdata(dev);
	unsigned int reg;
	unsigned int val;
	unsigned int cmd;
	int rc;

	if (sscanf(buf, "%u %u %u\n", &cmd, &reg, &val) < 2) {
		dev_err(&ltr->i2c->dev, "argument error\n");
		return -EINVAL;
	}

	if (cmd == CMD_WRITE) {
		rc = regmap_write(ltr->regmap, reg, val);
		if (rc) {
			dev_err(&ltr->i2c->dev, "write %d failed\n", reg);
			return rc;
		}
	} else if (cmd == CMD_READ) {
		ltr->reg_addr = reg;
		dev_dbg(&ltr->i2c->dev, "register address set to 0x%x\n", reg);
	}

	return size;
}

static DEVICE_ATTR(register, S_IWUSR | S_IRUGO,
		ltr329_register_show,
		ltr329_register_store);

static struct attribute *ltr329_attr[] = {
	&dev_attr_register.attr,
	NULL
};

static const struct attribute_group ltr329_attr_group = {
	.attrs = ltr329_attr,
};

static int ltr329_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct ltr329_data *ltr;
	int res = 0;

	dev_dbg(&client->dev, "probling ltr329...\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "ltr329 i2c check failed.\n");
		return -ENODEV;
	}

	ltr = devm_kzalloc(&client->dev, sizeof(struct ltr329_data),
			GFP_KERNEL);
	if (!ltr) {
		dev_err(&client->dev, "memory allocation failed,\n");
		return -ENOMEM;
	}

	ltr->i2c = client;

	if (client->dev.of_node) {
		res = ltr329_parse_dt(&client->dev, ltr);
		if (res) {
			dev_err(&client->dev,
				"unable to parse device tree.(%d)\n", res);
			goto out;
		}
	} else {
		dev_err(&client->dev, "device tree not found.\n");
		res = -ENODEV;
		goto out;
	}

	dev_set_drvdata(&client->dev, ltr);
	mutex_init(&ltr->ops_lock);

	ltr->regmap = devm_regmap_init_i2c(client, &ltr329_regmap_config);
	if (IS_ERR(ltr->regmap)) {
		dev_err(&client->dev, "init regmap failed.(%ld)\n",
				PTR_ERR(ltr->regmap));
		res = PTR_ERR(ltr->regmap);
		goto out;
	}

	res = sensor_power_init(&client->dev, power_config,
			ARRAY_SIZE(power_config));
	if (res) {
		dev_err(&client->dev, "init power failed.\n");
		goto out;
	}

	res = sensor_power_config(&client->dev, power_config,
			ARRAY_SIZE(power_config), true);
	if (res) {
		dev_err(&client->dev, "power up sensor failed.\n");
		goto err_power_config;
	}

	res = sensor_pinctrl_init(&client->dev, &pin_config);
	if (res) {
		dev_err(&client->dev, "init pinctrl failed.\n");
		goto err_pinctrl_init;
	}

	msleep(LTR329_BOOT_TIME_MS);

	res = ltr329_check_device(ltr);
	if (res) {
		dev_err(&client->dev, "check device failed.\n");
		goto err_check_device;
	}

	ltr->als_measure_rate = LTR329_ALS_DEFAULT_MEASURE_RATE;

	res = ltr329_init_device(ltr);
	if (res) {
		dev_err(&client->dev, "check device failed.\n");
		goto err_init_device;
	}

	/* configure interrupt */
	if (gpio_is_valid(ltr->irq_gpio)) {
		res = gpio_request(ltr->irq_gpio, "ltr329_interrupt");
		if (res) {
			dev_err(&client->dev,
				"unable to request interrupt gpio %d\n",
				ltr->irq_gpio);
			goto err_request_gpio;
		}

		res = gpio_direction_input(ltr->irq_gpio);
		if (res) {
			dev_err(&client->dev,
				"unable to set direction for gpio %d\n",
				ltr->irq_gpio);
			goto err_set_direction;
		}

		ltr->irq = gpio_to_irq(ltr->irq_gpio);

		res = devm_request_irq(&client->dev, ltr->irq,
				ltr329_irq_handler,
				ltr->irq_flags | IRQF_ONESHOT,
				"ltr329", ltr);

		if (res) {
			dev_err(&client->dev,
					"request irq %d failed(%d),\n",
					ltr->irq, res);
			goto err_request_irq;
		}

		/* device wakeup initialization */
		device_init_wakeup(&client->dev, 1);

		ltr->workqueue = alloc_workqueue("ltr329_workqueue",
				WQ_NON_REENTRANT | WQ_FREEZABLE, 0);
		INIT_WORK(&ltr->report_work, ltr329_report_work);
		INIT_WORK(&ltr->als_enable_work, ltr329_als_enable_work);
		INIT_WORK(&ltr->als_disable_work, ltr329_als_disable_work);

	} else {
		res = -ENODEV;
		goto err_init_device;
	}

	res = sysfs_create_group(&client->dev.kobj, &ltr329_attr_group);
	if (res) {
		dev_err(&client->dev, "sysfs create group failed\n");
		goto err_create_group;
	}

	res = ltr329_init_input(ltr);
	if (res) {
		dev_err(&client->dev, "init input failed.\n");
		goto err_init_input;
	}

	ltr->als_cdev = als_cdev;
	ltr->als_cdev.sensors_enable = ltr329_cdev_enable_als;
	ltr->als_cdev.sensors_poll_delay = ltr329_cdev_set_als_delay;
	ltr->als_cdev.sensors_flush = ltr329_cdev_als_flush;
	res = sensors_classdev_register(&ltr->input_light->dev, &ltr->als_cdev);
	if (res) {
		dev_err(&client->dev, "sensors class register failed.\n");
		goto err_register_als_cdev;
	}

	sensor_power_config(&client->dev, power_config,
			ARRAY_SIZE(power_config), false);

	dev_dbg(&client->dev, "ltr329 successfully probed!\n");

	return 0;

err_register_als_cdev:
	sensors_classdev_unregister(&ltr->als_cdev);
err_init_input:
	sysfs_remove_group(&client->dev.kobj, &ltr329_attr_group);
err_create_group:
err_request_irq:
err_set_direction:
	gpio_free(ltr->irq_gpio);
err_request_gpio:
err_init_device:
	device_init_wakeup(&client->dev, 0);
err_check_device:
err_pinctrl_init:
	sensor_power_config(&client->dev, power_config,
			ARRAY_SIZE(power_config), false);
err_power_config:
	sensor_power_deinit(&client->dev, power_config,
			ARRAY_SIZE(power_config));
out:
	return res;
}

static int ltr329_remove(struct i2c_client *client)
{
	struct ltr329_data *ltr = dev_get_drvdata(&client->dev);

	sensors_classdev_unregister(&ltr->ps_cdev);
	sensors_classdev_unregister(&ltr->als_cdev);

	if (ltr->input_light)
		input_unregister_device(ltr->input_light);

	if (ltr->input_proximity)
		input_unregister_device(ltr->input_proximity);

	destroy_workqueue(ltr->workqueue);
	device_init_wakeup(&ltr->i2c->dev, 0);
	sensor_power_config(&client->dev, power_config,
			ARRAY_SIZE(power_config), false);
	sensor_power_deinit(&client->dev, power_config,
			ARRAY_SIZE(power_config));
	return 0;
}

static int ltr329_suspend(struct device *dev)
{
	int res = 0;
	struct ltr329_data *ltr = dev_get_drvdata(dev);
	u8 ps_data[4];
	unsigned int config;
	int idx = ltr->ps_wakeup_threshold;

	dev_dbg(dev, "suspending ltr329...");

	mutex_lock(&ltr->ops_lock);

		/* power off */
		disable_irq(ltr->irq);
		if (ltr->power_enabled) {
			res = sensor_power_config(dev, power_config,
					ARRAY_SIZE(power_config), false);
			if (res) {
				dev_err(dev, "failed to suspend ltr329\n");
				enable_irq(ltr->irq);
				goto exit;
			}
		}
		pinctrl_select_state(pin_config.pinctrl, pin_config.state[1]);
exit:
	mutex_unlock(&ltr->ops_lock);
	return res;
}

static int ltr329_resume(struct device *dev)
{
	int res = 0;
	struct ltr329_data *ltr = dev_get_drvdata(dev);
	unsigned int config;

	dev_dbg(dev, "resuming ltr329...");
	if (ltr->ps_enabled) {
		if (device_may_wakeup(&ltr->i2c->dev)) {
			dev_dbg(&ltr->i2c->dev, "disable irq wake\n");
			disable_irq_wake(ltr->irq);
		}

		if (ltr->als_enabled) {
			res = regmap_read(ltr->regmap, LTR329_REG_ALS_CTL,
					&config);
			if (res) {
				dev_err(&ltr->i2c->dev, "read %d failed.(%d)\n",
						LTR329_REG_ALS_CTL, res);
				goto exit;
			}

			res = regmap_write(ltr->regmap, LTR329_REG_ALS_CTL,
					config | 0x1);
			if (res) {
				dev_err(&ltr->i2c->dev, "write %d failed.(%d)\n",
						LTR329_REG_ALS_CTL, res);
				goto exit;
			}
		}
	} else {
		pinctrl_select_state(pin_config.pinctrl, pin_config.state[0]);
		/* Power up sensor */
		if (ltr->power_enabled) {
			res = sensor_power_config(dev, power_config,
					ARRAY_SIZE(power_config), true);
			if (res) {
				dev_err(dev, "failed to power up ltr329\n");
				goto exit;
			}
			msleep(LTR329_BOOT_TIME_MS);

			res = ltr329_init_device(ltr);
			if (res) {
				dev_err(dev, "failed to init ltr329\n");
				goto exit_power_off;
			}
		}

		if (ltr->als_enabled) {
			res = ltr329_enable_als(ltr, ltr->als_enabled);
			if (res) {
				dev_err(dev, "failed to enable ltr329\n");
				goto exit_power_off;
			}
		}

		enable_irq(ltr->irq);
	}

	return res;

exit_power_off:
	if ((!ltr->als_enabled) && (!ltr->ps_enabled) &&
			ltr->power_enabled) {
		if (sensor_power_config(&ltr->i2c->dev, power_config,
					ARRAY_SIZE(power_config), false)) {
			dev_err(&ltr->i2c->dev, "power up sensor failed.\n");
			goto exit;
		}
		ltr->power_enabled = false;
	}

exit:
	return res;
}

static const struct i2c_device_id ltr329_id[] = {
	{ LTR329_I2C_NAME, 0 },
	{ }
};

static struct of_device_id ltr329_match_table[] = {
	{ .compatible = "liteon,ltr329", },
	{ },
};

static const struct dev_pm_ops ltr329_pm_ops = {
	.suspend = ltr329_suspend,
	.resume = ltr329_resume,
};

static struct i2c_driver ltr329_driver = {
	.probe = ltr329_probe,
	.remove = ltr329_remove,
	.id_table = ltr329_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = LTR329_I2C_NAME,
		.of_match_table = ltr329_match_table,
		.pm = &ltr329_pm_ops,
	},
};

module_i2c_driver(ltr329_driver);

MODULE_DESCRIPTION("LTR-553ALPS Driver");
MODULE_LICENSE("GPL v2");
