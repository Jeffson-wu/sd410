/*
 *
 * Borderless hmts TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 * Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/sensors.h>
#include <linux/input/hmts.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define HM_SUSPEND_LEVEL 1
#endif

#define HMTS_POLL	1 // 1:polling 0:interrupt
#define MOUSE_MODE	0
#define AXIS_EN		0
#define GESTURE_EN	1

#define GPIO_BASE	902
#define IRQ_GPIO	GPIO_BASE + 26 //org:105
#define SYNC_GPIO	GPIO_BASE + 28

/*register address*/
#define HM_FW_MAJ	0
#define HM_FW_MIN	1
#define HM_APPID_H	2
#define HM_APPID_L	3
#define HM_CMD		4
#define HM_MODE		5
#define HM_MODECON	6
#define HM_STATE	0x10
#define HM_X_MSB	0x11
#define HM_Y_MSB	0x12
#define HM_XY_LSB	0x13
#define HM_GES_STA	0x14
#define HM_GES_DIAG	0x15
#define HM_X_CHAN	0x20
#define HM_Y_CHAN	0x21
#define HM_SCAN_CNT	0x22
#define HM_TRES_X	0x23
#define HM_TRES_Y	0x24
#define HM_HYSTER	0x2A
#define HM_FLT_TYP	0x31
#define HM_FLT_STR	0x32
#define HM_HSWIP_DST	0x37
#define HM_VSWIP_DST	0x38
#define HM_VSWIP_DST	0x38
#define HM_COMPE_RAM	0x50
#define HM_SEN_VAL		0x80
#define HM_RAW_VAL		0x90

#define HM_VTG_MIN_UV		2600000
#define HM_VTG_MAX_UV		3300000
#define HM_I2C_VTG_MIN_UV	1800000
#define HM_I2C_VTG_MAX_UV	1800000
#define HM_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4
#define POLL_INTERVAL		100
#define X_CHAN		8// DVK 9 HM 8
#define Y_CHAN		7// DVK 6 HM 7
#define MAX_XPOS	X_CHAN * 64// chan 8 x 512 chan 9 x 576
#define MAX_YPOS	Y_CHAN * 64// chan 7 y 448 chan 6 x 384
#define MAX_DX		640
#define MAX_DY		360

#define HM_DEBUG_DIR_NAME	"ts_debug"

struct hmts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct hmts_platform_data *pdata;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	u16 addr;
	bool suspended;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct delayed_work hmts_work;
	
};

int rep_x, rep_y;
u32 mrep_x, mrep_y;
u32 leave_x = 0;
u32 leave_y = 0;
u8	prv_ges;
u8	sensor_peak[15];
u8	sensor_comp[15] = {49,43,53,44,60,67,0,61,66,68,67,59,73,0,49};
bool poll_trigger = false;
bool volumn_bar = false;

//static int hmts_start(struct device *dev);
//static int hmts_stop(struct device *dev);

static int hmts_i2c_read(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error on write.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error on read r.\n", __func__);
	}
	return ret;
}

static int hmts_i2c_write(struct i2c_client *client, char *writebuf,
			    int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	return ret;
}

static int hmts_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return hmts_i2c_write(client, buf, sizeof(buf));
}

static DEVICE_ATTR(pocket, 0664, NULL, NULL);
static DEVICE_ATTR(enable, 0664, NULL, NULL);

#if !HMTS_POLL
static irqreturn_t hmts_interrupt(int irq, void *dev_id)
{
	struct hmts_data *data = dev_id;
	struct input_dev *ip_dev;
	int rc;
	u32 id, x, y;
	u8 fin, reg, *buf;

	if (!data) {
		pr_err("%s: Invalid data\n", __func__);
		return IRQ_HANDLED;
	}

	ip_dev = data->input_dev;
	buf = data->tch_data;

	/*
	 * Read touch data start from register FT_REG_DEV_MODE.
	 * The touch x/y value start from FT_TOUCH_X_H/L_POS and
	 * FT_TOUCH_Y_H/L_POS in buf.
	 */
	reg = HM_STATE;
	rc = hmts_i2c_read(data->client, &reg, 1, buf, 4);
	if (rc < 0) {
		dev_err(&data->client->dev, "%s: read data fail\n", __func__);
		return IRQ_HANDLED;
	}


	fin = buf[0] & 0x1;
	x = (buf[1] << 4) + ((buf[3] & 0xf0) >> 4);
	y = (buf[2] << 4) + (buf[3] & 0x0f);

//	printk("eztest hmts get x[%x] y[%x] state:%x pool trigger:%d\n",x,y,fin,poll_trigger);
	input_mt_slot(ip_dev, id);
		
#if AXIS_EN
	if(fin & 1){
		poll_trigger = true;
#if 0
		if(data->pdata->mirror_h)
			rep_x=data->pdata->panel_maxx-x;
		if(data->pdata->mirror_v)
			rep_y=data->pdata->panel_maxy-y;
#else
		rep_x = x;
		rep_y = y;
#endif
	}else if(!poll_trigger){
		return IRQ_HANDLED;
	}

	if(fin & 1){
		input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 1);
		input_report_abs(ip_dev, ABS_MT_POSITION_X, rep_x);
		input_report_abs(ip_dev, ABS_MT_POSITION_Y, rep_y);
		printk("hmts INT report x[%x] y[%x]\n",rep_x,rep_y);
	}else{
		poll_trigger = false;
		input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 0);
		printk("hmts INT report ------------> leave\n");
	}

	input_mt_report_pointer_emulation(ip_dev, false);
	input_sync(ip_dev);
	printk("hmts INT report ------------> SYNC\n");
#endif

#if GESTURE_EN
	if(buf[0] && 2){
		reg = HM_GES_STA;
		rc = hmts_i2c_read(data->client, &reg, 1, buf, 2);
		if(rc < 0){
			dev_err(&data->client->dev, "hmts poll read failed");
			return IRQ_HANDLED;
		}
		if(buf[0]) printk("eztest hmts----------->ges state:%x ges diag:%x\n",buf[0],buf[1]);
		switch(buf[0]){
			case 0x41:
			case 0x42:
				input_report_key(ip_dev, KEY_RIGHT, 1);
				input_sync(ip_dev);
				input_report_key(ip_dev, KEY_RIGHT, 0);
				input_sync(ip_dev);
				break;
			case 0x61:
			case 0x62:
				input_report_key(ip_dev, KEY_LEFT, 1);
				input_sync(ip_dev);
				input_report_key(ip_dev, KEY_LEFT, 0);
				input_sync(ip_dev);
				break;
			case 0x51:
			case 0x52:
				input_report_key(ip_dev, KEY_VOLUMEUP, 1);
				input_sync(ip_dev);
				input_report_key(ip_dev, KEY_VOLUMEUP, 0);
				input_sync(ip_dev);
				break;
			case 0x31:
			case 0x32:
				input_report_key(ip_dev, KEY_VOLUMEDOWN, 1);
				input_sync(ip_dev);
				input_report_key(ip_dev, KEY_VOLUMEDOWN, 0);
				input_sync(ip_dev);
				break;
			case 0x10:
			case 0x11:
				input_report_key(ip_dev, KEY_OK, 1);
				input_sync(ip_dev);
				input_report_key(ip_dev, KEY_OK, 0);
				input_sync(ip_dev);
				break;
			case 0x20:
				input_report_key(ip_dev, KEY_BACK, 1);
				input_sync(ip_dev);
				input_report_key(ip_dev, KEY_BACK, 0);
				input_sync(ip_dev);
				break;
				
		}
		
	}
#endif

	return IRQ_HANDLED;
}
#endif

//poll test
#if HMTS_POLL
static void hmts_poll(struct work_struct *work)
{
	struct hmts_data *data;
	struct input_dev *ip_dev;
	struct delayed_work *delayed_work;
//	struct ts_event *event = &data->event;
//	int err0,err1,err2;
	int err, x, y;
//	int i;
	u32 id = 1234;
	u8 reg, fin;
//	u8 reg_value;
#if GESTURE_EN
	u8 ges;
#endif
	u8 *buf;

	delayed_work = container_of(work, struct delayed_work, work);
	data = container_of(delayed_work, struct hmts_data, hmts_work);

	ip_dev = data->input_dev;
	buf = data->tch_data;

	reg = HM_STATE;
	err = hmts_i2c_read(data->client, &reg, 1, buf, 6);
	if(err < 0){
		dev_err(&data->client->dev, "hmts poll read failed");
		goto quiet_exit;
	}
	fin = buf[0] & 0x1;
	x = (buf[1] << 4) + ((buf[3] & 0xf0) >> 4);
	y = (buf[2] << 4) + (buf[3] & 0x0f);
//	y = (buf[1] << 4) + ((buf[3] & 0xf0) >> 4);
//	x = (buf[2] << 4) + (buf[3] & 0x0f);



	if(buf[0] & 0x3){
		printk("eztest hmts get x[%d] y[%d] state:%x pool trigger:%d\n",x,y,buf[0],poll_trigger);
#if 0
		for(i=0;i<15;i++)
		{
			reg = HM_COMPE_RAM + i;
			err = hmts_i2c_read(data->client, &reg, 1, &reg_value, 1);
			if(i == 0) printk("hmts COMPENSATION RAM:\n 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
			printk("[%02x]",reg_value);
			if(i == 14) printk("\n========================================\n");
		}
		for(i=0;i<15;i++)
		{
			reg = HM_SEN_VAL + i;
			err = hmts_i2c_read(data->client, &reg, 1, &reg_value, 1);
			if(i == 0) printk("hmts SENSOR VALUES:\n 0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F\n");
			if(reg_value > sensor_peak[i]) sensor_peak[i] = reg_value;
			printk("[%02x]",reg_value);
			if(i == 14) printk("\n========================================\n");
		}
		for(i=0;i<15;i++)
		{
			if(i == 0) printk("hmts SENSOR PEAK:\n 0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F\n");
			printk("[%02x]",sensor_peak[i]);
			if(i == 14) printk("\n========================================\n");
		}
		for(i=0;i<15;i++)
		{
			reg = HM_RAW_VAL + i;
			err = hmts_i2c_read(data->client, &reg, 1, &reg_value, 1);
			if(i == 0) printk("hmts RAW VALUES:\n 0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F\n");
			printk("[%02x]",reg_value);
			if(i == 14) printk("\n========================================\n");
		}
#endif
	}
	input_mt_slot(ip_dev, id);

#if AXIS_EN
	if(fin & 1){
#if GESTURE_EN
		x = x*2 -100;
		y = 58;
#endif
		if(x < 0) x = 0;
		if(y < 0) y = 0;
		if(!poll_trigger){
			rep_x = x;
			rep_y = y;
		}else{
			if((x == rep_x)&&(y != rep_y)) x = x + 1;
			if((y == rep_y)&&(x != rep_x)) y = y + 1;
		}
		rep_x = x;
		rep_y = y;
	}

	if((fin & 1)/*&&(volumn_bar)*/){
		poll_trigger = true;
		input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 1);
#if MOUSE_MODE
		input_report_rel(ip_dev, REL_X, x);
		input_report_rel(ip_dev, REL_Y, y);
		printk("hmts mouse report x[%d] y[%d] tri:%d\n",x,y,poll_trigger);
#else
		input_report_abs(ip_dev, ABS_MT_POSITION_X, x);
		input_report_abs(ip_dev, ABS_MT_POSITION_Y, y);
		printk("hmts touch report x[%d] y[%d] tri:%d\n",x,y,poll_trigger);
#endif
	}else if(poll_trigger){
		poll_trigger = false;
		input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, false);
		leave_x = x;
		leave_y = y;
		volumn_bar = false;
		printk("hmts report ------------> leave lx:%x ly:%x\n",leave_x,leave_y);
	}else{
		volumn_bar = false;
		if((buf[0] & 2) != 2) goto quiet_exit;
	}

	input_mt_report_pointer_emulation(ip_dev, true);
//	input_sync(ip_dev);
	printk("hmts report ------------> SYNC\n");
#endif

#if GESTURE_EN
	if((buf[0] & 2)||(buf[4] > 0)){
		printk("eztest hmts----------->mode[%x] gesture[%x] diag[%x]\n",buf[0],buf[4],buf[5]);
		ges = buf[4];
		switch(ges){
			case 0x41:
			case 0x42:
//				input_report_key(ip_dev, KEY_RIGHT, 1);
				input_report_key(ip_dev, KEY_NEXTSONG, 1);
				input_sync(ip_dev);
//				input_report_key(ip_dev, KEY_RIGHT, 0);
				input_report_key(ip_dev, KEY_NEXTSONG, 0);
//				volumn_bar = true;
//				input_sync(ip_dev);
				break;
			case 0x61:
			case 0x62:
//				input_report_key(ip_dev, KEY_LEFT, 1);
				input_report_key(ip_dev, KEY_PREVIOUSSONG, 1);
				input_sync(ip_dev);
//				input_report_key(ip_dev, KEY_LEFT, 0);
				input_report_key(ip_dev, KEY_PREVIOUSSONG, 0);
//				volumn_bar = true;
//				input_sync(ip_dev);
				break;
			case 0x31:
			case 0x32:
//				if(!volumn_bar){
					input_report_key(ip_dev, KEY_VOLUMEDOWN, 1);
					input_sync(ip_dev);
					input_report_key(ip_dev, KEY_VOLUMEDOWN, 0);
					prv_ges = KEY_VOLUMEDOWN;
//				}
//				input_sync(ip_dev);
				volumn_bar = true;
				break;
			case 0x51:
			case 0x52:
//				if(!volumn_bar){
					input_report_key(ip_dev, KEY_VOLUMEUP, 1);
					input_sync(ip_dev);
					input_report_key(ip_dev, KEY_VOLUMEUP, 0);
					prv_ges = KEY_VOLUMEUP;
//				}
//				input_sync(ip_dev);
				volumn_bar = true;
				break;
			case 0x20:
			case 0x10:
			case 0x11:
				if(volumn_bar){
					input_report_key(ip_dev, prv_ges, 1);
					input_sync(ip_dev);
					input_report_key(ip_dev, prv_ges, 0);
				}
				break;
			default:
				volumn_bar = false;
				break;
				
		}
	}
#endif
	if(buf[0] & 0x3) input_sync(ip_dev);
	else volumn_bar = false;
quiet_exit:
	schedule_delayed_work(&data->hmts_work,
			msecs_to_jiffies(POLL_INTERVAL));
}
#endif // end of HMTS_POLL

static int hmts_gpio_configure(struct hmts_data *data, bool on)
{
	int err = 0;

	if (on) {
		
		if (gpio_is_valid(data->pdata->irq_gpio)) {
			err = gpio_request(data->pdata->irq_gpio,
						"hmts_irq_gpio");
			if (err) {
				dev_err(&data->client->dev,
					"irq gpio request failed");
				goto err_irq_gpio_req;
			}

			err = gpio_direction_input(data->pdata->irq_gpio);
			if (err) {
				dev_err(&data->client->dev,
					"set_direction for irq gpio failed\n");
				goto err_irq_gpio_dir;
			}
		}
		return 0;
	} else {

		if (gpio_is_valid(data->pdata->irq_gpio))
			gpio_free(data->pdata->irq_gpio);
		return 0;
	}

err_irq_gpio_dir:
	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:
	return err;

}

static int hmts_power_on(struct hmts_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
		}
	}

	return rc;
}

static int hmts_power_init(struct hmts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, HM_VTG_MIN_UV,
					   HM_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, HM_I2C_VTG_MIN_UV,
					   HM_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, HM_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, HM_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, HM_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

#if 0
#ifdef CONFIG_PM
static int hmts_start(struct device *dev)
{
	struct hmts_data *data = dev_get_drvdata(dev);
	int err;

	if (data->pdata->power_on) {
		err = data->pdata->power_on(true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	} else {
		err = hmts_power_on(data, true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	}

	err = hmts_gpio_configure(data, true);
	if (err < 0) {
		dev_err(&data->client->dev,
			"failed to put gpios in resue state\n");
		goto err_gpio_configuration;
	}


	return 0;

err_gpio_configuration:
	if (data->pdata->power_on) {
		err = data->pdata->power_on(false);
		if (err)
			dev_err(dev, "power off failed");
	} else {
		err = hmts_power_on(data, false);
		if (err)
			dev_err(dev, "power off failed");
	}
	return err;
}

static int hmts_stop(struct device *dev)
{
	struct hmts_data *data = dev_get_drvdata(dev);
	int i, err;

	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);


	if (data->pdata->power_on) {
		err = data->pdata->power_on(false);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	} else {
		err = hmts_power_on(data, false);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	}


	err = hmts_gpio_configure(data, false);
	if (err < 0) {
		dev_err(&data->client->dev,
			"failed to put gpios in suspend state\n");
		goto gpio_configure_fail;
	}

	data->suspended = true;

	return 0;

gpio_configure_fail:
	if (data->pdata->power_on) {
		err = data->pdata->power_on(true);
		if (err)
			dev_err(dev, "power on failed");
	} else {
		err = hmts_power_on(data, true);
		if (err)
			dev_err(dev, "power on failed");
	}
pwr_off_fail:
	return err;
}
#if 0
static int hmts_suspend(struct device *dev)
{
	struct hmts_data *data = dev_get_drvdata(dev);

	if (data->suspended) {
		dev_info(dev, "Already in suspend state\n");
		return 0;
	}

	return hmts_stop(dev);
}

static int hmts_resume(struct device *dev)
{
	struct hmts_data *data = dev_get_drvdata(dev);
	int err;

	if (!data->suspended) {
		dev_dbg(dev, "Already in awake state\n");
		return 0;
	}

	err = hmts_start(dev);
	if (err < 0)
		return err;

	return 0;
}
#endif
static const struct dev_pm_ops hmts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = hmts_suspend,
	.resume = hmts_resume,
#endif
};

#else
static int hmts_suspend(struct device *dev)
{
	return 0;
}

static int hmts_resume(struct device *dev)
{
	return 0;
}

#endif
#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void hmts_early_suspend(struct early_suspend *handler)
{
	struct hmts_data *data = container_of(handler,
						   struct hmts_data,
						   early_suspend);

	hmts_suspend(&data->client->dev);
}

static void hmts_late_resume(struct early_suspend *handler)
{
	struct hmts_data *data = container_of(handler,
						   struct hmts_data,
						   early_suspend);

	hmts_resume(&data->client->dev);
}
#endif


#ifdef CONFIG_OF
static int hmts_get_dt_coords(struct device *dev, char *name,
				struct hmts_platform_data *pdata)
{
	u32 coords[HM_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != 4) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

#if 0
	if (!strcmp(name, "borderless,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "borderless,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	}
#endif

	pdata->x_min = 0;
	pdata->y_min = 0;
	pdata->x_max = MAX_DX;
	pdata->y_max = MAX_DY;

	pdata->panel_minx = 0;//coords[0];
	pdata->panel_miny = 0;//coords[1];
	pdata->panel_maxx = MAX_XPOS;//coords[2]; org 511
	pdata->panel_maxy = MAX_YPOS;//coords[3]; org 447

	return 0;
}

static int hmts_parse_dt(struct device *dev,
			struct hmts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;
	pdata->name = "borderless";
	rc = of_property_read_string(np, "borderless,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}
	printk("hmts config_of 0 name:%s\n",pdata->name);

	rc = hmts_get_dt_coords(dev, "borderless,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	printk("hmts config_of 1 panel max<%d,%d> min<%d,%d>\n",pdata->panel_maxx,pdata->panel_maxy,pdata->panel_minx,pdata->panel_miny);
	rc = hmts_get_dt_coords(dev, "borderless,display-coords", pdata);
	if (rc)
		return rc;

	printk("hmts config_of 2 display max<%d,%d> min<%d,%d>\n",pdata->x_max,pdata->y_max,pdata->x_min,pdata->x_min);
	pdata->irq_gpio = IRQ_GPIO;/*of_get_named_gpio_flags(np, "borderless,irq-gpio",
				0, &pdata->irq_gpio_flags);*/
	printk("hmts config_of 3 hard code irq:%d\n",pdata->irq_gpio);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	pdata->i2c_pull_up = of_property_read_bool(np,
						"borderless,i2c-pull-up");


	rc = of_property_read_u32(np, "borderless,num-max-touches", &temp_val);
	printk("hmts config_of 4 rc:%d max touch:%d\n",rc,temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;
	printk("hmts config_of 5\n");

	pdata->mirror_h = of_property_read_bool(np,"borderless,mirror_h");
	pdata->mirror_v = of_property_read_bool(np,"borderless,mirror_v");


	return 0;
}
#else
static int hmts_parse_dt(struct device *dev,
			struct hmts_platform_data *pdata)
{
	printk("hmts no config_of\n");
	return -ENODEV;
}
#endif


static int hmts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct hmts_platform_data *pdata;
	struct hmts_data *data;
	struct input_dev *input_dev;
	int err;
//	int i;
	u8 reg_value;
	u8 reg_addr;
	
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct hmts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = hmts_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			return err;
		}
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev,
			sizeof(struct hmts_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	data->tch_data = devm_kzalloc(&client->dev,
				5, GFP_KERNEL);
	if (!data->tch_data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	data->input_dev = input_dev;
	data->client = client;
	data->pdata = pdata;

	input_dev->name = "hmts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	__set_bit(EV_KEY, input_dev->evbit);
#if MOUSE_MODE
//	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_REL, input_dev->evbit);
	__set_bit(REL_X, input_dev->relbit);
	__set_bit(REL_Y, input_dev->relbit);
//	__set_bit(REL_WHEEL, input_dev->relbit);
	__set_bit(BTN_MOUSE, input_dev->keybit);
#else
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
#endif
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	__set_bit(KEY_VOLUMEUP, input_dev->keybit);   
	__set_bit(KEY_VOLUMEDOWN, input_dev->keybit);   
	__set_bit(KEY_UP, input_dev->keybit);
	__set_bit(KEY_DOWN, input_dev->keybit);
	__set_bit(KEY_LEFT, input_dev->keybit);
	__set_bit(KEY_RIGHT, input_dev->keybit);
	__set_bit(KEY_PREVIOUSSONG, input_dev->keybit);
	__set_bit(KEY_NEXTSONG, input_dev->keybit);
	__set_bit(KEY_OK, input_dev->keybit);
	__set_bit(KEY_BACK, input_dev->keybit);

#if MOUSE_MODE
	input_mt_init_slots(input_dev, pdata->num_max_touches, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->panel_minx,
			     pdata->panel_maxx, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->panel_miny,
			     pdata->panel_maxy, 0, 0);
#else
	input_mt_init_slots(input_dev, pdata->num_max_touches, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->panel_minx,
			     pdata->panel_maxx, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->panel_miny,
			     pdata->panel_maxy, 0, 0);
#endif

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "Input device registration failed\n");
		input_free_device(input_dev);
		return err;
	}

	if (pdata->power_init) {
		err = pdata->power_init(true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	} else {
		err = hmts_power_init(data, true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	}

	if (pdata->power_on) {
		err = pdata->power_on(true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	} else {
		err = hmts_power_on(data, true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	}

	err = hmts_gpio_configure(data, true);
	if (err < 0) {
		dev_err(&client->dev,
			"Failed to configure the gpios\n");
	}


	/* check the controller id */
	reg_addr = HM_FW_MAJ;
	err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	printk("hmts FW MAJOR [%x]\n",reg_value);
	reg_addr = HM_FW_MIN;
	err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	printk("hmts FW MINOR [%x]\n",reg_value);
	reg_addr = HM_APPID_H;
	err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	printk("hmts APPID H [%x]\n",reg_value);
	reg_addr = HM_APPID_L;
	err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	printk("hmts APPID L [%x]\n",reg_value);
	if (err < 0) {
		dev_err(&client->dev, "device read failed");
		goto free_gpio;
	}
#if 0
	for(i=0;i<15;i++)
	{
		reg_addr = HM_SEN_VAL + i;
		err = hmts_write_reg(client, reg_addr, 0);
		reg_addr = HM_COMPE_RAM + i;
		err = hmts_write_reg(client, reg_addr, sensor_comp[i]);
	}
	for(i=0;i<15;i++)
	{
		reg_addr = HM_COMPE_RAM + i;
		err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
		if(i == 0) printk("hmts COMPENSATION RAM:\n 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
		printk("[%x]",reg_value);
		if(i == 14) printk("\n================================\n");
	}
	for(i=0;i<15;i++)
	{
		reg_addr = HM_SEN_VAL + i;
		err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
		if(i == 0) printk("hmts SENSOR VALUES:\n 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
		sensor_peak[i] = reg_value;
		printk("[%x]",reg_value);
		if(i == 14) printk("\n================================\n");
	}
#endif
#if 0
	dev_info(&client->dev, "eztest Device ID = 0x%x\n", reg_value);
	if ((pdata->family_id != reg_value) && (!pdata->ignore_id_check)) {
		dev_err(&client->dev, "%s:Unsupported controller\n", __func__);
		goto free_gpio;
	}
#endif

#if 0
	reg_addr = HM_SCAN_CNT;
	err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	printk("hmts HM_SCAN_CNT [%x]\n",reg_value);
	reg_addr = HM_X_CHAN;
	err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	printk("hmts HM_X_CHAN [%x]\n",reg_value);
	reg_addr = HM_Y_CHAN;
	err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	printk("hmts HM_Y_CHAN [%x]\n",reg_value);
	reg_addr = HM_HYSTER;
	err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	printk("hmts HM_HYSTER [%x]\n",reg_value);
	reg_addr = HM_FLT_TYP;
	err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	printk("hmts HM_FLT_TYP [%x]\n",reg_value);
	reg_addr = HM_FLT_STR;
	err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	printk("hmts HM_FLT_STR [%x]\n",reg_value);
	reg_addr = HM_HSWIP_DST;
	err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	printk("hmts HM_HSWIP_DST [%x]\n",reg_value);
	reg_addr = HM_VSWIP_DST;
	err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	printk("hmts HM_VSWIP_DST [%x]\n",reg_value);
#endif

	err = hmts_write_reg(client, HM_SCAN_CNT, 6);
	err = hmts_write_reg(client, HM_HYSTER, 3);// org 4
	err = hmts_write_reg(client, HM_X_CHAN, X_CHAN);// org 8
	err = hmts_write_reg(client, HM_Y_CHAN, Y_CHAN);// org 7
	err = hmts_write_reg(client, HM_HSWIP_DST, 48);// org 64
	err = hmts_write_reg(client, HM_VSWIP_DST, 40);// org 64
	err = hmts_write_reg(client, HM_CMD, 0x21);
	mdelay(100);
	reg_addr = HM_SCAN_CNT;
	err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	printk("hmts HM_SCAN_CNT read back[%x]\n",reg_value);
	reg_addr = HM_HYSTER;
	err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	printk("hmts HM_HYSTER read back[%x]\n",reg_value);
	reg_addr = HM_X_CHAN;
	err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	printk("hmts HM_X_CHAN back[%x]\n",reg_value);
	reg_addr = HM_Y_CHAN;
	err = hmts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	printk("hmts HM_Y_CHAN back[%x]\n",reg_value);

#if !HMTS_POLL
	err = request_threaded_irq(client->irq, NULL,
				hmts_interrupt,
	/*
	* the interrupt trigger mode will be set in Device Tree with property
	* "interrupts", so here we just need to set the flag IRQF_ONESHOT
	*/
				IRQF_ONESHOT,
				client->dev.driver->name, data);
#endif

#if defined(CONFIG_FB)
	data->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&data->fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						    HM_SUSPEND_LEVEL;
	data->early_suspend.suspend = hmts_early_suspend;
	data->early_suspend.resume = hmts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

#if HMTS_POLL// poll test
	INIT_DELAYED_WORK(&data->hmts_work, hmts_poll);
	msleep(100);
	schedule_delayed_work(&data->hmts_work,
			msecs_to_jiffies(POLL_INTERVAL));
#endif

	return 0;
free_gpio:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);

pwr_deinit:
	if (pdata->power_init)
		pdata->power_init(false);
	else
		hmts_power_init(data, false);
unreg_inputdev:
	input_unregister_device(input_dev);
	return err;
}

static int hmts_remove(struct i2c_client *client)
{
	struct hmts_data *data = i2c_get_clientdata(client);


#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif


	if (data->pdata->power_on)
		data->pdata->power_on(false);
	else
		hmts_power_on(data, false);

	if (data->pdata->power_init)
		data->pdata->power_init(false);
	else
		hmts_power_init(data, false);

	input_unregister_device(data->input_dev);

	return 0;
}

static const struct i2c_device_id hmts_id[] = {
	{"hmts", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, hmts_id);

#ifdef CONFIG_OF
static struct of_device_id hmts_match_table[] = {
	{ .compatible = "borderless,hmts",},
	{ },
};
#else
#define hmts_match_table NULL
#endif

static struct i2c_driver hmts_driver = {
	.probe = hmts_probe,
	.remove = hmts_remove,
	.driver = {
		   .name = "hmts",
		   .owner = THIS_MODULE,
		.of_match_table = hmts_match_table,
#ifdef CONFIG_PM
//		   .pm = &hmts_pm_ops,
#endif
		   },
	.id_table = hmts_id,
};

static int __init hmts_init(void)
{
	return i2c_add_driver(&hmts_driver);
}
module_init(hmts_init);

static void __exit hmts_exit(void)
{
	i2c_del_driver(&hmts_driver);
}
module_exit(hmts_exit);

MODULE_DESCRIPTION("Borderless HMTS TouchScreen driver");
MODULE_LICENSE("GPL v2");
