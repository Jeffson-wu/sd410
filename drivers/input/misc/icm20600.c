#include "icm20600.h"
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>

/* Limit mininum delay to 10ms as we do not need higher rate so far */
#define ICM20600_ACCEL_MIN_POLL_INTERVAL_MS	10
#define ICM20600_ACCEL_MAX_POLL_INTERVAL_MS	5000
#define ICM20600_ACCEL_DEFAULT_POLL_INTERVAL_MS	200
#define ICM20600_ACCEL_INT_MAX_DELAY			19

#define ICM20600_GYRO_MIN_POLL_INTERVAL_MS	10
#define ICM20600_GYRO_MAX_POLL_INTERVAL_MS	5000
#define ICM20600_GYRO_DEFAULT_POLL_INTERVAL_MS	200
#define ICM20600_GYRO_INT_MAX_DELAY		18

struct icm20600_data {
	struct i2c_client *i2c;
	struct device *dev;
	struct hrtimer gyro_timer;
	struct hrtimer accel_timer;
	struct input_dev *accel_dev;
	struct input_dev *gyro_dev;
	struct sensors_classdev accel_cdev;
	struct sensors_classdev gyro_cdev;
	struct icm20600_platform_data *pdata;
	struct mutex op_lock;
#if 0
	enum inv_devices chip_type;
	struct workqueue_struct *data_wq;
	struct delayed_work fifo_flush_work;
	struct mpu_reg_map reg;
	struct mpu_chip_config cfg;
	struct axis_data axis;
	u32 gyro_poll_ms;
	u32 accel_poll_ms;
	u32 accel_latency_ms;
	u32 gyro_latency_ms;
	atomic_t accel_en;
	atomic_t gyro_en;
	bool use_poll;
	bool motion_det_en;
	bool batch_accel;
	bool batch_gyro;

	/* calibration */
	char acc_cal_buf[MPU_ACC_CAL_BUF_SIZE];
	int acc_cal_params[3];
	bool acc_use_cal;

	/* power control */
	struct regulator *vlogic;
	struct regulator *vdd;
	struct regulator *vi2c;
	int enable_gpio;
	bool power_enabled;

	/* pinctrl */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;

	u32 flush_count;
	u64 fifo_start_ns;
	int gyro_wkp_flag;
	int accel_wkp_flag;
	struct task_struct *gyr_task;
	struct task_struct *accel_task;
	bool gyro_delay_change;
	bool accel_delay_change;
	wait_queue_head_t	gyro_wq;
	wait_queue_head_t	accel_wq;
#endif
};

static struct icm20600_data *s_icm;

#if 0
/* Accelerometer information read by HAL */
static struct sensors_classdev icm20600_acc_cdev = {
	.name = "ICM20600-accel",
	.vendor = "Invensense",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "156.8",	/* m/s^2 */
	.resolution = "0.000598144",	/* m/s^2 */
	.sensor_power = "0.5",	/* 0.5 mA */
	.min_delay = ICM20600_ACCEL_MIN_POLL_INTERVAL_MS * 1000,
	.max_delay = ICM20600_ACCEL_MAX_POLL_INTERVAL_MS,
	.delay_msec = ICM20600_ACCEL_DEFAULT_POLL_INTERVAL_MS,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.max_latency = 0,
	.flags = 0, /* SENSOR_FLAG_CONTINUOUS_MODE */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_enable_wakeup = NULL,
	.sensors_set_latency = NULL,
	.sensors_flush = NULL,
};

/* gyroscope information read by HAL */
static struct sensors_classdev icm20600_gyro_cdev = {
	.name = "ICM20600-gyro",
	.vendor = "Invensense",
	.version = 1,
	.handle = SENSORS_GYROSCOPE_HANDLE,
	.type = SENSOR_TYPE_GYROSCOPE,
	.max_range = "34.906586",	/* rad/s */
	.resolution = "0.0010681152",	/* rad/s */
	.sensor_power = "3.6",	/* 3.6 mA */
	.min_delay = ICM20600_GYRO_MIN_POLL_INTERVAL_MS * 1000,
	.max_delay = ICM20600_GYRO_MAX_POLL_INTERVAL_MS,
	.delay_msec = ICM20600_ACCEL_DEFAULT_POLL_INTERVAL_MS,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.max_latency = 0,
	.flags = 0, /* SENSOR_FLAG_CONTINUOUS_MODE */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_enable_wakeup = NULL,
	.sensors_set_latency = NULL,
	.sensors_flush = NULL,
};
#endif

/***** I2C I/O function ***********************************************/
static int icm_i2c_rxdata(
	struct i2c_client *i2c,
	uint8_t *reg_addr,
	uint8_t *rxData,
	int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = reg_addr,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	ret = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
//	printk("icm20600 rx -------------> adr:%x reg:%x len:%d val:%x ret:%d\n",i2c->addr,reg_addr[0],length,rxData[0],ret);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&i2c->dev, "%s: transfer failed(size error).\n",
				__func__);
		return -ENXIO;
	}

	return 0;
}

static int icm_i2c_txdata(
	struct i2c_client *i2c,
	uint8_t *txData,
	int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
//	printk("icm20600 tx -------------> adr:%x reg:%x len:%d val:%x ret:%d\n",i2c->addr,txData[0],length,txData[1],ret);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msg)) {
		dev_err(&i2c->dev, "%s: transfer failed(size error).",
				__func__);
		return -ENXIO;
	}

	dev_vdbg(&i2c->dev, "TxData: len=%02x, addr=%02x data=%02x",
		length, txData[0], txData[1]);

	return 0;
}

uint8_t icm_readByte(uint8_t val) {
	uint8_t buf[2];
	int err;
	err = icm_i2c_rxdata(s_icm->i2c, &val, buf, 1);
    return buf[0];
}

uint8_t icm_writeByte(uint8_t addr, uint8_t val) {
	uint8_t buf[2];
	int err;
	buf[0] = addr;
	buf[1] = val;
	err = icm_i2c_txdata(s_icm->i2c, buf, sizeof(buf));
//	printk("icm20600 write byte ----------> addr:%x val:%x err:%d\n",addr,val,err);
    return err;
}

uint8_t ICM20600_getDeviceID(void) {
	uint8_t buf[2];
	int err;
	uint8_t reg = ICM20600_WHO_AM_I;
	err = icm_i2c_rxdata(s_icm->i2c, &reg, buf, 1);
    return buf[0];
}


void ICM20600_initialize(void) {
	int err;
    // configuration
	err = icm_writeByte(ICM20600_CONFIG, 0x00);
    // disable fifo
	err = icm_writeByte(ICM20600_FIFO_EN, 0x00);// tst 0x08

    // set default power mode
    ICM20600_setPowerMode(ICM_6AXIS_LOW_POWER);

    // gyro config
    ICM20600_setGyroScaleRange(RANGE_2K_DPS);
    ICM20600_setGyroOutputDataRate(GYRO_RATE_1K_BW_176);
    ICM20600_setGyroAverageSample(GYRO_AVERAGE_1);

    // accel config
    ICM20600_setAccScaleRange(RANGE_8G);
    ICM20600_setAccOutputDataRate(ACC_RATE_1K_BW_420);
    ICM20600_setAccAverageSample(ACC_AVERAGE_4);
	printk("icm20600 init ------------> done\n");
}


void ICM20600_setPowerMode(enum icm20600_power_type_t mode) {
	int err;
    uint8_t data_pwr1;
    uint8_t data_pwr2 = 0x00;
    uint8_t data_gyro_lp;
    data_pwr1 = icm_readByte(ICM20600_PWR_MGMT_1);
    data_pwr1 &= 0x8f;                  // 0b10001111
    data_gyro_lp = icm_readByte(ICM20600_GYRO_LP_MODE_CFG);
    // When set to ‘1’ low-power gyroscope mode is enabled. Default setting is 0
    data_gyro_lp &= 0x7f;               // 0b01111111
    switch (mode) {
        case ICM_SLEEP_MODE:
            data_pwr1 |= 0x40;          // set 0b01000000
            break;

        case ICM_STANDYBY_MODE:
            data_pwr1 |= 0x10;          // set 0b00010000
            data_pwr2 = 0x38;           // 0x00111000 disable acc
            break;

        case ICM_ACC_LOW_POWER:
            data_pwr1 |= 0x20;          // set bit5 0b00100000
            data_pwr2 = 0x07;           //0x00000111 disable gyro
            break;

        case ICM_ACC_LOW_NOISE:
            data_pwr1 |= 0x00;
            data_pwr2 = 0x07;           //0x00000111 disable gyro
            break;

        case ICM_GYRO_LOW_POWER:
            data_pwr1 |= 0x00;          // dont set bit5 0b00000000
            data_pwr2 = 0x38;           // 0x00111000 disable acc
            data_gyro_lp |= 0x80;
            break;

        case ICM_GYRO_LOW_NOISE:
            data_pwr1 |= 0x00;
            data_pwr2 = 0x38;           // 0x00111000 disable acc
            break;

        case ICM_6AXIS_LOW_POWER:
            data_pwr1 |= 0x00;          // dont set bit5 0b00100000 org: 0x00 tst:0x80
            data_gyro_lp |= 0x80;
            break;

        case ICM_6AXIS_LOW_NOISE:
            data_pwr1 |= 0x00;
            break;

        default:
            break;
    }
	mdelay(20);
//    printk("icm20600 set power write ---------> mgmt1:%x mgmt2:%x\n",data_pwr1,data_pwr2);
	err = icm_writeByte(ICM20600_PWR_MGMT_1, data_pwr1);
	mdelay(20);
	err = icm_writeByte(ICM20600_PWR_MGMT_2, data_pwr2);
	mdelay(20);
	err = icm_writeByte(ICM20600_GYRO_LP_MODE_CFG, data_gyro_lp);
	mdelay(20);
//	err = icm_writeByte(ICM20600_PWR_MGMT_1, 1);
}

// SAMPLE_RATE = 1KHz / (1 + div)
// work for low-power gyroscope and low-power accelerometer and low-noise accelerometer
void ICM20600_setSampleRateDivier(uint8_t div) {
	int err;
	err = icm_writeByte(ICM20600_SMPLRT_DIV, div);
}


void ICM20600_setAccScaleRange(enum acc_scale_type_t range) {
	int err;
    uint8_t data;
    data = icm_readByte(ICM20600_ACCEL_CONFIG);
    data &= 0xe7; // 0b 1110 0111

    switch (range) {
        case RANGE_2G:
            data |= 0x00;   // 0bxxx00xxx
            _acc_scale = 4000;
            break;

        case RANGE_4G:
            data |= 0x08;   // 0bxxx01xxx
            _acc_scale = 8000;
            break;

        case RANGE_8G:
            data |= 0x10;   // 0bxxx10xxx
            _acc_scale = 16000;
            break;

        case RANGE_16G:
            data |= 0x18;   // 0bxxx11xxx
            _acc_scale = 32000;
            break;

        default:
            break;
    }

	err = icm_writeByte(ICM20600_ACCEL_CONFIG, data);
}


// for low power mode only
void ICM20600_setAccAverageSample(enum acc_averaging_sample_type_t sample) {
	int err;
    uint8_t data = 0;
    data = icm_readByte(ICM20600_ACCEL_CONFIG2);

    data &= 0xcf; // & 0b11001111
    switch (sample) {
        case ACC_AVERAGE_4:
            data |= 0x00; // 0bxx00xxxx
            break;

        case ACC_AVERAGE_8:
            data |= 0x10; // 0bxx01xxxx
            break;

        case ACC_AVERAGE_16:
            data |= 0x20; // 0bxx10xxxx
            break;

        case ACC_AVERAGE_32:
            data |= 0x30; // 0bxx11xxxx
            break;

        default:
            break;
    }

	err = icm_writeByte(ICM20600_ACCEL_CONFIG2, data);
}


void ICM20600_setAccOutputDataRate(enum acc_lownoise_odr_type_t odr) {
	int err;
    uint8_t data;
    data = icm_readByte(ICM20600_ACCEL_CONFIG2);
    data &= 0xf0;  // 0b11110000

    switch (odr) {
        case ACC_RATE_4K_BW_1046:
            data |= 0x08;
            break;

        case ACC_RATE_1K_BW_420:
            data |= 0x07;
            break;

        case ACC_RATE_1K_BW_218:
            data |= 0x01;
            break;

        case ACC_RATE_1K_BW_99:
            data |= 0x02;
            break;

        case ACC_RATE_1K_BW_44:
            data |= 0x03;
            break;

        case ACC_RATE_1K_BW_21:
            data |= 0x04;
            break;

        case ACC_RATE_1K_BW_10:
            data |= 0x05;
            break;

        case ACC_RATE_1K_BW_5:
            data |= 0x06;
            break;

        default:
            break;
    }

	err = icm_writeByte(ICM20600_ACCEL_CONFIG2, data);
}


void ICM20600_setGyroScaleRange(enum gyro_scale_type_t range) {
	int err;
    uint8_t data = 0;
    data = icm_readByte(ICM20600_GYRO_CONFIG);
    data &= 0xe7; // 0b11100111

    switch (range) {
        case RANGE_250_DPS:
            data |= 0x00;   // 0bxxx00xxx
            _gyro_scale = 500;
            break;

        case RANGE_500_DPS:
            data |= 0x08;   // 0bxxx00xxx
            _gyro_scale = 1000;
            break;

        case RANGE_1K_DPS:
            data |= 0x10;   // 0bxxx10xxx
            _gyro_scale = 2000;
            break;

        case RANGE_2K_DPS:
            data |= 0x18;   // 0bxxx11xxx
            _gyro_scale = 4000;
            break;

        default:
            break;
    }

	err = icm_writeByte(ICM20600_GYRO_CONFIG, data);
}


// for low power mode only
void ICM20600_setGyroAverageSample(enum gyro_averaging_sample_type_t sample) {
    uint8_t data = 0;
    data = icm_readByte(ICM20600_GYRO_LP_MODE_CFG);

    data &= 0x8f;           // 0b10001111
    switch (sample) {
        case GYRO_AVERAGE_1:
            data |= 0x00; // 0bx000xxxx
            break;

        case GYRO_AVERAGE_2:
            data |= 0x10; // 0bx001xxxx
            break;

        case GYRO_AVERAGE_4:
            data |= 0x20; // 0bx010xxxx
            break;

        case GYRO_AVERAGE_8:
            data |= 0x30; // 0bx011xxxx
            break;

        case GYRO_AVERAGE_16:
            data |= 0x40; // 0bx100xxxx
            break;

        case GYRO_AVERAGE_32:
            data |= 0x50; // 0bx101xxxx
            break;

        case GYRO_AVERAGE_64:
            data |= 0x60;
            break;

        case GYRO_AVERAGE_128:
            data |= 0x70;
            break;


        default:
            break;
    }

	icm_writeByte(ICM20600_GYRO_LP_MODE_CFG, data);
}



void ICM20600_setGyroOutputDataRate(enum gyro_lownoise_odr_type_t odr) {
    uint8_t data;
    data = icm_readByte(ICM20600_CONFIG);
    data &= 0xf8;  // DLPF_CFG[2:0] 0b11111000

    switch (odr) {
        case GYRO_RATE_8K_BW_3281:
            data |= 0x07;
            break;
        case GYRO_RATE_8K_BW_250:
            data |= 0x00;
            break;
        case GYRO_RATE_1K_BW_176:
            data |= 0x01;
            break;
        case GYRO_RATE_1K_BW_92:
            data |= 0x02;
            break;
        case GYRO_RATE_1K_BW_41:
            data |= 0x03;
            break;
        case GYRO_RATE_1K_BW_20:
            data |= 0x04;
            break;
        case GYRO_RATE_1K_BW_10:
            data |= 0x05;
            break;
        case GYRO_RATE_1K_BW_5:
            data |= 0x06;
            break;
    }

	icm_writeByte(ICM20600_CONFIG, data);
}

void ICM20600_getAcceleration(int16_t* x, int16_t* y, int16_t* z) {
    *x = ICM20600_getAccelerationX();
    *y = ICM20600_getAccelerationY();
    *z = ICM20600_getAccelerationZ();
}

int16_t ICM20600_getAccelerationX(void) {
    int32_t raw_data = ICM20600_getRawAccelerationX();
    raw_data = (raw_data * _acc_scale) >> 16;
    return (int16_t)raw_data;
}
int16_t ICM20600_getAccelerationY(void) {
    int32_t raw_data = ICM20600_getRawAccelerationY();
    raw_data = (raw_data * _acc_scale) >> 16;
    return (int16_t)raw_data;
}
int16_t ICM20600_getAccelerationZ(void) {
    int32_t raw_data = ICM20600_getRawAccelerationZ();
    raw_data = (raw_data * _acc_scale) >> 16;
    return (int16_t)raw_data;
}

int16_t ICM20600_getRawAccelerationX(void) {
	uint8_t buf[2];
//	int err;
//	uint8_t reg = ICM20600_ACCEL_XOUT_H;
//	err = icm_i2c_rxdata(s_icm->i2c, &reg, buf, 2);
    buf[0] = icm_readByte(ICM20600_ACCEL_XOUT_H);
    buf[1] = icm_readByte(ICM20600_ACCEL_XOUT_L);
    return ((int16_t)buf[0] << 8) + buf[1];
}

int16_t ICM20600_getRawAccelerationY(void) {
	uint8_t buf[2];
//	int err;
//	uint8_t reg = ICM20600_ACCEL_YOUT_H;
//	err = icm_i2c_rxdata(s_icm->i2c, &reg, buf, 2);
    buf[0] = icm_readByte(ICM20600_ACCEL_YOUT_H);
    buf[1] = icm_readByte(ICM20600_ACCEL_YOUT_L);
    return ((int16_t)buf[0] << 8) + buf[1];
}

int16_t ICM20600_getRawAccelerationZ(void) {
	uint8_t buf[2];
//	int err;
//	uint8_t reg = ICM20600_ACCEL_ZOUT_H;
//	err = icm_i2c_rxdata(s_icm->i2c, &reg, buf, 2);
    buf[0] = icm_readByte(ICM20600_ACCEL_ZOUT_H);
    buf[1] = icm_readByte(ICM20600_ACCEL_ZOUT_L);
    return ((int16_t)buf[0] << 8) + buf[1];
}

void ICM20600_getGyroscope(int16_t* x, int16_t* y, int16_t* z) {
    *x = ICM20600_getGyroscopeX();
    *y = ICM20600_getGyroscopeY();
    *z = ICM20600_getGyroscopeZ();
}

int16_t ICM20600_getGyroscopeX(void) {
    int32_t raw_data = ICM20600_getRawGyroscopeX();
    raw_data = (raw_data * _gyro_scale) >> 16;
    return (int16_t)raw_data;
}

int16_t ICM20600_getGyroscopeY(void) {
    int32_t raw_data = ICM20600_getRawGyroscopeY();
    raw_data = (raw_data * _gyro_scale) >> 16;
    return (int16_t)raw_data;
}

int16_t ICM20600_getGyroscopeZ(void) {
    int32_t raw_data = ICM20600_getRawGyroscopeZ();
    raw_data = (raw_data * _gyro_scale) >> 16;
    return (int16_t)raw_data;
}

int16_t ICM20600_getRawGyroscopeX(void) {
	uint8_t buf[2];
	int err;
	uint8_t reg = ICM20600_GYRO_XOUT_H;
	err = icm_i2c_rxdata(s_icm->i2c, &reg, buf, 2);
    return ((int16_t)buf[0] << 8) + buf[1];
}

int16_t ICM20600_getRawGyroscopeY(void) {
	uint8_t buf[2];
	int err;
	uint8_t reg = ICM20600_GYRO_YOUT_H;
	err = icm_i2c_rxdata(s_icm->i2c, &reg, buf, 2);
    return ((int16_t)buf[0] << 8) + buf[1];
}

int16_t ICM20600_getRawGyroscopeZ(void) {
	uint8_t buf[2];
	int err;
	uint8_t reg = ICM20600_GYRO_ZOUT_H;
	err = icm_i2c_rxdata(s_icm->i2c, &reg, buf, 2);
    return ((int16_t)buf[0] << 8) + buf[1];
}

int16_t ICM20600_getRegVal(void) {
	uint8_t buf[16];
#if 1
	int err,i;
	uint8_t reg = ICM20600_SIGNAL_PATH_RESET;
	err = icm_i2c_rxdata(s_icm->i2c, &reg, buf, 16);
	printk("icm20600 ********** reg get**********\n");
	for(i=0;i<16;i++){
		printk("[%02d-%02x]",ICM20600_SIGNAL_PATH_RESET+i,buf[i]);
	}
	printk("\n********************************\n");
#else
    buf[0] = icm_readByte(ICM20600_PWR_MGMT_1);
    buf[1] = icm_readByte(ICM20600_PWR_MGMT_2);
#endif
    return ((int16_t)buf[0] << 8) + buf[1];
}

int16_t ICM20600_getTemperature(void) {
	uint8_t buf[2];
    uint16_t rawdata;
//	int err;
//	uint8_t reg = ICM20600_TEMP_OUT_H;
//	err = icm_i2c_rxdata(s_icm->i2c, &reg, buf, 2);
    buf[0] = icm_readByte(ICM20600_TEMP_OUT_H);
    buf[1] = icm_readByte(ICM20600_TEMP_OUT_L);
    rawdata = (((uint16_t)buf[0]) << 8) + buf[1];
    return (int16_t)(rawdata / 327 + 25);
}

void ICM20600_reset(void) {
    uint8_t data;
    data = icm_readByte(ICM20600_USER_CTRL);
    data &= 0xfe;  // ICM20600_USER_CTRL[0] 0b11111110
//    data |= 0x45;//ICM20600_RESET_BIT;
	printk("icm20600 reset write------------> %x\n",data);
	icm_writeByte(ICM20600_USER_CTRL, data);
	printk("icm20600 reset ------------> done rb:%x\n",icm_readByte(ICM20600_USER_CTRL));
}


int icm20600_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
//	int i;
	s_icm = kzalloc(sizeof(struct icm20600_data), GFP_KERNEL);

	s_icm->i2c = client;
	/* set client data */
	i2c_set_clientdata(client, s_icm);
	
	ICM20600_reset();
	mdelay(100);
	ICM20600_initialize();
	mdelay(100);

	ret = ICM20600_getDeviceID();
	printk("icm20600 probe get id: %x\n",ret);
	ICM_init_done = ret;
	return 0;
}

static int icm20600_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id icm20600_id[] = {
	{"icm20600", 0 },
	{ }
};
/*
static const struct dev_pm_ops icm20600_pm_ops = {
	.suspend	= icm20600_suspend,
	.resume		= icm20600_resume,
};
*/
static struct of_device_id icm20600_match_table[] = {
	{ .compatible = "invensense,icm20600", },
	{ },
};

static struct i2c_driver icm20600_driver = {
	.probe		= icm20600_probe,
	.remove		= icm20600_remove,
	.id_table	= icm20600_id,
	.driver = {
		.name	= "icm20600",
		.owner  = THIS_MODULE,
		.of_match_table = icm20600_match_table,
//		.pm		= &icm20600_pm_ops,
	},
};

static int __init icm20600_init(void)
{
	pr_info("AKM compass driver: initialize.");
	return i2c_add_driver(&icm20600_driver);
}

static void __exit icm20600_exit(void)
{
	pr_info("AKM compass driver: release.");
	i2c_del_driver(&icm20600_driver);
}

module_init(icm20600_init);
module_exit(icm20600_exit);

MODULE_AUTHOR("viral wang <viral_wang@htc.com>");
MODULE_DESCRIPTION("ICM20600 driver");
MODULE_LICENSE("GPL");


