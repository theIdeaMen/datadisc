/*
 * Copyright (c) 2021 Griffin Adams
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rohm_bm1422agmv

#include <stdlib.h>
#include <kernel.h>
#include <string.h>
#include <init.h>

#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <drivers/i2c.h>

#include <logging/log.h>
#include <sys/printk.h>
#include <sys/__assert.h>

#include "bm1422.h"

LOG_MODULE_REGISTER(BM1422, CONFIG_SENSOR_LOG_LEVEL);

static int bm1422_reg_access(struct bm1422_data *ctx, uint8_t cmd,
							 uint8_t reg_addr, void *data, size_t length)
{
	int ret;

	if (cmd == BM1422_READ_REG)
	{
		return i2c_burst_read(ctx->bus, ctx->i2c_addr,
							  BM1422_TO_I2C_REG(reg),
							  (uint8_t *)data, length);
	}
	else
	{
		if (length != 1)
		{
			return -EINVAL;
		}

		return i2c_reg_write_byte(ctx->bus, ctx->i2c_addr,
								  BM1422_TO_I2C_REG(reg),
								  *(uint8_t *)data);
	}
}

/**
 * Read from device.
 * @param dev - The device structure.
 * @param read_buf - The register data.
 * @param register_address - The register address.
 * @param count - Number of bytes to read.
 * @return 0 in case of success, negative error code otherwise.
 */
static int bm1422_get_reg(const struct device *dev, uint8_t *read_buf,
						  uint8_t register_address, uint8_t count)
{
	struct bm1422_data *bm1422_data = dev->data;

	return bm1422_reg_access(bm1422_data,
							 BM1422_READ_REG,
							 register_address,
							 read_buf, count);
}

/**
 * Write to device register.
 * @param dev - The device structure.
 * @param register_value - The register data.
 * @param register_address - The register address.
 * @param count - Number of bytes to write.
 * @return 0 in case of success, negative error code otherwise.
 */
static int bm1422_set_reg(const struct device *dev,
						  uint16_t register_value,
						  uint8_t register_address, uint8_t count)
{
	struct bm1422_data *bm1422_data = dev->data;

	return bm1422_reg_access(bm1422_data,
							BM1422_WRITE_REG,
							register_address,
							&register_value,
							count);
}

/**
 * Write to device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int bm1422_reg_write_mask(const struct device *dev, uint8_t register_address,
						  uint8_t mask, uint8_t data)
{
	int ret;
	uint8_t tmp;

	ret = bm1422_get_reg(dev, &tmp, register_address, 1);
	if (ret)
	{
		return ret;
	}

	tmp &= ~mask;
	tmp |= data;

	return bm1422_set_reg(dev, tmp, register_address, 1);
}

#if defined(CONFIG_BM1422_TRIGGER)

/**
 * Get the status register data
 * @param dev - The device structure.
 * @param status - Data stored in the STATUS register
 * @return 0 in case of success, negative error code otherwise.
 */
int bm1422_get_status(const struct device *dev, uint8_t *status)
{
	return bm1422_get_reg(dev, status, BM1422_STA1, 1);
}

int bm1422_clear_interrupts(const struct device *dev)
{
	/* Reading magnetometer data clears the DRDY flag? */
	//return bm1422_get_reg(dev, &buf, BM1422_INT_REL, 1);
}
#endif

/**
 * Software reset.
 * @param dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
static int bm1422_software_reset(const struct device *dev)
{
	/* Writing to SRST resets the device */
	return bm1422_reg_write_mask(dev, BM1422_CNTL1,
								 BM1422_CNTL1_RST_LV_MSK,
								 BM1422_CNTL1_RST_LV_MODE(1));
}

/**
 * Set the mode of operation.
 * @param dev - The device structure.
 * @param val - Mode of operation.
 *		Accepted values: 
 * @return 0 in case of success, negative error code otherwise.
 */
static int bm1422_set_op_mode(const struct device *dev,
							  const struct sensor_value *val)
{
	return -1; // TODO: Write an runtime op mode change like odr
}

/**
 * Set Output data rate.
 * @param dev - The device structure.
 * @param val - Output data rate.
 *		Accepted values:  
 * @return 0 in case of success, negative error code otherwise.
 */
static int bm1422_set_odr(const struct device *dev,
						  const struct sensor_value *val)
{
	enum bm1422_odr odr;

	switch (val->val1)
	{
	case 10:
		odr = BM1422_ODR_10HZ;
		break;
	case 20:
		odr = BM1422_ODR_20HZ;
		break;
	case 100:
		odr = BM1422_ODR_100HZ;
		break;
	case 1000:
		odr = BM1422_ODR_1000HZ;
		break;
	default:
		return -EINVAL;
	}

	return bm1422_reg_write_mask(dev, BM1422_CNTL1,
								 BM1422_CNTL1_ODR_MSK,
								 BM1422_CNTL1_ODR_MODE(odr));
}

/**
 * Set full scale range.
 * @param dev  - The device structure.
 * @param val  - Full scale range.
 *		 Accepted values:  
 * @return 0 in case of success, negative error code otherwise.
 */
static int bm1422_set_ave(const struct device *dev,
						   const struct sensor_value *val)
{
	enum bm1422_ave ave;

	switch (val->val1)
	{
	case 1:
		ave = BM1422_1_TIMES;
		break;
	case 2:
		ave = BM1422_2_TIMES;
		break;
	case 4:
		ave = BM1422_4_TIMES;
		break;
	case 8:
		ave = BM1422_8_TIMES;
		break;
	case 16:
		ave = BM1422_16_TIMES;
		break;
	default:
		return -EINVAL;
	}

	return bm1422_reg_write_mask(dev, BM1422_AVE_A,
								 BM1422_AVE_A_MSK,
								 BM1422_AVE_A_MODE(ave));
}

static int bm1422_attr_set(const struct device *dev,
						   enum sensor_channel chan,
						   enum sensor_attribute attr,
						   const struct sensor_value *val)
{
	switch (attr)
	{
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return bm1422_set_odr(dev, val);
	case SENSOR_ATTR_FULL_SCALE:
		return bm1422_set_ave(dev, val);
	default:
		return -ENOTSUP;
	}
	// TODO: Figure out how to make custom attributes
}

static int bm1422_attr_get(const struct device *dev,
						   enum sensor_channel chan,
						   enum sensor_attribute attr,
						   struct sensor_value *val)
{
	switch (attr)
	{
	case 0:
	default:
		return -ENOTSUP;
	}
	// TODO: Figure out how to make custom attributes
}

static inline int bm1422_range_to_scale(enum bm1422_gsel range)
{
	/* See table 1 in specifications section of datasheet */
	switch (range)
	{
	case BM1422_GSEL_8G:
		return BM1422_ACCEL_8G_LSB_PER_G;
	case BM1422_GSEL_16G:
		return BM1422_ACCEL_16G_LSB_PER_G;
	case BM1422_GSEL_32G:
		return BM1422_ACCEL_32G_LSB_PER_G;
	case BM1422_GSEL_64G:
		return BM1422_ACCEL_64G_LSB_PER_G;
	default:
		return -EINVAL;
	}
}

static void bm1422_accel_convert(struct sensor_value *val, int accel,
								 enum bm1422_gsel range)
{
	/*
	 * Sensor output is 2's compliment, 
         *    FSR / (2^(bit resolution - 1)) per LSB
	 */
	int scale = bm1422_range_to_scale(range);
	long micro_ms2 = accel * SENSOR_G / scale;

	__ASSERT_NO_MSG(scale != -EINVAL);

	val->val1 = micro_ms2 / 1000000;
	val->val2 = micro_ms2 % 1000000;
}

static int bm1422_sample_fetch(const struct device *dev,
							   enum sensor_channel chan)
{
	struct bm1422_data *data = dev->data;
	uint8_t buf[6];
	int ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	ret = bm1422_get_reg(dev, (uint8_t *)buf, BM1422_INS1, 3);
	if (ret)
	{
		return ret;
	}
	data->tap_int = buf[0];
	data->func_int = buf[1];
	data->wkup_int = buf[2];

	ret = bm1422_get_reg(dev, (uint8_t *)buf, BM1422_XOUT_L, sizeof(buf));
	if (ret)
	{
		return ret;
	}

	/* 16-bit, big endien, 2's compliment */
	data->acc_x = (int16_t)(buf[1] << 8 | (buf[0] & 0xFF));
	data->acc_y = (int16_t)(buf[3] << 8 | (buf[2] & 0xFF));
	data->acc_z = (int16_t)(buf[5] << 8 | (buf[4] & 0xFF));

	return 0;
}

static int bm1422_channel_get(const struct device *dev,
							  enum sensor_channel chan,
							  struct sensor_value *val)
{
	struct bm1422_data *data = dev->data;

	switch (chan)
	{
	case SENSOR_CHAN_ACCEL_X:
		bm1422_accel_convert(val, data->acc_x, data->selected_range);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		bm1422_accel_convert(val, data->acc_y, data->selected_range);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		bm1422_accel_convert(val, data->acc_z, data->selected_range);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		for (size_t i = 0; i < 3; i++)
		{
			bm1422_accel_convert(&val[i], data->acc_xyz[i], data->selected_range);
		}
		break;
	case BM1422_SENSOR_CHAN_INT_SOURCE:
		val->val1 = data->func_int;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api bm1422_api_funcs = {
	.attr_set = bm1422_attr_set,
	.attr_get = bm1422_attr_get,
	.sample_fetch = bm1422_sample_fetch,
	.channel_get = bm1422_channel_get,
#ifdef CONFIG_BM1422_TRIGGER
	.trigger_set = bm1422_trigger_set,
#endif

};

enum bm1422_osa bm1422_get_kconfig_odr(void)
{
#ifdef CONFIG_BM1422_OSA_0HZ781
	return BM1422_OSA_0HZ781;
#elif CONFIG_BM1422_OSA_1HZ563
	return BM1422_OSA_1HZ563;
#elif CONFIG_BM1422_OSA_3HZ125
	return BM1422_OSA_3HZ125;
#elif CONFIG_BM1422_OSA_6HZ25
	return BM1422_OSA_6HZ25;
#elif CONFIG_BM1422_OSA_12HZ5
	return BM1422_OSA_12HZ5;
#elif CONFIG_BM1422_OSA_25HZ
	return BM1422_OSA_25HZ;
#elif CONFIG_BM1422_OSA_50HZ
	return BM1422_OSA_50HZ;
#elif CONFIG_BM1422_OSA_100HZ
	return BM1422_OSA_100HZ;
#elif CONFIG_BM1422_OSA_200HZ
	return BM1422_OSA_200HZ;
#elif CONFIG_BM1422_OSA_400HZ
	return BM1422_OSA_400HZ;
#elif CONFIG_BM1422_OSA_800HZ
	return BM1422_OSA_800HZ;
#elif CONFIG_BM1422_OSA_1600HZ
	return BM1422_OSA_1600HZ;
#elif CONFIG_BM1422_OSA_3200HZ
	return BM1422_OSA_3200HZ;
#elif CONFIG_BM1422_OSA_6400HZ
	return BM1422_OSA_6400HZ;
#endif
}

enum bm1422_gsel bm1422_get_kconfig_gsel(void)
{
#ifdef CONFIG_BM1422_RANGE_8G
	return BM1422_GSEL_8G;
#elif CONFIG_BM1422_RANGE_16G
	return BM1422_GSEL_16G;
#elif CONFIG_BM1422_RANGE_32G
	return BM1422_GSEL_32G;
#elif CONFIG_BM1422_RANGE_64G
	return BM1422_GSEL_64G;
#endif
}

enum bm1422_op_mode bm1422_get_kconfig_op_mode(void)
{
#ifdef CONFIG_BM1422_LOW_POWER_MODE
	return BM1422_LOW_POWER;
#elif CONFIG_BM1422_HIGH_PERF_MODE
	return BM1422_HI_PERFORMANCE;
#endif
}

#if defined(CONFIG_BM1422_TRIGGER)

uint8_t bm1422_get_kconfig_inc1(void)
{
	uint8_t pw1 = 0;

#ifdef CONFIG_BM1422_PW1_USEC
	pw1 = 0;
#elif CONFIG_BM1422_PW1_1XOSA
	pw1 = 1;
#elif CONFIG_BM1422_PW1_2XOSA
	pw1 = 2;
#elif CONFIG_BM1422_PW1_RTIME
	pw1 = 3;
#endif

	return (BM1422_INC1_PW1_MODE(pw1) | BM1422_INC1_IEN1_MODE(IS_ENABLED(CONFIG_BM1422_INT1)) |
			BM1422_INC1_IEA1_MODE(IS_ENABLED(CONFIG_BM1422_IEA1)) |
			BM1422_INC1_IEL1_MODE(IS_ENABLED(CONFIG_BM1422_IEL1)));
}

uint8_t bm1422_get_kconfig_inc2(void)
{

	return (BM1422_INC2_AIO_MODE(IS_ENABLED(CONFIG_BM1422_AIO)) |
			BM1422_INC2_XNWUE_MODE(IS_ENABLED(CONFIG_BM1422_XNWUE)) |
			BM1422_INC2_XPWUE_MODE(IS_ENABLED(CONFIG_BM1422_XPWUE)) |
			BM1422_INC2_YNWUE_MODE(IS_ENABLED(CONFIG_BM1422_YNWUE)) |
			BM1422_INC2_YPWUE_MODE(IS_ENABLED(CONFIG_BM1422_YPWUE)) |
			BM1422_INC2_ZNWUE_MODE(IS_ENABLED(CONFIG_BM1422_ZNWUE)) |
			BM1422_INC2_ZPWUE_MODE(IS_ENABLED(CONFIG_BM1422_ZPWUE)));
}

uint8_t bm1422_get_kconfig_inc3(void)
{

	return (BM1422_INC3_TMEN_MODE(IS_ENABLED(CONFIG_BM1422_TMEN)) |
			BM1422_INC3_TLEM_MODE(IS_ENABLED(CONFIG_BM1422_TLEM)) |
			BM1422_INC3_TRIM_MODE(IS_ENABLED(CONFIG_BM1422_TRIM)) |
			BM1422_INC3_TDOM_MODE(IS_ENABLED(CONFIG_BM1422_TDOM)) |
			BM1422_INC3_TUPM_MODE(IS_ENABLED(CONFIG_BM1422_TUPM)) |
			BM1422_INC3_TFDM_MODE(IS_ENABLED(CONFIG_BM1422_TFDM)) |
			BM1422_INC3_TFUM_MODE(IS_ENABLED(CONFIG_BM1422_TFUM)));
}

uint8_t bm1422_get_kconfig_inc4(void)
{

	return (BM1422_INC4_FFI1_MODE(IS_ENABLED(CONFIG_BM1422_FFI1)) |
			BM1422_INC4_BFI1_MODE(IS_ENABLED(CONFIG_BM1422_BFI1)) |
			BM1422_INC4_WMI1_MODE(IS_ENABLED(CONFIG_BM1422_WMI1)) |
			BM1422_INC4_DRDYI1_MODE(IS_ENABLED(CONFIG_BM1422_DRDYI1)) |
			BM1422_INC4_BTSI1_MODE(IS_ENABLED(CONFIG_BM1422_BTSI1)) |
			BM1422_INC4_TDTI1_MODE(IS_ENABLED(CONFIG_BM1422_TDTI1)) |
			BM1422_INC4_WUFI1_MODE(IS_ENABLED(CONFIG_BM1422_WUFI1)) |
			BM1422_INC4_TPI1_MODE(IS_ENABLED(CONFIG_BM1422_TPI1)));
}

uint8_t bm1422_get_kconfig_inc5(void)
{
	uint8_t pw2 = 0;

#ifdef CONFIG_BM1422_PW2_USEC
	pw2 = 0;
#elif CONFIG_BM1422_PW2_1XOSA
	pw2 = 1;
#elif CONFIG_BM1422_PW2_2XOSA
	pw2 = 2;
#elif CONFIG_BM1422_PW2_RTIME
	pw2 = 3;
#endif

	return (BM1422_INC5_PW2_MODE(pw2) | BM1422_INC5_IEN2_MODE(IS_ENABLED(CONFIG_BM1422_INT2)) |
			BM1422_INC5_IEA2_MODE(IS_ENABLED(CONFIG_BM1422_IEA2)) |
			BM1422_INC5_IEL2_MODE(IS_ENABLED(CONFIG_BM1422_IEL2)));
}

uint8_t bm1422_get_kconfig_inc6(void)
{

	return (BM1422_INC6_FFI2_MODE(IS_ENABLED(CONFIG_BM1422_FFI2)) |
			BM1422_INC6_BFI2_MODE(IS_ENABLED(CONFIG_BM1422_BFI2)) |
			BM1422_INC6_WMI2_MODE(IS_ENABLED(CONFIG_BM1422_WMI2)) |
			BM1422_INC6_DRDYI2_MODE(IS_ENABLED(CONFIG_BM1422_DRDYI2)) |
			BM1422_INC6_BTSI2_MODE(IS_ENABLED(CONFIG_BM1422_BTSI2)) |
			BM1422_INC6_TDTI2_MODE(IS_ENABLED(CONFIG_BM1422_TDTI2)) |
			BM1422_INC6_WUFI2_MODE(IS_ENABLED(CONFIG_BM1422_WUFI2)) |
			BM1422_INC6_TPI2_MODE(IS_ENABLED(CONFIG_BM1422_TPI2)));
}

#endif /* CONFIG_BM1422_TRIGGER */

static int bm1422_chip_init(const struct device *dev)
{
	struct bm1422_data *data = dev->data;
	int ret;

	/* Device settings from kconfig */
	ret = bm1422_reg_write_mask(dev, BM1422_ODCNTL,
								BM1422_ODCNTL_OUT_ODR_MSK,
								BM1422_ODCNTL_OUT_ODR_MODE(bm1422_get_kconfig_odr()));
	if (ret)
	{
		return ret;
	}

	ret = bm1422_reg_write_mask(dev, BM1422_CNTL1,
								BM1422_CNTL1_GSEL_MSK,
								BM1422_CNTL1_GSEL_MODE(bm1422_get_kconfig_gsel()));
	if (ret)
	{
		return ret;
	}

#if defined(CONFIG_BM1422_TRIGGER)
	data->int1_config = bm1422_get_kconfig_inc1();
	data->int1_source = bm1422_get_kconfig_inc4();

	data->int2_config = bm1422_get_kconfig_inc5();
	data->int2_source = bm1422_get_kconfig_inc6();

	ret = bm1422_set_reg(dev, data->int1_config, BM1422_INC1, 1);
	if (ret)
	{
		return ret;
	}
	ret = bm1422_set_reg(dev, bm1422_get_kconfig_inc2(), BM1422_INC2, 1);
	if (ret)
	{
		return ret;
	}
	ret = bm1422_set_reg(dev, bm1422_get_kconfig_inc3(), BM1422_INC3, 1);
	if (ret)
	{
		return ret;
	}
	ret = bm1422_set_reg(dev, data->int1_source, BM1422_INC4, 1);
	if (ret)
	{
		return ret;
	}
	ret = bm1422_set_reg(dev, data->int2_config, BM1422_INC5, 1);
	if (ret)
	{
		return ret;
	}
	ret = bm1422_set_reg(dev, data->int2_source, BM1422_INC6, 1);
	if (ret)
	{
		return ret;
	}

#if defined(CONFIG_BM1422_DTRE)
	ret = bm1422_set_reg(dev, CONFIG_BM1422_TDTC, BM1422_TDTC, 1);
	if (ret)
	{
		return ret;
	}
	ret = bm1422_set_reg(dev, CONFIG_BM1422_TTH, BM1422_TTH, 1);
	if (ret)
	{
		return ret;
	}
	ret = bm1422_set_reg(dev, CONFIG_BM1422_TTL, BM1422_TTL, 1);
	if (ret)
	{
		return ret;
	}
	ret = bm1422_set_reg(dev, CONFIG_BM1422_FTD, BM1422_FTD, 1);
	if (ret)
	{
		return ret;
	}
	ret = bm1422_set_reg(dev, CONFIG_BM1422_STD, BM1422_STD, 1);
	if (ret)
	{
		return ret;
	}
	ret = bm1422_set_reg(dev, CONFIG_BM1422_TLT, BM1422_TLT, 1);
	if (ret)
	{
		return ret;
	}
	ret = bm1422_set_reg(dev, CONFIG_BM1422_TWS, BM1422_TWS, 1);
	if (ret)
	{
		return ret;
	}
#endif /* CONFIG_BM1422_DTRE */

	if (bm1422_init_interrupt(dev) < 0)
	{
		LOG_ERR("Failed to initialize interrupt!");
		return -EIO;
	}
#endif

	ret = bm1422_reg_write_mask(dev, BM1422_CNTL1,
								BM1422_CNTL1_ACTIVE_MSK,
								BM1422_CNTL1_ACTIVE_MODE(bm1422_get_kconfig_op_mode()));
	if (ret)
	{
		return ret;
	}

	return 0;
}

static int bm1422_init(const struct device *dev)
{
	const struct bm1422_config *cfg = dev->config;
	struct bm1422_data *data = dev->data;
	uint8_t value[2];
	int err;

#ifdef CONFIG_BM1422_I2C
	data->bus = device_get_binding(cfg->i2c_port);
	if (data->bus == NULL)
	{
		LOG_ERR("Failed to get pointer to %s device!",
				cfg->i2c_port);
		return -EINVAL;
	}
#endif
#ifdef CONFIG_BM1422_SPI
	data->bus = device_get_binding(cfg->spi_port);
	if (!data->bus)
	{
		LOG_ERR("spi device not found: %s", cfg->spi_port);
		return -EINVAL;
	}
	/* CPOL=0, CPHA=0, max 10MHz */
	data->spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
	data->spi_cfg.frequency = cfg->spi_max_frequency;
	data->spi_cfg.slave = cfg->spi_slave;

	data->bm1422_cs_ctrl.gpio_dev = device_get_binding(cfg->gpio_cs_port);
	if (!data->bm1422_cs_ctrl.gpio_dev)
	{
		LOG_ERR("Unable to get GPIO SPI CS device");
		return -ENODEV;
	}

	data->bm1422_cs_ctrl.gpio_pin = cfg->cs_gpio;
	data->bm1422_cs_ctrl.gpio_dt_flags = cfg->cs_flags;
	data->bm1422_cs_ctrl.delay = 0U;

	data->spi_cfg.cs = &data->bm1422_cs_ctrl;
#endif /* CONFIG_BM1422_SPI */

	err = bm1422_software_reset(dev);

	if (err)
	{
		LOG_ERR("bm1422_software_reset failed, error %d\n", err);
		return -ENODEV;
	}
	k_sleep(K_MSEC(5));

	//while(1){

	bm1422_get_reg(dev, value, BM1422_PART_ID, sizeof(value));
	if (value[0] != BM1422_WHO_AM_I_VAL)
	{
		LOG_ERR("Failed Part-ID: %d\n", value[0]);
		return -ENODEV;
	}

	//k_sleep(K_MSEC(500));

	//}; // debuggin

	if (bm1422_chip_init(dev) < 0)
	{
		return -ENODEV;
	}

	return 0;
}

//static const struct bm1422_config bm1422_config = {
//#ifdef CONFIG_BM1422_I2C
//	.i2c_port = DT_INST_BUS_LABEL(0),
//	.i2c_addr = DT_INST_REG_ADDR(0),
//#endif
//#ifdef CONFIG_BM1422_SPI
//	.spi_port = DT_INST_BUS_LABEL(1),
//	.spi_slave = DT_INST_REG_ADDR(1),
//	.spi_max_frequency = DT_INST_PROP(1, spi_max_frequency),
//#if DT_INST_SPI_DEV_HAS_CS_GPIOS(1)
//	.gpio_cs_port = DT_INST_SPI_DEV_CS_GPIOS_LABEL(1),
//	.cs_gpio = DT_INST_SPI_DEV_CS_GPIOS_PIN(1),
//	.cs_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(1),
//#endif
//#endif
//#ifdef CONFIG_BM1422_TRIGGER
//	.gpio_port = DT_INST_GPIO_LABEL(1, irq_gpios),
//	.int_gpio = DT_INST_GPIO_PIN(1, irq_gpios),
//	.int_flags = DT_INST_GPIO_FLAGS(1, irq_gpios),
//        .int1_config = BM1422_INC1_IEN1_MODE(1) | BM1422_INC1_IEA1_MODE(1), // Enable and active high
//	.int2_config = BM1422_INC5_IEN2_MODE(0), // Disabled for now
//#endif

//#ifdef CONFIG_BM1422_OSA_50HZ
//	.osa = BM1422_OSA_50HZ,
//#elif CONFIG_BM1422_OSA_800HZ
//	.osa = BM1422_OSA_800HZ,
//#elif CONFIG_BM1422_OSA_1600HZ
//	.osa = BM1422_OSA_1600HZ,
//#elif CONFIG_BM1422_OSA_3200HZ
//	.osa = BM1422_OSA_3200HZ,
//#elif CONFIG_BM1422_OSA_6400HZ
//	.osa = BM1422_OSA_6400HZ,
//#endif

//        /* Device Settings */
//	.buff_config = BM1422_BM_STREAM,

//	.op         = BM1422_LOW_POWER,
//	.gsel       = BM1422_GSEL_64G,
//	.otp        = BM1422_OTP_1HZ563,
//	.otdt       = BM1422_OTDT_12HZ5,
//	.owuf       = BM1422_OWUF_0HZ781,
//	.obts       = BM1422_OBTS_0HZ781,
//	.offi       = BM1422_OFFI_12HZ5,
//	.avc        = BM1422_NO_AVG,
//	.rms_avc    = BM1422_RMS_AVG_2_SAMPLES,
//	.oadp       = BM1422_OADP_0HZ781,
//};

//DEVICE_DT_INST_DEFINE(0, bm1422_init, device_pm_control_nop,
//		    &bm1422_data, &bm1422_config, POST_KERNEL,
//		    CONFIG_SENSOR_INIT_PRIORITY, &bm1422_api_funcs);

/*
 * This instantiation macro is named "CREATE_MY_DEVICE".
 * Its "inst" argument is an arbitrary instance number.
 *
 * Put this near the end of the file, e.g. after defining "my_api_funcs".
 */
#define CREATE_MY_DEVICE(inst)                                      \
	static struct bm1422_data bm1422_data_##inst = {                \
		/* initialize RAM values as needed, e.g.: */                \
	};                                                              \
	static const struct bm1422_config bm1422_config_##inst = {      \
		/* initialize ROM values as needed. */                      \
		.spi_port = DT_INST_BUS_LABEL(inst),                        \
		.spi_slave = DT_INST_REG_ADDR(inst),                        \
		.spi_max_frequency = DT_INST_PROP(inst, spi_max_frequency), \
		.gpio_cs_port = DT_INST_SPI_DEV_CS_GPIOS_LABEL(inst),       \
		.cs_gpio = DT_INST_SPI_DEV_CS_GPIOS_PIN(inst),              \
		.cs_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(inst),           \
		.gpio_port = DT_INST_GPIO_LABEL(inst, irq_gpios),           \
		.int_gpio = DT_INST_GPIO_PIN(inst, irq_gpios),              \
		.int_flags = DT_INST_GPIO_FLAGS(inst, irq_gpios),           \
	};                                                              \
	DEVICE_DT_INST_DEFINE(inst,                                     \
						  bm1422_init,                              \
						  NULL,                                     \
						  &bm1422_data_##inst,                      \
						  &bm1422_config_##inst,                    \
						  POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
						  &bm1422_api_funcs);

/* Call the device creation macro for each instance: */
DT_INST_FOREACH_STATUS_OKAY(CREATE_MY_DEVICE)