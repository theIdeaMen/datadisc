/*
 * Copyright (c) 2021 Griffin Adams
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT kionix_kx134_1211

#include <init.h>
#include <kernel.h>
#include <stdlib.h>
#include <string.h>

#include <drivers/sensor.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <drivers/i2c.h>

#include <logging/log.h>
#include <sys/__assert.h>
#include <sys/printk.h>

#include "kx134.h"

LOG_MODULE_REGISTER(KX134, CONFIG_SENSOR_LOG_LEVEL);


/**
 * Register access.
 * @param dev - The device structure.
 * @param cmd - Read or write.
 * @param reg_addr - The register address.
 * @param data - Data buffer.
 * @param length - Number of bytes to read.
 * @return 0 in case of success, negative error code otherwise.
 */
static int kx134_reg_access(const struct device *dev, uint8_t cmd,
							uint8_t reg_addr, void *data, size_t length)
{
	int ret;
	const struct kx134_config *cfg = dev->config;

#ifdef CONFIG_KX134_SPI
	if (cmd == KX134_READ_REG)
	{

		uint8_t buffer_tx[2] = {reg_addr | 0x80};
		const struct spi_buf tx_buf = {
			.buf = buffer_tx,
			.len = 1,
		};
		const struct spi_buf_set tx = {
			.buffers = &tx_buf,
			.count = 1};
		const struct spi_buf rx_buf[2] = {
			{
				.buf = NULL,
				.len = 1,
			},
			{
				.buf = data,
				.len = length,
			}};
		const struct spi_buf_set rx = {
			.buffers = rx_buf,
			.count = 2};

		if (length > 64)
		{
			return -EIO;
		}
		ret = spi_transceive_dt(&cfg->spi, &tx, &rx);

		LOG_DBG("Read %d bytes starting with 0x%02X from 0x%02X", length, *((uint8_t *)data), reg_addr);

		return ret;
	}

	uint8_t buffer_tx[1] = {reg_addr};
	const struct spi_buf tx_buf[2] = {
		{
			.buf = buffer_tx,
			.len = 1,
		},
		{
			.buf = data,
			.len = length,
		}};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2};

	if (length > 64)
	{
		return -EIO;
	}

	ret = spi_write_dt(&cfg->spi, &tx);

	LOG_DBG("Wrote 0x%02X to 0x%02X", *((uint8_t *)data), reg_addr);

	return ret;

#elif CONFIG_KX134_I2C
	if (cmd == KX134_READ_REG)
	{
		return i2c_burst_read_dt(cfg->i2c,
								 BM1422_TO_I2C_REG(reg_addr),
								 (uint8_t *)data, length);
	}
	else
	{
		if (length != 1)
		{
			return -EINVAL;
		}

		return i2c_reg_write_byte_dt(cfg->i2c,
									 BM1422_TO_I2C_REG(reg_addr),
									 *(uint8_t *)data);
	}

#endif
}

/**
 * Read from device.
 * @param dev - The device structure.
 * @param read_buf - The register data.
 * @param register_address - The register address.
 * @param count - Number of bytes to read.
 * @return 0 in case of success, negative error code otherwise.
 */
int kx134_get_reg(const struct device *dev, uint8_t *read_buf,
				  uint8_t register_address, uint8_t count)
{
	return kx134_reg_access(dev,
				  KX134_READ_REG,
				  register_address,
				  read_buf, count);
}

/**
 * Write to device register.
 * @param dev - The device structure.
 * @param register_value - The register data.
 * @param register_address - The register address.
 * @param count - Number of bytes to read.
 * @return 0 in case of success, negative error code otherwise.
 */
int kx134_set_reg(const struct device *dev,
				  uint16_t register_value,
				  uint8_t register_address, uint8_t count)
{
	int ret;
	uint8_t cntl1_before;
	uint8_t zero = 0;

	// KX134 needs to be in standby for write access
	ret = kx134_get_reg(dev, &cntl1_before, KX134_CNTL1, 1);
	if (ret)
	{
		return ret;
	}

	ret = kx134_reg_access(dev,
						   KX134_WRITE_REG,
						   KX134_CNTL1, &zero, 1);
	if (ret)
	{
		return ret;
	}

	ret = kx134_reg_access(dev,
						   KX134_WRITE_REG,
						   register_address,
						   &register_value,
						   count);
	if (ret)
	{
		return ret;
	}

	if (register_address == KX134_CNTL1)
	{
		return ret;
	}

	// Put CNTL1 back to its previous value
	return kx134_reg_access(dev,
							KX134_WRITE_REG,
							KX134_CNTL1, &cntl1_before, 1);
}

/**
 * SPI write to device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int kx134_reg_write_mask(const struct device *dev, uint8_t register_address,
			   uint8_t mask, uint8_t data)
{
	int ret;
	uint8_t tmp;

	ret = kx134_get_reg(dev, &tmp, register_address, 1);
	if (ret) {
		return ret;
	}

	tmp &= ~mask;
	tmp |= data;

	return kx134_set_reg(dev, tmp, register_address, 1);
}


#if defined(CONFIG_KX134_TRIGGER)

/**
 * Get the status register data
 * @param dev - The device structure.
 * @param status - Data stored in the STATUS register
 * @return 0 in case of success, negative error code otherwise.
 */
int kx134_get_status(const struct device *dev, uint8_t *status)
{
        return kx134_get_reg(dev, status, KX134_STATUS_REG, 1);
}

int kx134_clear_interrupts(const struct device *dev)
{
	uint8_t buf;
	/* Reading INT_REL register clears the interrupt flags */
	return kx134_get_reg(dev, &buf, KX134_INT_REL, 1);
}
#endif

/**
 * Software reset.
 * @param dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
static int kx134_software_reset(const struct device *dev)
{
	/* Writing to SRST resets the device */
	return kx134_reg_write_mask(dev, KX134_CNTL2, 
					KX134_CNTL2_SOFT_RST_MSK, 
					KX134_CNTL2_SOFT_RST_MODE(1));
}

/**
 * Set the mode of operation.
 * @param dev - The device structure.
 * @param val - Mode of operation.
 *		Accepted values: 
 * @return 0 in case of success, negative error code otherwise.
 */
static int kx134_set_op_mode(const struct device *dev,
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
static int kx134_set_odr(const struct device *dev,
				const struct sensor_value *val)
{
	enum kx134_osa odr;

	switch (val->val1) {
	case 0:
		odr = KX134_OSA_0HZ781;
		break;
	case 1:
		odr = KX134_OSA_1HZ563;
		break;
	case 3:
		odr = KX134_OSA_3HZ125;
		break;
	case 6:
		odr = KX134_OSA_6HZ25;
		break;
	case 12:
		odr = KX134_OSA_12HZ5;
		break;
        case 25:
		odr = KX134_OSA_25HZ;
		break;
	case 50:
		odr = KX134_OSA_50HZ;
		break;
	case 100:
		odr = KX134_OSA_100HZ;
		break;
	case 200:
		odr = KX134_OSA_200HZ;
		break;
        case 400:
		odr = KX134_OSA_400HZ;
		break;
	case 800:
		odr = KX134_OSA_800HZ;
		break;
	case 1600:
		odr = KX134_OSA_1600HZ;
		break;
	case 3200:
		odr = KX134_OSA_3200HZ;
		break;
	case 6400:
		odr = KX134_OSA_6400HZ;
		break;
	default:
		return -EINVAL;
	}

	return kx134_reg_write_mask(dev, KX134_ODCNTL,
				      KX134_ODCNTL_OUT_ODR_MSK,
				      KX134_ODCNTL_OUT_ODR_MODE(odr));
}

/**
 * Set full scale range.
 * @param dev  - The device structure.
 * @param val  - Full scale range.
 *		 Accepted values:  
 * @return 0 in case of success, negative error code otherwise.
 */
static int kx134_set_gsel(const struct device *dev,
				const struct sensor_value *val)
{
	enum kx134_gsel gsel;

	switch (val->val1) {
	case 8:
		gsel = KX134_GSEL_8G;
		break;
	case 16:
		gsel = KX134_GSEL_16G;
		break;
	case 32:
		gsel = KX134_GSEL_32G;
		break;
	case 64:
		gsel = KX134_GSEL_64G;
		break;
	default:
		return -EINVAL;
	}

	return kx134_reg_write_mask(dev, KX134_ODCNTL,
				      KX134_CNTL1_GSEL_MSK,
				      KX134_CNTL1_GSEL_MODE(gsel));
}


static int kx134_attr_set(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return kx134_set_odr(dev, val);
	case SENSOR_ATTR_FULL_SCALE:
                return kx134_set_gsel(dev, val);
	default:
		return -ENOTSUP;
	}
        // TODO: Figure out how to make custom attributes
}

static int kx134_attr_get(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    struct sensor_value *val)
{
	switch (attr) {
	case 0:
	default:
		return -ENOTSUP;
	}
        // TODO: Figure out how to make custom attributes
}


static inline int kx134_range_to_scale(enum kx134_gsel range)
{
	/* See table 1 in specifications section of datasheet */
	switch (range) {
	case KX134_GSEL_8G:
		return KX134_ACCEL_8G_LSB_PER_G;
	case KX134_GSEL_16G:
		return KX134_ACCEL_16G_LSB_PER_G;
	case KX134_GSEL_32G:
		return KX134_ACCEL_32G_LSB_PER_G;
        case KX134_GSEL_64G:
		return KX134_ACCEL_64G_LSB_PER_G;
	default:
		return -EINVAL;
	}
}

static void kx134_accel_convert(struct sensor_value *val, int accel,
				  enum kx134_gsel range)
{
	/*
	 * Sensor output is 2's compliment, 
         *    FSR / (2^(bit resolution - 1)) per LSB
	 */
        int scale = kx134_range_to_scale(range);
	long micro_ms2 = accel * SENSOR_G / scale;

	__ASSERT_NO_MSG(scale != -EINVAL);

	val->val1 = micro_ms2 / 1000000;
	val->val2 = micro_ms2 % 1000000;
}

static int kx134_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct kx134_data *data = dev->data;
	uint8_t buf[6];
	int ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

 //       ret = kx134_get_reg(dev, (uint8_t *)buf, KX134_INS1, 3);
 //       if (ret) {
	//	return ret;
	//}
 //       data->tap_int = buf[0];
 //       data->func_int = buf[1];
 //       data->wkup_int = buf[2];


	ret = kx134_get_reg(dev, (uint8_t *)buf, KX134_XOUT_L, sizeof(buf));
	if (ret) {
		return ret;
	}

        /* 16-bit, big endien, 2's compliment */
	data->acc_x = (int16_t)(buf[1] << 8 | (buf[0] & 0xFF));
	data->acc_y = (int16_t)(buf[3] << 8 | (buf[2] & 0xFF));
	data->acc_z = (int16_t)(buf[5] << 8 | (buf[4] & 0xFF));

	return 0;
}

static int kx134_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct kx134_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		kx134_accel_convert(val, data->acc_x, data->selected_range);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		kx134_accel_convert(val, data->acc_y, data->selected_range);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		kx134_accel_convert(val, data->acc_z, data->selected_range);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		for (size_t i = 0; i < 3; i++) {
			kx134_accel_convert(&val[i], data->acc_xyz[i], data->selected_range);
		}
		break;
    case KX134_SENSOR_CHAN_INT_SOURCE:
        val->val1 = data->func_int;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api kx134_api_funcs = {
    .attr_set     = kx134_attr_set,
    .attr_get     = kx134_attr_get,
	.sample_fetch = kx134_sample_fetch,
	.channel_get  = kx134_channel_get,
#ifdef CONFIG_KX134_TRIGGER
	.trigger_set = kx134_trigger_set,
#endif

};

enum kx134_osa kx134_get_kconfig_odr(void) {
  #ifdef CONFIG_KX134_OSA_0HZ781
         return KX134_OSA_0HZ781;
  #elif CONFIG_KX134_OSA_1HZ563
         return KX134_OSA_1HZ563;
  #elif CONFIG_KX134_OSA_3HZ125
         return KX134_OSA_3HZ125;
  #elif CONFIG_KX134_OSA_6HZ25
         return KX134_OSA_6HZ25;
  #elif CONFIG_KX134_OSA_12HZ5
         return KX134_OSA_12HZ5;
  #elif CONFIG_KX134_OSA_25HZ
         return KX134_OSA_25HZ;
  #elif CONFIG_KX134_OSA_50HZ
         return KX134_OSA_50HZ;
  #elif CONFIG_KX134_OSA_100HZ
         return KX134_OSA_100HZ;
  #elif CONFIG_KX134_OSA_200HZ
         return KX134_OSA_200HZ;
  #elif CONFIG_KX134_OSA_400HZ
         return KX134_OSA_400HZ;
  #elif CONFIG_KX134_OSA_800HZ
         return KX134_OSA_800HZ;
  #elif CONFIG_KX134_OSA_1600HZ
         return KX134_OSA_1600HZ;
  #elif CONFIG_KX134_OSA_3200HZ
         return KX134_OSA_3200HZ;
  #elif CONFIG_KX134_OSA_6400HZ
         return KX134_OSA_6400HZ;
  #endif
}

enum kx134_gsel kx134_get_kconfig_gsel(void) {
  #ifdef CONFIG_KX134_RANGE_8G
         return KX134_GSEL_8G;
  #elif CONFIG_KX134_RANGE_16G
         return KX134_GSEL_16G;
  #elif CONFIG_KX134_RANGE_32G
         return KX134_GSEL_32G;
  #elif CONFIG_KX134_RANGE_64G
         return KX134_GSEL_64G;
  #endif
}

enum kx134_op_mode kx134_get_kconfig_op_mode(void) {
  #ifdef CONFIG_KX134_LOW_POWER_MODE
         return KX134_LOW_POWER;
  #elif CONFIG_KX134_HIGH_PERF_MODE
         return KX134_HI_PERFORMANCE;
  #endif
}



uint8_t kx134_get_kconfig_cntl2(void) {

  return (  KX134_CNTL2_LEM_MODE(IS_ENABLED(CONFIG_KX134_TILT_AXIS_LEM)) | 
            KX134_CNTL2_RIM_MODE(IS_ENABLED(CONFIG_KX134_TILT_AXIS_RIM)) | 
            KX134_CNTL2_DOM_MODE(IS_ENABLED(CONFIG_KX134_TILT_AXIS_DOM)) |
            KX134_CNTL2_UPM_MODE(IS_ENABLED(CONFIG_KX134_TILT_AXIS_UPM)) |
            KX134_CNTL2_FDM_MODE(IS_ENABLED(CONFIG_KX134_TILT_AXIS_FDM)) |
            KX134_CNTL2_FUM_MODE(IS_ENABLED(CONFIG_KX134_TILT_AXIS_FUM)) );
	
}

  //enum kx134_offi offi_odr = KX134_OFFI_12HZ5;

  //#ifdef CONFIG_KX134_OFFI_ODR_12HZ5
  //  offi_odr = KX134_OFFI_12HZ5;
  //#elif CONFIG_KX134_OFFI_ODR_25HZ
  //  offi_odr = KX134_OFFI_25HZ;
  //#elif CONFIG_KX134_OFFI_ODR_50HZ
  //  offi_odr = KX134_OFFI_50HZ;
  //#elif CONFIG_KX134_OFFI_ODR_100HZ
  //  offi_odr = KX134_OFFI_100HZ;
  //#elif CONFIG_KX134_OFFI_ODR_200HZ
  //  offi_odr = KX134_OFFI_200HZ;
  //#elif CONFIG_KX134_OFFI_ODR_400HZ
  //  offi_odr = KX134_OFFI_400HZ;
  //#elif CONFIG_KX134_OFFI_ODR_800HZ
  //  offi_odr = KX134_OFFI_800HZ;
  //#elif CONFIG_KX134_OFFI_ODR_1600HZ
  //  offi_odr = KX134_OFFI_1600HZ;
  //#endif

uint8_t kx134_get_kconfig_cntl3(void) {
  enum kx134_otp otp_odr = KX134_OTP_1HZ563;
  enum kx134_otdt otdt_odr = KX134_OTDT_12HZ5;
  enum kx134_owuf owuf_odr = KX134_OWUF_0HZ781;

  #ifdef CONFIG_KX134_OTP_ODR_1HZ563
    otp_odr = KX134_OTP_1HZ563;
  #elif CONFIG_KX134_OTP_ODR_6HZ25
    otp_odr = KX134_OTP_6HZ25;
  #elif CONFIG_KX134_OTP_ODR_12HZ5
    otp_odr = KX134_OTP_12HZ5;
  #elif CONFIG_KX134_OTP_ODR_50HZ
    otp_odr = KX134_OTP_50HZ;
  #endif

  #ifdef CONFIG_KX134_OTDT_ODR_12HZ5
    otdt_odr = KX134_OTDT_12HZ5;
  #elif CONFIG_KX134_OTDT_ODR_25HZ
    otdt_odr = KX134_OTDT_25HZ;
  #elif CONFIG_KX134_OTDT_ODR_50HZ
    otdt_odr = KX134_OTDT_50HZ;
  #elif CONFIG_KX134_OTDT_ODR_100HZ
    otdt_odr = KX134_OTDT_100HZ;
  #elif CONFIG_KX134_OTDT_ODR_200HZ
    otdt_odr = KX134_OTDT_200HZ;
  #elif CONFIG_KX134_OTDT_ODR_400HZ
    otdt_odr = KX134_OTDT_400HZ;
  #elif CONFIG_KX134_OTDT_ODR_800HZ
    otdt_odr = KX134_OTDT_800HZ;
  #elif CONFIG_KX134_OTDT_ODR_1600HZ
    otdt_odr = KX134_OTDT_1600HZ;
  #endif

  #ifdef CONFIG_KX134_OWUF_ODR_12HZ5
    owuf_odr = KX134_OWUF_0HZ781;
  #elif CONFIG_KX134_OWUF_ODR_1HZ563
    owuf_odr = KX134_OWUF_1HZ563;
  #elif CONFIG_KX134_OWUF_ODR_3HZ125
    owuf_odr = KX134_OWUF_3HZ125;
  #elif CONFIG_KX134_OWUF_ODR_6HZ25
    owuf_odr = KX134_OWUF_6HZ25;
  #elif CONFIG_KX134_OWUF_ODR_12HZ5
    owuf_odr = KX134_OWUF_12HZ5;
  #elif CONFIG_KX134_OWUF_ODR_25HZ
    owuf_odr = KX134_OWUF_25HZ;
  #elif CONFIG_KX134_OWUF_ODR_50HZ
    owuf_odr = KX134_OWUF_50HZ;
  #elif CONFIG_KX134_OWUF_ODR_100HZ
    owuf_odr = KX134_OWUF_100HZ;
  #endif

return (KX134_CNTL3_TILT_ODR_MODE(otp_odr) |
        KX134_CNTL3_TAP_ODR_MODE(otdt_odr) |
        KX134_CNTL3_WAKE_ODR_MODE(owuf_odr));
	
}

uint8_t kx134_get_kconfig_cntl4(void) {
  enum kx134_obts obts_odr = KX134_OBTS_0HZ781;

  #ifdef CONFIG_KX134_OBTS_ODR_12HZ5
    obts_odr = KX134_OBTS_0HZ781;
  #elif CONFIG_KX134_OBTS_ODR_1HZ563
    obts_odr = KX134_OBTS_1HZ563;
  #elif CONFIG_KX134_OBTS_ODR_3HZ125
    obts_odr = KX134_OBTS_3HZ125;
  #elif CONFIG_KX134_OBTS_ODR_6HZ25
    obts_odr = KX134_OBTS_6HZ25;
  #elif CONFIG_KX134_OBTS_ODR_12HZ5
    obts_odr = KX134_OBTS_12HZ5;
  #elif CONFIG_KX134_OBTS_ODR_25HZ
    obts_odr = KX134_OBTS_25HZ;
  #elif CONFIG_KX134_OBTS_ODR_50HZ
    obts_odr = KX134_OBTS_50HZ;
  #elif CONFIG_KX134_OBTS_ODR_100HZ
    obts_odr = KX134_OBTS_100HZ;
  #endif

  return (  KX134_CNTL4_DBNC_CNTR_MODE(IS_ENABLED(CONFIG_KX134_C_MODE)) | 
            KX134_CNTL4_WAKE_TH_REL_MODE(IS_ENABLED(CONFIG_KX134_TH_MODE)) | 
            KX134_CNTL4_WAKE_EN_MODE(0) |
            KX134_CNTL4_BTSLEEP_EN_MODE(0) |
            KX134_CNTL4_PULSE_REJ_MODE(IS_ENABLED(CONFIG_KX134_PR_MODE)) |
            KX134_CNTL4_BTSLEEP_ODR_MODE(obts_odr) );
}

#if defined(CONFIG_KX134_TRIGGER)

uint8_t kx134_get_kconfig_inc1(void) {
  uint8_t pw1 = 0;

  #ifdef CONFIG_KX134_PW1_USEC
         pw1 = 0;
  #elif CONFIG_KX134_PW1_1XOSA
         pw1 = 1;
  #elif CONFIG_KX134_PW1_2XOSA
         pw1 = 2;
  #elif CONFIG_KX134_PW1_RTIME
         pw1 = 3;
  #endif

  return (  KX134_INC1_PW1_MODE(pw1) | KX134_INC1_IEN1_MODE(IS_ENABLED(CONFIG_KX134_INT1)) | 
            KX134_INC1_IEA1_MODE(IS_ENABLED(CONFIG_KX134_IEA1)) | 
            KX134_INC1_IEL1_MODE(IS_ENABLED(CONFIG_KX134_IEL1)) );
}

uint8_t kx134_get_kconfig_inc2(void) {

  return (  KX134_INC2_AIO_MODE(IS_ENABLED(CONFIG_KX134_AIO)) | 
            KX134_INC2_XNWUE_MODE(IS_ENABLED(CONFIG_KX134_XNWUE)) | 
            KX134_INC2_XPWUE_MODE(IS_ENABLED(CONFIG_KX134_XPWUE)) | 
            KX134_INC2_YNWUE_MODE(IS_ENABLED(CONFIG_KX134_YNWUE)) |
            KX134_INC2_YPWUE_MODE(IS_ENABLED(CONFIG_KX134_YPWUE)) |
            KX134_INC2_ZNWUE_MODE(IS_ENABLED(CONFIG_KX134_ZNWUE)) |
            KX134_INC2_ZPWUE_MODE(IS_ENABLED(CONFIG_KX134_ZPWUE)) );
	
}

uint8_t kx134_get_kconfig_inc3(void) {

  return (  KX134_INC3_TMEN_MODE(IS_ENABLED(CONFIG_KX134_TMEN)) | 
            KX134_INC3_TLEM_MODE(IS_ENABLED(CONFIG_KX134_TLEM)) | 
            KX134_INC3_TRIM_MODE(IS_ENABLED(CONFIG_KX134_TRIM)) | 
            KX134_INC3_TDOM_MODE(IS_ENABLED(CONFIG_KX134_TDOM)) |
            KX134_INC3_TUPM_MODE(IS_ENABLED(CONFIG_KX134_TUPM)) |
            KX134_INC3_TFDM_MODE(IS_ENABLED(CONFIG_KX134_TFDM)) |
            KX134_INC3_TFUM_MODE(IS_ENABLED(CONFIG_KX134_TFUM)) );
	
}

uint8_t kx134_get_kconfig_inc4(void) {

  return (  KX134_INC4_FFI1_MODE(IS_ENABLED(CONFIG_KX134_FFI1)) | 
            KX134_INC4_BFI1_MODE(IS_ENABLED(CONFIG_KX134_BFI1)) | 
            KX134_INC4_WMI1_MODE(IS_ENABLED(CONFIG_KX134_WMI1)) | 
            KX134_INC4_DRDYI1_MODE(IS_ENABLED(CONFIG_KX134_DRDYI1)) |
            KX134_INC4_BTSI1_MODE(IS_ENABLED(CONFIG_KX134_BTSI1)) |
            KX134_INC4_TDTI1_MODE(IS_ENABLED(CONFIG_KX134_TDTI1)) |
            KX134_INC4_WUFI1_MODE(IS_ENABLED(CONFIG_KX134_WUFI1)) |
            KX134_INC4_TPI1_MODE(IS_ENABLED(CONFIG_KX134_TPI1)) );
	
}

uint8_t kx134_get_kconfig_inc5(void) {
  uint8_t pw2 = 0;

  #ifdef CONFIG_KX134_PW2_USEC
         pw2 = 0;
  #elif CONFIG_KX134_PW2_1XOSA
         pw2 = 1;
  #elif CONFIG_KX134_PW2_2XOSA
         pw2 = 2;
  #elif CONFIG_KX134_PW2_RTIME
         pw2 = 3;
  #endif

  return (  KX134_INC5_PW2_MODE(pw2) | KX134_INC5_IEN2_MODE(IS_ENABLED(CONFIG_KX134_INT2)) | 
            KX134_INC5_IEA2_MODE(IS_ENABLED(CONFIG_KX134_IEA2)) | 
            KX134_INC5_IEL2_MODE(IS_ENABLED(CONFIG_KX134_IEL2)) );
}

uint8_t kx134_get_kconfig_inc6(void) {

  return (  KX134_INC6_FFI2_MODE(IS_ENABLED(CONFIG_KX134_FFI2)) | 
            KX134_INC6_BFI2_MODE(IS_ENABLED(CONFIG_KX134_BFI2)) | 
            KX134_INC6_WMI2_MODE(IS_ENABLED(CONFIG_KX134_WMI2)) | 
            KX134_INC6_DRDYI2_MODE(IS_ENABLED(CONFIG_KX134_DRDYI2)) |
            KX134_INC6_BTSI2_MODE(IS_ENABLED(CONFIG_KX134_BTSI2)) |
            KX134_INC6_TDTI2_MODE(IS_ENABLED(CONFIG_KX134_TDTI2)) |
            KX134_INC6_WUFI2_MODE(IS_ENABLED(CONFIG_KX134_WUFI2)) |
            KX134_INC6_TPI2_MODE(IS_ENABLED(CONFIG_KX134_TPI2)) );
	
}

#endif /* CONFIG_KX134_TRIGGER */

static int kx134_chip_init(const struct device *dev)
{
	struct kx134_data *data = dev->data;
	int ret;

	/* Device settings from kconfig */
	ret = kx134_reg_write_mask(dev, KX134_ODCNTL,
							   KX134_ODCNTL_OUT_ODR_MSK,
							   KX134_ODCNTL_OUT_ODR_MODE(kx134_get_kconfig_odr()));
	if (ret)
	{
		return ret;
	}

	data->selected_range = kx134_get_kconfig_gsel();
	ret = kx134_reg_write_mask(dev, KX134_CNTL1,
							   KX134_CNTL1_GSEL_MSK,
							   KX134_CNTL1_GSEL_MODE(data->selected_range));
	if (ret)
	{
		return ret;
	}

#if defined(CONFIG_KX134_TRIGGER)
	data->int1_config = kx134_get_kconfig_inc1();
	data->int1_source = kx134_get_kconfig_inc4();

	data->int2_config = kx134_get_kconfig_inc5();
	data->int2_source = kx134_get_kconfig_inc6();

	ret = kx134_set_reg(dev, data->int1_config, KX134_INC1, 1);
	if (ret)
	{
		return ret;
	}
	ret = kx134_set_reg(dev, kx134_get_kconfig_inc2(), KX134_INC2, 1);
	if (ret)
	{
		return ret;
	}
	ret = kx134_set_reg(dev, kx134_get_kconfig_inc3(), KX134_INC3, 1);
	if (ret)
	{
		return ret;
	}
	ret = kx134_set_reg(dev, data->int1_source, KX134_INC4, 1);
	if (ret)
	{
		return ret;
	}
	ret = kx134_set_reg(dev, data->int2_config, KX134_INC5, 1);
	if (ret)
	{
		return ret;
	}
	ret = kx134_set_reg(dev, data->int2_source, KX134_INC6, 1);
	if (ret)
	{
		return ret;
	}

	if (kx134_init_interrupt(dev) < 0)
	{
		LOG_ERR("Failed to initialize interrupt!");
		return -EIO;
	}
#endif

	ret = kx134_set_reg(dev, kx134_get_kconfig_cntl2(), KX134_CNTL2, 1);
	if (ret)
	{
		return ret;
	}
	ret = kx134_set_reg(dev, kx134_get_kconfig_cntl3(), KX134_CNTL3, 1);
	if (ret)
	{
		return ret;
	}
	ret = kx134_set_reg(dev, kx134_get_kconfig_cntl4(), KX134_CNTL4, 1);
	if (ret)
	{
		return ret;
	}

	ret = kx134_reg_write_mask(dev, KX134_CNTL1,
							   KX134_CNTL1_ACTIVE_MSK,
							   KX134_CNTL1_ACTIVE_MODE(kx134_get_kconfig_op_mode()));
	if (ret)
	{
		return ret;
	}

	return 0;
}

static int kx134_init(const struct device *dev)
{
	const struct kx134_config *cfg = dev->config;
	struct kx134_data *data = dev->data;
	uint8_t value[2];
	int err;

	data->dev = dev;

#ifdef CONFIG_KX134_I2C
	if (!device_is_ready(cfg->i2c.bus))
	{
		LOG_ERR("I2C bus not ready!");
		return -ENODEV;
	}
#endif
#ifdef CONFIG_KX134_SPI
	if (!spi_is_ready(&cfg->spi))
	{
		LOG_ERR("SPI device not found");
		return -ENODEV;
	}
#endif /* CONFIG_KX134_SPI */

	err = kx134_software_reset(dev);

	if (err)
	{
		LOG_ERR("kx134_software_reset failed, error %d\n", err);
		return -ENODEV;
	}
	k_sleep(K_MSEC(5));

	value[0] = 0;

	kx134_get_reg(dev, value, KX134_PART_ID, sizeof(value));
	if (value[0] != KX134_WHO_AM_I_VAL)
	{
		LOG_ERR("Failed Part-ID: %d\n", value[0]);
		return -ENODEV;
	}

	if (kx134_chip_init(dev) < 0)
	{
		return -ENODEV;
	}

	return 0;
}


/*
 * This instantiation macro is named "CREATE_KX134_DEVICE".
 * Its "inst" argument is an arbitrary instance number.
 *
 * Put this near the end of the file, e.g. after defining "my_api_funcs".
 */
#define CREATE_KX134_DEVICE(inst)                                                                                    \
	static struct kx134_data kx134_data_##inst = {                                                                   \
		/* initialize RAM values as needed, e.g.: */                                                                 \
	};                                                                                                               \
	static const struct kx134_config kx134_config_##inst = {                                                         \
		/* initialize ROM values as needed. */                                                                       \
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),                                                                       \
					(.spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8), 0)), \
					(.i2c = I2C_DT_SPEC_INST_GET(inst))),                                                            \
		.gpio_drdy =                                                                                                 \
			GPIO_DT_SPEC_INST_GET_BY_IDX(inst, irq_gpios, 0),                                                        \
		.gpio_int =                                                                                                  \
			GPIO_DT_SPEC_INST_GET_BY_IDX(inst, irq_gpios, 1),                                                        \
	};                                                                                                               \
	DEVICE_DT_INST_DEFINE(inst,                                                                                      \
						  kx134_init,                                                                                \
						  NULL,                                                                                      \
						  &kx134_data_##inst,                                                                        \
						  &kx134_config_##inst,                                                                      \
						  POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,                                                  \
						  &kx134_api_funcs);

/* Call the device creation macro for each instance: */
DT_INST_FOREACH_STATUS_OKAY(CREATE_KX134_DEVICE)