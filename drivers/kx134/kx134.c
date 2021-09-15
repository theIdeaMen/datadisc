/*
 * Copyright (c) 2021 Griffin Adams
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT kionix_kx134_1211

#include <kernel.h>
#include <string.h>
#include <drivers/sensor.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <stdlib.h>
#include <drivers/spi.h>
#include <drivers/i2c.h>
#include <logging/log.h>

#include "kx134.h"

LOG_MODULE_REGISTER(KX134, CONFIG_SENSOR_LOG_LEVEL);

static int kx134_reg_access(struct kx134_data *ctx, uint8_t cmd,
			      uint8_t reg_addr, void *data, size_t length)
{
        int ret;
#ifdef CONFIG_KX134_SPI
	if (cmd == KX134_READ_REG) {

              uint8_t buffer_tx[2] = { reg_addr | 0x80 };
              const struct spi_buf tx_buf = {
                              .buf = buffer_tx,
                              .len = 1,
              };
              const struct spi_buf_set tx = {
                      .buffers = &tx_buf,
                      .count = 1
              };
              const struct spi_buf rx_buf[2] = {
                      {
                              .buf = NULL,
                              .len = 1,
                      },
                      {
                              .buf = data,
                              .len = length,
                      }
              };
              const struct spi_buf_set rx = {
                      .buffers = rx_buf,
                      .count = 2
              };


              if (length > 64) {
                      return -EIO;
              }
              ret = spi_transceive(ctx->bus, &ctx->spi_cfg, &tx, &rx);

              LOG_DBG("Read %d bytes starting with 0x%02X from 0x%02X", length, *((uint8_t*)data), reg_addr);

              return ret;
	}

        uint8_t buffer_tx[1] = { reg_addr };
	const struct spi_buf tx_buf[2] = {
		{
			.buf = buffer_tx,
			.len = 1,
		},
		{
			.buf = data,
			.len = length,
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2
	};

        if (length > 64) {
		return -EIO;
	}

	ret = spi_write(ctx->bus, &ctx->spi_cfg, &tx);

        LOG_DBG("Wrote 0x%02X to 0x%02X", *((uint8_t*)data), reg_addr);

        return ret;

#elif CONFIG_KX134_I2C
	if (cmd == KX134_READ_REG) {
		return i2c_burst_read(ctx->bus, ctx->i2c_addr,
				      KX134_TO_I2C_REG(reg),
				      (uint8_t *) data, length);
	} else {
		if (length != 1) {
			return -EINVAL;
		}

		return i2c_reg_write_byte(ctx->bus, ctx->i2c_addr,
					  KX134_TO_I2C_REG(reg),
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
static int kx134_get_reg(const struct device *dev, uint8_t *read_buf,
				  uint8_t register_address, uint8_t count)
{
	struct kx134_data *kx134_data = dev->data;

	return kx134_reg_access(kx134_data,
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
static int kx134_set_reg(const struct device *dev,
				  uint16_t register_value,
				  uint8_t register_address, uint8_t count)
{
        int ret;
	uint8_t cntl1_before;
        uint8_t zero = 0;
        struct kx134_data *kx134_data = dev->data;

        // KX134 needs to be in standby for write access
	ret = kx134_get_reg(dev, &cntl1_before, KX134_CNTL1, 1);
	if (ret) {
		return ret;
	}

        ret = kx134_reg_access(kx134_data,
				  KX134_WRITE_REG,
				  KX134_CNTL1, &zero, 1);
        if (ret) {
		return ret;
	}

	ret = kx134_reg_access(kx134_data,
				  KX134_WRITE_REG,
				  register_address,
				  &register_value,
				  count);
        if (ret) {
		return ret;
	}

        if (register_address == KX134_CNTL1) {
                return ret;
        }

        // Put CNTL1 back to its previous value
        return kx134_reg_access(kx134_data,
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
 * Configure the INT1 and INT2 interrupt pins.
 * @param dev - The device structure.
 * @param int1 -  INT1 interrupt pins.
 * @param int2 -  INT2 interrupt pins.
 * @return 0 in case of success, negative error code otherwise.
 */
static int kx134_interrupt_config(const struct device *dev,
				    uint8_t int1,
				    uint8_t int2)
{
	int ret;

	ret = kx134_set_reg(dev, int1, KX134_INC1, 1);
	if (ret) {
		return ret;
	}

	return kx134_set_reg(dev, int2, KX134_INC5, 1);
}

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

/**
 * Get interrupt source.
 * @param dev  - The device structure.
 * @param val  - Interrupt source register value. 
 * @return 0 in case of success, negative error code otherwise.
 */
static int kx134_get_int_source(const struct device *dev,
				struct sensor_value *val)
{
	uint8_t int_src[3];
        int ret;

	ret = kx134_get_reg(dev, int_src, KX134_INS1, 3);
        if (ret) {
		return ret;
	}
        val->val1 = int_src[0];
        val->val1 = (val->val1 << 8) | int_src[1];
        val->val1 = (val->val1 << 8) | int_src[2];
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

        ret = kx134_get_reg(dev, (uint8_t *)buf, KX134_INS1, 3);
        if (ret) {
		return ret;
	}
        data->tap_int = buf[0];
        data->func_int = buf[1];
        data->wkup_int = buf[2];


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

#endif // CONFIG_KX134_TRIGGER

static int kx134_chip_init(const struct device *dev)
{
        struct kx134_data *data = dev->data;
	int ret;

	/* Device settings from kconfig */
	ret = kx134_reg_write_mask(dev, KX134_ODCNTL,
				      KX134_ODCNTL_OUT_ODR_MSK,
				      KX134_ODCNTL_OUT_ODR_MODE(kx134_get_kconfig_odr()));
	if (ret) {
		return ret;
	}

        ret = kx134_reg_write_mask(dev, KX134_CNTL1,
				      KX134_CNTL1_GSEL_MSK,
				      KX134_CNTL1_GSEL_MODE(kx134_get_kconfig_gsel()));
	if (ret) {
		return ret;
	}


#if defined(CONFIG_KX134_TRIGGER)
        data->int1_config = kx134_get_kconfig_inc1();
        data->int1_source = kx134_get_kconfig_inc4();

        data->int2_config = kx134_get_kconfig_inc5();
        data->int2_source = kx134_get_kconfig_inc6();

        if (kx134_interrupt_config(dev, data->int1_config, data->int2_config) < 0) {
		LOG_ERR("Failed to configure interrupt");
		return -EIO;
	}

	if (kx134_init_interrupt(dev) < 0) {
		LOG_ERR("Failed to initialize interrupt!");
		return -EIO;
	}
#endif

        ret = kx134_reg_write_mask(dev, KX134_CNTL1,
				      KX134_CNTL1_ACTIVE_MSK,
				      KX134_CNTL1_ACTIVE_MODE(kx134_get_kconfig_op_mode()));
	if (ret) {
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

#ifdef CONFIG_KX134_I2C
	data->bus  = device_get_binding(cfg->i2c_port);
	if (data->bus  == NULL) {
		LOG_ERR("Failed to get pointer to %s device!",
			    cfg->i2c_port);
		return -EINVAL;
	}
#endif
#ifdef CONFIG_KX134_SPI
	data->bus = device_get_binding(cfg->spi_port);
	if (!data->bus) {
		LOG_ERR("spi device not found: %s", cfg->spi_port);
		return -EINVAL;
	}
	/* CPOL=0, CPHA=0, max 10MHz */
	data->spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
	data->spi_cfg.frequency = cfg->spi_max_frequency;
	data->spi_cfg.slave = cfg->spi_slave;

	data->kx134_cs_ctrl.gpio_dev = device_get_binding(cfg->gpio_cs_port);
	if (!data->kx134_cs_ctrl.gpio_dev) {
		LOG_ERR("Unable to get GPIO SPI CS device");
		return -ENODEV;
	}

	data->kx134_cs_ctrl.gpio_pin = cfg->cs_gpio;
	data->kx134_cs_ctrl.gpio_dt_flags = cfg->cs_flags;
	data->kx134_cs_ctrl.delay = 0U;

	data->spi_cfg.cs = &data->kx134_cs_ctrl;
#endif /* CONFIG_KX134_SPI */

	err = kx134_software_reset(dev);

	if (err) {
		LOG_ERR("kx134_software_reset failed, error %d\n", err);
		return -ENODEV;
	}
        k_sleep(K_MSEC(5));

        //while(1){

        kx134_get_reg(dev, value, KX134_PART_ID, sizeof(value));
	if (value[0] != KX134_WHO_AM_I_VAL) {
		LOG_ERR("Failed Part-ID: %d\n", value[0]);
		return -ENODEV;
	}

        //k_sleep(K_MSEC(500));

        //}; // debuggin

	if (kx134_chip_init(dev) < 0) {
		return -ENODEV;
	}

        return 0;
}



//static const struct kx134_config kx134_config = {
//#ifdef CONFIG_KX134_I2C
//	.i2c_port = DT_INST_BUS_LABEL(0),
//	.i2c_addr = DT_INST_REG_ADDR(0),
//#endif
//#ifdef CONFIG_KX134_SPI
//	.spi_port = DT_INST_BUS_LABEL(1),
//	.spi_slave = DT_INST_REG_ADDR(1),
//	.spi_max_frequency = DT_INST_PROP(1, spi_max_frequency),
//#if DT_INST_SPI_DEV_HAS_CS_GPIOS(1)
//	.gpio_cs_port = DT_INST_SPI_DEV_CS_GPIOS_LABEL(1),
//	.cs_gpio = DT_INST_SPI_DEV_CS_GPIOS_PIN(1),
//	.cs_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(1),
//#endif
//#endif
//#ifdef CONFIG_KX134_TRIGGER
//	.gpio_port = DT_INST_GPIO_LABEL(1, irq_gpios),
//	.int_gpio = DT_INST_GPIO_PIN(1, irq_gpios),
//	.int_flags = DT_INST_GPIO_FLAGS(1, irq_gpios),
//        .int1_config = KX134_INC1_IEN1_MODE(1) | KX134_INC1_IEA1_MODE(1), // Enable and active high
//	.int2_config = KX134_INC5_IEN2_MODE(0), // Disabled for now
//#endif

//#ifdef CONFIG_KX134_OSA_50HZ
//	.osa = KX134_OSA_50HZ,
//#elif CONFIG_KX134_OSA_800HZ
//	.osa = KX134_OSA_800HZ,
//#elif CONFIG_KX134_OSA_1600HZ
//	.osa = KX134_OSA_1600HZ,
//#elif CONFIG_KX134_OSA_3200HZ
//	.osa = KX134_OSA_3200HZ,
//#elif CONFIG_KX134_OSA_6400HZ
//	.osa = KX134_OSA_6400HZ,
//#endif

//        /* Device Settings */
//	.buff_config = KX134_BM_STREAM,

//	.op         = KX134_LOW_POWER,
//	.gsel       = KX134_GSEL_64G,
//	.otp        = KX134_OTP_1HZ563,
//	.otdt       = KX134_OTDT_12HZ5,
//	.owuf       = KX134_OWUF_0HZ781,
//	.obts       = KX134_OBTS_0HZ781,
//	.offi       = KX134_OFFI_12HZ5,
//	.avc        = KX134_NO_AVG,
//	.rms_avc    = KX134_RMS_AVG_2_SAMPLES,
//	.oadp       = KX134_OADP_0HZ781,
//};

//DEVICE_DT_INST_DEFINE(0, kx134_init, device_pm_control_nop,
//		    &kx134_data, &kx134_config, POST_KERNEL,
//		    CONFIG_SENSOR_INIT_PRIORITY, &kx134_api_funcs);


/*
 * This instantiation macro is named "CREATE_MY_DEVICE".
 * Its "inst" argument is an arbitrary instance number.
 *
 * Put this near the end of the file, e.g. after defining "my_api_funcs".
 */
#define CREATE_MY_DEVICE(inst)                                    \
  static struct kx134_data kx134_data_##inst = {                  \
      /* initialize RAM values as needed, e.g.: */                \
  };                                                              \
  static const struct kx134_config kx134_config_##inst = {        \
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
      kx134_init,                                                 \
      NULL,                                                       \
      &kx134_data_##inst,                                         \
      &kx134_config_##inst,                                       \
      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,                   \
      &kx134_api_funcs);

/* Call the device creation macro for each instance: */
DT_INST_FOREACH_STATUS_OKAY(CREATE_MY_DEVICE)