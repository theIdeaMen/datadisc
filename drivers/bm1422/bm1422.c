/*
 * Copyright (c) 2021 Griffin Adams
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rohm_bm1422agmv

#include <init.h>
#include <kernel.h>
#include <stdlib.h>
#include <string.h>

#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>

#include <logging/log.h>
#include <sys/__assert.h>
#include <sys/printk.h>

#include "bm1422.h"

LOG_MODULE_REGISTER(BM1422, CONFIG_SENSOR_LOG_LEVEL);

static int bm1422_reg_access(const struct device *dev, uint8_t cmd,
    uint8_t reg_addr, void *data, size_t length) {

  struct bm1422_data *bm1422_data = dev->data;
  const struct bm1422_config *cfg = dev->config;

  if (cmd == BM1422_READ_REG) {
    return i2c_burst_read(bm1422_data->bus, cfg->i2c_addr,
        BM1422_TO_I2C_REG(reg_addr),
        (uint8_t *)data, length);
  } else {
    if (length != 1) {
      return -EINVAL;
    }

    return i2c_reg_write_byte(bm1422_data->bus, cfg->i2c_addr,
        BM1422_TO_I2C_REG(reg_addr),
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
static int bm1422_get_reg(const struct device *dev, uint8_t register_address, uint8_t *read_buf, uint8_t count) {

  return bm1422_reg_access(dev,
      BM1422_READ_REG,
      BM1422_REG_READ(register_address),
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
static int bm1422_set_reg(const struct device *dev, uint8_t register_address, uint16_t register_value, uint8_t count) {

  return bm1422_reg_access(dev,
      BM1422_WRITE_REG,
      BM1422_REG_WRITE(register_address),
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
    uint8_t mask, uint8_t data) {
  int ret;
  uint8_t tmp;

  ret = bm1422_get_reg(dev, register_address, &tmp, 1);
  if (ret) {
    return ret;
  }

  tmp &= ~mask;
  tmp |= data;

  return bm1422_set_reg(dev, register_address, tmp, 1);
}

#if defined(CONFIG_BM1422_TRIGGER)

/**
 * Get the status register data
 * @param dev - The device structure.
 * @param status - Data stored in the STATUS register
 * @return 0 in case of success, negative error code otherwise.
 */
int bm1422_get_status(const struct device *dev, uint8_t *status) {
  return bm1422_get_reg(dev, BM1422_STA1, status, 1);
}

int bm1422_clear_interrupts(const struct device *dev) {
  uint8_t temp[6];
  /* Reading magnetometer data clears the DRDY flag? */
  return bm1422_get_reg(dev, BM1422_FINE_DATAX_LO, (uint8_t *)temp, sizeof(temp));
  /* Set the FORCE flag again? */
  //return bm1422_reg_write_mask(dev, BM1422_CNTL3,
  //                             BM1422_CNTL3_FORCE_MSK,
  //                             BM1422_CNTL3_FORCE_MODE(1));
}
#endif

/**
 * Software reset.
 * @param dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
static int bm1422_software_reset(const struct device *dev) {
  /* Writing to SRST resets the device */
  return bm1422_reg_write_mask(dev, BM1422_CNTL1,
      BM1422_CNTL1_RST_LV_MSK,
      BM1422_CNTL1_RST_LV_MODE(1));
}

/**
 * Set Output data rate.
 * @param dev - The device structure.
 * @param val - Output data rate.
 *		Accepted values:  
 * @return 0 in case of success, negative error code otherwise.
 */
static int bm1422_set_odr(const struct device *dev,
    const struct sensor_value *val) {
  enum bm1422_odr odr;

  switch (val->val1) {
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
    const struct sensor_value *val) {
  enum bm1422_ave ave;

  switch (val->val1) {
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
    const struct sensor_value *val) {
  switch (attr) {
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
    struct sensor_value *val) {
  switch (attr) {
  case 0:
  default:
    return -ENOTSUP;
  }
  // TODO: Figure out how to make custom attributes
}

void bm1422_mag_convert(struct sensor_value *val, int16_t mag, enum bm1422_out_bit bits) {
  uint16_t divisor = BM1422_MAG_12BIT_LSB_PER_MILLI_T;
  if (bits == BM1422_14BIT) {
    divisor = BM1422_MAG_14BIT_LSB_PER_MILLI_T;
  }
  int64_t temp = (int64_t)mag * 1000LL * 1000LL * 1000LL;
  int64_t pico_t = temp / divisor;

  val->val1 = (int32_t)(pico_t / 1000000LL); // Saved in sensor_value as micro T
  val->val2 = (int32_t)(pico_t % 1000000LL); // Saved in sensor_value as pico T
}
//static void bm1422_mag_convert(struct sensor_value *val, int16_t mag, enum bm1422_out_bit bits) {


//  val->val1 = 0;
//  val->val2 = mag;
//}

static void bm1422_temp_convert(struct sensor_value *val, int16_t temp, enum bm1422_out_bit bits) {
  uint16_t divisor = BM1422_12BIT_LSB_PER_MILLI_DEGREE_C;
  if (bits == BM1422_14BIT) {
    divisor = BM1422_14BIT_LSB_PER_MILLI_DEGREE_C;
  }
  int64_t micro_c = temp * 1000 * 1000 * 1000 / divisor;

  val->val1 = micro_c / 1000000; // Saved in sensor_value as micro C
  val->val2 = micro_c % 1000000; // Saved in sensor_value as pico C
}

static int bm1422_sample_fetch(const struct device *dev, enum sensor_channel chan) {
  struct bm1422_data *data = dev->data;
  uint8_t buf[6];
  int ret;

  __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

  // Magnetometer data
  ret = bm1422_get_reg(dev, BM1422_DATAX_LO, (uint8_t *)buf, sizeof(buf));
  if (ret) {
    return ret;
  }

  /* 16-bit, big endien, 2's compliment */
  data->mag_x = ((int16_t)buf[1] << 8) | (buf[0]);
  data->mag_y = ((int16_t)buf[3] << 8) | (buf[2]);
  data->mag_z = ((int16_t)buf[5] << 8) | (buf[4]);

  // Temperature data
  ret = bm1422_get_reg(dev, BM1422_TEMP_LO, (uint8_t *)buf, 2);
  if (ret) {
    return ret;
  }

  /* 16-bit, big endien, 2's compliment */
  data->temperature = ((int16_t)buf[1] << 8) | (buf[0]);

  return 0;
}

static int bm1422_channel_get(const struct device *dev,
    enum sensor_channel chan,
    struct sensor_value *val) {
  struct bm1422_data *data = dev->data;

  switch (chan) {
  case SENSOR_CHAN_MAGN_X:
    bm1422_mag_convert(val, data->mag_x, data->selected_bits);
    break;
  case SENSOR_CHAN_MAGN_Y:
    bm1422_mag_convert(val, data->mag_y, data->selected_bits);
    break;
  case SENSOR_CHAN_MAGN_Z:
    bm1422_mag_convert(val, data->mag_z, data->selected_bits);
    break;
  case SENSOR_CHAN_MAGN_XYZ:
    for (size_t i = 0; i < 3; i++) {
      bm1422_mag_convert(&val[i], data->mag_xyz[i], data->selected_bits);
    }
    break;
  case SENSOR_CHAN_DIE_TEMP:
    bm1422_temp_convert(val, data->temperature, data->selected_bits);
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

enum bm1422_odr bm1422_get_kconfig_odr(void) {
#ifdef CONFIG_BM1422_ODR_10HZ
  return BM1422_ODR_10HZ;
#elif CONFIG_BM1422_ODR_20HZ
  return BM1422_ODR_20HZ;
#elif CONFIG_BM1422_ODR_100HZ
  return BM1422_ODR_100HZ;
#elif CONFIG_BM1422_ODR_1000HZ
  return BM1422_ODR_1000HZ;
#endif
}

enum bm1422_out_bit bm1422_get_kconfig_bits(void) {
#ifdef CONFIG_BM1422_12_BIT
  return BM1422_12BIT;
#elif CONFIG_BM1422_14_BIT
  return BM1422_14BIT;
#endif
}

enum bm1422_ave bm1422_get_kconfig_ave(void) {
#ifdef CONFIG_BM1422_AVC_NONE
  return BM1422_1_TIMES;
#elif CONFIG_BM1422_AVC_2SAMPLES
  return BM1422_2_TIMES;
#elif CONFIG_BM1422_AVC_4SAMPLES
  return BM1422_4_TIMES;
#elif CONFIG_BM1422_AVC_8SAMPLES
  return BM1422_8_TIMES;
#elif CONFIG_BM1422_AVC_16SAMPLES
  return BM1422_16_TIMES;
#endif
}

#if defined(CONFIG_BM1422_TRIGGER)

uint8_t bm1422_get_kconfig_cntl2(void) {
  return (BM1422_CNTL2_DREN_MODE(IS_ENABLED(CONFIG_BM1422_DREN)) | BM1422_CNTL2_DRP_MODE(IS_ENABLED(CONFIG_BM1422_DRP)));
}

#endif /* CONFIG_BM1422_TRIGGER */

static int bm1422_offset_adj(const struct device *dev) {
  struct bm1422_data *data = dev->data;
  int ret;
  uint8_t wk_dat = 1;
  uint16_t diff_x = 9999;
  uint16_t diff_y = 9999;
  uint16_t diff_z = 9999;
  uint8_t offx_dat = 1;
  uint8_t offy_dat = 1;
  uint8_t offz_dat = 1;

  ret = bm1422_set_reg(dev, BM1422_CNTL1, 0xC2, 1);
  if (ret) {
    return ret;
  }

  k_sleep(K_MSEC(1));

  ret = bm1422_set_reg(dev, BM1422_CNTL4_LO, 0x00, 1);
  if (ret) {
    return ret;
  }

  ret = bm1422_set_reg(dev, BM1422_CNTL4_HI, 0x00, 1);
  if (ret) {
    return ret;
  }

  ret = bm1422_set_reg(dev, BM1422_CNTL2, 0x0C, 1);
  if (ret) {
    return ret;
  }

  while (wk_dat < 96) {
    ret = bm1422_set_reg(dev, BM1422_OFFX_LO, wk_dat, 1);
    if (ret) {
      return ret;
    }

    ret = bm1422_set_reg(dev, BM1422_OFFY_LO, wk_dat, 1);
    if (ret) {
      return ret;
    }

    ret = bm1422_set_reg(dev, BM1422_OFFZ_LO, wk_dat, 1);
    if (ret) {
      return ret;
    }

    ret = bm1422_set_reg(dev, BM1422_CNTL3, 0x40, 1);
    if (ret) {
      return ret;
    }

    k_sleep(K_MSEC(101));

    if (sensor_sample_fetch(dev)) {
      LOG_INF("sensor_sample_fetch failed\n");
      return -EIO;
    }

    if (diff_x > abs(data->mag_x)) {
      offx_dat = wk_dat;
      diff_x = abs(data->mag_x);
    }

    if (diff_y > abs(data->mag_y)) {
      offy_dat = wk_dat;
      diff_y = abs(data->mag_y);
    }

    if (diff_z > abs(data->mag_z)) {
      offz_dat = wk_dat;
      diff_z = abs(data->mag_z);
    }

    wk_dat += 1;
  }

  LOG_INF("%d,%d,%d\n", offx_dat, offy_dat, offz_dat);

  ret = bm1422_set_reg(dev, BM1422_OFFX_LO, 46, 1);
  if (ret) {
    return ret;
  }

  ret = bm1422_set_reg(dev, BM1422_OFFY_LO, 46, 1);
  if (ret) {
    return ret;
  }

  ret = bm1422_set_reg(dev, BM1422_OFFZ_LO, 45, 1);
  if (ret) {
    return ret;
  }

  return 0;
}

static int bm1422_chip_init(const struct device *dev) {
  struct bm1422_data *data = dev->data;
  int ret;

  /* Device settings from kconfig */
  data->selected_bits = bm1422_get_kconfig_bits();
  ret = bm1422_set_reg(dev, BM1422_CNTL1, 
                        0 | BM1422_CNTL1_PC1_MODE(1) | 
                        BM1422_CNTL1_OUT_BIT_MODE(data->selected_bits) | 
                        BM1422_CNTL1_ODR_MODE(bm1422_get_kconfig_odr()), 1);
  if (ret) {
    return ret;
  }

  ret = bm1422_set_reg(dev, BM1422_AVE_A, BM1422_AVE_A_MODE(bm1422_get_kconfig_ave()), 1);
  if (ret) {
    return ret;
  }

#if defined(CONFIG_BM1422_TRIGGER)
  data->int_config = bm1422_get_kconfig_cntl2();

  ret = bm1422_set_reg(dev, BM1422_CNTL2, data->int_config, 1);
  if (ret) {
    return ret;
  }
  k_sleep(K_MSEC(1));

  if (bm1422_init_interrupt(dev) < 0) {
    LOG_ERR("Failed to initialize interrupt!");
    return -EIO;
  }
#endif

  k_sleep(K_MSEC(1));

  ret = bm1422_set_reg(dev, BM1422_CNTL4_LO, 0x00, 1);
  if (ret) {
    return ret;
  }

  ret = bm1422_set_reg(dev, BM1422_CNTL4_HI, 0x00, 1);
  if (ret) {
    return ret;
  }

  ret = bm1422_set_reg(dev, BM1422_OFFX_LO, 47, 1);
  if (ret) {
    return ret;
  }
  ret = bm1422_set_reg(dev, BM1422_OFFX_HI, 0, 1);
  if (ret) {
    return ret;
  }

  ret = bm1422_set_reg(dev, BM1422_OFFY_LO, 47, 1);
  if (ret) {
    return ret;
  }
  ret = bm1422_set_reg(dev, BM1422_OFFY_HI, 0, 1);
  if (ret) {
    return ret;
  }

  ret = bm1422_set_reg(dev, BM1422_OFFZ_LO, 46, 1);
  if (ret) {
    return ret;
  }
  ret = bm1422_set_reg(dev, BM1422_OFFZ_HI, 0, 1);
  if (ret) {
    return ret;
  }

  k_sleep(K_MSEC(1));

  ret = bm1422_set_reg(dev, BM1422_CNTL3, 0x40, 1);
  if (ret) {
    return ret;
  }

  return 0;
}

static int bm1422_init(const struct device *dev) {
  const struct bm1422_config *cfg = dev->config;
  struct bm1422_data *data = dev->data;
  uint8_t value[2];
  int err;

  data->bus = device_get_binding(cfg->i2c_port);
  if (data->bus == NULL) {
    LOG_ERR("Failed to get pointer to %s device!",
        cfg->i2c_port);
    return -EINVAL;
  }

  err = bm1422_software_reset(dev);
  if (err) {
    LOG_ERR("bm1422_software_reset failed, error %d\n", err);
    return -ENODEV;
  }
  k_sleep(K_MSEC(5));

  bm1422_get_reg(dev, BM1422_WHO_AM_I, value, 1);
  if (value[0] != BM1422_WHO_AM_I_VAL) {
    LOG_ERR("Failed Part-ID: %d\n", value[0]);
    return -ENODEV;
  }

  //if (bm1422_offset_adj(dev) < 0) {
  //  return -ENODEV;
  //}

  if (bm1422_chip_init(dev) < 0) {
    return -ENODEV;
  }

  return 0;
}

/*
 * This instantiation macro is named "CREATE_BM1422_DEVICE".
 * Its "inst" argument is an arbitrary instance number.
 *
 * Put this near the end of the file, e.g. after defining "my_api_funcs".
 */
#define CREATE_BM1422_DEVICE(inst)                               \
  static struct bm1422_data bm1422_data_##inst = {           \
      /* initialize RAM values as needed, e.g.: */           \
  };                                                         \
  static const struct bm1422_config bm1422_config_##inst = { \
      /* initialize ROM values as needed. */                 \
      .i2c_port = DT_INST_BUS_LABEL(inst),                   \
      .i2c_addr = DT_INST_REG_ADDR(inst),                    \
      .gpio_port = DT_INST_GPIO_LABEL(inst, irq_gpios),      \
      .int_gpio = DT_INST_GPIO_PIN(inst, irq_gpios),         \
      .int_flags = DT_INST_GPIO_FLAGS(inst, irq_gpios),      \
  };                                                         \
  DEVICE_DT_INST_DEFINE(inst,                                \
      bm1422_init,                                           \
      NULL,                                                  \
      &bm1422_data_##inst,                                   \
      &bm1422_config_##inst,                                 \
      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,              \
      &bm1422_api_funcs);

/* Call the device creation macro for each instance: */
DT_INST_FOREACH_STATUS_OKAY(CREATE_BM1422_DEVICE)