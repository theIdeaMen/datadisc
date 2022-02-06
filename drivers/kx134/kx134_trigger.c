/*
 * Copyright (c) 2021 Griffin Adams
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT kionix_kx134_1211

#include "kx134.h"
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <kernel.h>
#include <sys/util.h>
#include <logging/log.h>

LOG_MODULE_DECLARE(KX134, CONFIG_SENSOR_LOG_LEVEL);

#define TRIGGED_INT1    1
#define TRIGGED_INT2    2

static void kx134_thread_cb(const struct device *dev) {
const struct kx134_config *cfg = dev->config;
  struct kx134_data *drv_data = dev->data;

  k_mutex_lock(&drv_data->trigger_mutex, K_FOREVER);

  if (cfg->gpio_drdy.port &&
      atomic_test_and_clear_bit(&drv_data->trig_flags, TRIGGED_INT1)) {

    if (drv_data->drdy_handler != NULL) {
      drv_data->drdy_handler(dev, &drv_data->drdy_trigger);
      kx134_clear_interrupts(dev);
    }
    k_mutex_unlock(&drv_data->trigger_mutex);
    return;
  }

  if (cfg->gpio_int.port &&
      atomic_test_and_clear_bit(&drv_data->trig_flags, TRIGGED_INT2)) {

    if (drv_data->any_handler != NULL) {
      drv_data->any_handler(dev, &drv_data->any_trigger);
      kx134_clear_interrupts(dev);
    }
    k_mutex_unlock(&drv_data->trigger_mutex);
    return;
  }
}

static void kx134_gpio_int1_callback(const struct device *dev, 
                                     struct gpio_callback *cb, uint32_t pins) {
  struct kx134_data *drv_data = CONTAINER_OF(cb, struct kx134_data, gpio_int1_cb);

  ARG_UNUSED(pins);

  atomic_set_bit(&drv_data->trig_flags, TRIGGED_INT1);
  
#if defined(CONFIG_KX134_TRIGGER_OWN_THREAD)
  k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_KX134_TRIGGER_GLOBAL_THREAD)
  k_work_submit(&drv_data->work);
#endif
}

static void kx134_gpio_int2_callback(const struct device *dev, 
                                     struct gpio_callback *cb, uint32_t pins) {
  struct kx134_data *drv_data = CONTAINER_OF(cb, struct kx134_data, gpio_int2_cb);

  ARG_UNUSED(pins);

  atomic_set_bit(&drv_data->trig_flags, TRIGGED_INT2);
  
#if defined(CONFIG_KX134_TRIGGER_OWN_THREAD)
  k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_KX134_TRIGGER_GLOBAL_THREAD)
  k_work_submit(&drv_data->work);
#endif
}

#if defined(CONFIG_KX134_TRIGGER_OWN_THREAD)
static void kx134_thread(struct kx134_data *drv_data) {
  while (true) {
    k_sem_take(&drv_data->gpio_sem, K_FOREVER);
    kx134_thread_cb(drv_data->dev);
  }
}

#elif defined(CONFIG_KX134_TRIGGER_GLOBAL_THREAD)
static void kx134_work_cb(struct k_work *work) {
  struct kx134_data *drv_data = CONTAINER_OF(work, struct kx134_data, work);

  kx134_thread_cb(drv_data->dev);
}
#endif

int kx134_trigger_drdy_set(const struct device *dev,
                            sensor_trigger_handler_t handler) {
  const struct kx134_config *cfg = dev->config;
  struct kx134_data *drv_data = dev->data;
  int status;

  if (cfg->gpio_drdy.port == NULL) {
    LOG_ERR("trigger_set DRDY int not supported");
    return -ENOTSUP;
  }

  k_mutex_lock(&drv_data->trigger_mutex, K_FOREVER);
  drv_data->drdy_handler = handler;
  k_mutex_unlock(&drv_data->trigger_mutex);

  status = kx134_reg_write_mask(dev, KX134_INC4, KX134_INC4_DRDYI1_MSK, KX134_INC4_DRDYI1_MODE(1));
  if ((handler == NULL) || (status < 0)) {
    return status;
  }

  kx134_clear_interrupts(dev);

  status = gpio_pin_interrupt_configure_dt(&cfg->gpio_drdy, GPIO_INT_EDGE_TO_ACTIVE);
  if (status < 0) {
    return status;
  }

  return kx134_reg_write_mask(dev, KX134_CNTL1, KX134_CNTL1_DRDY_EN_MSK, KX134_CNTL1_DRDY_EN_MODE(1));
}

int kx134_trigger_any_set(const struct device *dev,
                            const struct sensor_trigger *trig,
                            sensor_trigger_handler_t handler) {
  const struct kx134_config *cfg = dev->config;
  struct kx134_data *drv_data = dev->data;
  uint8_t cntl1_mask = 0, cntl1_mode = 0;
  uint8_t cntl4_mask = 0, cntl4_mode = 0;
  int status;

  if (cfg->gpio_int.port == NULL) {
    LOG_ERR("trigger_set ANY int not supported");
    return -ENOTSUP;
  }

  k_mutex_lock(&drv_data->trigger_mutex, K_FOREVER);
  drv_data->any_trigger.chan = trig->chan;
  drv_data->any_trigger.type = trig->type;
  drv_data->any_handler = handler;
  k_mutex_unlock(&drv_data->trigger_mutex);

  if (trig->type == SENSOR_TRIG_DOUBLE_TAP) {
    // Turn off WU/BTS while using double tap for this accel
    status = kx134_reg_write_mask(dev, KX134_INC6, (KX134_INC6_BTSI2_MSK | KX134_INC6_WUFI2_MSK), 
                                  (KX134_INC6_BTSI2_MODE(0) | KX134_INC6_WUFI2_MODE(0)));
    if (status < 0) {
      return status;
    }
    cntl1_mask |= KX134_CNTL1_TAP_EN_MSK;
    cntl1_mode |= KX134_CNTL1_TAP_EN_MODE(1);
    cntl4_mask |= (KX134_CNTL4_WAKE_EN_MSK | KX134_CNTL4_BTSLEEP_EN_MSK);
    cntl4_mode |= (KX134_CNTL4_WAKE_EN_MODE(0) | KX134_CNTL4_BTSLEEP_EN_MODE(0));

    #if defined(CONFIG_KX134_DTRE)
      status = kx134_set_reg(dev, CONFIG_KX134_TDTC, KX134_TDTC, 1);
      if (status < 0) {
        return status;
      }
      status = kx134_set_reg(dev, CONFIG_KX134_TTH, KX134_TTH, 1);
      if (status < 0) {
        return status;
      }
      status = kx134_set_reg(dev, CONFIG_KX134_TTL, KX134_TTL, 1);
      if (status < 0) {
        return status;
      }
      status = kx134_set_reg(dev, CONFIG_KX134_FTD, KX134_FTD, 1);
      if (status < 0) {
        return status;
      }
      status = kx134_set_reg(dev, CONFIG_KX134_STD, KX134_STD, 1);
      if (status < 0) {
        return status;
      }
      status = kx134_set_reg(dev, CONFIG_KX134_TLT, KX134_TLT, 1);
      if (status < 0) {
        return status;
      }
      status = kx134_set_reg(dev, CONFIG_KX134_TWS, KX134_TWS, 1);
      if (status < 0) {
        return status;
      }
    #endif /* CONFIG_KX134_DTRE */
  }

  if (trig->type == KX134_SENSOR_TRIG_IDLE) {
    // Turn off double tap while using WU/BTS for this accel
    status = kx134_reg_write_mask(dev, KX134_INC6, KX134_INC6_TDTI2_MSK, KX134_INC6_TDTI2_MODE(0));
    if (status < 0) {
      return status;
    }
    cntl1_mask |= KX134_CNTL1_TAP_EN_MSK;
    cntl1_mode |= KX134_CNTL1_TAP_EN_MODE(0);
    cntl4_mask |= (KX134_CNTL4_WAKE_EN_MSK | KX134_CNTL4_BTSLEEP_EN_MSK);
    cntl4_mode |= (KX134_CNTL4_WAKE_EN_MODE(IS_ENABLED(CONFIG_KX134_WUFI2)) | KX134_CNTL4_BTSLEEP_EN_MODE(IS_ENABLED(CONFIG_KX134_BTSI2)));

    #if defined(CONFIG_KX134_WUFTH)
      status = kx134_set_reg(dev, CONFIG_KX134_WUFTH, KX134_WUFTH, 1);
      if (status < 0) {
        return status;
      }
      status = kx134_reg_write_mask(dev, KX134_BTSWUFTH, KX134_WUFTH_HI_REG_MSK, KX134_WUFTH_HI_REG_MODE(CONFIG_KX134_WUFTH));
      if (status < 0) {
        return status;
      }
      status = kx134_set_reg(dev, CONFIG_KX134_WUFC, KX134_WUFC, 1);
      if (status < 0) {
        return status;
      }
    #endif /* CONFIG_KX134_WUFTH */

    #if defined(CONFIG_KX134_BTSTH)
      status = kx134_set_reg(dev, CONFIG_KX134_BTSTH, KX134_BTSTH, 1);
      if (status < 0) {
        return status;
      }
      status = kx134_reg_write_mask(dev, KX134_BTSWUFTH, KX134_BTSTH_HI_REG_MSK, KX134_BTSTH_HI_REG_MODE(CONFIG_KX134_BTSTH));
      if (status < 0) {
        return status;
      }
      status = kx134_set_reg(dev, CONFIG_KX134_BTSC, KX134_BTSC, 1);
      if (status < 0) {
        return status;
      }
    #endif /* CONFIG_KX134_BTSTH */
  }

  kx134_clear_interrupts(dev);
  
  status = gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_EDGE_TO_ACTIVE);
  if (status < 0) {
    return status;
  }

  status = kx134_reg_write_mask(dev, KX134_CNTL4, cntl4_mask, cntl4_mode);
  if (status < 0) {
    return status;
  }

  return kx134_reg_write_mask(dev, KX134_CNTL1, cntl1_mask, cntl1_mode);
}

int kx134_trigger_set(const struct device *dev,
                      const struct sensor_trigger *trig,
                      sensor_trigger_handler_t handler) {

  if (trig->type == SENSOR_TRIG_DATA_READY &&
      trig->chan == SENSOR_CHAN_ACCEL_XYZ) {
    return kx134_trigger_drdy_set(dev, handler);
  } else if (trig->chan == KX134_SENSOR_CHAN_INT_SOURCE) {
    return kx134_trigger_any_set(dev, trig, handler);
  }
  LOG_ERR("Unsupported sensor trigger");

  return -ENOTSUP;
}

int kx134_init_interrupt(const struct device *dev) {
  struct kx134_data *drv_data = dev->data;
  const struct kx134_config *cfg = dev->config;
  int status;

  k_mutex_init(&drv_data->trigger_mutex);

  /* data ready int1 gpio configuration */
  status = gpio_pin_configure_dt(&cfg->gpio_drdy, GPIO_INPUT);
  if (status < 0) {
    LOG_ERR("Could not configure %s.%02u",
        cfg->gpio_drdy.port->name, cfg->gpio_drdy.pin);
    return status;
  }

  gpio_init_callback(&drv_data->gpio_int1_cb,
      kx134_gpio_int1_callback,
      BIT(cfg->gpio_drdy.pin));

  status = gpio_add_callback(cfg->gpio_drdy.port, &drv_data->gpio_int1_cb);
  if (status < 0) {
    LOG_ERR("Could not add gpio int1 callback");
    return status;
  }

  LOG_INF("%s: int1 on %s.%02u", dev->name,
      cfg->gpio_drdy.port->name,
      cfg->gpio_drdy.pin);

  /* any other int2 gpio configuration */
  status = gpio_pin_configure_dt(&cfg->gpio_int, GPIO_INPUT);
  if (status < 0) {
    LOG_ERR("Could not configure %s.%02u",
        cfg->gpio_int.port->name, cfg->gpio_int.pin);
    return status;
  }

  gpio_init_callback(&drv_data->gpio_int2_cb,
      kx134_gpio_int2_callback,
      BIT(cfg->gpio_int.pin));

  status = gpio_add_callback(cfg->gpio_int.port, &drv_data->gpio_int2_cb);
  if (status < 0) {
    LOG_ERR("Could not add gpio int2 callback");
    return status;
  }

  LOG_INF("%s: int2 on %s.%02u", dev->name,
      cfg->gpio_int.port->name,
      cfg->gpio_int.pin);

  drv_data->dev = dev;

#if defined(CONFIG_KX134_TRIGGER_OWN_THREAD)
  k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);

  k_thread_create(&drv_data->thread,
      drv_data->thread_stack,
      CONFIG_KX134_THREAD_STACK_SIZE,
      (k_thread_entry_t)kx134_thread,
      drv_data,
      NULL,
      NULL,
      K_PRIO_COOP(CONFIG_KX134_THREAD_PRIORITY),
      0,
      K_NO_WAIT);
#elif defined(CONFIG_KX134_TRIGGER_GLOBAL_THREAD)
  drv_data->work.handler = kx134_work_cb;
#endif

  //gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_gpio,
		//		     GPIO_INT_EDGE_TO_ACTIVE);
  //gpio_pin_interrupt_configure_dt(&cfg->gpio_drdy, GPIO_INT_LEVEL_ACTIVE);
  //gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_LEVEL_ACTIVE);

  return 0;
}