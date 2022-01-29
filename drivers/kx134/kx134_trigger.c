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

static void kx134_thread_cb(const struct device *dev) {
  struct kx134_data *drv_data = dev->data;

  k_mutex_lock(&drv_data->trigger_mutex, K_FOREVER);

  if (drv_data->drdy_handler != NULL) {
    drv_data->drdy_handler(dev, &drv_data->drdy_trigger);
    kx134_clear_interrupts(dev);
  }

  if (drv_data->any_handler != NULL) {
    drv_data->any_handler(dev, &drv_data->any_trigger);
    kx134_clear_interrupts(dev);
  }
  k_mutex_unlock(&drv_data->trigger_mutex);
}

static void kx134_gpio_int1_callback(const struct device *dev, 
                                     struct gpio_callback *cb, uint32_t pins) {
  struct kx134_data *drv_data = CONTAINER_OF(cb, struct kx134_data, gpio_int1_cb);

  ARG_UNUSED(pins);
  
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

int kx134_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler) {
  struct kx134_data *drv_data = dev->data;
  uint8_t int_mask, int_en;

  switch (trig->type) {
  case SENSOR_TRIG_THRESHOLD:
    k_mutex_lock(&drv_data->trigger_mutex, K_FOREVER);
    drv_data->th_handler = handler;
    drv_data->th_trigger = *trig;
    int_mask = 0;
    break;
  case SENSOR_TRIG_DATA_READY:
    k_mutex_lock(&drv_data->trigger_mutex, K_FOREVER);
    drv_data->drdy_handler = handler;
    drv_data->drdy_trigger = *trig;
    k_mutex_unlock(&drv_data->trigger_mutex);
    int_mask = KX134_INC4_DRDYI1_MSK;
    kx134_reg_write_mask(dev, KX134_CNTL1, KX134_CNTL1_DRDY_EN_MSK, KX134_CNTL1_DRDY_EN_MODE(1));
    kx134_clear_interrupts(dev);
    break;
  case KX134_SENSOR_TRIG_ANY:
    k_mutex_lock(&drv_data->trigger_mutex, K_FOREVER);
    drv_data->any_handler = handler;
    drv_data->any_trigger = *trig;
    k_mutex_unlock(&drv_data->trigger_mutex);
    int_mask = drv_data->int1_source;
    kx134_reg_write_mask(dev, KX134_CNTL1, KX134_CNTL1_DRDY_EN_MSK, KX134_CNTL1_DRDY_EN_MODE(1));
    kx134_reg_write_mask(dev, KX134_CNTL1, KX134_CNTL1_TILT_EN_MSK, KX134_CNTL1_TAP_EN_MODE(1));
    kx134_clear_interrupts(dev);
    break;
  default:
    LOG_ERR("Unsupported sensor trigger");
    return -ENOTSUP;
  }

  if (handler) {
    int_en = int_mask;
  } else {
    int_en = 0U;
  }

  return kx134_reg_write_mask(dev, KX134_INC4, int_mask, int_en);
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
  gpio_pin_interrupt_configure_dt(&cfg->gpio_drdy, GPIO_INT_LEVEL_ACTIVE);
  gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_LEVEL_ACTIVE);

  return 0;
}
