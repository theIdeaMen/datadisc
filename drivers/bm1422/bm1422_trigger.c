/*
 * Copyright (c) 2021 Griffin Adams
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rohm_bm1422agmv

#include "bm1422.h"
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <kernel.h>
#include <sys/util.h>

#include <logging/log.h>
LOG_MODULE_DECLARE(BM1422, CONFIG_SENSOR_LOG_LEVEL);

static void bm1422_thread_cb(const struct device *dev) {
  struct bm1422_data *drv_data = dev->data;

  k_mutex_lock(&drv_data->trigger_mutex, K_FOREVER);

  if (drv_data->drdy_handler != NULL) {
    drv_data->drdy_handler(dev, &drv_data->drdy_trigger);
  }

  k_mutex_unlock(&drv_data->trigger_mutex);
}

static void bm1422_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
  struct bm1422_data *drv_data = CONTAINER_OF(cb, struct bm1422_data, gpio_cb);

#if defined(CONFIG_BM1422_TRIGGER_OWN_THREAD)
  k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_BM1422_TRIGGER_GLOBAL_THREAD)
  k_work_submit(&drv_data->work);
#endif
}

#if defined(CONFIG_BM1422_TRIGGER_OWN_THREAD)
static void bm1422_thread(struct bm1422_data *drv_data) {
  while (true) {
    k_sem_take(&drv_data->gpio_sem, K_FOREVER);
    bm1422_thread_cb(drv_data->dev);
  }
}

#elif defined(CONFIG_BM1422_TRIGGER_GLOBAL_THREAD)
static void bm1422_work_cb(struct k_work *work) {
  struct bm1422_data *drv_data = CONTAINER_OF(work, struct bm1422_data, work);

  bm1422_thread_cb(drv_data->dev);
}
#endif

int bm1422_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler) {
  struct bm1422_data *drv_data = dev->data;
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
    int_mask = BM1422_CNTL2_DREN_MSK;
    bm1422_clear_interrupts(dev);
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

  return 0;
}

int bm1422_init_interrupt(const struct device *dev) {
  struct bm1422_data *drv_data = dev->data;
  const struct bm1422_config *cfg = dev->config;
  int status;

  k_mutex_init(&drv_data->trigger_mutex);

  status = gpio_pin_configure_dt(&cfg->gpio_drdy, GPIO_INPUT);
  if (status < 0) {
    LOG_ERR("Could not configure %s.%02u",
        cfg->gpio_drdy.port->name, cfg->gpio_drdy.pin);
    return status;
  }
  
  gpio_init_callback(&drv_data->gpio_cb,
      bm1422_gpio_callback,
      BIT(cfg->gpio_drdy.pin));

  status = gpio_add_callback(cfg->gpio_drdy.port, &drv_data->gpio_cb);
  if (status < 0) {
    LOG_ERR("Could not add gpio int callback");
    return status;
  }

  LOG_INF("%s: int on %s.%02u", dev->name,
    cfg->gpio_drdy.port->name,
    cfg->gpio_drdy.pin);

  drv_data->dev = dev;

#if defined(CONFIG_BM1422_TRIGGER_OWN_THREAD)
  k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);

  k_thread_create(&drv_data->thread,
      drv_data->thread_stack,
      CONFIG_BM1422_THREAD_STACK_SIZE,
      (k_thread_entry_t)bm1422_thread,
      drv_data,
      NULL,
      NULL,
      K_PRIO_COOP(CONFIG_BM1422_THREAD_PRIORITY),
      0,
      K_NO_WAIT);
#elif defined(CONFIG_BM1422_TRIGGER_GLOBAL_THREAD)
  drv_data->work.handler = bm1422_work_cb;
#endif

  status = gpio_pin_interrupt_configure_dt(&cfg->gpio_drdy, GPIO_INT_EDGE_TO_ACTIVE);
  if (status < 0) {
    LOG_ERR("Could not config gpio int");
    return status;
  }

  return 0;
}