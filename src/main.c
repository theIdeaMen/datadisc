/*
 * Copyright (c) 2022 Griffin Adams
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <errno.h>
#include <version.h>

#include <SEGGER_RTT.h>
#include <logging/log.h>
#include <logging/log_ctrl.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <sys/byteorder.h>
#include <shell/shell.h>

#include <ff.h>
#include <fs/fs.h>
#include <storage/flash_map.h>
#include <usb/usb_device.h>

#include <drivers/flash.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include <drivers/sensor.h>
#include <drivers/uart.h>

//#include <settings/settings.h>

//#include <bluetooth/bluetooth.h>
//#include <bluetooth/conn.h>
//#include <bluetooth/gatt.h>
//#include <bluetooth/hci.h>
//#include <bluetooth/services/bas.h>
//#include <bluetooth/uuid.h>

//#include "battery.h"
#include "kx134.h"
#include "bm1422.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);


/***************************************************************
*   Global Variables
*
****************************************************************/
typedef enum {
  IDLE,
  INIT,
  LOG,
  ERASE,
  DUMP,
  SLEEP
} Machine_State;
Machine_State datadisc_state = INIT;


/* file system things */
#define MAX_PATH_LEN 150

#if CONFIG_DISK_DRIVER_FLASH
#include <storage/flash_map.h>
#endif

#if CONFIG_FILE_SYSTEM_LITTLEFS
#include <fs/littlefs.h>
//FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);
FS_LITTLEFS_DECLARE_CUSTOM_CONFIG(storage, 32, 32, 64, 16);
#endif

static struct fs_mount_t fs_mnt;


/** A discharge curve specific to the power source. */
// TODO measure DataDisc battery curve
//static const struct battery_level_point levels[] = {
//#if DT_NODE_HAS_PROP(DT_INST(0, voltage_divider), io_channels)
    /* "Curve" here eyeballed from captured data for the [Adafruit
	 * 3.7v 2000 mAh](https://www.adafruit.com/product/2011) LIPO
	 * under full load that started with a charge of 3.96 V and
	 * dropped about linearly to 3.58 V over 15 hours.  It then
	 * dropped rapidly to 3.10 V over one hour, at which point it
	 * stopped transmitting.
	 *
	 * Based on eyeball comparisons we'll say that 15/16 of life
	 * goes between 3.95 and 3.55 V, and 1/16 goes between 3.55 V
	 * and 3.1 V.
	 */

//    {10000, 3950},
//    {625, 3550},
//    {0, 3100},
//#else
//    /* Linear from maximum voltage to minimum voltage. */
//    {10000, 3600},
//    {0, 1700},
//#endif
//};


unsigned int soc_percent = 0;


/* Thread things */
/* size of stack area used by most threads */
#define STACKSIZE 2048

/* scheduling priority used by each thread */
#define PRIORITY 7

/* delay for each thread to allow inits */
#define TDELAY 100



/***************************************************************
*   Bluetooth Functions
*
****************************************************************/
//#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
//#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

//// Set Advertisement data.
//static const struct bt_data ad[] = {
//    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
//};

//// Set Scan Response data
//static const struct bt_data sd[] = {
//    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
//};

//static void bt_ready(void) {
//  int err;

//  printk("Bluetooth initialized\n");

//  if (IS_ENABLED(CONFIG_SETTINGS)) {
//    settings_load();
//  }

//  err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
//  if (err) {
//    printk("Advertising failed to start (err %d)\n", err);
//    return;
//  }

//  printk("Advertising successfully started\n");
//}

/***************************************************************
*   Utility Functions
*
****************************************************************/

static void wait_on_log_flushed(void) {
  while (log_buffered_cnt()) {
    k_sleep(K_MSEC(15));
  }
}

uint64_t uptime_get_us(void) {

  return k_uptime_ticks() * 1000000 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;

}

static inline int32_t sensor_value_to_32(const struct sensor_value *val)
{
	return val->val1 * 1000000 + val->val2;
}

static const char *now_str(void) {
  static char buf[16]; /* ...HH:MM:SS.MMM */
  uint32_t now = k_uptime_get();
  unsigned int ms = now % MSEC_PER_SEC;
  unsigned int s;
  unsigned int min;
  unsigned int h;

  now /= MSEC_PER_SEC;
  s = now % 60U;
  now /= 60U;
  min = now % 60U;
  now /= 60U;
  h = now;

  snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u", h, min, s, ms);
  return buf;
}

static int setup_flash(struct fs_mount_t *mnt) {
  int rc = 0;
#if CONFIG_DISK_DRIVER_FLASH
  unsigned int id;
  const struct flash_area *pfa;

  mnt->storage_dev = (void *)FLASH_AREA_ID(storage);
  id = (uintptr_t)mnt->storage_dev;

  rc = flash_area_open(id, &pfa);
  printk("Area %u at 0x%x on %s for %u bytes\n",
      id, (unsigned int)pfa->fa_off, pfa->fa_dev_name,
      (unsigned int)pfa->fa_size);

  if (rc == 0 && IS_ENABLED(CONFIG_APP_WIPE_STORAGE)) {
    printk("Erasing flash area ... ");
    rc = flash_area_erase(pfa, 0, pfa->fa_size);
    printk("%d\n", rc);
  }

  if (rc < 0) {
    flash_area_close(pfa);
  }
#endif
  return rc;
}

static int mount_app_fs(struct fs_mount_t *mnt) {
  int rc;

#if CONFIG_FAT_FILESYSTEM_ELM
  static FATFS fat_fs;

  mnt->type = FS_FATFS;
  mnt->fs_data = &fat_fs;
  if (IS_ENABLED(CONFIG_DISK_DRIVER_RAM)) {
    mnt->mnt_point = "/RAM:";
  } else if (IS_ENABLED(CONFIG_DISK_DRIVER_SDMMC)) {
    mnt->mnt_point = "/SD:";
  } else {
    mnt->mnt_point = "/NAND:";
  }

#elif CONFIG_FILE_SYSTEM_LITTLEFS
  mnt->type = FS_LITTLEFS;
  mnt->mnt_point = "/lfs";
  mnt->fs_data = &storage;
#endif
  rc = fs_mount(mnt);

  return rc;
}

uint8_t boot_count;

static void setup_disk(void) {
  struct fs_mount_t *mp = &fs_mnt;
  struct fs_dir_t dir;
  struct fs_statvfs sbuf;
  char fname[MAX_PATH_LEN];
  int rc;

  fs_dir_t_init(&dir);

  if (IS_ENABLED(CONFIG_DISK_DRIVER_FLASH)) {
    rc = setup_flash(mp);
    if (rc < 0) {
      LOG_ERR("Failed to setup flash area");
      return;
    }
  }

  if (!IS_ENABLED(CONFIG_FILE_SYSTEM_LITTLEFS) &&
      !IS_ENABLED(CONFIG_FAT_FILESYSTEM_ELM)) {
    LOG_INF("No file system selected");
    return;
  }

  rc = mount_app_fs(mp);
  if (rc < 0) {
    LOG_ERR("Failed to mount filesystem");
    return;
  }

  /* Allow log messages to flush to avoid interleaved output */
  k_sleep(K_MSEC(50));

  printk("Mount %s: %d\n", fs_mnt.mnt_point, rc);

  rc = fs_statvfs(mp->mnt_point, &sbuf);
  if (rc < 0) {
    printk("FAIL: statvfs: %d\n", rc);
    return;
  }

  printk("%s: bsize = %lu ; frsize = %lu ;"
         " blocks = %lu ; bfree = %lu\n",
      mp->mnt_point,
      sbuf.f_bsize, sbuf.f_frsize,
      sbuf.f_blocks, sbuf.f_bfree);

  rc = fs_opendir(&dir, mp->mnt_point);
  printk("%s opendir: %d\n", mp->mnt_point, rc);

  if (rc < 0) {
    LOG_ERR("Failed to open directory");
  }

  while (rc >= 0) {
    struct fs_dirent ent = {0};

    rc = fs_readdir(&dir, &ent);
    if (rc < 0) {
      LOG_ERR("Failed to read directory entries");
      break;
    }
    if (ent.name[0] == 0) {
      printk("End of files\n");
      break;
    }
    printk("  %c %u %s\n",
        (ent.type == FS_DIR_ENTRY_FILE) ? 'F' : 'D',
        ent.size,
        ent.name);
  }

  (void)fs_closedir(&dir);

  snprintf(fname, sizeof(fname), "%s/boot_count", mp->mnt_point);

  struct fs_file_t file;

  fs_file_t_init(&file);

  rc = fs_open(&file, fname, FS_O_CREATE | FS_O_RDWR);
  if (rc < 0) {
    printk("FAIL: open %s: %d\n", fname, rc);
    return;
  }

  boot_count = 0;

  if (rc >= 0) {
    rc = fs_read(&file, &boot_count, sizeof(boot_count));
    printk("%s read count %u: %d\n", fname, boot_count, rc);
    rc = fs_seek(&file, 0, FS_SEEK_SET);
    printk("%s seek start: %d\n", fname, rc);
  }

  boot_count += 1;
  rc = fs_write(&file, &boot_count, sizeof(boot_count));
  printk("%s write new boot count %u: %d\n", fname,
      boot_count, rc);

  rc = fs_close(&file);
  printk("%s close: %d\n", fname, rc);

  return;
}

/***************************************************************
*   Threads
*
****************************************************************/
/* Condition variable for main initialize done */
K_MUTEX_DEFINE(init_mut);
K_CONDVAR_DEFINE(init_cond);


/********************************************
 * Battery check
 ********************************************/
//void batt_check_thread(void) {

//  int off_time; // turn off divider to save power (ms)

//  while (1) {
//    switch (datadisc_state) {
//    case LOG:
//      off_time = 700;
//      break;

//    default: // TODO: decide on times for other states
//      off_time = 2700;
//      break;
//    }

//    battery_measure_enable(true);

//    k_msleep(300);
//    int batt_mV = battery_sample();

//    battery_measure_enable(false);

//    if (batt_mV < 0) {
//      printk("Failed to read battery voltage: %d\n", batt_mV);
//    }

//    unsigned int batt_pptt = battery_level_pptt(batt_mV, levels);

//    printk("[%s]: %d mV; %u pptt\n", now_str(), batt_mV, batt_pptt);

//    soc_percent = batt_pptt / 100;

//    k_msleep(off_time);
//  }
//}

//K_THREAD_DEFINE(batt_check_id, STACKSIZE, batt_check_thread,
//    NULL, NULL, NULL, PRIORITY, 0, TDELAY);


/********************************************
 * LED Control
 ********************************************/
#define PWM_LED0_NODE DT_ALIAS(pwm_led0)

#if DT_NODE_HAS_STATUS(PWM_LED0_NODE, okay)
#define PWM_CTLR DT_PWMS_CTLR(PWM_LED0_NODE)
#define PWM_CHANNEL DT_PWMS_CHANNEL(PWM_LED0_NODE)
#define PWM_FLAGS DT_PWMS_FLAGS(PWM_LED0_NODE)
#define PWM_NAME DT_LABEL(PWM_LED0_NODE)
#else
#error "Unsupported board: pwm-led0 devicetree alias is not defined"
#define PWM_CTLR DT_INVALID_NODE
#define PWM_CHANNEL 0
#define PWM_FLAGS 0
#endif

#define MAX_BRIGHTNESS 75

#define FADE_DELAY 8

#define MIN_PERIOD_USEC (USEC_PER_SEC / 64U)
#define MAX_PERIOD_USEC USEC_PER_SEC

#define M_E 2.71828182845904523536
#define SCALING_CONST (MAX_BRIGHTNESS / (M_E - (1 / M_E)))

void led_control_thread(void) {

  const struct device *pwm;
  int err;
  uint16_t level = 0;
  uint16_t time_now;

  pwm = DEVICE_DT_GET(PWM_CTLR);
  if (pwm) {
    LOG_INF("Found device %s", PWM_NAME);
  } else {
    LOG_ERR("Device %s not found", PWM_NAME);
    return;
  }

  while (1) {

    switch (datadisc_state) {
    case INIT:
      // Fast breathe
      level = (exp(sin(10.0*(k_uptime_get()/1000.0))) - (1.0 / M_E)) * SCALING_CONST;
      
      // Slow breathe
      //level = (exp(sin(3.0*(k_uptime_get()/1000.0))) - (1.0 / M_E)) * SCALING_CONST;

      break;

    case IDLE:
      // One dim, slow pulse
      time_now = (uint16_t)k_uptime_get();

      if (time_now % 5000 < 300) {
        level = MAX_BRIGHTNESS / 3;
      }
      else {
        level = 0;
      }

      break;

    case LOG:
      // Three fast pulses
      time_now = (uint16_t)k_uptime_get();

      if ((time_now % 200 < 20) ^ (time_now % 300 < 20)) {
        level = MAX_BRIGHTNESS;
      }
      else {
        level = 0;
      }
      
      break;

    default: // TODO: decide on times for other states
      
      level = MAX_BRIGHTNESS;

      break;
    }

    err = pwm_pin_set_usec(pwm, PWM_CHANNEL, MIN_PERIOD_USEC, (MIN_PERIOD_USEC * level) / 100U, PWM_FLAGS);
    if (err < 0) {
      LOG_ERR("err=%d", err);
      return;
    }
    k_msleep(10);
  }
}

K_THREAD_DEFINE(led_control_id, STACKSIZE, led_control_thread,
    NULL, NULL, NULL, PRIORITY, 0, 0);



/********************************************
 * MSGQ buffers
 ********************************************/
struct accel_msgq_item_t {
  uint8_t id;
  uint64_t timestamp;
  struct sensor_value data[3];
} __packed;

struct datalog_msgq_item_t {
  size_t length;
  uint8_t data[50];
} __packed;

K_MSGQ_DEFINE(accel_msgq, sizeof(struct accel_msgq_item_t), 500, 4);
K_MSGQ_DEFINE(datalog_msgq, sizeof(struct datalog_msgq_item_t), 4000, 4);


/********************************************
 * Accelerometers
 ********************************************/
#define ACCEL_ALPHA_DEVICE DT_LABEL(DT_INST(0, kionix_kx134_1211))
#define ACCEL_BETA_DEVICE DT_LABEL(DT_INST(1, kionix_kx134_1211))

/* Unique IDs to carry into CSV */
#define ACCEL_ALPHA_ID  0x1A;
#define ACCEL_BETA_ID   0x2B;

K_SEM_DEFINE(sem_a, 0, 1);
K_SEM_DEFINE(sem_b, 0, 1);

static void accel_alpha_trigger_handler(const struct device *dev, struct sensor_trigger *trigger) {

  if (sensor_sample_fetch(dev)) {
    LOG_ERR("sensor_sample_fetch failed");
    return;
  }

  k_sem_give(&sem_a);
}

void accel_alpha_thread(void) {

  k_mutex_lock(&init_mut, K_FOREVER);

  while (datadisc_state == INIT) {
    k_condvar_wait(&init_cond, &init_mut, K_FOREVER);
  }

  k_mutex_unlock(&init_mut);

  const struct device *dev = device_get_binding(ACCEL_ALPHA_DEVICE);
  struct sensor_value int_source;
  struct accel_msgq_item_t msgq_item;

  if (!dev) {
    LOG_ERR("Devicetree has no kionix,kx134-1211 node");
    return;
  }
  if (!device_is_ready(dev)) {
    LOG_ERR("Device %s is not ready", log_strdup(dev->name));
    return;
  }

  struct sensor_trigger trig = {
      .type = KX134_SENSOR_TRIG_ANY,
      .chan = SENSOR_CHAN_ACCEL_XYZ,
  };

  if (sensor_trigger_set(dev, &trig, accel_alpha_trigger_handler)) {
    LOG_ERR("Could not set trigger");
    return;
  }

  struct kx134_data *drv_data = dev->data;
  LOG_INF("dev: %p, cb: %p", dev, drv_data->any_handler);

  while (1) {
    k_sem_take(&sem_a, K_FOREVER);

    sensor_channel_get(dev, KX134_SENSOR_CHAN_INT_SOURCE, &int_source);

    if (KX134_INS2_DRDY(int_source.val1) && datadisc_state == LOG) {

      msgq_item.id = ACCEL_ALPHA_ID;
      msgq_item.timestamp = uptime_get_us();

      sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, msgq_item.data);
      
      /* send data to consumers */
      while (k_msgq_put(&accel_msgq, &msgq_item, K_NO_WAIT) != 0) {
        /* message queue is full: purge old data & try again */
        k_msgq_purge(&accel_msgq);
      }
    }
  }
}

K_THREAD_DEFINE(accel_alpha_id, STACKSIZE, accel_alpha_thread,
    NULL, NULL, NULL, PRIORITY, 0, TDELAY);


static void accel_beta_trigger_handler(const struct device *dev, struct sensor_trigger *trigger) {

  if (sensor_sample_fetch(dev)) {
    LOG_ERR("sensor_sample_fetch failed");
    return;
  }

  k_sem_give(&sem_b);
}


void accel_beta_thread(void) {

  k_mutex_lock(&init_mut, K_FOREVER);

  while (datadisc_state == INIT) {
    k_condvar_wait(&init_cond, &init_mut, K_FOREVER);
  }

  k_mutex_unlock(&init_mut);

  const struct device *dev = device_get_binding(ACCEL_BETA_DEVICE);
  struct sensor_value int_source;
  struct accel_msgq_item_t msgq_item;

  if (!dev) {
    LOG_ERR("Devicetree has no kionix,kx134-1211 node");
    return;
  }
  if (!device_is_ready(dev)) {
    LOG_ERR("Device %s is not ready", log_strdup(dev->name));
    return;
  }

  struct sensor_trigger trig = {
      .type = KX134_SENSOR_TRIG_ANY,
      .chan = SENSOR_CHAN_ACCEL_XYZ,
  };

  if (sensor_trigger_set(dev, &trig, accel_beta_trigger_handler)) {
    LOG_ERR("Could not set trigger");
    return;
  }

  struct kx134_data *drv_data = dev->data;
  LOG_INF("dev: %p, cb: %p", dev, drv_data->any_handler);

  while (1) {

    k_sem_take(&sem_b, K_FOREVER);

    sensor_channel_get(dev, KX134_SENSOR_CHAN_INT_SOURCE, &int_source);

    if (KX134_INS2_DRDY(int_source.val1) && datadisc_state == LOG) {

      msgq_item.id = ACCEL_BETA_ID;
      msgq_item.timestamp = uptime_get_us();

      sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, msgq_item.data);
      
      /* send data to consumers */
      while (k_msgq_put(&accel_msgq, &msgq_item, K_NO_WAIT) != 0) {
        /* message queue is full: purge old data & try again */
        k_msgq_purge(&accel_msgq);
      }
    }

    // Double tap int source
    if (KX134_INS2_DTS(int_source.val1)) {

      msgq_item.id = 0x00;
      msgq_item.timestamp = uptime_get_us();

      /* Toggle DataDisc State */
      //if (datadisc_state == IDLE) {
      //  datadisc_state = LOG;
      //} else {
      //  datadisc_state = IDLE;
      //}

      printk("[%s] Double Tap!\n", now_str());
      
      /* send data to consumers */
      while (k_msgq_put(&accel_msgq, &msgq_item, K_NO_WAIT) != 0) {
        /* message queue is full: purge old data & try again */
        k_msgq_purge(&accel_msgq);
      }
    }
  }
}

K_THREAD_DEFINE(accel_beta_id, STACKSIZE, accel_beta_thread,
    NULL, NULL, NULL, PRIORITY, 0, TDELAY);


/********************************************
 * Magnetometer
 ********************************************/
#define MAGN_DEVICE DT_LABEL(DT_INST(0, rohm_bm1422agmv))

/* Unique IDs to carry into CSV */
#define MAGN_ID  0x3C;

K_SEM_DEFINE(magn_sem, 0, 1);

static void magn_trigger_handler(const struct device *dev, struct sensor_trigger *trigger) {

  if (sensor_sample_fetch(dev)) {
    LOG_ERR("sensor_sample_fetch failed");
    return;
  }

  k_sem_give(&magn_sem);
}

void magn_thread(void) {

  k_mutex_lock(&init_mut, K_FOREVER);

  while (datadisc_state == INIT) {
    k_condvar_wait(&init_cond, &init_mut, K_FOREVER);
  }

  k_mutex_unlock(&init_mut);

  const struct device *dev = device_get_binding(MAGN_DEVICE);
  struct accel_msgq_item_t msgq_item;

  if (!dev) {
    LOG_ERR("Devicetree has no rohm,bm1422agmv node");
    return;
  }
  if (!device_is_ready(dev)) {
    LOG_ERR("Device %s is not ready", log_strdup(dev->name));
    return;
  }

  struct sensor_trigger trig = {
      .type = SENSOR_TRIG_DATA_READY,
      .chan = SENSOR_CHAN_MAGN_XYZ,
  };

  if (sensor_trigger_set(dev, &trig, magn_trigger_handler)) {
    LOG_ERR("Could not set trigger");
    return;
  }

  while (1) {

    k_sem_take(&magn_sem, K_FOREVER);

    if (datadisc_state == LOG) {

      msgq_item.id = MAGN_ID;
      msgq_item.timestamp = uptime_get_us();

      sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, msgq_item.data);

      /* send data to consumers */
      while (k_msgq_put(&accel_msgq, &msgq_item, K_NO_WAIT) != 0) {
        /* message queue is full: purge old data & try again */
        k_msgq_purge(&accel_msgq);
      }
    }
  }
}

K_THREAD_DEFINE(magn_id, STACKSIZE, magn_thread,
    NULL, NULL, NULL, PRIORITY, 0, TDELAY);


/********************************************
 * Thread for crunching data during runtime
 ********************************************/
void runtime_compute_thread(void) {

  k_mutex_lock(&init_mut, K_FOREVER);

  while (datadisc_state == INIT) {
    k_condvar_wait(&init_cond, &init_mut, K_FOREVER);
  }

  k_mutex_unlock(&init_mut);

  struct accel_msgq_item_t msgq_item;
  struct datalog_msgq_item_t data_item;
  struct datalog_msgq_item_t throw_away_item;
  int32_t x_value, y_value, z_value;

  // TODO: Accel averaging, spin rate, etc.
  while (1) {

    k_msgq_get(&accel_msgq, &msgq_item, K_FOREVER);

    switch (msgq_item.id) {
    case 0x00:
      data_item.length = snprintf(data_item.data, sizeof(data_item.data), "%dt,%lu,-99999,-99999,-99999\n",
          msgq_item.id, msgq_item.timestamp);
      break;

    case 0x1A:
    case 0x2B:
    case 0x3C:
      x_value = sensor_value_to_32(&msgq_item.data[0]);
      y_value = sensor_value_to_32(&msgq_item.data[1]);
      z_value = sensor_value_to_32(&msgq_item.data[2]);

      data_item.length = snprintf(data_item.data, sizeof(data_item.data), "%02X,%u,%d,%d,%d\n",
          msgq_item.id, (uint32_t)msgq_item.timestamp, x_value, y_value, z_value);
      break;

    case 0xFF:
    default:
      data_item.length = snprintf(data_item.data, sizeof(data_item.data), "%no_data,%lu\n",
          msgq_item.id, msgq_item.timestamp);
      break;
    }
    LOG_INF("[0x%X] %d", msgq_item.id, k_msgq_num_free_get(&datalog_msgq));

    /* Send the string to the FLASH write thread */
    while (k_msgq_put(&datalog_msgq, &data_item, K_NO_WAIT) != 0) {
      /* message queue is full: purge old data & try again */
      k_msgq_get(&datalog_msgq, &throw_away_item, K_NO_WAIT);
    }
  }
}

K_THREAD_DEFINE(runtime_compute_id, STACKSIZE, runtime_compute_thread,
    NULL, NULL, NULL, PRIORITY+1, 0, TDELAY);



/********************************************
 * Q-SPI FLASH
 ********************************************/
uint8_t fname[MAX_PATH_LEN];    // Buffer created outside thread to avoid stack overflow

void spi_flash_thread(void) {

  k_mutex_lock(&init_mut, K_FOREVER);

  while (datadisc_state == INIT) {
    k_condvar_wait(&init_cond, &init_mut, K_FOREVER);
  }

  k_mutex_unlock(&init_mut);

  //k_thread_system_pool_assign(k_current_get());

  struct fs_mount_t *mp = &fs_mnt;
  struct fs_file_t file;
  unsigned int id = (uintptr_t)mp->storage_dev;
  uint64_t log_start_time;
  int rc;
  uint16_t data_size = 0;
  uint8_t buffer[150];
  struct datalog_msgq_item_t data_item;

  log_start_time = k_uptime_get();

  if (!mp->mnt_point) {
    LOG_ERR("FAIL: mount id %u at %s", id, log_strdup(mp->mnt_point));
    return;
  }
  LOG_INF("%s mount\n", log_strdup(mp->mnt_point));

  snprintf(fname, sizeof(fname), "%s/%u_%lu.csv", mp->mnt_point, boot_count, (uint32_t)log_start_time);

  fs_file_t_init(&file);

  rc = fs_open(&file, fname, FS_O_CREATE | FS_O_RDWR);
  if (rc < 0) {
    LOG_ERR("FAIL: open %s: %d\n", log_strdup(fname), rc);
    goto out;
  }

  snprintf(buffer, 5, "SOL\n");

  rc = fs_write(&file, buffer, strlen(buffer));
  if (rc < 0) {
    LOG_ERR("FAIL: write %s: %d\n", log_strdup(buffer), rc);
    goto out;
  }

  rc = fs_sync(&file);
  if (rc < 0) {
    LOG_ERR("FAIL: sync %s: %d\n", log_strdup(fname), rc);
    goto out;
  }

  //datadisc_state = LOG;

  while (1) {

    k_msgq_get(&datalog_msgq, &data_item, K_FOREVER);

    rc = fs_write(&file, data_item.data, data_item.length);
    
    if (rc < 0) {
      LOG_ERR("FAIL: write %s: %d\n", log_strdup(fname), rc);
      goto out;
    }
    data_size += data_item.length;

    if (data_size >= 1024) {
      rc = fs_sync(&file);
      if (rc < 0) {
        LOG_ERR("FAIL: sync %s: %d\n", fname, rc);
        goto out;
      }
      data_size = 0;
    }

    if (datadisc_state != LOG && k_msgq_num_used_get(&datalog_msgq) <= 0) {
      goto out;
    }

    if ((uint64_t)k_uptime_get() - log_start_time > 20000) {
      datadisc_state = IDLE;
    }
  }

out:
  datadisc_state = IDLE;

  rc = fs_close(&file);
  LOG_INF("%s close: %d\n", log_strdup(fname), rc);

  k_msgq_purge(&accel_msgq);
  k_msgq_purge(&datalog_msgq);
}

K_THREAD_DEFINE(spi_flash_id, STACKSIZE, spi_flash_thread,
    NULL, NULL, NULL, PRIORITY+2, 0, TDELAY);


/***************************************************************
*   Main
*
****************************************************************/
void main(void) {

  int err;

  k_mutex_lock(&init_mut, K_FOREVER);

  SEGGER_RTT_Init();

  printk("Starting DataDisc v2\n");

  setup_disk();

  k_msleep(2);

  err = usb_enable(NULL);
  if (err != 0) {
    LOG_ERR("Failed to enable USB");
    return;
  }

  LOG_INF("USB mass storage setup complete.\n");

  #if DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_shell_uart), zephyr_cdc_acm_uart)
      const struct device *dev;
      uint32_t dtr = 0;

      dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
      if (!device_is_ready(dev) || usb_enable(NULL)) {
              return;
      }

      while (!dtr) {
              uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
              k_sleep(K_MSEC(100));
      }
      LOG_INF("Virtual COM port setup complete.\n");
  #endif

  datadisc_state = IDLE;

  k_condvar_signal(&init_cond);
  k_mutex_unlock(&init_mut);

  // Initialize the Bluetooth Subsystem
  //err = bt_enable(NULL);
  //if (err) {
  //  printk("Bluetooth init failed (err %d)\n", err);
  //  return;
  //}

  //bt_ready();

  //dev = device_get_binding("GPIO_0");

  //err = gpio_pin_configure(dev, 10, (GPIO_OUTPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW));
  //if (err < 0) {
  //        return;
  //}


  // Main loop
  while (1) {

    /* Battery level */
    //bt_bas_set_battery_level(soc_percent);

    //gpio_pin_toggle(dev, 10);

    k_msleep(200);
  }
}



/***************************************************************
*   Shell Commands
*
****************************************************************/
/* DataDisc Shell Commands */
static int setstate_cmd_handler(const struct shell *shell,
                            size_t argc, char **argv, void *data)
{
        datadisc_state = (int)data;

        shell_print(shell, "State set to: %s\n"
                           "Value sent to Main: %d",
                           argv[0],
                           (int)data);

        return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_setstate, setstate_cmd_handler,
        (idle, 0), (init, 1), (log, 2), (erase, 3), (dump, 4), (sleep, 5)
);

SHELL_CMD_REGISTER(setstate, &sub_setstate, "Set DataDisc State", NULL);


/* Utility Shell Commands */
static int cmd_demo_ping(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "pong");

	return 0;
}


static int cmd_version(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "Zephyr version %s", KERNEL_VERSION_STRING);

	return 0;
}


SHELL_CMD_REGISTER(ping, NULL, "Demo commands", cmd_demo_ping);

SHELL_CMD_ARG_REGISTER(version, NULL, "Show kernel version", cmd_version, 1, 0);