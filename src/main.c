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
#include <sys/util.h>
#include <sys/byteorder.h>
#include <sys/reboot.h>
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

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>
#include <bluetooth/services/bas.h>

//#include <mgmt/mcumgr/smp_bt.h>
//#include "os_mgmt/os_mgmt.h"
//#include "img_mgmt/img_mgmt.h"

#include "battery.h"
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
volatile Machine_State datadisc_state = INIT;

static const char *const state_strings[] = {
    [IDLE]  = "IDLE",
    [INIT]  = "INIT",
    [LOG]   = "LOG",
    [ERASE] = "ERASE",
    [DUMP]  = "DUMP",
    [SLEEP] = "SLEEP"
};

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


/********************************************
 * MSGQ buffers
 ********************************************/
typedef struct {
  uint32_t timestamp;
  float data_x;
  float data_y;
  float data_z;
  uint8_t id;
  uint8_t length;
}__attribute__((aligned(4))) datalog_msgq_item_t;

K_MSGQ_DEFINE(accel_msgq, sizeof(datalog_msgq_item_t), 500, 4);
K_MSGQ_DEFINE(datalog_msgq, sizeof(datalog_msgq_item_t), 5000, 4);

struct Comp_Data {
  float prev_magn;
  float now_magn;
  uint64_t prev_time;
  uint64_t now_time;
};


/** A discharge curve specific to the power source. */
// TODO measure DataDisc battery curve
static const struct battery_level_point levels[] = {
#if DT_NODE_HAS_PROP(DT_INST(0, voltage_divider), io_channels)
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

    {10000, 3950},
    {625, 3550},
    {0, 3100},
#else
    /* Linear from maximum voltage to minimum voltage. */
    {10000, 3600},
    {0, 1700},
#endif
};


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
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

// Set Advertisement data.
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

// Set Scan Response data
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      0x84, 0xaa, 0x60, 0x74, 0x52, 0x8a, 0x8b, 0x86,
		      0xd3, 0x4c, 0xb7, 0x1d, 0x1d, 0xdc, 0x53, 0x8d),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void connected(struct bt_conn *conn, uint8_t err) {
  if (err) {
    LOG_ERR("Connection failed (err %u)\n", err);
    return;
  }

  LOG_INF("Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
  LOG_INF("Disconnected (reason %u)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void datadisc_bt_init() {
  int err;

  err = bt_enable(NULL);
  if (err) {
    LOG_ERR("Bluetooth init failed (err %d)\n", err);
    return;
  }

  LOG_INF("Bluetooth initialized\n");

  err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (err) {
    LOG_ERR("Advertising failed to start (err %d)\n", err);
    return;
  }

  LOG_INF("Advertising successfully started\n");
}


/***************************************************************
*   Utility Functions
*
****************************************************************/
static inline float magnitude(const datalog_msgq_item_t *item) {
  return sqrtf(item->data_x*item->data_x + item->data_y*item->data_y + item->data_z*item->data_z);
}

float running_jerk(struct Comp_Data *data) {
  float jerk;
  jerk = abs(data->now_magn - data->prev_magn) / ((float)(data->now_time - data->prev_time)/1000000.0);
  data->prev_magn = data->now_magn;
  data->prev_time = data->now_time;
  return jerk;
}

static void wait_on_log_flushed(void) {
  while (log_buffered_cnt()) {
    k_sleep(K_MSEC(15));
  }
}

uint32_t uptime_get_us(void) {
  return (uint32_t)(k_uptime_ticks() * 1000000 / CONFIG_SYS_CLOCK_TICKS_PER_SEC);
}

static inline int32_t sensor_value_to_32(const struct sensor_value *val) {
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
  LOG_INF("Area %u at 0x%x on %s for %u bytes\n",
      id, (unsigned int)pfa->fa_off, log_strdup(pfa->fa_dev_name),
      (unsigned int)pfa->fa_size);

  if (rc == 0 && (IS_ENABLED(CONFIG_APP_WIPE_STORAGE) || datadisc_state == ERASE)) {
    LOG_INF("Erasing flash area ... ");
    rc = flash_area_erase(pfa, 0, pfa->fa_size);
    LOG_INF("%d\n", rc);
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

  if (mp->mnt_point != NULL) {
    rc = fs_unmount(mp);
    if (rc < 0) {
      LOG_ERR("Failed to unmount filesystem");
      return;
    }
  }

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

  LOG_INF("Mount %s: %d\n", log_strdup(fs_mnt.mnt_point), rc);

  rc = fs_statvfs(mp->mnt_point, &sbuf);
  if (rc < 0) {
    LOG_ERR("FAIL: statvfs: %d\n", rc);
    return;
  }

  LOG_INF("%s: bsize = %lu ; frsize = %lu ;"
         " blocks = %lu ; bfree = %lu\n",
      log_strdup(mp->mnt_point),
      sbuf.f_bsize, sbuf.f_frsize,
      sbuf.f_blocks, sbuf.f_bfree);

  rc = fs_opendir(&dir, mp->mnt_point);
  LOG_INF("%s opendir: %d\n", log_strdup(mp->mnt_point), rc);

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
      LOG_INF("End of files\n");
      break;
    }
    LOG_INF("  %c %u %s\n",
        (ent.type == FS_DIR_ENTRY_FILE) ? 'F' : 'D',
        ent.size,
        log_strdup(ent.name));
  }

  (void)fs_closedir(&dir);

  snprintf(fname, sizeof(fname), "%s/boot_count", log_strdup(mp->mnt_point));

  struct fs_file_t file;

  fs_file_t_init(&file);

  rc = fs_open(&file, fname, FS_O_CREATE | FS_O_RDWR);
  if (rc < 0) {
    LOG_ERR("FAIL: open %s: %d\n", log_strdup(fname), rc);
    return;
  }

  boot_count = 0;

  if (rc >= 0) {
    rc = fs_read(&file, &boot_count, sizeof(boot_count));
    LOG_INF("%s read count %u: %d\n", log_strdup(fname), boot_count, rc);
    rc = fs_seek(&file, 0, FS_SEEK_SET);
    LOG_INF("%s seek start: %d\n", log_strdup(fname), rc);
  }

  boot_count += 1;
  rc = fs_write(&file, &boot_count, sizeof(boot_count));
  LOG_INF("%s write new boot count %u: %d\n", log_strdup(fname),
      boot_count, rc);

  rc = fs_close(&file);
  LOG_INF("%s close: %d\n", log_strdup(fname), rc);

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
extern void batt_check_thread(void) {

  int off_time; // turn off divider to save power (ms)
  unsigned int soc_percent = 0; // State of charge percentage
  unsigned int prev_soc_percent = 0;
  unsigned int batt_pptt;
  int batt_mV;

  while (1) {
    switch (datadisc_state) {
    case LOG:
      off_time = 1700;
      break;

    case IDLE:
      off_time = 4700;
      break;

    default: // TODO: decide on times for other states
      off_time = 9700;
      break;
    }

    battery_measure_enable(true);

    k_msleep(300); // Wait for measurment to settle
    batt_mV = battery_sample();

    battery_measure_enable(false);

    if (batt_mV < 0) {
      LOG_ERR("Failed to read battery voltage: %d\n", batt_mV);
    }

    batt_pptt = battery_level_pptt(batt_mV, levels);

    LOG_DBG("[%s]: %d mV; %u pptt\n", log_strdup(now_str()), batt_mV, batt_pptt);

    soc_percent = batt_pptt / 100;

    if (soc_percent != prev_soc_percent) {
      /* Send battery level over BLE */
      bt_bas_set_battery_level(soc_percent);
      prev_soc_percent = soc_percent;
    }

    k_msleep(off_time);
  }
}

K_THREAD_DEFINE(batt_check_id, STACKSIZE, batt_check_thread,
    NULL, NULL, NULL, PRIORITY, 0, TDELAY);


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

extern void led_control_thread(void) {

  const struct device *pwm;
  int err;
  uint16_t level = 0;
  uint16_t time_now;

  pwm = DEVICE_DT_GET(PWM_CTLR);
  if (pwm) {
    LOG_INF("Found device %s", log_strdup(PWM_NAME));
  } else {
    LOG_ERR("Device %s not found", log_strdup(PWM_NAME));
    return;
  }

  while (1) {

    switch (datadisc_state) {
    case INIT:
      // Fast breathe
      level = (exp(sin(10.0*(k_uptime_get()/1000.0))) - (1.0 / M_E)) * SCALING_CONST;

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

    case ERASE:
      // Slow breathe
      level = (exp(sin(3.0*(k_uptime_get()/1000.0))) - (1.0 / M_E)) * SCALING_CONST;
      
      break;

    case SLEEP:
      level = 0;
      
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
 * Accelerometers
 ********************************************/
#define ACCEL_ALPHA_DEVICE DT_LABEL(DT_INST(0, kionix_kx134_1211))
#define ACCEL_BETA_DEVICE DT_LABEL(DT_INST(1, kionix_kx134_1211))

/* Unique IDs to carry into CSV */
#define ACCEL_ALPHA_ID  0x1A;
#define ACCEL_BETA_ID   0x2B;

K_SEM_DEFINE(sem_accel_alpha_drdy, 0, 1);
K_SEM_DEFINE(sem_accel_alpha_tap, 0, 1);
K_SEM_DEFINE(sem_accel_beta_drdy, 0, 1);
K_SEM_DEFINE(sem_accel_beta_idle, 0, 1);

static void accel_alpha_drdy_trigger_handler(const struct device *dev, struct sensor_trigger *trigger) {

  ARG_UNUSED(trigger);

  if (sensor_sample_fetch(dev)) {
    LOG_ERR("sensor_sample_fetch failed");
    return;
  }

  k_sem_give(&sem_accel_alpha_drdy);
}

extern void accel_alpha_drdy_thread(void) {

  k_mutex_lock(&init_mut, K_FOREVER);

  while (datadisc_state == INIT) {
    k_condvar_wait(&init_cond, &init_mut, K_FOREVER);
  }

  k_mutex_unlock(&init_mut);

  const struct device *dev = device_get_binding(ACCEL_ALPHA_DEVICE);
  datalog_msgq_item_t msgq_item;
  struct sensor_value acc_xyz[3];
  
  uint16_t sample_counter = 0;
  uint8_t sample_mod = 1;
    
  float any_jerk = 0;

  struct Comp_Data jerk_help;
  jerk_help.prev_magn = 0;
  jerk_help.prev_time = 0;

  uint32_t prev_time = 0;

  if (!dev) {
    LOG_ERR("Devicetree has no kionix,kx134-1211 node");
    return;
  }
  if (!device_is_ready(dev)) {
    LOG_ERR("Device %s is not ready", log_strdup(dev->name));
    return;
  }

  struct sensor_trigger trig = {
      .type = SENSOR_TRIG_DATA_READY,
      .chan = SENSOR_CHAN_ACCEL_XYZ,
  };

  if (sensor_trigger_set(dev, &trig, accel_alpha_drdy_trigger_handler)) {
    LOG_ERR("Could not set trigger");
    return;
  }

  struct kx134_data *drv_data = dev->data;
  LOG_INF("dev: %p, cb: %p", dev, drv_data->drdy_handler);

  msgq_item.id = ACCEL_ALPHA_ID;
  msgq_item.length = 18; // Number of bytes saved to log

  while (1) {

    k_sem_take(&sem_accel_alpha_drdy, K_FOREVER);

    if (datadisc_state == LOG) {

      sample_counter += 1;

      msgq_item.timestamp = uptime_get_us();

      sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc_xyz);
      msgq_item.data_x = (float)sensor_value_to_double(&acc_xyz[0]);
      msgq_item.data_y = (float)sensor_value_to_double(&acc_xyz[1]);
      msgq_item.data_z = (float)sensor_value_to_double(&acc_xyz[2]);

      jerk_help.now_magn = magnitude(&msgq_item);
      jerk_help.now_time = msgq_item.timestamp;
      any_jerk = running_jerk(&jerk_help);

      if (any_jerk > 500) {
        // reset speed-up timer
        prev_time = msgq_item.timestamp;
        sample_mod = 1;
      }

      if (msgq_item.timestamp - prev_time > 10000) {
        // Only save every 10th sample now
        sample_mod = 10;
      }

      /* send data to consumers */
      if (sample_counter % sample_mod == 0) {
        while (k_msgq_put(&accel_msgq, &msgq_item, K_NO_WAIT) != 0) {
          /* message queue is full: purge old data & try again */
          k_msgq_purge(&accel_msgq);
        }
      }
    }
  }
}

K_THREAD_DEFINE(accel_alpha_drdy_id, STACKSIZE, accel_alpha_drdy_thread,
    NULL, NULL, NULL, PRIORITY, 0, TDELAY);


static void accel_alpha_tap_trigger_handler(const struct device *dev, struct sensor_trigger *trigger) {

  ARG_UNUSED(dev);

  if (trigger->type == SENSOR_TRIG_DOUBLE_TAP) {
    k_sem_give(&sem_accel_alpha_tap);
    return;
  }

  LOG_ERR("Unrecognized trigger");
}

extern void accel_alpha_tap_thread(void) {

  k_mutex_lock(&init_mut, K_FOREVER);

  while (datadisc_state == INIT) {
    k_condvar_wait(&init_cond, &init_mut, K_FOREVER);
  }

  k_mutex_unlock(&init_mut);

  const struct device *dev = device_get_binding(ACCEL_ALPHA_DEVICE);

  if (!dev) {
    LOG_ERR("Devicetree has no kionix,kx134-1211 node");
    return;
  }
  if (!device_is_ready(dev)) {
    LOG_ERR("Device %s is not ready", log_strdup(dev->name));
    return;
  }

  struct sensor_trigger trig = {
      .type = SENSOR_TRIG_DOUBLE_TAP,
      .chan = KX134_SENSOR_CHAN_INT_SOURCE,
  };

  if (sensor_trigger_set(dev, &trig, accel_alpha_tap_trigger_handler)) {
    LOG_ERR("Could not set trigger");
    return;
  }

  struct kx134_data *drv_data = dev->data;
  LOG_INF("dev: %p, cb: %p", dev, drv_data->dbtp_handler);

  while (1) {
    k_sem_take(&sem_accel_alpha_tap, K_FOREVER);

    LOG_INF("Double Tap!\n");

    datadisc_state = LOG;
    k_msleep(250);
  }
}

K_THREAD_DEFINE(accel_alpha_tap_id, STACKSIZE, accel_alpha_tap_thread,
    NULL, NULL, NULL, PRIORITY+1, 0, TDELAY);


static void accel_beta_drdy_trigger_handler(const struct device *dev, struct sensor_trigger *trigger) {

  ARG_UNUSED(trigger);

  if (sensor_sample_fetch(dev)) {
    LOG_ERR("sensor_sample_fetch failed");
    return;
  }

  k_sem_give(&sem_accel_beta_drdy);
}

extern void accel_beta_drdy_thread(void) {

  k_mutex_lock(&init_mut, K_FOREVER);

  while (datadisc_state == INIT) {
    k_condvar_wait(&init_cond, &init_mut, K_FOREVER);
  }

  k_mutex_unlock(&init_mut);

  const struct device *dev = device_get_binding(ACCEL_BETA_DEVICE);
  datalog_msgq_item_t msgq_item;
  struct sensor_value acc_xyz[3];

  uint16_t sample_counter = 0;
  uint8_t sample_mod = 1;
    
  float any_jerk = 0;

  struct Comp_Data jerk_help;
  jerk_help.prev_magn = 0;
  jerk_help.prev_time = 0;

  uint32_t prev_time = 0;

  if (!dev) {
    LOG_ERR("Devicetree has no kionix,kx134-1211 node");
    return;
  }
  if (!device_is_ready(dev)) {
    LOG_ERR("Device %s is not ready", log_strdup(dev->name));
    return;
  }

  struct sensor_trigger trig = {
      .type = SENSOR_TRIG_DATA_READY,
      .chan = SENSOR_CHAN_ACCEL_XYZ,
  };

  if (sensor_trigger_set(dev, &trig, accel_beta_drdy_trigger_handler)) {
    LOG_ERR("Could not set trigger");
    return;
  }

  struct kx134_data *drv_data = dev->data;
  LOG_INF("dev: %p, cb: %p", dev, drv_data->drdy_handler);

  msgq_item.id = ACCEL_BETA_ID;
  msgq_item.length = 18; // Number of bytes saved to log

  while (1) {

    k_sem_take(&sem_accel_beta_drdy, K_FOREVER);

    if (datadisc_state == LOG) {

      sample_counter += 1;

      msgq_item.timestamp = uptime_get_us();

      sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc_xyz);
      msgq_item.data_x = (float)sensor_value_to_double(&acc_xyz[0]);
      msgq_item.data_y = (float)sensor_value_to_double(&acc_xyz[1]);
      msgq_item.data_z = (float)sensor_value_to_double(&acc_xyz[2]);

      jerk_help.now_magn = magnitude(&msgq_item);
      jerk_help.now_time = msgq_item.timestamp;
      any_jerk = running_jerk(&jerk_help);

      if (any_jerk > 500) {
        // reset speed-up timer
        prev_time = msgq_item.timestamp;
        sample_mod = 1;
      }

      if (msgq_item.timestamp - prev_time > 10000) {
        // Only save every 10th sample now
        sample_mod = 10;
      }

      /* send data to consumers */
      if (sample_counter % sample_mod == 0) {
        while (k_msgq_put(&accel_msgq, &msgq_item, K_NO_WAIT) != 0) {
          /* message queue is full: purge old data & try again */
          k_msgq_purge(&accel_msgq);
        }
      }
    }
  }
}

K_THREAD_DEFINE(accel_beta_drdy_id, STACKSIZE, accel_beta_drdy_thread,
    NULL, NULL, NULL, PRIORITY, 0, TDELAY);


static void accel_beta_idle_trigger_handler(const struct device *dev, struct sensor_trigger *trigger) {

  ARG_UNUSED(dev);

  if (trigger->type == KX134_SENSOR_TRIG_IDLE) {
    k_sem_give(&sem_accel_beta_idle);
    return;
  }

  LOG_ERR("Unrecognized trigger");
}

extern void accel_beta_idle_thread(void) {

  k_mutex_lock(&init_mut, K_FOREVER);

  while (datadisc_state == INIT) {
    k_condvar_wait(&init_cond, &init_mut, K_FOREVER);
  }

  k_mutex_unlock(&init_mut);

  const struct device *dev = device_get_binding(ACCEL_BETA_DEVICE);

  if (!dev) {
    LOG_ERR("Devicetree has no kionix,kx134-1211 node");
    return;
  }
  if (!device_is_ready(dev)) {
    LOG_ERR("Device %s is not ready", log_strdup(dev->name));
    return;
  }

  struct sensor_trigger trig = {
      .type = KX134_SENSOR_TRIG_IDLE,
      .chan = KX134_SENSOR_CHAN_INT_SOURCE,
  };

  if (sensor_trigger_set(dev, &trig, accel_beta_idle_trigger_handler)) {
    LOG_ERR("Could not set trigger");
    return;
  }

  struct kx134_data *drv_data = dev->data;
  LOG_INF("dev: %p, cb: %p", dev, drv_data->idle_handler);

  while (1) {
    k_sem_take(&sem_accel_beta_idle, K_FOREVER);

    LOG_INF("Idle detected!\n");

    datadisc_state = IDLE;
    k_msleep(250);
  }
}

K_THREAD_DEFINE(accel_beta_idle_id, STACKSIZE, accel_beta_idle_thread,
    NULL, NULL, NULL, PRIORITY+1, 0, TDELAY);


/********************************************
 * Magnetometer
 ********************************************/
#define MAGN_DEVICE DT_LABEL(DT_INST(0, rohm_bm1422agmv))

/* Unique IDs to carry into log */
#define MAGN_ID  0x3C;

K_SEM_DEFINE(magn_sem, 0, 1);

static void magn_trigger_handler(const struct device *dev, struct sensor_trigger *trigger) {

  if (sensor_sample_fetch(dev)) {
    LOG_ERR("sensor_sample_fetch failed");
    return;
  }

  k_sem_give(&magn_sem);
}

extern void magn_thread(void) {

  k_mutex_lock(&init_mut, K_FOREVER);

  while (datadisc_state == INIT) {
    k_condvar_wait(&init_cond, &init_mut, K_FOREVER);
  }

  k_mutex_unlock(&init_mut);

  const struct device *dev = device_get_binding(MAGN_DEVICE);
  datalog_msgq_item_t msgq_item;
  struct sensor_value magn_xyz[3];

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

  msgq_item.id = MAGN_ID;
  msgq_item.length = 18; // Number of bytes saved to log

  while (1) {

    k_sem_take(&magn_sem, K_FOREVER);

    if (datadisc_state == LOG) {
      
      msgq_item.timestamp = uptime_get_us();

      sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, magn_xyz);
      msgq_item.data_x = (float)sensor_value_to_double(&magn_xyz[0]);
      msgq_item.data_y = (float)sensor_value_to_double(&magn_xyz[1]);
      msgq_item.data_z = (float)sensor_value_to_double(&magn_xyz[2]);

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
uint16_t temp_counter = 0;

extern void runtime_compute_thread(void) {

  k_mutex_lock(&init_mut, K_FOREVER);

  while (datadisc_state == INIT) {
    k_condvar_wait(&init_cond, &init_mut, K_FOREVER);
  }

  k_mutex_unlock(&init_mut);

  datalog_msgq_item_t msgq_item;
  datalog_msgq_item_t data_item;
  datalog_msgq_item_t throw_away_item;

  // TODO: Accel averaging, spin rate, etc.
  while (1) {

    k_msgq_get(&accel_msgq, &msgq_item, K_FOREVER);

    memcpy(&data_item, &msgq_item, sizeof(datalog_msgq_item_t));

    switch (msgq_item.id) {
    case 0x00:
      break;

    case 0x1A:
    case 0x2B:
    case 0x3C:
      break;

    case 0xFF:
    default:
      data_item.length = 10;
      data_item.data_x = 9;
      break;
    }

    if (k_msgq_num_free_get(&datalog_msgq) <= 0) {
      temp_counter += 1;
    }

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
extern void spi_flash_thread(void) {

  struct fs_mount_t *mp = &fs_mnt;
  struct fs_file_t file;
  unsigned int id = (uintptr_t)mp->storage_dev;
  uint32_t log_start_time;
  int rc;
  uint16_t data_size = 0;
  uint8_t buffer[20];
  uint8_t latest_fname[20];
  uint8_t oldest_fname[20];
  datalog_msgq_item_t data_item;

  snprintf(latest_fname, sizeof(latest_fname), "%s/latest.ddl", mp->mnt_point);
  snprintf(oldest_fname, sizeof(oldest_fname), "%s/oldest.ddl", mp->mnt_point);

  while (1) {

    log_start_time = k_uptime_get_32();
  
    if (!mp->mnt_point) {
      LOG_ERR("FAIL: mount id %u at %s", id, log_strdup(mp->mnt_point));
      return;
    }
    LOG_INF("%s mount\n", log_strdup(mp->mnt_point));

    // *.ddl = DataDiscLog file, binary list of numbers
    // delete oldest log
    rc = fs_unlink(oldest_fname);
    if (rc < 0) {
      LOG_INF("'oldest.ddl' not found or read-only: %d\n", rc);
    }

    // rename old log to oldest log
    rc = fs_rename(latest_fname, oldest_fname);
    if (rc < 0) {
      LOG_INF("'latest.ddl' not found or read-only: %d\n", rc);
    }

    // create new log
    fs_file_t_init(&file);
    rc = fs_open(&file, latest_fname, FS_O_CREATE | FS_O_RDWR);
    if (rc < 0) {
      LOG_ERR("FAIL: open %s: %d\n", log_strdup(latest_fname), rc);
      goto out;
    }

    snprintf(buffer, 5, "DDL\n");

    rc = fs_write(&file, buffer, strlen(buffer));
    if (rc < 0) {
      LOG_ERR("FAIL: write %s: %d\n", log_strdup(buffer), rc);
      goto out;
    }

    rc = fs_sync(&file);
    if (rc < 0) {
      LOG_ERR("FAIL: sync %s: %d\n", log_strdup(latest_fname), rc);
      goto out;
    }

    while (1) {

      k_msgq_get(&datalog_msgq, &data_item, K_FOREVER);

      memcpy(buffer, &data_item.length, 1);
      memcpy(buffer+1, &data_item.id, 1);
      memcpy(buffer+2, &data_item.timestamp, sizeof(data_item.timestamp));
      memcpy(buffer+2+sizeof(data_item.timestamp), &data_item.data_x, data_item.length - 6);

      rc = fs_write(&file, buffer, data_item.length);
      if (rc < 0) {
        LOG_ERR("FAIL: write %s: %d\n", log_strdup(latest_fname), rc);
        goto out;
      }
      data_size += data_item.length;

      if (data_size >= 4096) {
        rc = fs_sync(&file);
        if (rc < 0) {
          LOG_ERR("FAIL: sync %s: %d\n", log_strdup(latest_fname), rc);
          goto out;
        }
        data_size = 0;
      }

      if (datadisc_state != LOG && k_msgq_num_used_get(&datalog_msgq) <= 0) {
        goto out;
      }

      if (k_uptime_get_32() - log_start_time > 15000) {
        datadisc_state = IDLE;
      }
    }

  out:
    datadisc_state = IDLE;

    snprintf(buffer, 5, "EOF\n");

    rc = fs_write(&file, buffer, strlen(buffer));

    rc = fs_close(&file);
    LOG_INF("%s close: %d\n", log_strdup(latest_fname), rc);

    k_msgq_purge(&accel_msgq);
    k_msgq_purge(&datalog_msgq);

    k_thread_suspend(k_current_get());
  }
}

//K_THREAD_DEFINE(spi_flash_id, STACKSIZE, spi_flash_thread,
//    NULL, NULL, NULL, PRIORITY+2, 0, TDELAY);
K_THREAD_STACK_DEFINE(spi_flash_stack_area, STACKSIZE);
struct k_thread spi_flash_thread_data;
k_tid_t spi_flash_tid = NULL;


/********************************************
 * UART Control
 ********************************************/
#if DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_shell_uart), zephyr_cdc_acm_uart)

extern void uart_ctl_thread(void) {

  k_mutex_lock(&init_mut, K_FOREVER);

  while (datadisc_state == INIT) {
    k_condvar_wait(&init_cond, &init_mut, K_FOREVER);
  }

  k_mutex_unlock(&init_mut);

  const struct device *dev;
  uint32_t dtr = 0;

  dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
  if (!device_is_ready(dev) || usb_enable(NULL)) {
    return;
  }

  LOG_INF("Virtual COM port setup complete.\n");

  while (!dtr) {
    uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
    k_sleep(K_MSEC(100));
  }
}

K_THREAD_DEFINE(uart_ctl_id, STACKSIZE, uart_ctl_thread,
    NULL, NULL, NULL, PRIORITY, 0, TDELAY);

/***************************************************************
 *   Shell Commands
 *
 ****************************************************************/
/* DataDisc Shell Commands */
static int setstate_cmd_handler(const struct shell *shell,
    size_t argc, char **argv, void *data) {

  Machine_State prev_state = datadisc_state;
  datadisc_state = (int)data;

  shell_print(shell, "Previous State: %s\n"
                     "New State     : %s\n",
              state_strings[prev_state],
              state_strings[datadisc_state]);

  return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_setstate, setstate_cmd_handler,
    (idle, 0), (init, 1), (log, 2), (erase, 3), (dump, 4), (sleep, 5));

SHELL_CMD_REGISTER(setstate, &sub_setstate, "Set DataDisc State", NULL);

/* Utility Shell Commands */
static int cmd_demo_ping(const struct shell *shell, size_t argc, char **argv) {
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  shell_print(shell, "pong");

  return 0;
}

static int cmd_version(const struct shell *shell, size_t argc, char **argv) {
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  shell_print(shell, "Zephyr version %s", KERNEL_VERSION_STRING);

  return 0;
}

static int cmd_reboot(const struct shell *shell, size_t argc, char **argv) {
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  shell_print(shell, "Rebooting...");
  sys_reboot(SYS_REBOOT_COLD);

  return 0;
}

SHELL_CMD_REGISTER(ping, NULL, "Demo commands", cmd_demo_ping);

SHELL_CMD_ARG_REGISTER(version, NULL, "Show kernel version", cmd_version, 1, 0);

SHELL_CMD_REGISTER(reboot, NULL, "Reboot device", cmd_reboot);

#endif


/***************************************************************
*   Main
*
****************************************************************/
void main(void) {

  int err;
  Machine_State prev_state = IDLE;

  k_mutex_lock(&init_mut, K_FOREVER);

  SEGGER_RTT_Init();

  LOG_INF("Starting DataDisc v2\n");

  // Initialize the Bluetooth Subsystem
  datadisc_bt_init();

  // Initialize USB and mass storage
  setup_disk();

  k_msleep(2);

  err = usb_enable(NULL);
  if (err != 0) {
    LOG_ERR("Failed to enable USB");
    return;
  }

  LOG_INF("USB mass storage setup complete.\n");

  //dev = device_get_binding("GPIO_0");

  err = gpio_pin_configure(device_get_binding("GPIO_0"), 10, (GPIO_OUTPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW));
  if (err < 0) {
    return;
  }

  datadisc_state = IDLE;

  k_condvar_signal(&init_cond);
  k_mutex_unlock(&init_mut);


  // Main loop
  while (1) {
    
    /* State Changed */
    if (datadisc_state != prev_state) {
      LOG_INF("State changed to %s.\n", log_strdup(state_strings[datadisc_state]));
      prev_state = datadisc_state;

      switch (datadisc_state) {
      case IDLE:
        LOG_INF("Buffer underflows during log: %d\n", temp_counter);
        temp_counter = 0;
        break;

      case LOG:
        if (spi_flash_tid == NULL) {
          spi_flash_tid = k_thread_create(&spi_flash_thread_data, spi_flash_stack_area,
              K_THREAD_STACK_SIZEOF(spi_flash_stack_area),
              spi_flash_thread,
              NULL, NULL, NULL,
              PRIORITY + 2, 0, K_NO_WAIT);
        } else {
          k_thread_resume(spi_flash_tid);
        }
        break;

      case ERASE:
        // Make sure no activity on FLASH bus
        datadisc_state = IDLE;

        err = usb_disable();
        if (err != 0) {
          LOG_ERR("Failed to disable USB");
          return;
        }
        k_msleep(500);

        datadisc_state = ERASE;
        setup_disk();

        err = usb_enable(NULL);
        if (err != 0) {
          LOG_ERR("Failed to enable USB");
          return;
        }

        datadisc_state = IDLE;
        break;

      case DUMP:
        break;

      case SLEEP:
        break;

      default:
        break;
      }
    }

    k_msleep(100);
  }
}