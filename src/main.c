/*
 * Copyright (c) 2021 Griffin Adams
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <errno.h>

#include <SEGGER_RTT.h>
#include <logging/log.h>
#include <logging/log_ctrl.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <sys/byteorder.h>

#include <fs/fs.h>
#include <fs/littlefs.h>
#include <storage/flash_map.h>
#include <usb/usb_device.h>

#include <drivers/flash.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include <drivers/sensor.h>

//#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/services/bas.h>
#include <bluetooth/uuid.h>

#include "battery.h"
#include "kx134.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);


/***************************************************************
*   Global Variables
*
****************************************************************/
typedef enum {
  IDLE,
  LOG,
  ERASE,
  DUMP,
  SLEEP
} Machine_State;
Machine_State datadisc_state = LOG; // TODO make this IDLE for releases


/* file system things */
#define MAX_PATH_LEN 255

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

unsigned int soc_percent = 0;


/* Thread things */
/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

/* delay for each thread to allow inits */
#define TDELAY 100

K_FIFO_DEFINE(accel_fifo);
K_FIFO_DEFINE(datalog_fifo);

struct accel_fifo_item_t {
  void *fifo_reserved; /* 1st word reserved for use by FIFO */
  uint8_t id;
  uint32_t timestamp;
  struct kx134_xyz_accel_data *data;
};

struct datalog_fifo_item_t {
  void *fifo_reserved; /* 1st word reserved for use by FIFO */
  uint8_t id;
  uint32_t timestamp;
  void *data;
};


/***************************************************************
*   Bluetooth Functions
*
****************************************************************/
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

// Set Advertisement data.
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};

// Set Scan Response data
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void bt_ready(void) {
  int err;

  printk("Bluetooth initialized\n");

  if (IS_ENABLED(CONFIG_SETTINGS)) {
    settings_load();
  }

  err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
  if (err) {
    printk("Advertising failed to start (err %d)\n", err);
    return;
  }

  printk("Advertising successfully started\n");
}

/***************************************************************
*   Utility Functions
*
****************************************************************/

static void wait_on_log_flushed(void) {
  while (log_buffered_cnt()) {
    k_sleep(K_MSEC(5));
  }
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

  mnt->storage_dev = (void *)FLASH_AREA_ID(external_flash);
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

  uint8_t boot_count = 0;

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
/* Battery check */
void batt_check_thread(void) {

  int off_time; // turn off divider to save power (ms)

  while (1) {
    switch (datadisc_state) {
    case LOG:
      off_time = 700;
      break;

    default: // TODO: decide on times for other states
      off_time = 2700;
      break;
    }

    battery_measure_enable(true);

    k_msleep(300);
    int batt_mV = battery_sample();

    battery_measure_enable(false);

    if (batt_mV < 0) {
      printk("Failed to read battery voltage: %d\n", batt_mV);
    }

    unsigned int batt_pptt = battery_level_pptt(batt_mV, levels);

    printk("[%s]: %d mV; %u pptt\n", now_str(), batt_mV, batt_pptt);

    soc_percent = batt_pptt / 100;

    k_msleep(off_time);
  }
}

//K_THREAD_DEFINE(batt_check_id, STACKSIZE, batt_check_thread,
//    NULL, NULL, NULL, PRIORITY, 0, TDELAY);


/* LED Control */
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

void led_control_thread(void) {

  const struct device *pwm;
  int err;
  uint16_t level;

  pwm = DEVICE_DT_GET(PWM_CTLR);
  if (pwm) {
    LOG_INF("Found device %s", PWM_NAME);
  } else {
    LOG_ERR("Device %s not found", PWM_NAME);
    return;
  }

  while (1) {

    switch (datadisc_state) {
    case LOG:
      for (level = 0; level <= MAX_BRIGHTNESS; level++) {
        err = pwm_pin_set_usec(pwm, PWM_CHANNEL, MIN_PERIOD_USEC, (MIN_PERIOD_USEC * level) / 100U, PWM_FLAGS);
        if (err < 0) {
          LOG_ERR("err=%d brightness=%d", err, level);
          return;
        }
        k_msleep(FADE_DELAY);
      }
      k_msleep(1000);
      for (level = 0; level <= MAX_BRIGHTNESS; level++) {
        err = pwm_pin_set_usec(pwm, PWM_CHANNEL, MIN_PERIOD_USEC, (MIN_PERIOD_USEC * (MAX_BRIGHTNESS - level)) / 100U, PWM_FLAGS);
        if (err < 0) {
          LOG_ERR("err=%d brightness=%d", err, (MAX_BRIGHTNESS - level));
          return;
        }
        k_msleep(FADE_DELAY);
      }
      k_msleep(1000);
      break;

    default: // TODO: decide on times for other states

      break;
    }
  }
}

K_THREAD_DEFINE(led_control_id, STACKSIZE, led_control_thread,
    NULL, NULL, NULL, PRIORITY, 0, TDELAY);


/* Accelerometers */
#define ACCEL_ALPHA_DEVICE DT_LABEL(DT_INST(0, kionix_kx134_1211))
#define ACCEL_BETA_DEVICE DT_LABEL(DT_INST(1, kionix_kx134_1211))

/* Unique IDs to carry into CSV */
#define ACCEL_ALPHA_ID  0x1A;
#define ACCEL_BETA_ID   0x2B;

K_SEM_DEFINE(sem_a, 0, 1);
K_SEM_DEFINE(sem_b, 0, 1);

static void accel_alpha_trigger_handler(const struct device *dev, struct sensor_trigger *trigger) {
  ARG_UNUSED(trigger);

  if (sensor_sample_fetch(dev)) {
    printf("sensor_sample_fetch failed\n");
    return;
  }

  k_sem_give(&sem_a);
}

void accel_alpha_thread(void) {

  struct sensor_value accel[3];
  struct accel_fifo_item_t fifo_item;
  const struct device *dev = device_get_binding(ACCEL_ALPHA_DEVICE);

  if (!dev) {
    printf("Devicetree has no kionix,kx134-1211 node\n");
    return;
  }
  if (!device_is_ready(dev)) {
    printf("Device %s is not ready\n", dev->name);
    return;
  }

  struct sensor_trigger trig = {
      .type = SENSOR_TRIG_DATA_READY,
      .chan = SENSOR_CHAN_ACCEL_XYZ,
  };

  if (IS_ENABLED(CONFIG_KX134_TRIGGER)) {
    if (sensor_trigger_set(dev, &trig, accel_alpha_trigger_handler)) {
      printf("Could not set trigger\n");
      return;
    }
  }

  struct kx134_xyz_accel_data fifo_data; 
  fifo_item.id = ACCEL_ALPHA_ID;

  while (1) {
    if (IS_ENABLED(CONFIG_KX134_TRIGGER)) {
      k_sem_take(&sem_a, K_FOREVER);
    } else {
      if (sensor_sample_fetch(dev)) {
        printf("sensor_sample_fetch failed\n");
      }
    }

    fifo_item.timestamp = k_uptime_get();

    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);

    //printk("x: %d.%06d; y: %d.%06d; z: %d.%06d\n",
    //    accel[0].val1, accel[0].val2, accel[1].val1, accel[1].val2,
    //    accel[2].val1, accel[2].val2);

    fifo_data.x = (float)sensor_value_to_double(&accel[0]);
    fifo_data.y = (float)sensor_value_to_double(&accel[1]);
    fifo_data.z = (float)sensor_value_to_double(&accel[2]);

    fifo_item.data = &fifo_data;

    k_fifo_put(&accel_fifo, &fifo_item);

    if (!IS_ENABLED(CONFIG_KX134_TRIGGER)) {
      k_sleep(K_MSEC(2000));
    }
  }
}

K_THREAD_DEFINE(accel_alpha_id, STACKSIZE, accel_alpha_thread,
    NULL, NULL, NULL, PRIORITY, 0, TDELAY);


static void accel_beta_trigger_handler(const struct device *dev, struct sensor_trigger *trigger) {
  ARG_UNUSED(trigger);

  if (sensor_sample_fetch(dev)) {
    printf("sensor_sample_fetch failed\n");
    return;
  }

  k_sem_give(&sem_b);
}

void accel_beta_thread(void) {

  struct sensor_value accel[3];
  struct accel_fifo_item_t fifo_item;
  const struct device *dev = device_get_binding(ACCEL_BETA_DEVICE);

  if (!dev) {
    printf("Devicetree has no kionix,kx134-1211 node\n");
    return;
  }
  if (!device_is_ready(dev)) {
    printf("Device %s is not ready\n", dev->name);
    return;
  }

  struct sensor_trigger trig = {
      .type = SENSOR_TRIG_DATA_READY,
      .chan = SENSOR_CHAN_ACCEL_XYZ,
  };

  if (IS_ENABLED(CONFIG_KX134_TRIGGER)) {
    if (sensor_trigger_set(dev, &trig, accel_beta_trigger_handler)) {
      printf("Could not set trigger\n");
      return;
    }
  }

  struct kx134_xyz_accel_data fifo_data; 
  fifo_item.id = ACCEL_BETA_ID;

  while (1) {
    if (IS_ENABLED(CONFIG_KX134_TRIGGER)) {
      k_sem_take(&sem_b, K_FOREVER);
    } else {
      if (sensor_sample_fetch(dev)) {
        printf("sensor_sample_fetch failed\n");
      }
    }

    fifo_item.timestamp = k_uptime_get();

    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);

    //printk("x: %d.%06d; y: %d.%06d; z: %d.%06d\n",
    //    accel[0].val1, accel[0].val2, accel[1].val1, accel[1].val2,
    //    accel[2].val1, accel[2].val2);

    fifo_data.x = (float)sensor_value_to_double(&accel[0]);
    fifo_data.y = (float)sensor_value_to_double(&accel[1]);
    fifo_data.z = (float)sensor_value_to_double(&accel[2]);

    fifo_item.data = &fifo_data;

    k_fifo_put(&accel_fifo, &fifo_item);

    if (!IS_ENABLED(CONFIG_KX134_TRIGGER)) {
      k_sleep(K_MSEC(2000));
    }
  }
}

K_THREAD_DEFINE(accel_beta_id, STACKSIZE, accel_beta_thread,
    NULL, NULL, NULL, PRIORITY, 0, TDELAY);

/* Thread for crunching data during runtime */
void runtime_compute_thread(void) {
  struct accel_fifo_item_t *fifo_item;

  // TODO: Accel averaging, spin rate, etc.
  while (1) {
    fifo_item = k_fifo_get(&accel_fifo, K_FOREVER);
    k_fifo_put(&datalog_fifo, fifo_item);
  }
}

K_THREAD_DEFINE(runtime_compute_id, STACKSIZE, runtime_compute_thread,
    NULL, NULL, NULL, PRIORITY+1, 0, TDELAY);


/* Q-SPI FLASH */
uint8_t fname[MAX_PATH_LEN];     // Buffer created outside thread to avoid stack overflow
uint8_t data[MAX_PATH_LEN];   // Buffer created outside thread to avoid stack overflow

uint64_t time_stamp;

void spi_flash_thread(void) {
  struct datalog_fifo_item_t *fifo_item;
  struct fs_mount_t *mp = &fs_mnt;
  unsigned int id = (uintptr_t)mp->storage_dev;
  int rc;

  snprintf(fname, sizeof(fname), "%s/datalog.csv", mp->mnt_point);

  if (!mp->mnt_point) {
    printk("FAIL: mount id %u at %s\n", id, mp->mnt_point);
    return;
  }
  printk("%s mount\n", mp->mnt_point);

  wait_on_log_flushed();

  struct fs_file_t file;

  fs_file_t_init(&file);

  rc = fs_open(&file, fname, FS_O_CREATE | FS_O_RDWR);
  if (rc < 0) {
    printk("FAIL: open %s: %d\n", fname, rc);
    goto out;
  }

  //snprintf(data, sizeof(data), "SOL\n");

  //rc = fs_write(&file, data, 4);

  /* capture initial time stamp */
  time_stamp = (uint64_t)k_uptime_get();

  while (1) {
    fifo_item = k_fifo_get(&datalog_fifo, K_FOREVER);

    struct kx134_xyz_accel_data *fifo_data = (struct kx134_xyz_accel_data*)fifo_item->data;

    snprintf(data, sizeof(data), "%d,%d,%d,%d\n", fifo_item->timestamp,
                                  fifo_data->x, fifo_data->y, fifo_data->z);

    printk("Data is: %s, length: %d", data, strlen(data));

    rc = fs_write(&file, data, strlen(data));
    if (rc < 0) {
      printk("FAIL: write %s: %d\n", fname, rc);
      goto out;
    }

    if ((uint64_t)k_uptime_get() - time_stamp > 10000) {
      goto out;
    }
  }

out:
  rc = fs_close(&file);
  printk("%s close: %d\n", fname, rc);
  rc = fs_unmount(mp);
  printk("%s unmount: %d\n", mp->mnt_point, rc);
}

K_THREAD_DEFINE(spi_flash_id, STACKSIZE, spi_flash_thread,
    NULL, NULL, NULL, PRIORITY+2, 0, TDELAY);



/***************************************************************
*   Main
*
****************************************************************/
void main(void) {

  int err;
  struct device *dev;

  SEGGER_RTT_Init();

  printk("Starting DataDisc v2\n");

  setup_disk();

  err = usb_enable(NULL);
  if (err != 0) {
    LOG_ERR("Failed to enable USB");
    return;
  }

  LOG_INF("The device is put in USB mass storage mode.\n");

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

    k_msleep(2000);
  }
}