/*
 * Copyright (c) 2021 Griffin Adams
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BM1422_H_
#define ZEPHYR_DRIVERS_SENSOR_BM1422_H_

#include <zephyr/types.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <sys/util.h>

/* BM1422 communication commands */
#define BM1422_WRITE_REG           0x0A
#define BM1422_READ_REG            0x0B
#define BM1422_WRITE_FIFO          0x0D

/*
 * BM1422 registers definition
 */
#define BM1422_INFO_LO		0x0Du  /* Device Info Low Byte */
#define BM1422_INFO_HI		0x0Eu  /* Device Info High Byte */
#define BM1422_WHO_AM_I		0x0Fu  /* Supplier Recognition ID */

#define BM1422_DATAX_LO		0x10u  /* X Channel Output Low Byte */
#define BM1422_DATAX_HI		0x11u  /* X Channel Output High Byte */
#define BM1422_DATAY_LO		0x12u  /* Y Channel Output Low Byte */
#define BM1422_DATAY_HI		0x13u  /* Y Channel Output High Byte */
#define BM1422_DATAZ_LO		0x14u  /* Z Channel Output Low Byte */
#define BM1422_DATAZ_HI		0x15u  /* Z Channel Output High Byte */

#define BM1422_STA1		0x18u  /* Status Register */

#define BM1422_CNTL1		0x1Bu  /* Main Feature Controls */
#define BM1422_CNTL2		0x1Cu  /* Data Ready Controls */
#define BM1422_CNTL3		0x1Du  /* Force Start Control */

#define BM1422_AVE_A		0x40u  /* Averaging Setting */

#define BM1422_CNTL4_LO		0x5Cu  /* Reset Low Byte */
#define BM1422_CNTL4_HI		0x5Du  /* Reset High Byte */

#define BM1422_TEMP_LO		0x60u  /* Temperature Value Low Byte */
#define BM1422_TEMP_HI		0x61u  /* Temperature Value High Byte */

#define BM1422_OFFX_LO		0x6Cu  /* X channel Offset Value Low Byte */
#define BM1422_OFFX_HI		0x6Du  /* X channel Offset Value High Byte */

#define BM1422_OFFY_LO		0x72u  /* Y channel Offset Value Low Byte */
#define BM1422_OFFY_HI		0x73u  /* Y channel Offset Value High Byte */

#define BM1422_OFFZ_LO		0x78u  /* Z channel Offset Value Low Byte */
#define BM1422_OFFZ_HI		0x79u  /* Z channel Offset Value High Byte */

#define BM1422_FINE_DATAX_LO	0x90u  /* X Channel Fine Output Low Byte */
#define BM1422_FINE_DATAX_HI	0x91u  /* X Channel Fine Output High Byte */
#define BM1422_FINE_DATAY_LO	0x92u  /* Y Channel Fine Output Low Byte */
#define BM1422_FINE_DATAY_HI	0x93u  /* Y Channel Fine Output High Byte */
#define BM1422_FINE_DATAZ_LO	0x94u  /* Z Channel Fine Output Low Byte */
#define BM1422_FINE_DATAZ_HI	0x95u  /* Z Channel Fine Output High Byte */

#define BM1422_GAIN_PARAX_LO	0x9Cu  /* Axis Interference Xch to Zch */
#define BM1422_GAIN_PARAX_HI	0x9Du  /* Axis Interference Xch to Ych */
#define BM1422_GAIN_PARAY_LO	0x9Eu  /* Axis Interference Ych to Zch */
#define BM1422_GAIN_PARAY_HI	0x9Fu  /* Axis Interference Ych to Xch */

#define BM1422_GAIN_PARAZ_LO	0xA0u  /* Axis Interference Zch to Ych */
#define BM1422_GAIN_PARAZ_HI	0xA1u  /* Axis Interference Zch to Xch */

/* Some register values */
#define BM1422_MAN_ID_VAL        "?"	/* TBD */
#define BM1422_PART_ID_VAL	0x33u   /* TBD */
#define BM1422_WHO_AM_I_VAL	0x41u  	/* Supplier Recognition ID */


#define BM1422_READ			0x01u
#define BM1422_REG_READ(x)		(((x & 0xFF) << 1) | BM1422_READ)
#define BM1422_REG_WRITE(x)		((x & 0xFF) << 1)
#define BM1422_TO_I2C_REG(x)            ((x) >> 1)

#define PWRTWO(x)                       (1 << (x))


/* BM1422_STA1 */
#define BM1422_STA1_RD_DRDY(x)			(((x) >> 6) & 0x1)

/* BM1422_CNTL1 */
#define BM1422_CNTL1_PC1_MSK			BIT(7)
#define BM1422_CNTL1_PC1_MODE(x)		(((x) & 0x1) << 7)
#define BM1422_CNTL1_OUT_BIT_MSK		BIT(6)
#define BM1422_CNTL1_OUT_BIT_MODE(x)            (((x) & 0x1) << 6)
#define BM1422_CNTL1_RST_LV_MSK			BIT(5)
#define BM1422_CNTL1_RST_LV_MODE(x)		(((x) & 0x1) << 5)
#define BM1422_CNTL1_ODR_MSK			GENMASK(4, 3)
#define BM1422_CNTL1_ODR_MODE(x)		(((x) & 0x3) << 3)
#define BM1422_CNTL1_FS1_MSK			BIT(1)
#define BM1422_CNTL1_FS1_MODE(x)		(((x) & 0x1) << 1)

/* BM1422_CNTL2 */
#define BM1422_CNTL2_DREN_MSK			BIT(3)
#define BM1422_CNTL2_DREN_MODE(x)		(((x) & 0x1) << 3)
#define BM1422_CNTL2_DRP_MSK			BIT(2)
#define BM1422_CNTL2_DRP_MODE(x)		(((x) & 0x1) << 2)

/* BM1422_CNTL3 */
#define BM1422_CNTL3_FORCE_MSK			BIT(6)
#define BM1422_CNTL3_FORCE_MODE(x)		(((x) & 0x1) << 6)

/* BM1422_AVE_A */
#define BM1422_AVE_A_MSK			GENMASK(4, 2)
#define BM1422_AVE_A_MODE(x)			(((x) & 0x7) << 2)


/* BM1422 scale factors from specifications */
#define BM1422_MAG_12BIT_LSB_PER_MILLI_T	5953LL
#define BM1422_MAG_14BIT_LSB_PER_MILLI_T	23810LL
#define BM1422_12BIT_LSB_PER_DEGREE_C           33		// Maybe? Datasheet sucks
#define BM1422_14BIT_LSB_PER_DEGREE_C           131		// Maybe? Datasheet sucks


enum bm1422_op_mode {
	BM1422_STANDBY,
	BM1422_ACTIVE,
};

enum bm1422_out_bit {
	BM1422_12BIT,
	BM1422_14BIT
};

enum bm1422_odr {
	BM1422_ODR_10HZ,
	BM1422_ODR_100HZ,
	BM1422_ODR_20HZ,
	BM1422_ODR_1000HZ
};

enum bm1422_fs1 {
	BM1422_CONTINUOUS,
	BM1422_SINGLE
};

enum bm1422_ave {
	BM1422_4_TIMES,
	BM1422_1_TIMES,
	BM1422_2_TIMES,
	BM1422_8_TIMES,
	BM1422_16_TIMES
};


struct bm1422_data {
	const struct device *bus;

	union {
		int16_t mag_xyz[3];
		struct {
			int16_t mag_x;
			int16_t mag_y;
			int16_t mag_z;
		};
	} __packed;

	int16_t temperature;
	enum bm1422_out_bit selected_bits;

#if defined(CONFIG_BM1422_TRIGGER)
	const struct device *dev;
	const struct device *gpio;
	struct gpio_callback gpio_cb;
	struct k_mutex trigger_mutex;

	sensor_trigger_handler_t th_handler;
	struct sensor_trigger th_trigger;
	sensor_trigger_handler_t drdy_handler;
	struct sensor_trigger drdy_trigger;

	uint8_t int_config;

#if defined(CONFIG_BM1422_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_BM1422_THREAD_STACK_SIZE);
	struct k_sem gpio_sem;
	struct k_thread thread;
#elif defined(CONFIG_BM1422_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
#endif /* CONFIG_BM1422_TRIGGER */
};

struct bm1422_config {
	const char *i2c_port;
	uint16_t i2c_addr;

#if defined(CONFIG_BM1422_TRIGGER)
	const char *gpio_port;
	gpio_pin_t int_gpio;
	gpio_dt_flags_t int_flags;
#endif

	/* Device Settings */
	enum bm1422_op_mode op;
	enum bm1422_out_bit out_bit;
	enum bm1422_odr odr;
	enum bm1422_fs1 mode;
	enum bm1422_ave ave;
};

#ifdef CONFIG_BM1422_TRIGGER
int bm1422_get_status(const struct device *dev, uint8_t *status);

int bm1422_reg_write_mask(const struct device *dev,
			   uint8_t reg_addr, uint8_t mask, uint8_t data);

int bm1422_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int bm1422_init_interrupt(const struct device *dev);

int bm1422_clear_interrupts(const struct device *dev);
#endif /* CONFIG_BM1422_TRIGGER */

#endif /* ZEPHYR_DRIVERS_SENSOR_BM1422_H_ */
