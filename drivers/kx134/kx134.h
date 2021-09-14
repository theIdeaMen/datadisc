/*
 * Copyright (c) 2021 Griffin Adams
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_KX134_H_
#define ZEPHYR_DRIVERS_SENSOR_KX134_H_

#include <zephyr/types.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <sys/util.h>

/* KX134 communication commands */
#define KX134_WRITE_REG           0x0A
#define KX134_READ_REG            0x0B
#define KX134_WRITE_FIFO          0x0D

/*
 * KX134 registers definition
 */
#define KX134_MAN_ID		0x00u  /* Manufacturer ID "Kion" */
#define KX134_PART_ID		0x01u  /* Silicon specific ID */
#define KX134_XADP_L		0x02u  /* X-axis ADP Output LSB */
#define KX134_XADP_H		0x03u  /* X-axis ADP Output MSB */
#define KX134_YADP_L		0x04u  /* Y-axis ADP Output LSB */
#define KX134_YADP_H		0x05u  /* Y-axis ADP Output MSB */
#define KX134_ZADP_L		0x06u  /* Z-axis ADP Output LSB */
#define KX134_ZADP_H		0x07u  /* Z-axis ADP Output MSB */
#define KX134_XOUT_L		0x08u  /* X-axis Accelerometer Output LSB */
#define KX134_XOUT_H		0x09u  /* X-axis Accelerometer Output MSB */
#define KX134_YOUT_L		0x0Au  /* Y-axis Accelerometer Output LSB */
#define KX134_YOUT_H		0x0Bu  /* Y-axis Accelerometer Output MSB */
#define KX134_ZOUT_L		0x0Cu  /* Z-axis Accelerometer Output LSB */
#define KX134_ZOUT_H		0x0Du  /* Z-axis Accelerometer Output MSB */

#define KX134_COTR		0x12u  /* Commmand Test Response */
#define KX134_WHO_AM_I		0x13u  /* Supplier Recognition */
#define KX134_TSCP		0x14u  /* Current Tilt Position */
#define KX134_TSPP		0x15u  /* Previous Tilt Position */
#define KX134_INS1		0x16u  /* Directional Tap Reporting */
#define KX134_INS2		0x17u  /* Interrupt Source */
#define KX134_INS3		0x18u  /* Motion Detection Reporting */
#define KX134_STATUS_REG	0x19u  /* Interrupt and Wake Status */
#define KX134_INT_REL		0x1Au  /* Interrupt Latch Release */
#define KX134_CNTL1		0x1Bu  /* Main Feature Controls */
#define KX134_CNTL2		0x1Cu  /* Tilt Controls */
#define KX134_CNTL3		0x1Du  /* Data Rate For Tilt, Tap, Wake */
#define KX134_CNTL4		0x1Eu  /* More Feature Set Controls */
#define KX134_CNTL5		0x1Fu  /* More Feature Set Controls */
#define KX134_CNTL6		0x20u  /* I2C Feature Set Controls */
#define KX134_ODCNTL		0x21u  /* Accelerometer Output Controls */
#define KX134_INC1		0x22u  /* Physical Interrupt Pin INT1 Controls */
#define KX134_INC2		0x23u  /* Wake/Sleep Engine Controls */
#define KX134_INC3		0x24u  /* Tap/Double-Tap Direction Controls */
#define KX134_INC4		0x25u  /* INT1 Routing Controls */
#define KX134_INC5		0x26u  /* Physical Interrupt Pin INT1 Controls */
#define KX134_INC6		0x27u  /* INT2 Routing Controls */

#define KX134_TILT_TIMER	0x29u  /* Tilt Position State Timer */
#define KX134_TDTRC		0x2Au  /* Tap/Double-Tap Report Control */
#define KX134_TDTC		0x2Bu  /* Double Tap Detection Counter */
#define KX134_TTH		0x2Cu  /* Tap Threshold High */
#define KX134_TTL		0x2Du  /* Tap Threshold Low */
#define KX134_FTD		0x2Eu  /* Any Tap Detection Counter */
#define KX134_STD		0x2Fu  /* Double Tap Detection Counter */
#define KX134_TLT		0x30u  /* Tap Detection Time Limit */
#define KX134_TWS		0x31u  /* Tap Detection Window */
#define KX134_FFTH		0x32u  /* Free Fall Threshold */
#define KX134_FFC		0x33u  /* Free Fall Counter */
#define KX134_FFCNTL		0x34u  /* Free Fall Controls */

#define KX134_TILT_ANGLE_LL	0x37u  /* Tilt Angle Low Limit */
#define KX134_TILT_ANGLE_HL	0x38u  /* Tilt Angle High Limit */
#define KX134_HYST_SET		0x39u  /* Tilt Hysteresis Setting */
#define KX134_LP_CNTL1		0x3Au  /* Low Power Control 1 */
#define KX134_LP_CNTL2		0x3Bu  /* Low Power Control 2 */

#define KX134_WUFTH		0x49u  /* Wake-up Function Threshold */
#define KX134_BTSWUFTH		0x4Au  /* Wake/Sleep Function Threshold */
#define KX134_BTSTH		0x4Bu  /* Back-To-Sleep Function Threshold */
#define KX134_BTSC		0x4Cu  /* Sleep Debounce Counter */
#define KX134_WUFC		0x4Du  /* Wake Debounce Counter */

#define KX134_SELF_TEST		0x5Du  /* Self-Test Enable */
#define KX134_BUF_CNTL1		0x5Eu  /* Buffer Sample Threshold */
#define KX134_BUF_CNTL2		0x5Fu  /* Buffer Operation Controls */
#define KX134_BUF_STATUS1	0x60u  /* Buffer Status (Level) */
#define KX134_BUF_STATUS2	0x61u  /* Buffer Status (Level, Trigger) */
#define KX134_BUF_CLEAR		0x62u  /* Buffer Clear */
#define KX134_BUF_READ		0x63u  /* Buffer Read */

#define KX134_ADP_CNTL1		0x64u  /* ADP Output Controls 1 */
#define KX134_ADP_CNTL2		0x65u  /* ADP Routing Controls */
#define KX134_ADP_CNTL3		0x66u  /* ADP Filter 1 Coefficient (1/A) */
#define KX134_ADP_CNTL4		0x67u  /* ADP Filter 1 Coefficient (B/A) */
#define KX134_ADP_CNTL5		0x68u  /* ADP Filter 1 Coefficient (B/A) */
#define KX134_ADP_CNTL6		0x69u  /* ADP Filter 1 Coefficient (B/A) */
#define KX134_ADP_CNTL7		0x6Au  /* ADP Filter 1 Coefficient (C/A) */
#define KX134_ADP_CNTL8		0x6Bu  /* ADP Filter 1 Coefficient (C/A) */
#define KX134_ADP_CNTL9		0x6Cu  /* ADP Filter 1 Coefficient (C/A) */
#define KX134_ADP_CNTL10	0x6Du  /* ADP Filter 1 Input Scale Shift */
#define KX134_ADP_CNTL11	0x6Eu  /* ADP Filter 2 (1/A) & Filter 1 Output Shift */
#define KX134_ADP_CNTL12	0x6Fu  /* ADP Filter 2 Coefficient (B/A) */
#define KX134_ADP_CNTL13	0x70u  /* ADP Filter 2 Coefficient (B/A) */
#define KX134_ADP_CNTL14	0x71u  /* Unused */
#define KX134_ADP_CNTL15	0x72u  /* Unused */
#define KX134_ADP_CNTL16	0x73u  /* Unused */
#define KX134_ADP_CNTL17	0x74u  /* Unused */
#define KX134_ADP_CNTL18	0x75u  /* ADP Filter 2 Input Scale Shift */
#define KX134_ADP_CNTL19	0x76u  /* ADP Filter 2 Output Scale Shift */

/* Some register values */
#define KX134_MAN_ID_VAL        "Kion" /* Kionix ID */
#define KX134_PART_ID_VAL	0x33   /* Device ID */
#define KX134_WHO_AM_I_VAL	0x46u  /* Supplier Recognition ID */


#define KX134_READ		0x01u
#define KX134_REG_READ(x)	(x)
#define KX134_REG_WRITE(x)	(x)
#define KX134_TO_I2C_REG(x)	((x) >> 1)

#define PWRTWO(x)               (1 << (x))


/* KX134_TSCP */
#define KX134_TSCP_LE(x)			(((x) >> 5) & 0x1)
#define KX134_TSCP_RI(x)			(((x) >> 4) & 0x1)
#define KX134_TSCP_DO(x)			(((x) >> 3) & 0x1)
#define KX134_TSCP_UP(x)			(((x) >> 2) & 0x1)
#define KX134_TSCP_FD(x)			(((x) >> 1) & 0x1)
#define KX134_TSCP_FU(x)			(((x) >> 0) & 0x1)

/* KX134_TSPP */
#define KX134_TSPP_LE(x)			(((x) >> 5) & 0x1)
#define KX134_TSPP_RI(x)			(((x) >> 4) & 0x1)
#define KX134_TSPP_DO(x)			(((x) >> 3) & 0x1)
#define KX134_TSPP_UP(x)			(((x) >> 2) & 0x1)
#define KX134_TSPP_FD(x)			(((x) >> 1) & 0x1)
#define KX134_TSPP_FU(x)			(((x) >> 0) & 0x1)

/* KX134_INS1 */
#define KX134_INS1_TLE(x)			(((x) >> 5) & 0x1)
#define KX134_INS1_TRI(x)			(((x) >> 4) & 0x1)
#define KX134_INS1_TDO(x)			(((x) >> 3) & 0x1)
#define KX134_INS1_TUP(x)			(((x) >> 2) & 0x1)
#define KX134_INS1_TFD(x)			(((x) >> 1) & 0x1)
#define KX134_INS1_TFU(x)			(((x) >> 0) & 0x1)

/* KX134_INS2 */
#define KX134_INS2_FFS(x)			(((x) >> 7) & 0x1)
#define KX134_INS2_BFI(x)			(((x) >> 6) & 0x1)
#define KX134_INS2_WMI(x)			(((x) >> 5) & 0x1)
#define KX134_INS2_DRDY(x)			(((x) >> 4) & 0x1)
#define KX134_INS2_TDTS(x)			(((x) >> 2) & 0x3)
#define KX134_INS2_TPS(x)			(((x) >> 0) & 0x1)

/* KX134_INS3 */
#define KX134_INS3_WUFS(x)			(((x) >> 7) & 0x1)
#define KX134_INS3_BTS(x)			(((x) >> 6) & 0x1)
#define KX134_INS3_XNWU(x)			(((x) >> 5) & 0x1)
#define KX134_INS3_XPWU(x)			(((x) >> 4) & 0x1)
#define KX134_INS3_YNWU(x)			(((x) >> 3) & 0x1)
#define KX134_INS3_YPWU(x)			(((x) >> 2) & 0x1)
#define KX134_INS3_ZNWU(x)			(((x) >> 1) & 0x1)
#define KX134_INS3_ZPWU(x)			(((x) >> 0) & 0x1)

/* KX134_STATUS_REG */
#define KX134_STATUS_REG_INT(x)		(((x) >> 4) & 0x1)
#define KX134_STATUS_REG_WAKE(x)		(((x) >> 0) & 0x1)

/* KX134_CNTL1 */
#define KX134_CNTL1_ACTIVE_MSK			GENMASK(7, 6)
#define KX134_CNTL1_ACTIVE_MODE(x)		(((x) & 0x3) << 6)
#define KX134_CNTL1_DRDY_EN_MSK			BIT(5)
#define KX134_CNTL1_DRDY_EN_MODE(x)		(((x) & 0x1) << 5)
#define KX134_CNTL1_GSEL_MSK			GENMASK(4, 3)
#define KX134_CNTL1_GSEL_MODE(x)		(((x) & 0x3) << 3)
#define KX134_CNTL1_TAP_EN_MSK			BIT(2)
#define KX134_CNTL1_TAP_EN_MODE(x)		(((x) & 0x1) << 2)
#define KX134_CNTL1_TILT_EN_MSK			BIT(0)
#define KX134_CNTL1_TILT_EN_MODE(x)		(((x) & 0x1) << 0)

/* KX134_CNTL2 */
#define KX134_CNTL2_SOFT_RST_MSK		BIT(7)
#define KX134_CNTL2_SOFT_RST_MODE(x)		(((x) & 0x1) << 7)
#define KX134_CNTL2_LEM_MSK			BIT(5)
#define KX134_CNTL2_LEM_MODE(x)			(((x) & 0x1) << 5)
#define KX134_CNTL2_RIM_MSK			BIT(4)
#define KX134_CNTL2_RIM_MODE(x)			(((x) & 0x1) << 4)
#define KX134_CNTL2_DOM_MSK			BIT(3)
#define KX134_CNTL2_DOM_MODE(x)			(((x) & 0x1) << 3)
#define KX134_CNTL2_UPM_MSK			BIT(2)
#define KX134_CNTL2_UPM_MODE(x)			(((x) & 0x1) << 2)
#define KX134_CNTL2_FDM_MSK			BIT(1)
#define KX134_CNTL2_FDM_MODE(x)			(((x) & 0x1) << 1)
#define KX134_CNTL2_FUM_MSK			BIT(0)
#define KX134_CNTL2_FUM_MODE(x)			(((x) & 0x1) << 0)

/* KX134_CNTL3 */
#define KX134_CNTL3_TILT_ODR_MSK		GENMASK(7, 6)
#define KX134_CNTL3_TILT_ODR_MODE(x)		(((x) & 0x3) << 6)
#define KX134_CNTL3_TAP_ODR_MSK			GENMASK(5, 4, 3)
#define KX134_CNTL3_TAP_ODR_MODE(x)		(((x) & 0x7) << 3)
#define KX134_CNTL3_WAKE_ODR_MSK		GENMASK(2, 0)
#define KX134_CNTL3_WAKE_ODR_MODE(x)		(((x) & 0x7) << 0)

/* KX134_CNTL4 */
#define KX134_CNTL4_DBNC_CNTR_MSK		BIT(7)
#define KX134_CNTL4_DBNC_CNTR_MODE(x)		(((x) & 0x1) << 7)
#define KX134_CNTL4_WAKE_TH_REL_MSK		BIT(6)
#define KX134_CNTL4_WAKE_TH_REL_MODE(x)		(((x) & 0x1) << 6)
#define KX134_CNTL4_WAKE_EN_MSK			BIT(5)
#define KX134_CNTL4_WAKE_EN_MODE(x)		(((x) & 0x1) << 5)
#define KX134_CNTL4_BTSLEEP_EN_MSK		BIT(4)
#define KX134_CNTL4_BTSLEEP_EN_MODE(x)		(((x) & 0x1) << 4)
#define KX134_CNTL4_PULSE_REJ_MSK		BIT(3)
#define KX134_CNTL4_PULSE_REJ_MODE(x)		(((x) & 0x1) << 3)
#define KX134_CNTL4_BTSLEEP_ODR_MSK		GENMASK(2, 0)
#define KX134_CNTL4_BTSLEEP_ODR_MODE(x)		(((x) & 0x7) << 0)

/* KX134_CNTL5 */
#define KX134_CNTL5_ADP_EN_MSK			BIT(4)
#define KX134_CNTL5_ADP_EN_MODE(x)		(((x) & 0x1) << 4)
#define KX134_CNTL5_FORCE_WAKE_MSK		BIT(1)
#define KX134_CNTL5_FORCE_WAKE_MODE(x)		(((x) & 0x1) << 1)
#define KX134_CNTL5_FORCE_SLEEP_MSK		BIT(0)
#define KX134_CNTL5_FORCE_SLEEP_MODE(x)		(((x) & 0x1) << 0)

/* KX134_CNTL6 */
#define KX134_CNTL6_I2C_ALE_MSK			BIT(7)
#define KX134_CNTL6_I2C_ALE_MODE(x)		(((x) & 0x1) << 7)
#define KX134_CNTL6_I2C_ALC_MSK			GENMASK(1, 0)
#define KX134_CNTL6_I2C_ALC_MODE(x)		(((x) & 0x3) << 0)

/* KX134_ODCNTL */
#define KX134_ODCNTL_LPF_ODR_BY2_MSK		BIT(6)
#define KX134_ODCNTL_LPF_ODR_BY2_MODE(x)	(((x) & 0x1) << 6)
#define KX134_ODCNTL_FAST_START_MSK		BIT(5)
#define KX134_ODCNTL_FAST_START_MODE(x)		(((x) & 0x1) << 5)
#define KX134_ODCNTL_OUT_ODR_MSK		GENMASK(3, 0)
#define KX134_ODCNTL_OUT_ODR_MODE(x)		(((x) & 0xF) << 0)

/* KX134_INC1 */
#define KX134_INC1_PW1_MSK			GENMASK(7, 6)
#define KX134_INC1_PW1_MODE(x)			(((x) & 0x3) << 6)
#define KX134_INC1_IEN1_MSK			BIT(5)
#define KX134_INC1_IEN1_MODE(x)			(((x) & 0x1) << 5)
#define KX134_INC1_IEA1_MSK			BIT(4)
#define KX134_INC1_IEA1_MODE(x)			(((x) & 0x1) << 4)
#define KX134_INC1_IEL1_MSK			BIT(3)
#define KX134_INC1_IEL1_MODE(x)			(((x) & 0x1) << 3)
#define KX134_INC1_STPOL_MSK			BIT(1)
#define KX134_INC1_STPOL_MODE(x)		(((x) & 0x1) << 1)
#define KX134_INC1_SPI3E_MSK			BIT(0)
#define KX134_INC1_SPI3E_MODE(x)		(((x) & 0x1) << 0)

/* KX134_INC2 */
#define KX134_INC2_AIO_MSK			BIT(6)
#define KX134_INC2_AIO_MODE(x)			(((x) & 0x1) << 6)
#define KX134_INC2_XNWUE_MSK			BIT(5)
#define KX134_INC2_XNWUE_MODE(x)		(((x) & 0x1) << 5)
#define KX134_INC2_XPWUE_MSK			BIT(4)
#define KX134_INC2_XPWUE_MODE(x)		(((x) & 0x1) << 4)
#define KX134_INC2_YNWUE_MSK			BIT(3)
#define KX134_INC2_YNWUE_MODE(x)		(((x) & 0x1) << 3)
#define KX134_INC2_YPWUE_MSK			BIT(2)
#define KX134_INC2_YPWUE_MODE(x)		(((x) & 0x1) << 2)
#define KX134_INC2_ZNWUE_MSK			BIT(1)
#define KX134_INC2_ZNWUE_MODE(x)		(((x) & 0x1) << 1)
#define KX134_INC2_ZPWUE_MSK			BIT(0)
#define KX134_INC2_ZPWUE_MODE(x)		(((x) & 0x1) << 0)

/* KX134_INC3 */
#define KX134_INC3_TMEN_MSK			BIT(6)
#define KX134_INC3_TMEN_MODE(x)			(((x) & 0x1) << 6)
#define KX134_INC3_TLEM_MSK			BIT(5)
#define KX134_INC3_TLEM_MODE(x)			(((x) & 0x1) << 5)
#define KX134_INC3_TRIM_MSK			BIT(4)
#define KX134_INC3_TRIM_MODE(x)			(((x) & 0x1) << 4)
#define KX134_INC3_TDOM_MSK			BIT(3)
#define KX134_INC3_TDOM_MODE(x)			(((x) & 0x1) << 3)
#define KX134_INC3_TUPM_MSK			BIT(2)
#define KX134_INC3_TUPM_MODE(x)			(((x) & 0x1) << 2)
#define KX134_INC3_TFDM_MSK			BIT(1)
#define KX134_INC3_TFDM_MODE(x)			(((x) & 0x1) << 1)
#define KX134_INC3_TFUM_MSK			BIT(0)
#define KX134_INC3_TFUM_MODE(x)			(((x) & 0x1) << 0)

/* KX134_INC4 */
#define KX134_INC4_FFI1_MSK			BIT(7)
#define KX134_INC4_FFI1_MODE(x)			(((x) & 0x1) << 7)
#define KX134_INC4_BFI1_MSK			BIT(6)
#define KX134_INC4_BFI1_MODE(x)			(((x) & 0x1) << 6)
#define KX134_INC4_WMI1_MSK			BIT(5)
#define KX134_INC4_WMI1_MODE(x)			(((x) & 0x1) << 5)
#define KX134_INC4_DRDYI1_MSK			BIT(4)
#define KX134_INC4_DRDYI1_MODE(x)		(((x) & 0x1) << 4)
#define KX134_INC4_BTSI1_MSK			BIT(3)
#define KX134_INC4_BTSI1_MODE(x)		(((x) & 0x1) << 3)
#define KX134_INC4_TDTI1_MSK			BIT(2)
#define KX134_INC4_TDTI1_MODE(x)		(((x) & 0x1) << 2)
#define KX134_INC4_WUFI1_MSK			BIT(1)
#define KX134_INC4_WUFI1_MODE(x)		(((x) & 0x1) << 1)
#define KX134_INC4_TPI1_MSK			BIT(0)
#define KX134_INC4_TPI1_MODE(x)			(((x) & 0x1) << 0)

/* KX134_INC5 */
#define KX134_INC5_PW2_MSK			GENMASK(7, 6)
#define KX134_INC5_PW2_MODE(x)			(((x) & 0x3) << 6)
#define KX134_INC5_IEN2_MSK			BIT(5)
#define KX134_INC5_IEN2_MODE(x)			(((x) & 0x1) << 5)
#define KX134_INC5_IEA2_MSK			BIT(4)
#define KX134_INC5_IEA2_MODE(x)			(((x) & 0x1) << 4)
#define KX134_INC5_IEL2_MSK			BIT(3)
#define KX134_INC5_IEL2_MODE(x)			(((x) & 0x1) << 3)
#define KX134_INC5_ACLR2_MSK			BIT(1)
#define KX134_INC5_ACLR2_MODE(x)		(((x) & 0x1) << 1)
#define KX134_INC5_ACLR1_MSK			BIT(0)
#define KX134_INC5_ACLR1_MODE(x)		(((x) & 0x1) << 0)

/* KX134_INC6 */
#define KX134_INC6_FFI2_MSK			BIT(7)
#define KX134_INC6_FFI2_MODE(x)			(((x) & 0x1) << 7)
#define KX134_INC6_BFI2_MSK			BIT(6)
#define KX134_INC6_BFI2_MODE(x)			(((x) & 0x1) << 6)
#define KX134_INC6_WMI2_MSK			BIT(5)
#define KX134_INC6_WMI2_MODE(x)			(((x) & 0x1) << 5)
#define KX134_INC6_DRDYI2_MSK			BIT(4)
#define KX134_INC6_DRDYI2_MODE(x)		(((x) & 0x1) << 4)
#define KX134_INC6_BTSI2_MSK			BIT(3)
#define KX134_INC6_BTSI2_MODE(x)		(((x) & 0x1) << 3)
#define KX134_INC6_TDTI2_MSK			BIT(2)
#define KX134_INC6_TDTI2_MODE(x)		(((x) & 0x1) << 2)
#define KX134_INC6_WUFI2_MSK			BIT(1)
#define KX134_INC6_WUFI2_MODE(x)		(((x) & 0x1) << 1)
#define KX134_INC6_TPI2_MSK			BIT(0)
#define KX134_INC6_TPI2_MODE(x)			(((x) & 0x1) << 0)

/* KX134_TDTRC */
#define KX134_TDTRC_DTRE_MSK			BIT(1)
#define KX134_TDTRC_DTRE_MODE(x)		(((x) & 0x1) << 1)
#define KX134_TDTRC_STRE_MSK			BIT(0)
#define KX134_TDTRC_STRE_MODE(x)		(((x) & 0x1) << 0)

/* KX134_FFCNTL */
#define KX134_FFCNTL_FFIE_MSK			BIT(7)
#define KX134_FFCNTL_FFIE_MODE(x)		(((x) & 0x1) << 7)
#define KX134_FFCNTL_ULMODE_MSK			BIT(6)
#define KX134_FFCNTL_ULMODE_MODE(x)		(((x) & 0x1) << 6)
#define KX134_FFCNTL_FFDC_MSK			GENMASK(5, 4)
#define KX134_FFCNTL_FFDC_MODE(x)		(((x) & 0x3) << 4)
#define KX134_FFCNTL_DCRM_MSK			BIT(3)
#define KX134_FFCNTL_DCRM_MODE(x)		(((x) & 0x1) << 3)
#define KX134_FFCNTL_OFFI_MSK			GENMASK(2, 0)
#define KX134_FFCNTL_OFFI_MODE(x)		(((x) & 0x7) << 0)

/* KX134_HYST_SET */
#define KX134_HYST_SET_MSK			GENMASK(5, 0)
#define KX134_HYST_SET_MODE(x)			(((x) & 0x3F) << 0)

/* KX134_LP_CNTL1 */
#define KX134_LP_CNTL1_AVC_MSK			GENMASK(6, 4)
#define KX134_LP_CNTL1_AVC_MODE(x)		(((x) & 0x7) << 4)

/* KX134_LP_CNTL2 */
#define KX134_LP_CNTL2_LPSTPSEL_MSK		BIT(0)
#define KX134_LP_CNTL2_LPSTPSEL_MODE(x)		(((x) & 0x1) << 0)

/* KX134_WUFTH */
#define KX134_WUFTH_HI_REG_MSK			GENMASK(2, 0)
#define KX134_WUFTH_HI_REG_MODE(x)		(((x) & 0x7) << 0)

/* KX134_BTSTH */
#define KX134_BTSTH_HI_REG_MSK			GENMASK(6, 4)
#define KX134_BTSTH_HI_REG_MODE(x)		(((x) & 0x7) << 4)

/* KX134_BUF_CNTL2 */
#define KX134_BUF_CNTL2_BUFE_MSK		BIT(7)
#define KX134_BUF_CNTL2_BUFE_MODE(x)		(((x) & 0x1) << 7)
#define KX134_BUF_CNTL2_BRES_MSK		BIT(6)
#define KX134_BUF_CNTL2_BRES_MODE(x)		(((x) & 0x1) << 6)
#define KX134_BUF_CNTL2_BFIE_MSK		BIT(5)
#define KX134_BUF_CNTL2_BFIE_MODE(x)		(((x) & 0x1) << 5)
#define KX134_BUF_CNTL2_BM_MSK			GENMASK(1, 0)
#define KX134_BUF_CNTL2_BM_MODE(x)		(((x) & 0x3) << 0)

/* KX134_BUF_STATUS */
#define KX134_BUF_STATUS_SMP_LEV(x)		(((x) >> 0) & 0x3FF)
#define KX134_BUF_STATUS_BUF_TRIG(x)		(((x) >> 7) & 0x1)

/* KX134_ADP_CNTL1 */
#define KX134_ADP_CNTL1_RMS_AVC_MSK		GENMASK(6, 4)
#define KX134_ADP_CNTL1_RMS_AVC_MODE(x)		(((x) & 0x7) << 4)
#define KX134_ADP_CNTL1_OADP_MSK		GENMASK(3, 0)
#define KX134_ADP_CNTL1_OADP_MODE(x)		(((x) & 0xF) << 0)

/* KX134_ADP_CNTL2 */
#define KX134_ADP_CNTL2_ADP_BUF_SEL_MSK		BIT(7)
#define KX134_ADP_CNTL2_ADP_BUF_SEL_MODE(x)	(((x) & 0x1) << 7)
#define KX134_ADP_CNTL2_ADP_WB_ISEL_MSK		BIT(6)
#define KX134_ADP_CNTL2_ADP_WB_ISEL_MODE(x)	(((x) & 0x1) << 6)
#define KX134_ADP_CNTL2_RMS_WB_OSEL_MSK		BIT(5)
#define KX134_ADP_CNTL2_RMS_WB_OSEL_MODE(x)	(((x) & 0x1) << 5)
#define KX134_ADP_CNTL2_ADP_FLT2_BYP_MSK	BIT(4)
#define KX134_ADP_CNTL2_ADP_FLT2_BYP_MODE(x)	(((x) & 0x1) << 4)
#define KX134_ADP_CNTL2_ADP_FLT1_BYP_MSK	BIT(3)
#define KX134_ADP_CNTL2_ADP_FLT1_BYP_MODE(x)	(((x) & 0x1) << 3)
#define KX134_ADP_CNTL2_ADP_RMS_OSEL_MSK	BIT(1)
#define KX134_ADP_CNTL2_ADP_RMS_OSEL_MODE(x)	(((x) & 0x1) << 1)
#define KX134_ADP_CNTL2_ADP_F2_HP_MSK		BIT(0)
#define KX134_ADP_CNTL2_ADP_F2_HP_MODE(x)	(((x) & 0x1) << 0)

/* KX134_ADP_CNTL11 */
#define KX134_ADP_CNTL11_ADP_F1_OSH_MSK		BIT(7)
#define KX134_ADP_CNTL11_ADP_F1_OSH_MODE(x)	(((x) & 0x1) << 7)
#define KX134_ADP_CNTL11_ADP_F2_1A_MSK		GENMASK(6, 0)
#define KX134_ADP_CNTL11_ADP_F2_1A_MODE(x)	(((x) & 0x7F) << 0)


/* KX134 scale factors from specifications */
#define KX134_ACCEL_8G_LSB_PER_G	4167
#define KX134_ACCEL_16G_LSB_PER_G	2041
#define KX134_ACCEL_32G_LSB_PER_G	1020
#define KX134_ACCEL_64G_LSB_PER_G	513


/* Extended Sensor trigger types */
enum kx134_sensor_trigger_type {
	/* End of parent enum, beginning of extended */
	KX134_SENSOR_TRIG_PRIV_START = SENSOR_TRIG_PRIV_START,
	
	/* Any 1 of 8 triggers available to physical INT pin */
	KX134_SENSOR_TRIG_ANY,
};

/* Extended Sensor attributes */
enum kx134_sensor_attribute {
	/* End of parent enum, beginning of extended */
        KX134_SENSOR_ATTR_PRIV_START = SENSOR_ATTR_PRIV_START,

	/* */
	KX134_SENSOR_ATTR_INT_SOURCE,
};


enum kx134_op_mode {
	KX134_STANDBY = 0,
	KX134_LOW_POWER = 2,
	KX134_HI_PERFORMANCE = 3
};

enum kx134_gsel {
	KX134_GSEL_8G,
	KX134_GSEL_16G,
	KX134_GSEL_32G,
	KX134_GSEL_64G
};

enum kx134_otp {
	KX134_OTP_1HZ563,
	KX134_OTP_6HZ25,
	KX134_OTP_12HZ5,
	KX134_OTP_50HZ
};

enum kx134_otdt {
	KX134_OTDT_12HZ5,
	KX134_OTDT_25HZ,
	KX134_OTDT_50HZ,
	KX134_OTDT_100HZ,
	KX134_OTDT_200HZ,
	KX134_OTDT_400HZ,
	KX134_OTDT_800HZ,
	KX134_OTDT_1600HZ
};

enum kx134_owuf {
	KX134_OWUF_0HZ781,
	KX134_OWUF_1HZ563,
	KX134_OWUF_3HZ125,
	KX134_OWUF_6HZ25,
	KX134_OWUF_12HZ5,
	KX134_OWUF_25HZ,
	KX134_OWUF_50HZ,
	KX134_OWUF_100HZ
};

enum kx134_obts {
	KX134_OBTS_0HZ781,
	KX134_OBTS_1HZ563,
	KX134_OBTS_3HZ125,
	KX134_OBTS_6HZ25,
	KX134_OBTS_12HZ5,
	KX134_OBTS_25HZ,
	KX134_OBTS_50HZ,
	KX134_OBTS_100HZ
};

enum kx134_osa {
	KX134_OSA_0HZ781,
	KX134_OSA_1HZ563,
	KX134_OSA_3HZ125,
	KX134_OSA_6HZ25,
	KX134_OSA_12HZ5,
	KX134_OSA_25HZ,
	KX134_OSA_50HZ,
	KX134_OSA_100HZ,
	KX134_OSA_200HZ,
	KX134_OSA_400HZ,
	KX134_OSA_800HZ,
	KX134_OSA_1600HZ,
	KX134_OSA_3200HZ,
	KX134_OSA_6400HZ,
	KX134_OSA_12800HZ,
	KX134_OSA_25600HZ
};

enum kx134_offi {
	KX134_OFFI_12HZ5,
	KX134_OFFI_25HZ,
	KX134_OFFI_50HZ,
	KX134_OFFI_100HZ,
	KX134_OFFI_200HZ,
	KX134_OFFI_400HZ,
	KX134_OFFI_800HZ,
	KX134_OFFI_1600HZ
};

enum kx134_avc {
	KX134_NO_AVG,
	KX134_AVG_2_SAMPLES,
	KX134_AVG_4_SAMPLES,
	KX134_AVG_8_SAMPLES,
	KX134_AVG_16_SAMPLES,
	KX134_AVG_32_SAMPLES,
	KX134_AVG_64_SAMPLES,
	KX134_AVG_128_SAMPLES
};

enum kx134_rms_avc {
	KX134_RMS_AVG_2_SAMPLES,
	KX134_RMS_AVG_4_SAMPLES,
	KX134_RMS_AVG_8_SAMPLES,
	KX134_RMS_AVG_16_SAMPLES,
	KX134_RMS_AVG_32_SAMPLES,
	KX134_RMS_AVG_64_SAMPLES,
	KX134_RMS_AVG_128_SAMPLES,
	KX134_RMS_AVG_256_SAMPLES
};

enum kx134_oadp {
	KX134_OADP_0HZ781,
	KX134_OADP_1HZ563,
	KX134_OADP_3HZ125,
	KX134_OADP_6HZ25,
	KX134_OADP_12HZ5,
	KX134_OADP_25HZ,
	KX134_OADP_50HZ,
	KX134_OADP_100HZ,
	KX134_OADP_200HZ,
	KX134_OADP_400HZ,
	KX134_OADP_800HZ,
	KX134_OADP_1600HZ,
	KX134_OADP_3200HZ,
	KX134_OADP_6400HZ,
	KX134_OADP_12800HZ,
	KX134_OADP_25600HZ
};

enum kx134_buff_mode {
	KX134_BM_FIFO,
	KX134_BM_STREAM,
	KX134_BM_TRIGGER
};

struct kx134_buff_config {
	enum kx134_buff_mode buff_mode;
        uint16_t fifo_samples;
};

struct kx134_xyz_accel_data {
	float x;
	float y;
	float z;
};

struct kx134_data {
	const struct device *bus;
#if defined(CONFIG_KX134_SPI)
	struct spi_config spi_cfg;
	struct spi_cs_control kx134_cs_ctrl;
#endif
	union {
		int16_t acc_xyz[3];
		struct {
			int16_t acc_x;
			int16_t acc_y;
			int16_t acc_z;
		};
	} __packed;

        uint8_t selected_range;

#if defined(CONFIG_KX134_TRIGGER)
        const struct device *dev;
	const struct device *gpio;
	struct gpio_callback gpio_cb;
        struct k_mutex trigger_mutex;

	sensor_trigger_handler_t th_handler;
	struct sensor_trigger th_trigger;
	sensor_trigger_handler_t drdy_handler;
	struct sensor_trigger drdy_trigger;
        sensor_trigger_handler_t any_handler;
	struct sensor_trigger any_trigger;

        uint8_t int1_config;
        uint8_t int1_source;
	uint8_t int2_config;
        uint8_t int2_source;

#if defined(CONFIG_KX134_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_KX134_THREAD_STACK_SIZE);
	struct k_sem gpio_sem;
	struct k_thread thread;
#elif defined(CONFIG_KX134_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
#endif /* CONFIG_KX134_TRIGGER */
};

struct kx134_config {
#ifdef CONFIG_KX134_I2C
	const char *i2c_port;
	uint16_t i2c_addr;
#endif

#ifdef CONFIG_KX134_SPI
	const char *spi_port;
	uint16_t spi_slave;
	uint32_t spi_max_frequency;
	const char *gpio_cs_port;
	gpio_pin_t cs_gpio;
	gpio_dt_flags_t cs_flags;
#endif /* CONFIG_KX134_SPI */

#if defined(CONFIG_KX134_TRIGGER)
	const char *gpio_port;
	gpio_pin_t int_gpio;
	gpio_dt_flags_t int_flags;
#endif

	/* Device Settings */
	struct kx134_buff_config buff_config;

	enum kx134_op_mode op;
	enum kx134_gsel gsel;
	enum kx134_otp otp;
	enum kx134_otdt otdt;
	enum kx134_owuf owuf;
	enum kx134_obts obts;
	enum kx134_osa osa;
	enum kx134_offi offi;
	enum kx134_avc avc;
	enum kx134_rms_avc rms_avc;
	enum kx134_oadp oadp;
};

#ifdef CONFIG_KX134_TRIGGER
int kx134_get_status(const struct device *dev, uint8_t *status);

int kx134_reg_write_mask(const struct device *dev,
			   uint8_t reg_addr, uint8_t mask, uint8_t data);

int kx134_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int kx134_init_interrupt(const struct device *dev);

int kx134_clear_interrupts(const struct device *dev);
#endif /* CONFIG_KX134_TRIGGER */

#endif /* ZEPHYR_DRIVERS_SENSOR_KX134_H_ */
