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

#define BM1422_STA1			0x18u  /* Status Register */

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
#define BM1422_PART_ID_VAL		0x33u   /* TBD */
#define BM1422_WHO_AM_I_VAL		0x41u  	/* Supplier Recognition ID */


#define BM1422_READ				0x01u
#define BM1422_REG_READ(x)		(((x & 0xFF) << 1) | BM1422_READ)
#define BM1422_REG_WRITE(x)		((x & 0xFF) << 1)
#define BM1422_TO_I2C_REG(x)	((x) >> 1)

#define PWRTWO(x)               (1 << (x))


/* BM1422_TSCP */
#define BM1422_TSCP_LE(x)			(((x) >> 5) & 0x1)
#define BM1422_TSCP_RI(x)			(((x) >> 4) & 0x1)
#define BM1422_TSCP_DO(x)			(((x) >> 3) & 0x1)
#define BM1422_TSCP_UP(x)			(((x) >> 2) & 0x1)
#define BM1422_TSCP_FD(x)			(((x) >> 1) & 0x1)
#define BM1422_TSCP_FU(x)			(((x) >> 0) & 0x1)

/* BM1422_TSPP */
#define BM1422_TSPP_LE(x)			(((x) >> 5) & 0x1)
#define BM1422_TSPP_RI(x)			(((x) >> 4) & 0x1)
#define BM1422_TSPP_DO(x)			(((x) >> 3) & 0x1)
#define BM1422_TSPP_UP(x)			(((x) >> 2) & 0x1)
#define BM1422_TSPP_FD(x)			(((x) >> 1) & 0x1)
#define BM1422_TSPP_FU(x)			(((x) >> 0) & 0x1)

/* BM1422_INS1 */
#define BM1422_INS1_TLE(x)			(((x) >> 5) & 0x1)
#define BM1422_INS1_TRI(x)			(((x) >> 4) & 0x1)
#define BM1422_INS1_TDO(x)			(((x) >> 3) & 0x1)
#define BM1422_INS1_TUP(x)			(((x) >> 2) & 0x1)
#define BM1422_INS1_TFD(x)			(((x) >> 1) & 0x1)
#define BM1422_INS1_TFU(x)			(((x) >> 0) & 0x1)

/* BM1422_INS2 */
#define BM1422_INS2_FFS(x)			(((x) >> 7) & 0x1)
#define BM1422_INS2_BFI(x)			(((x) >> 6) & 0x1)
#define BM1422_INS2_WMI(x)			(((x) >> 5) & 0x1)
#define BM1422_INS2_DRDY(x)			(((x) >> 4) & 0x1)
#define BM1422_INS2_STS(x)			(((x) >> 3) & 0x1)
#define BM1422_INS2_DTS(x)			(((x) >> 2) & 0x1)
#define BM1422_INS2_TPS(x)			(((x) >> 0) & 0x1)

/* BM1422_INS3 */
#define BM1422_INS3_WUFS(x)			(((x) >> 7) & 0x1)
#define BM1422_INS3_BTS(x)			(((x) >> 6) & 0x1)
#define BM1422_INS3_XNWU(x)			(((x) >> 5) & 0x1)
#define BM1422_INS3_XPWU(x)			(((x) >> 4) & 0x1)
#define BM1422_INS3_YNWU(x)			(((x) >> 3) & 0x1)
#define BM1422_INS3_YPWU(x)			(((x) >> 2) & 0x1)
#define BM1422_INS3_ZNWU(x)			(((x) >> 1) & 0x1)
#define BM1422_INS3_ZPWU(x)			(((x) >> 0) & 0x1)

/* BM1422_STATUS_REG */
#define BM1422_STATUS_REG_INT(x)                 (((x) >> 4) & 0x1)
#define BM1422_STATUS_REG_WAKE(x)		(((x) >> 0) & 0x1)

/* BM1422_CNTL1 */
#define BM1422_CNTL1_ACTIVE_MSK			GENMASK(7, 6)
#define BM1422_CNTL1_ACTIVE_MODE(x)		(((x) & 0x3) << 6)
#define BM1422_CNTL1_DRDY_EN_MSK			BIT(5)
#define BM1422_CNTL1_DRDY_EN_MODE(x)		(((x) & 0x1) << 5)
#define BM1422_CNTL1_GSEL_MSK			GENMASK(4, 3)
#define BM1422_CNTL1_GSEL_MODE(x)		(((x) & 0x3) << 3)
#define BM1422_CNTL1_TAP_EN_MSK			BIT(2)
#define BM1422_CNTL1_TAP_EN_MODE(x)		(((x) & 0x1) << 2)
#define BM1422_CNTL1_TILT_EN_MSK			BIT(0)
#define BM1422_CNTL1_TILT_EN_MODE(x)		(((x) & 0x1) << 0)

/* BM1422_CNTL2 */
#define BM1422_CNTL2_SOFT_RST_MSK		BIT(7)
#define BM1422_CNTL2_SOFT_RST_MODE(x)		(((x) & 0x1) << 7)
#define BM1422_CNTL2_LEM_MSK			BIT(5)
#define BM1422_CNTL2_LEM_MODE(x)			(((x) & 0x1) << 5)
#define BM1422_CNTL2_RIM_MSK			BIT(4)
#define BM1422_CNTL2_RIM_MODE(x)			(((x) & 0x1) << 4)
#define BM1422_CNTL2_DOM_MSK			BIT(3)
#define BM1422_CNTL2_DOM_MODE(x)			(((x) & 0x1) << 3)
#define BM1422_CNTL2_UPM_MSK			BIT(2)
#define BM1422_CNTL2_UPM_MODE(x)			(((x) & 0x1) << 2)
#define BM1422_CNTL2_FDM_MSK			BIT(1)
#define BM1422_CNTL2_FDM_MODE(x)			(((x) & 0x1) << 1)
#define BM1422_CNTL2_FUM_MSK			BIT(0)
#define BM1422_CNTL2_FUM_MODE(x)			(((x) & 0x1) << 0)

/* BM1422_CNTL3 */
#define BM1422_CNTL3_TILT_ODR_MSK		GENMASK(7, 6)
#define BM1422_CNTL3_TILT_ODR_MODE(x)		(((x) & 0x3) << 6)
#define BM1422_CNTL3_TAP_ODR_MSK			GENMASK(5, 4, 3)
#define BM1422_CNTL3_TAP_ODR_MODE(x)		(((x) & 0x7) << 3)
#define BM1422_CNTL3_WAKE_ODR_MSK		GENMASK(2, 0)
#define BM1422_CNTL3_WAKE_ODR_MODE(x)		(((x) & 0x7) << 0)

/* BM1422_CNTL4 */
#define BM1422_CNTL4_DBNC_CNTR_MSK		BIT(7)
#define BM1422_CNTL4_DBNC_CNTR_MODE(x)		(((x) & 0x1) << 7)
#define BM1422_CNTL4_WAKE_TH_REL_MSK		BIT(6)
#define BM1422_CNTL4_WAKE_TH_REL_MODE(x)		(((x) & 0x1) << 6)
#define BM1422_CNTL4_WAKE_EN_MSK			BIT(5)
#define BM1422_CNTL4_WAKE_EN_MODE(x)		(((x) & 0x1) << 5)
#define BM1422_CNTL4_BTSLEEP_EN_MSK		BIT(4)
#define BM1422_CNTL4_BTSLEEP_EN_MODE(x)		(((x) & 0x1) << 4)
#define BM1422_CNTL4_PULSE_REJ_MSK		BIT(3)
#define BM1422_CNTL4_PULSE_REJ_MODE(x)		(((x) & 0x1) << 3)
#define BM1422_CNTL4_BTSLEEP_ODR_MSK		GENMASK(2, 0)
#define BM1422_CNTL4_BTSLEEP_ODR_MODE(x)		(((x) & 0x7) << 0)

/* BM1422_CNTL5 */
#define BM1422_CNTL5_ADP_EN_MSK			BIT(4)
#define BM1422_CNTL5_ADP_EN_MODE(x)		(((x) & 0x1) << 4)
#define BM1422_CNTL5_FORCE_WAKE_MSK		BIT(1)
#define BM1422_CNTL5_FORCE_WAKE_MODE(x)		(((x) & 0x1) << 1)
#define BM1422_CNTL5_FORCE_SLEEP_MSK		BIT(0)
#define BM1422_CNTL5_FORCE_SLEEP_MODE(x)		(((x) & 0x1) << 0)

/* BM1422_CNTL6 */
#define BM1422_CNTL6_I2C_ALE_MSK			BIT(7)
#define BM1422_CNTL6_I2C_ALE_MODE(x)		(((x) & 0x1) << 7)
#define BM1422_CNTL6_I2C_ALC_MSK			GENMASK(1, 0)
#define BM1422_CNTL6_I2C_ALC_MODE(x)		(((x) & 0x3) << 0)

/* BM1422_ODCNTL */
#define BM1422_ODCNTL_LPF_ODR_BY2_MSK		BIT(6)
#define BM1422_ODCNTL_LPF_ODR_BY2_MODE(x)	(((x) & 0x1) << 6)
#define BM1422_ODCNTL_FAST_START_MSK		BIT(5)
#define BM1422_ODCNTL_FAST_START_MODE(x)		(((x) & 0x1) << 5)
#define BM1422_ODCNTL_OUT_ODR_MSK		GENMASK(3, 0)
#define BM1422_ODCNTL_OUT_ODR_MODE(x)		(((x) & 0xF) << 0)

/* BM1422_INC1 */
#define BM1422_INC1_PW1_MSK			GENMASK(7, 6)
#define BM1422_INC1_PW1_MODE(x)			(((x) & 0x3) << 6)
#define BM1422_INC1_IEN1_MSK			BIT(5)
#define BM1422_INC1_IEN1_MODE(x)			(((x) & 0x1) << 5)
#define BM1422_INC1_IEA1_MSK			BIT(4)
#define BM1422_INC1_IEA1_MODE(x)			(((x) & 0x1) << 4)
#define BM1422_INC1_IEL1_MSK			BIT(3)
#define BM1422_INC1_IEL1_MODE(x)			(((x) & 0x1) << 3)
#define BM1422_INC1_STPOL_MSK			BIT(1)
#define BM1422_INC1_STPOL_MODE(x)		(((x) & 0x1) << 1)
#define BM1422_INC1_SPI3E_MSK			BIT(0)
#define BM1422_INC1_SPI3E_MODE(x)		(((x) & 0x1) << 0)

/* BM1422_INC2 */
#define BM1422_INC2_AIO_MSK			BIT(6)
#define BM1422_INC2_AIO_MODE(x)			(((x) & 0x1) << 6)
#define BM1422_INC2_XNWUE_MSK			BIT(5)
#define BM1422_INC2_XNWUE_MODE(x)		(((x) & 0x1) << 5)
#define BM1422_INC2_XPWUE_MSK			BIT(4)
#define BM1422_INC2_XPWUE_MODE(x)		(((x) & 0x1) << 4)
#define BM1422_INC2_YNWUE_MSK			BIT(3)
#define BM1422_INC2_YNWUE_MODE(x)		(((x) & 0x1) << 3)
#define BM1422_INC2_YPWUE_MSK			BIT(2)
#define BM1422_INC2_YPWUE_MODE(x)		(((x) & 0x1) << 2)
#define BM1422_INC2_ZNWUE_MSK			BIT(1)
#define BM1422_INC2_ZNWUE_MODE(x)		(((x) & 0x1) << 1)
#define BM1422_INC2_ZPWUE_MSK			BIT(0)
#define BM1422_INC2_ZPWUE_MODE(x)		(((x) & 0x1) << 0)

/* BM1422_INC3 */
#define BM1422_INC3_TMEN_MSK			BIT(6)
#define BM1422_INC3_TMEN_MODE(x)			(((x) & 0x1) << 6)
#define BM1422_INC3_TLEM_MSK			BIT(5)
#define BM1422_INC3_TLEM_MODE(x)			(((x) & 0x1) << 5)
#define BM1422_INC3_TRIM_MSK			BIT(4)
#define BM1422_INC3_TRIM_MODE(x)			(((x) & 0x1) << 4)
#define BM1422_INC3_TDOM_MSK			BIT(3)
#define BM1422_INC3_TDOM_MODE(x)			(((x) & 0x1) << 3)
#define BM1422_INC3_TUPM_MSK			BIT(2)
#define BM1422_INC3_TUPM_MODE(x)			(((x) & 0x1) << 2)
#define BM1422_INC3_TFDM_MSK			BIT(1)
#define BM1422_INC3_TFDM_MODE(x)			(((x) & 0x1) << 1)
#define BM1422_INC3_TFUM_MSK			BIT(0)
#define BM1422_INC3_TFUM_MODE(x)			(((x) & 0x1) << 0)

/* BM1422_INC4 */
#define BM1422_INC4_FFI1_MSK			BIT(7)
#define BM1422_INC4_FFI1_MODE(x)			(((x) & 0x1) << 7)
#define BM1422_INC4_BFI1_MSK			BIT(6)
#define BM1422_INC4_BFI1_MODE(x)			(((x) & 0x1) << 6)
#define BM1422_INC4_WMI1_MSK			BIT(5)
#define BM1422_INC4_WMI1_MODE(x)			(((x) & 0x1) << 5)
#define BM1422_INC4_DRDYI1_MSK			BIT(4)
#define BM1422_INC4_DRDYI1_MODE(x)		(((x) & 0x1) << 4)
#define BM1422_INC4_BTSI1_MSK			BIT(3)
#define BM1422_INC4_BTSI1_MODE(x)		(((x) & 0x1) << 3)
#define BM1422_INC4_TDTI1_MSK			BIT(2)
#define BM1422_INC4_TDTI1_MODE(x)		(((x) & 0x1) << 2)
#define BM1422_INC4_WUFI1_MSK			BIT(1)
#define BM1422_INC4_WUFI1_MODE(x)		(((x) & 0x1) << 1)
#define BM1422_INC4_TPI1_MSK			BIT(0)
#define BM1422_INC4_TPI1_MODE(x)			(((x) & 0x1) << 0)

/* BM1422_INC5 */
#define BM1422_INC5_PW2_MSK			GENMASK(7, 6)
#define BM1422_INC5_PW2_MODE(x)			(((x) & 0x3) << 6)
#define BM1422_INC5_IEN2_MSK			BIT(5)
#define BM1422_INC5_IEN2_MODE(x)			(((x) & 0x1) << 5)
#define BM1422_INC5_IEA2_MSK			BIT(4)
#define BM1422_INC5_IEA2_MODE(x)			(((x) & 0x1) << 4)
#define BM1422_INC5_IEL2_MSK			BIT(3)
#define BM1422_INC5_IEL2_MODE(x)			(((x) & 0x1) << 3)
#define BM1422_INC5_ACLR2_MSK			BIT(1)
#define BM1422_INC5_ACLR2_MODE(x)		(((x) & 0x1) << 1)
#define BM1422_INC5_ACLR1_MSK			BIT(0)
#define BM1422_INC5_ACLR1_MODE(x)		(((x) & 0x1) << 0)

/* BM1422_INC6 */
#define BM1422_INC6_FFI2_MSK			BIT(7)
#define BM1422_INC6_FFI2_MODE(x)			(((x) & 0x1) << 7)
#define BM1422_INC6_BFI2_MSK			BIT(6)
#define BM1422_INC6_BFI2_MODE(x)			(((x) & 0x1) << 6)
#define BM1422_INC6_WMI2_MSK			BIT(5)
#define BM1422_INC6_WMI2_MODE(x)			(((x) & 0x1) << 5)
#define BM1422_INC6_DRDYI2_MSK			BIT(4)
#define BM1422_INC6_DRDYI2_MODE(x)		(((x) & 0x1) << 4)
#define BM1422_INC6_BTSI2_MSK			BIT(3)
#define BM1422_INC6_BTSI2_MODE(x)		(((x) & 0x1) << 3)
#define BM1422_INC6_TDTI2_MSK			BIT(2)
#define BM1422_INC6_TDTI2_MODE(x)		(((x) & 0x1) << 2)
#define BM1422_INC6_WUFI2_MSK			BIT(1)
#define BM1422_INC6_WUFI2_MODE(x)		(((x) & 0x1) << 1)
#define BM1422_INC6_TPI2_MSK			BIT(0)
#define BM1422_INC6_TPI2_MODE(x)			(((x) & 0x1) << 0)

/* BM1422_TDTRC */
#define BM1422_TDTRC_DTRE_MSK			BIT(1)
#define BM1422_TDTRC_DTRE_MODE(x)		(((x) & 0x1) << 1)
#define BM1422_TDTRC_STRE_MSK			BIT(0)
#define BM1422_TDTRC_STRE_MODE(x)		(((x) & 0x1) << 0)

/* BM1422_FFCNTL */
#define BM1422_FFCNTL_FFIE_MSK			BIT(7)
#define BM1422_FFCNTL_FFIE_MODE(x)		(((x) & 0x1) << 7)
#define BM1422_FFCNTL_ULMODE_MSK			BIT(6)
#define BM1422_FFCNTL_ULMODE_MODE(x)		(((x) & 0x1) << 6)
#define BM1422_FFCNTL_FFDC_MSK			GENMASK(5, 4)
#define BM1422_FFCNTL_FFDC_MODE(x)		(((x) & 0x3) << 4)
#define BM1422_FFCNTL_DCRM_MSK			BIT(3)
#define BM1422_FFCNTL_DCRM_MODE(x)		(((x) & 0x1) << 3)
#define BM1422_FFCNTL_OFFI_MSK			GENMASK(2, 0)
#define BM1422_FFCNTL_OFFI_MODE(x)		(((x) & 0x7) << 0)

/* BM1422_HYST_SET */
#define BM1422_HYST_SET_MSK			GENMASK(5, 0)
#define BM1422_HYST_SET_MODE(x)			(((x) & 0x3F) << 0)

/* BM1422_LP_CNTL1 */
#define BM1422_LP_CNTL1_AVC_MSK			GENMASK(6, 4)
#define BM1422_LP_CNTL1_AVC_MODE(x)		(((x) & 0x7) << 4)

/* BM1422_LP_CNTL2 */
#define BM1422_LP_CNTL2_LPSTPSEL_MSK		BIT(0)
#define BM1422_LP_CNTL2_LPSTPSEL_MODE(x)		(((x) & 0x1) << 0)

/* BM1422_WUFTH */
#define BM1422_WUFTH_HI_REG_MSK			GENMASK(2, 0)
#define BM1422_WUFTH_HI_REG_MODE(x)		(((x) & 0x7) << 0)

/* BM1422_BTSTH */
#define BM1422_BTSTH_HI_REG_MSK			GENMASK(6, 4)
#define BM1422_BTSTH_HI_REG_MODE(x)		(((x) & 0x7) << 4)

/* BM1422_BUF_CNTL2 */
#define BM1422_BUF_CNTL2_BUFE_MSK		BIT(7)
#define BM1422_BUF_CNTL2_BUFE_MODE(x)		(((x) & 0x1) << 7)
#define BM1422_BUF_CNTL2_BRES_MSK		BIT(6)
#define BM1422_BUF_CNTL2_BRES_MODE(x)		(((x) & 0x1) << 6)
#define BM1422_BUF_CNTL2_BFIE_MSK		BIT(5)
#define BM1422_BUF_CNTL2_BFIE_MODE(x)		(((x) & 0x1) << 5)
#define BM1422_BUF_CNTL2_BM_MSK			GENMASK(1, 0)
#define BM1422_BUF_CNTL2_BM_MODE(x)		(((x) & 0x3) << 0)

/* BM1422_BUF_STATUS */
#define BM1422_BUF_STATUS_SMP_LEV(x)		(((x) >> 0) & 0x3FF)
#define BM1422_BUF_STATUS_BUF_TRIG(x)		(((x) >> 7) & 0x1)

/* BM1422_ADP_CNTL1 */
#define BM1422_ADP_CNTL1_RMS_AVC_MSK		GENMASK(6, 4)
#define BM1422_ADP_CNTL1_RMS_AVC_MODE(x)		(((x) & 0x7) << 4)
#define BM1422_ADP_CNTL1_OADP_MSK		GENMASK(3, 0)
#define BM1422_ADP_CNTL1_OADP_MODE(x)		(((x) & 0xF) << 0)

/* BM1422_ADP_CNTL2 */
#define BM1422_ADP_CNTL2_ADP_BUF_SEL_MSK		BIT(7)
#define BM1422_ADP_CNTL2_ADP_BUF_SEL_MODE(x)	(((x) & 0x1) << 7)
#define BM1422_ADP_CNTL2_ADP_WB_ISEL_MSK		BIT(6)
#define BM1422_ADP_CNTL2_ADP_WB_ISEL_MODE(x)	(((x) & 0x1) << 6)
#define BM1422_ADP_CNTL2_RMS_WB_OSEL_MSK		BIT(5)
#define BM1422_ADP_CNTL2_RMS_WB_OSEL_MODE(x)	(((x) & 0x1) << 5)
#define BM1422_ADP_CNTL2_ADP_FLT2_BYP_MSK	BIT(4)
#define BM1422_ADP_CNTL2_ADP_FLT2_BYP_MODE(x)	(((x) & 0x1) << 4)
#define BM1422_ADP_CNTL2_ADP_FLT1_BYP_MSK	BIT(3)
#define BM1422_ADP_CNTL2_ADP_FLT1_BYP_MODE(x)	(((x) & 0x1) << 3)
#define BM1422_ADP_CNTL2_ADP_RMS_OSEL_MSK	BIT(1)
#define BM1422_ADP_CNTL2_ADP_RMS_OSEL_MODE(x)	(((x) & 0x1) << 1)
#define BM1422_ADP_CNTL2_ADP_F2_HP_MSK		BIT(0)
#define BM1422_ADP_CNTL2_ADP_F2_HP_MODE(x)	(((x) & 0x1) << 0)

/* BM1422_ADP_CNTL11 */
#define BM1422_ADP_CNTL11_ADP_F1_OSH_MSK		BIT(7)
#define BM1422_ADP_CNTL11_ADP_F1_OSH_MODE(x)	(((x) & 0x1) << 7)
#define BM1422_ADP_CNTL11_ADP_F2_1A_MSK		GENMASK(6, 0)
#define BM1422_ADP_CNTL11_ADP_F2_1A_MODE(x)	(((x) & 0x7F) << 0)


/* BM1422 scale factors from specifications */
#define BM1422_ACCEL_8G_LSB_PER_G	4167
#define BM1422_ACCEL_16G_LSB_PER_G	2041
#define BM1422_ACCEL_32G_LSB_PER_G	1020
#define BM1422_ACCEL_64G_LSB_PER_G	513


/* Extended Sensor trigger types */
enum bm1422_sensor_trigger_type {
	/* End of parent enum, beginning of extended */
	BM1422_SENSOR_TRIG_PRIV_START = SENSOR_TRIG_PRIV_START,
	
	/* Any 1 of 8 triggers available to physical INT pin */
	BM1422_SENSOR_TRIG_ANY,
};

/* Extended Sensor attributes */
enum bm1422_sensor_channel {
	/* End of parent enum, beginning of extended */
        BM1422_SENSOR_CHAN_PRIV_START = SENSOR_CHAN_PRIV_START,

	/* */
	BM1422_SENSOR_CHAN_INT_SOURCE,
};


enum bm1422_op_mode {
	BM1422_STANDBY = 0,
	BM1422_LOW_POWER = 2,
	BM1422_HI_PERFORMANCE = 3
};

enum bm1422_gsel {
	BM1422_GSEL_8G,
	BM1422_GSEL_16G,
	BM1422_GSEL_32G,
	BM1422_GSEL_64G
};

enum bm1422_otp {
	BM1422_OTP_1HZ563,
	BM1422_OTP_6HZ25,
	BM1422_OTP_12HZ5,
	BM1422_OTP_50HZ
};

enum bm1422_otdt {
	BM1422_OTDT_12HZ5,
	BM1422_OTDT_25HZ,
	BM1422_OTDT_50HZ,
	BM1422_OTDT_100HZ,
	BM1422_OTDT_200HZ,
	BM1422_OTDT_400HZ,
	BM1422_OTDT_800HZ,
	BM1422_OTDT_1600HZ
};

enum bm1422_owuf {
	BM1422_OWUF_0HZ781,
	BM1422_OWUF_1HZ563,
	BM1422_OWUF_3HZ125,
	BM1422_OWUF_6HZ25,
	BM1422_OWUF_12HZ5,
	BM1422_OWUF_25HZ,
	BM1422_OWUF_50HZ,
	BM1422_OWUF_100HZ
};

enum bm1422_obts {
	BM1422_OBTS_0HZ781,
	BM1422_OBTS_1HZ563,
	BM1422_OBTS_3HZ125,
	BM1422_OBTS_6HZ25,
	BM1422_OBTS_12HZ5,
	BM1422_OBTS_25HZ,
	BM1422_OBTS_50HZ,
	BM1422_OBTS_100HZ
};

enum bm1422_osa {
	BM1422_OSA_0HZ781,
	BM1422_OSA_1HZ563,
	BM1422_OSA_3HZ125,
	BM1422_OSA_6HZ25,
	BM1422_OSA_12HZ5,
	BM1422_OSA_25HZ,
	BM1422_OSA_50HZ,
	BM1422_OSA_100HZ,
	BM1422_OSA_200HZ,
	BM1422_OSA_400HZ,
	BM1422_OSA_800HZ,
	BM1422_OSA_1600HZ,
	BM1422_OSA_3200HZ,
	BM1422_OSA_6400HZ,
	BM1422_OSA_12800HZ,
	BM1422_OSA_25600HZ
};

enum bm1422_offi {
	BM1422_OFFI_12HZ5,
	BM1422_OFFI_25HZ,
	BM1422_OFFI_50HZ,
	BM1422_OFFI_100HZ,
	BM1422_OFFI_200HZ,
	BM1422_OFFI_400HZ,
	BM1422_OFFI_800HZ,
	BM1422_OFFI_1600HZ
};

enum bm1422_avc {
	BM1422_NO_AVG,
	BM1422_AVG_2_SAMPLES,
	BM1422_AVG_4_SAMPLES,
	BM1422_AVG_8_SAMPLES,
	BM1422_AVG_16_SAMPLES,
	BM1422_AVG_32_SAMPLES,
	BM1422_AVG_64_SAMPLES,
	BM1422_AVG_128_SAMPLES
};

enum bm1422_rms_avc {
	BM1422_RMS_AVG_2_SAMPLES,
	BM1422_RMS_AVG_4_SAMPLES,
	BM1422_RMS_AVG_8_SAMPLES,
	BM1422_RMS_AVG_16_SAMPLES,
	BM1422_RMS_AVG_32_SAMPLES,
	BM1422_RMS_AVG_64_SAMPLES,
	BM1422_RMS_AVG_128_SAMPLES,
	BM1422_RMS_AVG_256_SAMPLES
};

enum bm1422_oadp {
	BM1422_OADP_0HZ781,
	BM1422_OADP_1HZ563,
	BM1422_OADP_3HZ125,
	BM1422_OADP_6HZ25,
	BM1422_OADP_12HZ5,
	BM1422_OADP_25HZ,
	BM1422_OADP_50HZ,
	BM1422_OADP_100HZ,
	BM1422_OADP_200HZ,
	BM1422_OADP_400HZ,
	BM1422_OADP_800HZ,
	BM1422_OADP_1600HZ,
	BM1422_OADP_3200HZ,
	BM1422_OADP_6400HZ,
	BM1422_OADP_12800HZ,
	BM1422_OADP_25600HZ
};

enum bm1422_buff_mode {
	BM1422_BM_FIFO,
	BM1422_BM_STREAM,
	BM1422_BM_TRIGGER
};

struct bm1422_buff_config {
	enum bm1422_buff_mode buff_mode;
        uint16_t fifo_samples;
};


struct bm1422_data {
	const struct device *bus;
#if defined(CONFIG_BM1422_SPI)
	struct spi_config spi_cfg;
	struct spi_cs_control bm1422_cs_ctrl;
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

#if defined(CONFIG_BM1422_TRIGGER)
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

        uint8_t tap_int;
        uint8_t func_int;
        uint8_t wkup_int;

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
#ifdef CONFIG_BM1422_I2C
	const char *i2c_port;
	uint16_t i2c_addr;
#endif

#ifdef CONFIG_BM1422_SPI
	const char *spi_port;
	uint16_t spi_slave;
	uint32_t spi_max_frequency;
	const char *gpio_cs_port;
	gpio_pin_t cs_gpio;
	gpio_dt_flags_t cs_flags;
#endif /* CONFIG_BM1422_SPI */

#if defined(CONFIG_BM1422_TRIGGER)
	const char *gpio_port;
	gpio_pin_t int_gpio;
	gpio_dt_flags_t int_flags;
#endif

	/* Device Settings */
	struct bm1422_buff_config buff_config;

	enum bm1422_op_mode op;
	enum bm1422_gsel gsel;
	enum bm1422_otp otp;
	enum bm1422_otdt otdt;
	enum bm1422_owuf owuf;
	enum bm1422_obts obts;
	enum bm1422_osa osa;
	enum bm1422_offi offi;
	enum bm1422_avc avc;
	enum bm1422_rms_avc rms_avc;
	enum bm1422_oadp oadp;
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
