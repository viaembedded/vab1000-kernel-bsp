/*
 * arch/arm/mach-elite/include/mach/elite_rtc.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* Be sure that virtual mapping is defined right */

#ifndef __MACH_ELITE_RTC_H
#define __MACH_ELITE_RTC_H

/******************************************************************************
 *
 * Define the register access macros.
 *
 * Note: Current policy in standalone program is using register as a pointer.
 *
 ******************************************************************************/

#define BIT0                0x00000001
#define BIT1                0x00000002
#define BIT2                0x00000004
#define BIT3                0x00000008
#define BIT4                0x00000010
#define BIT5                0x00000020
#define BIT6                0x00000040
#define BIT7                0x00000080
#define BIT8                0x00000100
#define BIT9                0x00000200
#define BIT10               0x00000400
#define BIT11               0x00000800
#define BIT12               0x00001000
#define BIT13               0x00002000
#define BIT14               0x00004000
#define BIT15               0x00008000
#define BIT16               0x00010000
#define BIT17               0x00020000
#define BIT18               0x00040000
#define BIT19               0x00080000
#define BIT20               0x00100000
#define BIT21               0x00200000
#define BIT22               0x00400000
#define BIT23               0x00800000
#define BIT24               0x01000000
#define BIT25               0x02000000
#define BIT26               0x04000000
#define BIT27               0x08000000
#define BIT28               0x10000000
#define BIT29               0x20000000
#define BIT30               0x40000000
#define BIT31               0x80000000

#define REG32_PTR(x) ((volatile u32 *)(x))
#define REG16_PTR(x) ((volatile u16 *)(x))
#define REG8_PTR(x) ((volatile u8 *)(x))

#define REG32_VAL(x) (*(REG32_PTR(x)))
#define REG16_VAL(x) (*(REG16_PTR(x)))
#define REG8_VAL(x) (*(REG8_PTR(x)))


/******************************************************************************
 *
 * ELITE System Configuration Control .
 *
 ******************************************************************************/ 
#define ELITE_SYSTEM_CFG_CTRL_BASE               0xD8120000	/* 64K  */
#define RTC_ENABLE     (*(volatile unsigned int *)IO_ADDRESS(ELITE_SYSTEM_CFG_CTRL_BASE))

/******************************************************************************
 *
 * macros to translate to/from binary and binary-coded decimal
 *
 ******************************************************************************/
#define BCD2BIN(x)    (((x)&0x0f) + ((x) >> 4)*10)
#define BIN2BCD(x)    ((((x)/10) << 4) + (x)%10)

/******************************************************************************
 * ELITE Real Time Clock (RTC) control registers.
 * Registers Abbreviations:
 *
 * RTTS_REG     RTC Time Set Register.
 * RTDS_REG     RTC Date Set Register.
 * RTAS_REG     RTC Alarm Set Register.
 * RTCC_REG     RTC Control Register.
 * RTCT_REG     RTC Current Time Register.
 * RTCD_REG     RTC Current Date Register.
 * RTWS_REG     RTC Write Status Register.
 * RTTM_REG     RTC Test Mode Register.
 * RTTC_REG     RTC Time Calibration Register.
 * RTIS_REG     RTC Interrupt Status Register.
 *
 ******************************************************************************/
/******************************************************************************
 *
 * Address constant for each register.
 *
 ******************************************************************************/
#define	RTTS_ADDR       (0x00)
#define	RTDS_ADDR       (0x04)
#define	RTAS_ADDR       (0x08)
#define	RTCC_ADDR       (0x0C)
#define	RTCT_ADDR       (0x10)
#define	RTCD_ADDR       (0x14)
#define	RTWS_ADDR       (0x18)
#define	RTTM_ADDR       (0x1C)
#define	RTTC_ADDR        (0x20)
#define	RTIS_ADDR        (0x24)
#define	RTSR_ADDR        (0x28)

/******************************************************************************
 *
 * Register pointer.
 *
 ******************************************************************************/
#define	RTTS_REG        (REG32_PTR(RTTS_ADDR))
#define	RTDS_REG        (REG32_PTR(RTDS_ADDR))
#define	RTAS_REG        (REG32_PTR(RTAS_ADDR))
#define	RTCC_REG        (REG32_PTR(RTCC_ADDR))
#define	RTCT_REG        (REG32_PTR(RTCT_ADDR))
#define	RTCD_REG        (REG32_PTR(RTCD_ADDR))
#define	RTWS_REG        (REG32_PTR(RTWS_ADDR))
#define	RTTM_REG        (REG32_PTR(RTTM_ADDR))
#define	RTTC_REG        (REG32_PTR(RTTC_ADDR))
#define	RTIS_REG        (REG32_PTR(RTIS_ADDR))
#define	RTSR_REG        (REG32_PTR(RTSR_ADDR))

/*16'h002c-16'hFFFF Reserved (Read-only, all zeros) */

/******************************************************************************
 *
 * Register value.
 *
 ******************************************************************************/
#define	RTTS_VAL        (REG32_VAL(RTTS_ADDR))
#define	RTDS_VAL        (REG32_VAL(RTDS_ADDR))
#define	RTAS_VAL        (REG32_VAL(RTAS_ADDR))
#define	RTCC_VAL        (REG32_VAL(RTCC_ADDR))
#define	RTCT_VAL        (REG32_VAL(RTCT_ADDR))
#define	RTCD_VAL        (REG32_VAL(RTCD_ADDR))
#define	RTWS_VAL        (REG32_VAL(RTWS_ADDR))
#define	RTTM_VAL        (REG32_VAL(RTTM_ADDR))
#define	RTTC_VAL        (REG32_VAL(RTTC_ADDR))
#define	RTIS_VAL        (REG32_VAL(RTIS_ADDR))
#define	RTSR_VAL        (REG32_VAL(RTSR_ADDR))
/*16'h002c-16'hFFFF Reserved (Read-only, all zeros) */

/******************************************************************************
 *
 * RTTS_REG     RTC Time Set Register bits functions.
 *
 ******************************************************************************/
#define	RTTS_OSEC               (BIT0 | BIT1 | BIT2 | BIT3)         /* One digit */
#define	RTTS_TSEC               (BIT4 | BIT5 | BIT6)                /* Ten digit */
#define	RTTS_OMIN               (BIT7 | BIT8 | BIT9 | BIT10)
#define	RTTS_TMIN               (BIT11 | BIT12 | BIT13)
#define	RTTS_OHOUR              (BIT14 | BIT15 | BIT16 | BIT17)
#define	RTTS_THOUR              (BIT18 | BIT19)
#define	RTTS_WDAY               (BIT20 | BIT21 | BIT22)             /* wday      */
#define	RTTS_TIME               0x7FFFFF                            /* Bits 0-22 */
/* Bits 23-31: Reserved */

/* BIN2BCD macros
 * in  : sec, min, hour, wday (in binary)
 * out : RTTS_VAL
 */
#define	RTTS_SEC(x)             ((BIN2BCD(x) << 0) & (RTTS_OSEC | RTTS_TSEC))
#define	RTTS_MIN(x)             ((BIN2BCD(x) << 7) & (RTTS_OMIN | RTTS_TMIN))
#define	RTTS_HOUR(x)            ((BIN2BCD(x) << 14) & (RTTS_OHOUR | RTTS_THOUR))
#define	RTTS_DAY(x)             ((BIN2BCD(x) << 20) &  RTTS_WDAY)

/******************************************************************************
 *
 * RTDS_REG     RTC Date Set Register bits functions.
 *
 ******************************************************************************/
#define	RTDS_ODAY               (BIT0 | BIT1 | BIT2 | BIT3)       /* One digit */
#define	RTDS_TDAY               (BIT4 | BIT5)                     /* Ten digit */
#define	RTDS_OMON               (BIT6 | BIT7 | BIT8 | BIT9)
#define	RTDS_TMON               BIT10
#define	RTDS_OYEAR              (BIT11 | BIT12 | BIT13 | BIT14)
#define	RTDS_TYEAR              (BIT15 | BIT16 | BIT17 | BIT18)
#define RTDS_CEN                BIT19
#define	RTDS_DATE               0x000FFFFF                        /* Bits 0-19 */
/* Bits 20-31: Reserved */

/* BIN2BCD macros
 * in  : mday, mon, year, century (in binary)
 * out : RTDS_VAL
 */
#define	RTDS_MDAY(x)            ((BIN2BCD(x) << 0) & (RTDS_ODAY | RTDS_TDAY))
#define	RTDS_MON(x)             ((BIN2BCD(x) << 6) & (RTDS_OMON | RTDS_TMON))
#define	RTDS_YEAR(x)            ((BIN2BCD(x) << 11) & (RTDS_OYEAR | RTDS_TYEAR))
#define	RTDS_CENT(x)            ((BIN2BCD(x) << 19) & RTDS_CEN)

/******************************************************************************
 *
 * RTAS_REG     RTC Alarm Set Register bits functions.
 *
 ******************************************************************************/
#define	RTAS_OSEC               (BIT0 | BIT1 | BIT2 | BIT3)         /* One digit */
#define	RTAS_TSEC               (BIT4 | BIT5 | BIT6)                /* Ten digit */
#define	RTAS_OMIN               (BIT7 | BIT8 | BIT9 | BIT10)
#define	RTAS_TMIN               (BIT11 | BIT12 | BIT13)
#define	RTAS_OHOUR              (BIT14 | BIT15 | BIT16 | BIT17)
#define	RTAS_THOUR              (BIT18 | BIT19)
#define	RTAS_ODAY               (BIT20 | BIT21 | BIT22 | BIT23)     /* mday      */
#define	RTAS_TDAY               (BIT24 | BIT25)
#define	RTAS_ALMASK             0x03FFFFFF                          /* Bits 0-25 */
#define RTAS_CMPSEC             BIT26
#define RTAS_CMPMIN             BIT27
#define RTAS_CMPHOUR            BIT28
#define RTAS_CMPDAY             BIT29
#define RTAS_CMPMASK            (BIT26 | BIT27 | BIT28 | BIT29)
/* Bits 30-31: Reserved */

/* BIN2BCD macros
 * in  : sec, min, hour, mday (in binary)
 * out : RTAS_VAL
 */
#define	RTAS_SEC(x)             ((BIN2BCD(x) << 0) & (RTAS_OSEC | RTAS_TSEC))
#define	RTAS_MIN(x)             ((BIN2BCD(x) << 7) & (RTAS_OMIN | RTAS_TMIN))
#define	RTAS_HOUR(x)            ((BIN2BCD(x) << 14) & (RTAS_OHOUR | RTAS_THOUR))
#define	RTAS_DAY(x)             ((BIN2BCD(x) << 20) & (RTAS_ODAY | RTAS_TDAY))

/******************************************************************************
 *
 * RTCC_REG     RTC Control Register bit function.
 *
 ******************************************************************************/
#define	RTCC_ENA                BIT0    /* Real Time Clock Enable           */
#define	RTCC_12HR               BIT1    /* Time Format 1:12-hour 0:24-hour  */
#define	RTCC_INTENA             BIT2    /* Sec/Min Interrupt Request Enable */
#define	RTCC_INTTYPE            BIT3    /* Sec/Min Type Select
										 * 0:Generate interrupt every minute.
										 * 1:Generate interrupt every sec.  */
#define	RTCC_CALIBRATION        BIT4    /* Calibration Enable               */
#define	RTCC_CTRLMASK           0x1F    /* Bits 0-4                         */
/* Bits 5-31: Reserved */

/******************************************************************************
 *
 * RTCT_REG     RTC Current Time Register bits definitions.
 *
 ******************************************************************************/
#define	RTCT_OSEC               (BIT0 | BIT1 | BIT2 | BIT3)       /* One digit */
#define	RTCT_TSEC               (BIT4 | BIT5 | BIT6)              /* Ten digit */
#define	RTCT_OMIN               (BIT7 | BIT8 | BIT9 | BIT10)
#define	RTCT_TMIN               (BIT11 | BIT12 | BIT13)
#define	RTCT_OHOUR              (BIT14 | BIT15 | BIT16 | BIT17)
#define	RTCT_THOUR              (BIT18 | BIT19)
#define	RTCT_WDAY               (BIT20 | BIT21 | BIT22)           /* wday      */
/* Bits 23-30: Reserved */
#define	RTCT_INVALID            BIT31                             /* 1:invalid */

/* BCD2BIN macros
 * in  : RTCT_VAL
 * out : sec, min, hour, wday (in binary)
 */
#define	RTCT_SEC(x)             BCD2BIN(((x) & (RTCT_OSEC | RTCT_TSEC)) >> 0)
#define	RTCT_MIN(x)             BCD2BIN(((x) & (RTCT_OMIN | RTCT_TMIN)) >> 7)
#define	RTCT_HOUR(x)            BCD2BIN(((x) & (RTCT_OHOUR | RTCT_THOUR)) >> 14)
#define	RTCT_DAY(x)             BCD2BIN(((x) & (RTCT_WDAY))             >> 20)

/******************************************************************************
 *
 * RTCD_REG     RTC Current Date Register bits definitions.
 *
 ******************************************************************************/
#define	RTCD_ODAY               (BIT0 | BIT1 | BIT2 | BIT3)      /* One digit */
#define	RTCD_TDAY               (BIT4 | BIT5)                    /* Ten digit */
#define	RTCD_OMON               (BIT6 | BIT7 | BIT8 | BIT9)
#define	RTCD_TMON               BIT10
#define	RTCD_OYEAR              (BIT11 | BIT12 | BIT13 | BIT14)
#define	RTCD_TYEAR              (BIT15 | BIT16 | BIT17 | BIT18)
#define	RTCD_CEN                BIT19
/* Bits 20-30: Reserved */
#define	RTCD_INVALID            BIT31                            /* 1:invalid */

/* BCD2BIN macros
 * in  : RTCD_VAL
 * out : mday, mon, year, century (in binary)
 */
#define	RTCD_MDAY(x)            BCD2BIN(((x) & (RTCD_ODAY | RTCD_TDAY)) >> 0)
#define	RTCD_MON(x)             BCD2BIN(((x) & (RTCD_OMON | RTCD_TMON)) >> 6)
#define	RTCD_YEAR(x)            BCD2BIN(((x) & (RTCD_OYEAR | RTCD_TYEAR)) >> 11)
#define	RTCD_CENT(x)            BCD2BIN(((x) & (RTCD_CEN))              >> 19)

/******************************************************************************
 *
 * RTWS_REG     RTC Write Status Register bits definitions.
 *
 ******************************************************************************/
#define	RTWS_TIMESET            BIT0	/* RTC Time Set Register Busy         */
#define	RTWS_DATESET            BIT1	/* RTC Date Set Register Busy         */
#define	RTWS_ALARMSET           BIT2	/* RTC Alarm Set Register Busy        */
#define	RTWS_CONTROL            BIT3	/* RTC Control Register Busy          */
#define	RTWS_TESTMODE           BIT4	/* RTC Test Mode Register Busy        */
#define	RTWS_CALIBRATION        BIT5	/* RTC Time Calibration Register Busy */
/* Bits 6-30: Reserved */

/******************************************************************************
 *
 * RTTM_REG     RTC Test Mode Register bit definition.
 *
 ******************************************************************************/
#define	RTTM_ENABLE             BIT0	/* RTC Test Mode Enable */
#define	RTTM_TESTMASK           0x01	/* Bits 0 only now      */
/* Bits 1-30: Reserved */

/******************************************************************************
 *
 * RTTC_REG     RTC Time Calibration Register bits definitions.
 *
 ******************************************************************************/

/* Calibration value Bit0-14 If all one(0x7FFF), means add or sub one second. */
#define	RTTC_VALUEMASK          0x7FFF

#define	RTTC_TYPE               BIT15
#define	RTTC_CALIMASK           0xFFFF	/* Bits 0-15 */
/* Bits 16-30: Reserved */

/******************************************************************************
 *
 * RTIS_REG	RTC Interrupt Status bits definition.
 *
 ******************************************************************************/
#define RTIS_ALARM              BIT0	/* RTC Alarm interrupt          */
#define RTIS_UPDATE             BIT1	/* RTC sec/min update interrupt */
/* Bits 2-31: Reserved */

/******************************************************************************
 *
 * RTSR_REG	RTC RTC Status Register bits definition.
 *
 ******************************************************************************/
#define RTSR_VAILD              BIT0	/* RTC Vaild Time status     */
/* Bits 1-31: Reserved */

#endif
