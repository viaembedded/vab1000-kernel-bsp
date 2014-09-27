/****************************************************************************
*
*           Copyright @ 2012 by S3 Graphics.
*           All Rights Reserved.
*
*****************************************************************************
*
*   File        : elite_sci.h
*
*   Version     : 1.0
*
*   Description : SC implementation.
*
*   Notes       :
*
*****************************************************************************
*                      C H A N G E   R E C O R D
*
*   Date                ID                      Description
* -------- -------- ------------------------------------------------------
* 2012/08/22       bret.zhang           Created this file.
*****************************************************************************/

#ifndef _ELITE_SMARTCARD_H
/* To assert that only one occurrence is included */
#define _ELITE_SMARTCARD_H

#include <linux/types.h>

#ifdef __KERNEL__

#include <linux/list.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#endif
#include <linux/ioctl.h>
#include <linux/synclink.h>

/* Include your headers here*/

#define TRUE     1
#define FALSE    0
#define SCTMA   0x00000000
#define SCRMA   0x00000004
#define SCS       0x00000008
#define SCTMS   0x0000000A
#define SCRMS   0x0000000C
#define SCCMD   0x0000000E
#define SCCWI     0x00000010
#define SCETU     0x00000012
#define SCGT      0x00000014
#define SCCD      0X00000016
#define SCRA      0x00000018
#define SCIE      0x0000001A
#define SCFC      0x0000001C
#define SCES      0x0000001E
#define SCPC      0x00000020
#define SCBC      0x00000022
#define SCNC      0x00000024
#define SCIM      0x00000026
#define SCIS      0x00000028
#define SCD_T14   0x0000002A

#define SCCD_VALUE_MASK        0x1FF
#define SCES_VALUE_MASK        0x7
#define SCBC_VALUE_MASK        0x1FF
#define SCPC_VALUE_MASK        0x1FF
#define SCNC_VALUE_MASK        0x7F
#define SCRMS_VALUE_MASK       0xFF
#define SCIM_VALUE_MASK        0x1F
#define SCIS_VALUE_MASK        0x1F

#define SC_SCCMND_CRC   BIT6
#define SC_SCCMND_PT     BIT4
#define SC_SCCMND_PTS    BIT1
#define SC_SCES_FG           BIT0

#define SC_SCCMND_SAR   BIT0
#define SC_SCCD_EXT     BIT9
#define SC_SCCD_C       BIT8
#define SC_SCCMND_IE    BIT5
#define SC_SCCMND_VCC   BIT9 
#define SC_SCCMND_COE   BIT11
#define SC_SCCMND_OFF   BIT10
#define SC_SCCMND_RL    BIT13
#define SC_SCCMND_RST   BIT12
#define SC_SCCMND_SMR   BIT15
#define SC_SCCMND_IRS   BIT8
#define SC_SCCMND_RD    BIT14
#define SC_SCCMND_TE    BIT3
#define SC_SCCMND_RE    BIT2
#define SC_SCCWI_CWI   (BIT0|BIT1|BIT2|BIT3)
#define SC_SCETU_CP     BIT14
#define SC_SCETU_C      BIT13
#define SC_SCETU_EP     BIT12
#define  SC_SCETU_ETU  (BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7|BIT8|BIT9|BIT10|BIT11)
#define  SC_SCES_IA            BIT2
#define  SC_INT_ISR_CWT        BIT0
#define  SC_INT_ISR_OFF        BIT3
#define  SC_INT_ISR_NOREPLY    BIT2
#define  SC_INT_ISR_PWROFF     BIT5
#define  SC_INT_ISR_MESS       BIT1

#define  SC_INT_ISR_PARITY     BIT4

#define SC_BUFFER_SIZE          0x140

#define ELITE_SCI_MAJOR        0X10


/* freq = RSACLK in Khz, Half = bit C of SCCD_x, delay = TDA800x delay in us */
#define COMPUTE_SCRA(freq, half, delay )   (((freq) * (delay) / 1000) / ( half + 1 ))

/* I'm multiplying by 1000 because freq is in kHz*/
#define COMPUTE_US_PER_CLKCYCLES(clkcycles_max, clkcycles, freq, half)  ( ((clkcycles_max - clkcycles)*10)/((freq/100) /(half+1)) )
#define COMPUTE_US_PER_ETUS(etus_max, etus, clk_per_bit, freq, half)   ( ((etus_max - etus) * (clk_per_bit)*10)/((freq/100) /(half+1)) )

/* 14.896+40ms>51ms */
//#define ATR_NR_INTERVAL          100000 
#define ATR_NR_INTERVAL       240000 

/* Checking every 2000 clock cycles */
#define ATR_NR_SAMPLE            40000 

/* 14.896+40ms>51ms */
//#define ATR_NR_INTERVAL       240000 
/* Checking every 40000 clock cycles */
//#define ATR_NR_SAMPLE         40000 

/* max clockcycles to consider a NR event occured 42,001[5% tolerance] + 10*372! */
#define ATR_NR_MAX              44500 
/* etus for min ICD */
#define ATR_ICD_MIN             12 
/* 10080 is too high 9600 + 5% etus for max ICD */
#define ATR_ICD_MAX             9900 
/* min number of etus between two opposite characters sent in opposite direction */
#define T1_BGT_TIME             22 
/* all the characters shall be received during this time */
#define ATR_TIME                19200 
/* as defined in test */
#define ATR_TIME_MAX            4000 

#define TIMER                   0
#define RST_ACT_CNT_TDA8004     135
#define IN_EN_CNT_TDA8004       300

/*CL_SC_SCCD_CLK_DIV_TDA8004 is set in the SC Clock Devide Register. The value is due to the 
*following fact: the hardware input circuit to the TDA is dividing the actual clock output from the
*AViA (hw->freq_rsaclk) by 2. This is the frequency that then goes to the smart card's clock.
*So what I'm doing is:
*- tell the HW not to devide the value coming from sel_clk[=> 0x001f, for more info see ERM part 1]
*- tell to the Smart Card Controller State Machine to run at half of the frquency of the RSACLK: this is
*   due to the fact (as said above) that the input circuit to the TDA devides RSACLK by 2. [GB]
*/
/* No Divide of sel_clk needed */
#define CL_SC_SCCD_CLK_DIV_TDA8004      ( SC_SCCD_C | 0x001f )  
#define CLK_DIV_TDA8004                 ((s->chip_type==SC_IO_TDA8004))

 /* if set to 0, allows printing of debug msgs from ISR */
#define ISR_DEBUG_MSG          1            

/* Values for chip type in CRR register */
#define B0_SC                  0
#define A6_SC                  1
#define CRR_A6_VALUE_SC        0x00000440
#define CRR_B0_VALUE_SC        0x00000460

/*chip ID is in CRR[31:16]*/
#define CHIP_TYPE_SHIFT_SC     16 

/*clk frequency used to set RSACLK*/
#define INTERNAL_CLK_FREQ_KHZ  216000 
#define CLK_FREQ_MHZ           2.5
#define INITIAL_F_VALUE        372

#define SC_INVERSE_CONVENTION  0x3F
#define SC_DIRECT_CONVENTION   0x3B
#define SC_PROTOCOL_TYPE_0     0x0
#define SC_PROTOCOL_TYPE_1     0x1
#define SC_PROTOCOL_TYPE_14    0xE

#define SMART_CARD_IOCTL_BASE         'C'

#define SMART_CARD_GETBASECONFIG      _IOR(SMART_CARD_IOCTL_BASE, 0,  t_sc9600_config)
#define SMART_CARD_GETHWCONFIG        _IOR(SMART_CARD_IOCTL_BASE, 1,  t_sc9600_config)
#define SMART_CARD_GETATRCONFIG       _IOR(SMART_CARD_IOCTL_BASE, 2,  t_sc9600_config)
#define SMART_CARD_CARDDETECT         _IOR(SMART_CARD_IOCTL_BASE, 4,  int)
#define SMART_CARD_GET_STATUS         _IOR(SMART_CARD_IOCTL_BASE, 5,  int)
#define SMART_CARD_GET_SCSSTATUS      _IOR(SMART_CARD_IOCTL_BASE, 6,  unsigned short)
#define SMART_CARD_GET_SCBGT          _IOR(SMART_CARD_IOCTL_BASE, 7,  unsigned char)
#define SMART_CARD_GET_SCPE           _IOR(SMART_CARD_IOCTL_BASE, 8,  unsigned char)
#define SMART_CARD_GET_SCIM           _IOR(SMART_CARD_IOCTL_BASE, 9,  unsigned short)
#define SMART_CARD_GET_RXSIZE         _IOR(SMART_CARD_IOCTL_BASE, 10, unsigned short)
#define SMART_CARD_GET_SCRMS          _IOR(SMART_CARD_IOCTL_BASE, 11, unsigned short)
#define SMART_CARD_GET_ATRINFO        _IOR(SMART_CARD_IOCTL_BASE, 12, sc_atr_info_type)
#define SMART_CARD_GET_SCIS           _IOR(SMART_CARD_IOCTL_BASE, 13, unsigned short)

#define SMART_CARD_SETCONFIG          _IOW(SMART_CARD_IOCTL_BASE, 14,  t_sc9600_config)
#define SMART_CARD_DEACT              _IO(SMART_CARD_IOCTL_BASE, 15)
#define SMART_CARD_RESET              _IOW(SMART_CARD_IOCTL_BASE, 16,  int)
#define SMART_CARD_SET_STATUS         _IOW(SMART_CARD_IOCTL_BASE, 17,  unsigned char)
#define SMART_CARD_SET_SCRMS          _IOW(SMART_CARD_IOCTL_BASE, 18,  unsigned short)
#define SMART_CARD_SET_ATR            _IOW(SMART_CARD_IOCTL_BASE, 19,  unsigned char[SC_BUFFER_SIZE])
#define SMART_CARD_CHECK_ATR          _IOW(SMART_CARD_IOCTL_BASE, 20,  int)
#define SMART_CARD_SET_ATRINFO        _IOW(SMART_CARD_IOCTL_BASE, 21,  sc_atr_info_type)
#define SMART_CARD_DETECT_ENABLE      _IO(SMART_CARD_IOCTL_BASE, 22)
#define SMART_CARD_GET_EVENT           _IOR(SMART_CARD_IOCTL_BASE, 23, struct sc_rx_message)


/* smart card state */
typedef enum {
	SC_STATE_NULL,
	SC_STATE_INIT,
	SC_STATE_ATR,
	SC_STATE_IDLE,
	SC_STATE_DEACTIVATED,
	SC_STATE_NOCARD,
	SC_STATE_IO,
	SC_STATE_RESET,
	SC_STATE_MAX
} sc_state_types;

/* the standard the smart card flow */
typedef enum {
	SC_STANDARD_ISO7816,
	SC_STANDARD_EMV96,
	SC_STANDARD_EMV2000,
	SC_STANDARD_MAX,
} sc_standard_type;

/* smart card reset type */
typedef enum {
	SC_COLD_RESET,
	SC_WARM_RESET,
	SC_PIPELINE_RESET,
} sc_reset_level_type;


/* smart card event type */
typedef enum {
	SC_EVENT_NULL,
	SC_EVENT_MC,
	SC_EVENT_NR,
	SC_EVENT_PO,
	SC_EVENT_CD,
	SC_EVENT_PE,
	SC_EVENT_ICD,
	SC_EVENT_WRONG_TS,
	SC_EVENT_LONG_ATR,
	SC_EVENT_WWT,
	SC_EVENT_CWT,
	SC_EVENT_BGT,
	SC_EVENT_BWT,
	SC_EVENT_IO_REQ,
	SC_EVENT_RESET_REQ,
	SC_EVENT_TASK_STOP,
} sc_event_types;
	
/*smart card's IO type*/
/*smart card's IO type*/
typedef enum {
	SC_IO_MODE_T0=SC_PROTOCOL_TYPE_0,
	SC_IO_MODE_T1=SC_PROTOCOL_TYPE_1,
	SC_IO_MODE_T14=SC_PROTOCOL_TYPE_14,
	SC_IO_MODE_MAX,
} sc_io_mode_type;

typedef enum {
        SC_IO_TDA8001,
        SC_IO_TDA8002,
        SC_IO_TDA8004,
        SC_IO_TDA8020,
        SC_IO_TDA8034,
   }sc_io_chip_type;


/* smart card hardware config */
typedef struct {
	unsigned int standard;           /* SC_STANDARD_ISO7816 / SC_STANDARD_EMV96 / SC_STANDARD_EMV2000  */
	unsigned int inverse_reset;      /* 0 = No effect on RSARESET; or 1 = Invert the RSARESET signal */
	unsigned int two_crc;            /* 1 = it transmits two CRC check bytes in the epilogue field; 0 = only one byte */
	unsigned int protocol_type;      /* T=0, T=1,T=14*/
	unsigned int use_pts;            /* 1 = use PTS/PPS if avaliable, 0 = not use PTS/PPS        */
	unsigned int cwi;                /* CWI. Max time between characters from card: CWT = (2^CWI + 11) work ETU  */
	unsigned int auto_convention;    /* 1 = auto convention;  0 = Prevent the autodetection circuit from overwriting the convention value. */
	unsigned int inverse_convention; /* 0 = Use the direct convention; 1 = Use the inverse convention */
	unsigned int auto_etu;           /* 1 = auto ETU;  0 = Prevent the automatic baud rate determining circuit from overwriting the ETU value. */
	unsigned int etu;                /* ETU */
	unsigned int guard_time;         /* Guardtime */  
	unsigned int reset_time;         /* Time delay (in us) between the activation of RSACMDVCC and the activation of RSARESET. */
	unsigned int atr_response_time;  /* the time delay (how many clock cycle) between the activation of RSARESET and the enabling of smart card input data. range: 200 ~ 400 clock cycle       */
	unsigned int force_guard_time;   /* Force GuardTime. This allows the support of 0 ETU extra guard time of characters sent in
	                                    the same direction, while providing 15 ETU extra guard time during a change in the direction of the data transfer. */    
	unsigned int clock;              /* clock rate, now only 2.5MHz  */
	unsigned int valid_protocols;    /* Bitfield of valid protocols from ATR       */
	unsigned int bwi;                /* BWI  */
	unsigned int wi;                 /* WI  */
	unsigned int pts0;               /* PTS0 */
	unsigned int pts1;               /* PTS1 */
	unsigned int pts2;               /* PTS2 */
	unsigned int pts3;               /* PTS3 */
	unsigned int n_value;            /* N */
	unsigned int p_value;            /* P */
	unsigned int b_value;            /* B */
} t_sc9600_config;


/* smart card's response message content,ICC->TTL */
struct  sc_rx_message{
	sc_event_types     event;
	unsigned short     size;
	unsigned char      data[SC_BUFFER_SIZE];
} ;

/* smart card's response message content,TTL->ICC */
struct  sc_tx_message{
	unsigned char    buffer[SC_BUFFER_SIZE];
	unsigned short   size;
	unsigned short   reply_size;
} ;

/* smart card's ATR info */
typedef struct {
    unsigned char      buffer[SC_BUFFER_SIZE]; //[256]
    unsigned char      length;
    unsigned char      ts;
    unsigned char      t0;
    unsigned char      fi;
    unsigned char      di;
    unsigned short     etu;
    unsigned char      specific_mode;
    unsigned char      mode;
    unsigned char      mode_changable;
    unsigned char      mode_parameters;
    unsigned char      mode_t;
    unsigned char      ifsc;
    unsigned char      tb1;
    unsigned char      ii;
    unsigned char      pi1;
    unsigned char      tb2;
    unsigned char      pi2;
    unsigned char      tb3;
    unsigned char      cwi;
    unsigned char      bwi;
    unsigned char      n;
    unsigned char      wi;
    unsigned char      rc;
    unsigned char      td2;
    unsigned char      td2_isnibble;
    unsigned char      ta4;
    unsigned char      tb4;
    unsigned char      tc4;
    unsigned char      history_size;
    unsigned char *    history_ptr;
    sc_reset_level_type   level;
 } sc_atr_info_type;

#ifdef __KERNEL__

/* smart card's response message,ICC->TTL */
struct sc_message {
	struct  sc_rx_message io_resp;
	struct list_head          queue;
};

/* smart card device description */
struct elite_sci {
	unsigned char *    mmio;
	int                irq;
	dma_addr_t         dma_handle;
	void        *      dma_buffer;
	int                dma_size;
	unsigned char*     tx_buffer;
	unsigned char*     rx_buffer;
	unsigned short     tx_size;
	unsigned short     rx_size;
	dma_addr_t         tx_buffer_phy;
	dma_addr_t         rx_buffer_phy;
	unsigned char      state;
	unsigned short     protocol;         /* T=0, T=1, etc.*/
	sc_io_chip_type    chip_type;
	/* Timing parameters: */
	unsigned short     t_sample;        /*period for sampling the RD bit.*/
	unsigned char      nr_trigg;
	unsigned char      rst_val;
	unsigned short     freq_rsaclk;
	unsigned int       ms_per_tick;     /* Number of ms in one tick.*/
	unsigned int       block_length;    /* Lenght of a block sent by the card in T=1 protocol (in tick) */
	unsigned char      got_byte;
	unsigned short     atr_long_count;  /* ATR longer than 19,200 etus*/
	unsigned int       atr_long_max;
	unsigned short     atr_nr_count;    /* No reply from card within 40,000 clkcycles*/
	unsigned int       atr_nr_max;
	unsigned short     atr_icd_count;
	unsigned int       atr_icd_max;
	unsigned short     atr_time;
	unsigned char      bgt;
	unsigned short     bwt_count;       /*Block Wait Time*/
	unsigned int       bwt_max;
	unsigned short     cwt_count;       /*Character Waiting Time*/
	unsigned int       cwt_max;
	unsigned short     wwt_count;       /*Work Waiting Time*/
	unsigned int       wwt_max;
	unsigned int       wwt;
	unsigned char      parity_error;
	sc_atr_info_type   atr_info;
	struct list_head   messages;
    int                need_timer;
	struct fasync_struct * cmd_async;
	struct semaphore       sem ;
	spinlock_t             sc_lock;
	struct timer_list      timer;
	struct cdev            cdev ;
	struct list_head   event_messages;
};



/*read the register of smart card*/
static unsigned short inline sc_readw(struct elite_sci *s, int offset)
{
    return readw(s->mmio + offset);
}

/*write the register of smart card*/
static void inline sc_writew(struct elite_sci *s, unsigned short val, int offset)
{
	writew(val, s->mmio + offset);
}

/*read the register of smart card*/
static unsigned int inline sc_readl(struct elite_sci *s, int offset)
{
	return readl(s->mmio + offset);
}

/*write the register of smart card*/
static void inline sc_writel(struct elite_sci *s, unsigned int val, int offset)
{
	writel(val, s->mmio + offset);
}
#endif 

#endif //_ELITE_SMARTCARD_H

