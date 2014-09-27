#ifndef _ELITE_I2S
#define _ELITE_I2S

//#include <mach/generics.h>

#define I2S_BASE_ADDR                   IO_ADDRESS(0xD80ED800)
/******************************************************************************
 *
 * Address constant for each register.
 *
 ******************************************************************************/
#define	DACCFG_ADDR                     (I2S_BASE_ADDR + 0x0040)
#define ADCCFG_ADDR                     (I2S_BASE_ADDR + 0x0080)
#define AADCFEN_ADDR                    (I2S_BASE_ADDR + 0x008C)
#define AADCFSTATE_ADDR                 (I2S_BASE_ADDR + 0x0090)




#define DGOCFG_ADDR                     (I2S_BASE_ADDR + 0x00C0)
#define ADGICFG_ADDR                    (I2S_BASE_ADDR + 0x00D0)
#define ASMPFCFG_ADDR                   (I2S_BASE_ADDR + 0x0180)
#define ASMPFRDY_ADDR                   (I2S_BASE_ADDR + 0x0184)
#define ASMPFBS_TYPE_ADDR               (I2S_BASE_ADDR + 0x0188)
#define ASMPFCHCFG_ADDR                 (I2S_BASE_ADDR + 0x0194)
#define AUDPRFRST_ADDR                  (I2S_BASE_ADDR + 0x0244)
#define AADCFOBDOUT_DMA_ADDR            (I2S_BASE_ADDR + 0x0300)
#define ASMPFDP_DMA_ADDR                (I2S_BASE_ADDR + 0x0360)
#define DZDRQ8_CFG_ADDR                 (I2S_BASE_ADDR + 0x03A0)
#define DZDRQ9_CFG_ADDR                 (I2S_BASE_ADDR + 0x03A4)
#define DZDRQA_CFG_ADDR                 (I2S_BASE_ADDR + 0x03A8)

#define DGOCS0A_ADDR                    (I2S_BASE_ADDR + 0x0114)
#define DGOCS1A_ADDR                    (I2S_BASE_ADDR + 0x012C)

#define DGOCSRDY_ADDR			(I2S_BASE_ADDR + 0x00e0)

/******************************************************************************
 *
 * Register pointer.
 *
 ******************************************************************************/
#define	DACCFG_REG                      (REG32_PTR(DACCFG_ADDR))
#define	ADCCFG_REG                      (REG32_PTR(ADCCFG_ADDR))
#define	AADCFEN_REG                     (REG32_PTR(AADCFEN_ADDR))
#define	AADCFSTATE_REG        	        (REG32_PTR(AADCFSTATE_ADDR))
#define	DGOCFG_REG        		(REG32_PTR(DGOCFG_ADDR))
#define	ADGICFG_REG        		(REG32_PTR(ADGICFG_ADDR))
#define	ASMPFCFG_REG        	        (REG32_PTR(ASMPFCFG_ADDR))
#define	ASMPFRDY_REG        	        (REG32_PTR(ASMPFRDY_ADDR))
#define	ASMPFBS_TYPE_REG                (REG32_PTR(ASMPFBS_TYPE_ADDR))
#define	ASMPFCHCFG_REG        	        (REG32_PTR(ASMPFCHCFG_ADDR))
#define	AUDPRFRST_REG        	        (REG32_PTR(AUDPRFRST_ADDR))
#define	AADCFOBDOUT_DMA_REG		(REG32_PTR(AADCFOBDOUT_DMA_ADDR))
#define	ASMPFDP_DMA_REG			(REG32_PTR(ASMPFDP_DMA_ADDR))
#define	DZDRQ8_CFG_REG			(REG32_PTR(DZDRQ8_CFG_ADDR))
#define	DZDRQ9_CFG_REG			(REG32_PTR(DZDRQ9_CFG_ADDR))
#define	DZDRQA_CFG_REG			(REG32_PTR(DZDRQA_CFG_ADDR))


/******************************************************************************
 *
 * Register value.
 *
 ******************************************************************************/
#define	DACCFG_VAL        		(REG32_VAL(DACCFG_ADDR))
#define	ADCCFG_VAL        		(REG32_VAL(ADCCFG_ADDR))
#define	AADCFEN_VAL        		(REG32_VAL(AADCFEN_ADDR))
#define	AADCFSTATE_VAL     		(REG32_VAL(AADCFSTATE_ADDR))
#define	DGOCFG_VAL     		        (REG32_VAL(DGOCFG_ADDR))
#define	ASMPFCFG_VAL        	        (REG32_VAL(ASMPFCFG_ADDR))
#define	ASMPFRDY_VAL        	        (REG32_VAL(ASMPFRDY_ADDR))
#define	ASMPFBS_TYPE_VAL                (REG32_VAL(ASMPFBS_TYPE_ADDR))
#define	ASMPFCHCFG_VAL        	        (REG32_VAL(ASMPFCHCFG_ADDR))
#define	AUDPRFRST_VAL        	        (REG32_VAL(AUDPRFRST_ADDR))
#define	AADCFOBDOUT_DMA_VAL		(REG32_VAL(AADCFOBDOUT_DMA_ADDR))
#define	ASMPFDP_DMA_VAL			(REG32_VAL(ASMPFDP_DMA_ADDR))
#define	DZDRQ8_CFG_VAL			(REG32_VAL(DZDRQ8_CFG_ADDR))
#define	DZDRQ9_CFG_VAL			(REG32_VAL(DZDRQ9_CFG_ADDR))
#define	DZDRQA_CFG_VAL			(REG32_VAL(DZDRQA_CFG_ADDR))


/******************************************************************************
 *
 * 
 *
 ******************************************************************************/
#define DACITF_ENABLE                   BIT22			/* DAC interface enable */
#define ADCITF_ENABLE                   BIT23			/* ADC interface enable */

#define ADCITF_MASTER                   BIT22                   /* ADC Master/Slave mode selection */
#define ASMPF_8BIT_SMP                  0x00			/* sample quantization config for 8 bit */
#define ASMPF_16BIT_SMP                 0x10			/* sample quantization config for 16 bit */
#define ASMPF_32BIT_SMP                 0x20			/* sample quantization config for 32 bit */
#define ASMPF_ENABLE                    BIT6			/* sample FIFO enable */
#define ASMPF_EXCH_FMT                  BIT7			/* sample FIFO exchange unsigned/signed format enable */
#define ASMPF_EXCH_ENDIAN               BIT8			/* sample FIFO exchange little/big endian enable */

#define AADCF_ENABLE                    BIT0			/* ADC FIFO enable */
#define AADCF16_ENABLE                  BIT1			/* ADC FIFO 16-bits enable */

#define DGOITF_ENABLE                   BIT7			/* ADGO(SPDIF-out) interface enable */
#define ADGIF16_ENABLE                  BIT14			/* ADGI FIFO 16-bits enable */
#define ADGIITF_ENABLE                  BIT1			/* ADGI(SPDIF-in) interface enable */
#define ADGI_EXTRACTOR_ENABLE           BIT0			/* ADGI-Extractor enable */

#define ASMPF_RESET                     BIT1			/* sample FIFO reset */
#define DACITF_RESET                    BIT2			/* DAC interface reset */
#define ADCITF_RESET                    BIT3			/* ADC interface & ADC FIFO reset */
#define DGOITF_RESET                    BIT4			/* SPDIF out reset */


#define I2S_TX_FIFO 0xD80EDB60
#define I2S_RX_FIFO 0xD80EDB00

#define AUD_SPDIF_ENABLE

struct elite_i2s_regs {
    unsigned int DGOCFG;
    unsigned int DGOCS0A;
    unsigned int DGOCS1A;
    unsigned int ADGICFG;
    unsigned int ASMPFCFG;
    unsigned int AADCFEN;
    unsigned int DACCFG;
    unsigned int ADCCFG;
} ;

struct elite_i2s_dev {
    struct device *dev;
    struct clk *clk;
    int    clk_count;
    struct elite_i2s_regs regs;
    struct elite_pcm_dma_params playback_dma_params;
    struct elite_pcm_dma_params capture_dma_params;
};

#endif
