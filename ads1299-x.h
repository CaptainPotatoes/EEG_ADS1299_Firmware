/* Copyright (c) 2017 Musa Mahmood
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
 
/** @file
 *
 * @brief Functions for initializing and controlling Texas Instruments ADS1291/2 analog front-end.
 */
 
#ifndef ADS1291_2_H__
#define ADS1291_2_H__
 
#include <stdint.h>
#include "nrf_drv_spi.h"

#ifdef __cplusplus
	extern "C" {
#endif

#ifdef BOARD_PCA10028
	#define ADS1291_2_PWDN_PIN		 					4		 	 /**< ADS1291 power-down/reset pin - A3 on Arduino */
	#define ADS1291_2_DRDY_PIN							15		 /**< ADS1291 data ready interrupt pin - D3 on Arduino */	
#elif defined(BOARD_CUSTOM)
	#define ADS1299_RESET_PIN							12
	#define ADS1299_PWDN_PIN							13		 /**< ADS1291 power-down/reset pin */
	#define ADS1299_DRDY_PIN							 8		 /**< ADS1291 data ready interrupt pin */
		
	#define ADS1291_2_PWDN_PIN							16		 /**< ADS1291 power-down/reset pin */
	#define ADS1291_2_DRDY_PIN							11		 /**< ADS1291 data ready interrupt pin */
#endif
	
#define SIGN_EXT_24(VAL)								((int32_t)((uint32_t)(VAL) ^ (1UL<<(23))) - (1L<<(23)))	

/**
 *	\brief Error codes for interacting with the ADS1291_2.
 *
 */
typedef enum
{
	ADS1291_2_STATUS_OK	= 								0,				///< No error.
	ADS1291_2_ERROR_SPI_TIMEOUT	= 				1,				///< SPI timed out. Check SPI configuration and hardware connections.
	/* Expand with other codes if desired */
} ads1291_2_error_t;

#define ADS1291_2_NUM_REGS							12
#define ADS1299_NUM_REGS								24

/**
 *	\brief ADS1291_2 register addresses.
 *
 * Consult the ADS1291/2 datasheet and user's guide for more information.
 */
#define	ADS1299_REGADDR_ID			 		0x00			///< Chip ID register. Read-only.
#define	ADS1299_REGADDR_CONFIG1		 		0x01			///< Configuration register 1. Controls conversion mode and data rate.
#define	ADS1299_REGADDR_CONFIG2		 		0x02			///< Configuration register 2. Controls LOFF comparator, reference, CLK pin, and test signal.
#define ADS1299_REGADDR_CONFIG3				0x03			
#define	ADS1299_REGADDR_LOFF		 		0x04			///< Lead-off control register. Controls lead-off frequency, magnitude, and threshold.
#define	ADS1299_REGADDR_CH1SET		 		0x05					///< Channel 1 settings register. Controls channel 1 input mux, gain, and power-down.
#define	ADS1299_REGADDR_CH2SET		 		0x06			///< Channel 2 settings register (ADS1292x only). Controls channel 2 input mux, gain, and power-down.
#define	ADS1299_REGADDR_CH3SET		 		0x07
#define	ADS1299_REGADDR_CH4SET		 		0x08
#define	ADS1299_REGADDR_CH5SET		 		0x09
#define	ADS1299_REGADDR_CH6SET		 		0x0A
#define	ADS1299_REGADDR_CH7SET		 		0x0B
#define	ADS1299_REGADDR_CH8SET		 		0x0C
#define	ADS1299_REGADDR_BIAS_SENSP	 		0x0D			///< RLD sense selection. Controls PGA chop frequency, RLD buffer, and channels for RLD derivation.
#define	ADS1299_REGADDR_BIAS_SENSN	 		0x0E
#define	ADS1299_REGADDR_LOFF_SENSP	 		0x0F			///< Lead-off sense selection. Controls current direction and selects channels that will use lead-off detection.
#define	ADS1299_REGADDR_LOFF_SENSN	 		0x10
#define	ADS1299_REGADDR_LOFF_FLIP	 		0x11
#define	ADS1299_REGADDR_LOFF_STATP 			0x12			///< Lead-off status register. Bit 6 controls clock divider. For bits 4:0, 0: lead on, 1: lead off.
#define	ADS1299_REGADDR_LOFF_STATN	 		0x13
#define	ADS1299_REGADDR_GPIO		 		0x14			///< GPIO register. Controls state and direction of the ADS1291_2 GPIO pins.
#define	ADS1299_REGADDR_MISC1		 		0x15			///< Respiration 1 (ADS1292R only). See datasheet.
#define	ADS1299_REGADDR_MISC2		 		0x16			///< Respiration 2. Controls offset calibration, respiration modulator freq, and RLDREF signal source.
#define ADS1299_REGADDR_CONFIG4				0x17


/******************

#define	ADS1291_2_REGADDR_ID			 			0x00			///< Chip ID register. Read-only.
#define	ADS1291_2_REGADDR_CONFIG1		 		0x01			///< Configuration register 1. Controls conversion mode and data rate.
#define	ADS1291_2_REGADDR_CONFIG2		 		0x02			///< Configuration register 2. Controls LOFF comparator, reference, CLK pin, and test signal.
#define	ADS1291_2_REGADDR_LOFF		 			0x03			///< Lead-off control register. Controls lead-off frequency, magnitude, and threshold.
#define	ADS1291_2_REGADDR_CH1SET		 		0x04			///< Channel 1 settings register. Controls channel 1 input mux, gain, and power-down.
#define	ADS1291_2_REGADDR_CH2SET		 		0x05			///< Channel 2 settings register (ADS1292x only). Controls channel 2 input mux, gain, and power-down.
#define	ADS1291_2_REGADDR_RLD_SENS	 			0x06			///< RLD sense selection. Controls PGA chop frequency, RLD buffer, and channels for RLD derivation.
#define	ADS1291_2_REGADDR_LOFF_SENS	 		0x07			///< Lead-off sense selection. Controls current direction and selects channels that will use lead-off detection.
#define	ADS1291_2_REGADDR_LOFF_STAT	 		0x08			///< Lead-off status register. Bit 6 controls clock divider. For bits 4:0, 0: lead on, 1: lead off.
#define	ADS1291_2_REGADDR_RESP1		 			0x09			///< Respiration 1 (ADS1292R only). See datasheet.
#define	ADS1291_2_REGADDR_RESP2		 			0x0A			///< Respiration 2. Controls offset calibration, respiration modulator freq, and RLDREF signal source.
#define	ADS1291_2_REGADDR_GPIO		 			0x0B			///< GPIO register. Controls state and direction of the ADS1291_2 GPIO pins.
*/

/**
 *	\brief ADS1299 SPI communication opcodes.
 *	
 * Consult the ADS1299 datasheet and user's guide for more information.
 * For RREG and WREG opcodes, the first byte (opcode) must be ORed with the address of the register to be read/written. 
 * The command is completed with a second byte 000n nnnn, where n nnnn is (# registers to read) - 1.
 * \TODO: DOUBLE CHECK THESE VALS:
 */
 
#define	ADS1299_OPC_WAKEUP		 					0x02			///< Wake up from standby.
#define	ADS1299_OPC_STANDBY		 					0x04			///< Enter standby.
#define	ADS1299_OPC_RESET		 					0x06			///< Reset all registers.	
#define	ADS1299_OPC_START		 					0x08			///< Start data conversions.
#define	ADS1299_OPC_STOP		 					0x0A			///< Stop data conversions.
#define	ADS1299_OPC_OFFSETCAL						0x1A			///< Calibrate channel offset. RESP2.CALIB_ON must be 1. Execute after every PGA gain change.
#define	ADS1299_OPC_RDATAC		 					0x10			///< Read data continuously (registers cannot be read or written in this mode).
#define	ADS1299_OPC_SDATAC		 					0x11			///< Stop continuous data read.
#define	ADS1299_OPC_RDATA		 					0x12			///< Read single data value.

#define	ADS1299_OPC_RREG		 					0x20			///< Read register value. System must not be in RDATAC mode.
#define	ADS1299_OPC_WREG		 					0x40			///< Write register value.


 /**********************************/
#define	ADS1291_2_OPC_WAKEUP		 				0x02			///< Wake up from standby.
#define	ADS1291_2_OPC_STANDBY		 				0x04			///< Enter standby.
#define	ADS1291_2_OPC_RESET		 					0x06			///< Reset all registers.	
#define	ADS1291_2_OPC_START		 					0x08			///< Start data conversions.
#define	ADS1291_2_OPC_STOP		 					0x0A			///< Stop data conversions.
#define	ADS1291_2_OPC_OFFSETCAL					0x1A			///< Calibrate channel offset. RESP2.CALIB_ON must be 1. Execute after every PGA gain change.

#define	ADS1291_2_OPC_RDATAC		 				0x10			///< Read data continuously (registers cannot be read or written in this mode).
#define	ADS1291_2_OPC_SDATAC		 				0x11			///< Stop continuous data read.
#define	ADS1291_2_OPC_RDATA		 					0x12			///< Read single data value.
	
#define	ADS1291_2_OPC_RREG		 					0x20			///< Read register value. System must not be in RDATAC mode.
#define	ADS1291_2_OPC_WREG		 					0x40			///< Write register value. System must not be in RDATAC mode.

/* ID REGISTER ********************************************************************/

/**
 *  \brief Factory-programmed device ID for ADS1291/2.
 */ 
#define ADS1291_DEVICE_ID						0x52
#define ADS1292_DEVICE_ID						0x53	
#define ADS1292R_DEVICE_ID					    0x73

#define ADS1299_4_DEVICE_ID					    0x1C	//Not sure: should be [0b???11100]
#define ADS1299_6_DEVICE_ID					    0x1D	//Not sure: should be [0b???11101]
#define ADS1299_DEVICE_ID						0x1E	//Not sure: should be [0b???11101]

/* CONFIG1 REGISTER ***************************************************************/

/* CONFIG2 REGISTER ***************************************************************/

/* LOFF REGISTER ******************************************************************/

/* CHnSET REGISTERS *************************************************************/

/* RLD_SENS REGISTER **************************************************************/

/* LOFF_SENS REGISTER *************************************************************/

/* LOFF_STAT REGISTER *************************************************************/
/**
 *  \brief Bit mask definitions for LOFF_STAT.CLK_DIV (modulation clock division ratio).
 */

/* GPIO REGISTER ******************************************************************/

/* DEFAULT REGISTER VALUES ********************************************************/

//Use multi-line copy from excel file.
#define	ADS1299_REGDEFAULT_CONFIG1		 		0xB6			///< Configuration register 1. Controls conversion mode and data rate.
#define	ADS1299_REGDEFAULT_CONFIG2		 		0xD2			///< Configuration register 2. Controls LOFF comparator, reference, CLK pin, and test signal.
#define ADS1299_REGDEFAULT_CONFIG3				0xEC
#define	ADS1299_REGDEFAULT_LOFF		 			0x02			///< Lead-off control register. Controls lead-off frequency, magnitude, and threshold.
#define	ADS1299_REGDEFAULT_CH1SET		 		0x60			///< Channel 1 settings register. Controls channel 1 input mux, gain, and power-down.
#define	ADS1299_REGDEFAULT_CH2SET		 		0x60			///<
#define	ADS1299_REGDEFAULT_CH3SET		 		0x60
#define	ADS1299_REGDEFAULT_CH4SET		 		0x60
#define	ADS1299_REGDEFAULT_CH5SET		 		0xF1
#define	ADS1299_REGDEFAULT_CH6SET		 		0xF1
#define	ADS1299_REGDEFAULT_CH7SET		 		0xF1
#define	ADS1299_REGDEFAULT_CH8SET		 		0xF1
#define	ADS1299_REGDEFAULT_BIAS_SENSP	 		0x0F			///<
#define	ADS1299_REGDEFAULT_BIAS_SENSN	 		0x00
#define	ADS1299_REGDEFAULT_LOFF_SENSP	 		0x00			///<
#define	ADS1299_REGDEFAULT_LOFF_SENSN	 		0x00
#define	ADS1299_REGDEFAULT_LOFF_FLIP	 		0x00
#define	ADS1299_REGDEFAULT_LOFF_STATP 			0x00			///<
#define	ADS1299_REGDEFAULT_LOFF_STATN	 		0x00
#define	ADS1299_REGDEFAULT_GPIO		 			0x0F			///<
#define	ADS1299_REGDEFAULT_MISC1		 		0x20			///<
#define	ADS1299_REGDEFAULT_MISC2		 		0x00			///<
#define ADS1299_REGDEFAULT_CONFIG4				0x00


//
// 0x00 =  125SPS
// 0x01 =  250SPS
// 0x02 =  500SPS
// 0x03 = 1000SPS
// 0x04 = 2000SPS
// 0x05 = 4000SPS
// 0x06 = 8000SPS
//
// DON'T USE > 0x06
// 
#define ADS1291_2_REGDEFAULT_CONFIG1		    0x01			///< Continuous conversion, data rate = 1000SPS
#define ADS1291_2_REGDEFAULT_CONFIG2		    0xA3			///< LOFF off, REFBUF on, VREF=2.42, CLK_EN=0, INT_TEST=1, TEST_FREQ @ 1Hz
#define ADS1291_2_REGDEFAULT_LOFF				0x00			///< 95%/5% LOFF comparator threshold, DC lead-off at 6 nA	
#define ADS1291_2_REGDEFAULT_CH1SET			    0x60			///< Channel on, G=12, normal electrode
#define ADS1291_2_REGDEFAULT_CH2SET			    0x91			///< Channel off, G=1, input short
#define ADS1291_2_REGDEFAULT_RLD_SENS 	        0x23			///< Chop @ fmod/16, RLD buffer on, LOFF off, RLD derivation from CH1 P+N
#define ADS1291_2_REGDEFAULT_LOFF_SENS	        0x00			///< Current source @ IN+, sink @ IN-, all LOFF channels disconnected
#define ADS1291_2_REGDEFAULT_LOFF_STAT	        0x00			///< Fmod = fclk/4 (for fclk = 512 kHz)
#define ADS1291_2_REGDEFAULT_RESP1			    0x02			///< Resp measurement disabled
#define ADS1291_2_REGDEFAULT_RESP2			    0x07			///< Offset calibration disabled, RLD internally generated
#define ADS1291_2_REGDEFAULT_GPIO				0x00			///< All GPIO set to output, logic low



/**@TYPEDEFS: */
typedef int16_t body_voltage_t;

/**************************************************************************************************************************************************
*               Prototypes    ADS1299-x                                                                                                           *
**************************************************************************************************************************************************/
void ads_spi_init(void);


void init_buf(uint8_t * const p_tx_buffer,
                     uint8_t * const p_rx_buffer,
                     const uint16_t  len);

/**
 * TODO: ADS1299 FUNCTIONS:
 */
/**
 *	\brief Initialize the ADS1299-x.
 *
 * This function performs the power-on reset and initialization procedure documented on page 61 of the
 * ADS1299 datasheet, up to "Send SDATAC Command."
 *
 * \pre Requires spi_master.h from the nRF51 SDK.
 * \return Zero if successful, or an error code if unsuccessful.
 */
void ads1299_powerup_reset(void);

void ads1299_init_regs(void);

void ads1299_powerdn(void);

void ads1299_powerup(void);

void ads1299_standby(void);

void ads1299_wake(void);

void ads1299_soft_start_conversion(void);

void ads1299_stop_rdatac(void);

void ads1299_start_rdatac(void);

void ads1299_check_id(void);

void get_eeg_voltage_samples (int32_t *eeg1, int32_t *eeg2, int32_t *eeg3, int32_t *eeg4);
//Alt funcs:
void ads_spi_init_alt(void);
void get_eeg_voltage_samples_alt (int32_t *eeg1, int32_t *eeg2, int32_t *eeg3, int32_t *eeg4);
void ads1291_2_wake(void);
void ads1291_2_standby(void);
void ads1291_2_powerup(void);
void ads1291_2_init_regs(void);
void ads1291_2_soft_start_conversion(void);
void ads1291_2_stop_rdatac(void);
void ads1291_2_check_id(void);
void ads1291_2_start_rdatac(void);
void ads1291_2_powerdn(void);

#endif // ADS1291_2_H__
