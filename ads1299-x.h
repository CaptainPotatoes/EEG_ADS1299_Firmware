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
 * @brief Functions for initializing and controlling Texas Instruments ADS1299 analog front-end.
 */
 
#ifndef ADS1299_H__
#define ADS1299_H__
 
#include <stdint.h>
#include "nrf_drv_spi.h"

#ifdef __cplusplus
	extern "C" {
#endif
/**@SPI STUFF*/
#ifdef BOARD_PCA10028
	#define ADS1299_SPI_SCLK_PIN		10
	#define ADS1299_SPI_CS_PIN			11
	#define ADS1299_SPI_MOSI_PIN		14 //MASTER (nRF) OUT; SLAVE (ADS) DIN
	#define ADS1299_SPI_MISO_PIN		 9 //MASTER (nRF) IN ; SLAVE (ADS) DOUT
	#define ADS1299_PWDN_RST_PIN		12
	#define ADS1299_DRDY_PIN				 8
#elif defined(BOARD_NRF_BREAKOUT)
		//BOARD_CUSTOM BOARD_NRF_BREAKOUT
	#define ADS1299_SPI_SCLK_PIN		 9
	#define ADS1299_SPI_CS_PIN			10
	#define ADS1299_SPI_MOSI_PIN		12 //MASTER (nRF) OUT; SLAVE (ADS) DIN
	#define ADS1299_SPI_MISO_PIN		 8 //MASTER (nRF) IN ; SLAVE (ADS) DOUT
	#define ADS1299_PWDN_RST_PIN		11
	#define ADS1299_DRDY_PIN				15
#elif defined(BOARD_FULL_EEG_V1)
	#define ADS1299_DRDY_PIN					8
	#define ADS1299_SPI_MISO_PIN		 	9 //MASTER (nRF) IN ; SLAVE (ADS) DOUT
	#define ADS1299_SPI_SCLK_PIN		 	10
	#define ADS1299_SPI_CS_PIN				11
	#define ADS1299_RESET_PIN					12
	#define ADS1299_PWDN_PIN					13
	#define ADS1299_SPI_MOSI_PIN			14 //MASTER (nRF) OUT; SLAVE (ADS) DIN
#endif


#define ADS1299_NUM_REGS								24

/**
 *	\brief ADS1291_2 register addresses.
 *
 * Consult the ADS1291/2 datasheet and user's guide for more information.
 */
#define	ADS1299_REGADDR_ID			 				0x00			///< Chip ID register. Read-only.
#define	ADS1299_REGADDR_CONFIG1		 			0x01			///< Configuration register 1. Controls conversion mode and data rate.
#define	ADS1299_REGADDR_CONFIG2		 			0x02			///< Configuration register 2. Controls LOFF comparator, reference, CLK pin, and test signal.
#define ADS1299_REGADDR_CONFIG3					0x03			
#define	ADS1299_REGADDR_LOFF		 				0x04			///< Lead-off control register. Controls lead-off frequency, magnitude, and threshold.
#define	ADS1299_REGADDR_CH1SET		 			0x05					///< Channel 1 settings register. Controls channel 1 input mux, gain, and power-down.
#define	ADS1299_REGADDR_CH2SET		 			0x06			///< Channel 2 settings register (ADS1292x only). Controls channel 2 input mux, gain, and power-down.
#define	ADS1299_REGADDR_CH3SET		 			0x07
#define	ADS1299_REGADDR_CH4SET		 			0x08
#define	ADS1299_REGADDR_CH5SET		 			0x09
#define	ADS1299_REGADDR_CH6SET		 			0x0A
#define	ADS1299_REGADDR_CH7SET		 			0x0B
#define	ADS1299_REGADDR_CH8SET		 			0x0C
#define	ADS1299_REGADDR_BIAS_SENSP	 		0x0D			///< RLD sense selection. Controls PGA chop frequency, RLD buffer, and channels for RLD derivation.
#define	ADS1299_REGADDR_BIAS_SENSN	 		0x0E
#define	ADS1299_REGADDR_LOFF_SENSP	 		0x0F			///< Lead-off sense selection. Controls current direction and selects channels that will use lead-off detection.
#define	ADS1299_REGADDR_LOFF_SENSN	 		0x10
#define	ADS1299_REGADDR_LOFF_FLIP	 			0x11
#define	ADS1299_REGADDR_LOFF_STATP 			0x12			///< Lead-off status register. Bit 6 controls clock divider. For bits 4:0, 0: lead on, 1: lead off.
#define	ADS1299_REGADDR_LOFF_STATN	 		0x13
#define	ADS1299_REGADDR_GPIO		 				0x14			///< GPIO register. Controls state and direction of the ADS1291_2 GPIO pins.
#define	ADS1299_REGADDR_MISC1		 				0x15			///< Respiration 1 (ADS1292R only). See datasheet.
#define	ADS1299_REGADDR_MISC2		 				0x16			///< Respiration 2. Controls offset calibration, respiration modulator freq, and RLDREF signal source.
#define ADS1299_REGADDR_CONFIG4					0x17

/**
 *	\brief ADS1299 SPI communication opcodes.
 *	
 * Consult the ADS1299 datasheet and user's guide for more information.
 * For RREG and WREG opcodes, the first byte (opcode) must be ORed with the address of the register to be read/written. 
 * The command is completed with a second byte 000n nnnn, where n nnnn is (# registers to read) - 1.
 */
 
#define	ADS1299_OPC_WAKEUP		 				0x02			///< Wake up from standby.
#define	ADS1299_OPC_STANDBY		 				0x04			///< Enter standby.
#define	ADS1299_OPC_RESET		 					0x06			///< Reset all registers.	
#define	ADS1299_OPC_START		 					0x08			///< Start data conversions.
#define	ADS1299_OPC_STOP		 					0x0A			///< Stop data conversions.
#define	ADS1299_OPC_OFFSETCAL					0x1A			///< Calibrate channel offset. RESP2.CALIB_ON must be 1. Execute after every PGA gain change.
#define	ADS1299_OPC_RDATAC		 				0x10			///< Read data continuously (registers cannot be read or written in this mode).
#define	ADS1299_OPC_SDATAC		 				0x11			///< Stop continuous data read.
#define	ADS1299_OPC_RDATA		 					0x12			///< Read single data value.

#define	ADS1299_OPC_RREG		 					0x20			///< Read register value. System must not be in RDATAC mode.
#define	ADS1299_OPC_WREG		 					0x40			///< Write register value.


 /**********************************/

/* ID REGISTER ********************************************************************/

/**
 *  \brief Factory-programmed device ID for ADS1299 & ADS1299-x.
 */ 

#define ADS1299_4_DEVICE_ID					    0x1C	//Device ID [0bvvv11100] Where vvv is the version bits
#define ADS1299_6_DEVICE_ID					    0x1D	//Device ID [0bvvv11101]
#define ADS1299_DEVICE_ID								0x1E	//Device ID [0bvvv11101] 

/* DEFAULT REGISTER VALUES ********************************************************/

//Use multi-line copy from excel file.
#define	ADS1299_REGDEFAULT_CONFIG1		 		0xB6			///< Configuration register 1. Controls conversion mode and data rate.
#define	ADS1299_REGDEFAULT_CONFIG2		 		0xD2			///< Configuration register 2. Controls LOFF comparator, reference, CLK pin, and test signal.
#define ADS1299_REGDEFAULT_CONFIG3				0xEC
#define	ADS1299_REGDEFAULT_LOFF		 				0x02			///< Lead-off control register. Controls lead-off frequency, magnitude, and threshold.
#define	ADS1299_REGDEFAULT_CH1SET		 			0x60			///< Channel 1 settings register. Controls channel 1 input mux, gain, and power-down.
#define	ADS1299_REGDEFAULT_CH2SET		 			0x60			///<
#define	ADS1299_REGDEFAULT_CH3SET		 			0x60
#define	ADS1299_REGDEFAULT_CH4SET		 			0x60
#define	ADS1299_REGDEFAULT_CH5SET		 			0xF1
#define	ADS1299_REGDEFAULT_CH6SET		 			0xF1
#define	ADS1299_REGDEFAULT_CH7SET		 			0xF1
#define	ADS1299_REGDEFAULT_CH8SET		 			0xF1
#define	ADS1299_REGDEFAULT_BIAS_SENSP	 		0x0F			///<
#define	ADS1299_REGDEFAULT_BIAS_SENSN	 		0x00
#define	ADS1299_REGDEFAULT_LOFF_SENSP	 		0x00			///<
#define	ADS1299_REGDEFAULT_LOFF_SENSN	 		0x00
#define	ADS1299_REGDEFAULT_LOFF_FLIP	 		0x00
#define	ADS1299_REGDEFAULT_LOFF_STATP 		0x00			///<
#define	ADS1299_REGDEFAULT_LOFF_STATN	 		0x00
#define	ADS1299_REGDEFAULT_GPIO		 				0x0F			///<
#define	ADS1299_REGDEFAULT_MISC1		 			0x20			///<
#define	ADS1299_REGDEFAULT_MISC2		 			0x00			///<
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

/**@TYPEDEFS: */
typedef int16_t body_voltage_t;

/**************************************************************************************************************************************************
*              Function Prototypes ADS1299-x 																																																			*
**************************************************************************************************************************************************/
void ads_spi_init(void);


void init_buf(uint8_t * const p_tx_buffer,
                     uint8_t * const p_rx_buffer,
                     const uint16_t  len);

/**
 *	\brief Initialize the ADS1299-x.
 *
 * This function performs the power-on reset and initialization procedure documented on page 61 of the
 * ADS1299 datasheet, up to "Send SDATAC Command."
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


#endif // ADS1299_H__
