/* Copyright (c) 2015 Graham Kelly
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
//#include "ble_bms.h"

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


/*******************/

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
/**
 *	\brief ADS1291/2 SPI communication opcodes.
 *	
 * Consult the ADS1291/2 datasheet and user's guide for more information.
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
#define	ADS1291_2_OPC_OFFSETCAL						0x1A			///< Calibrate channel offset. RESP2.CALIB_ON must be 1. Execute after every PGA gain change.

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
#define ADS1292R_DEVICE_ID					0x73

#define ADS1299_4_DEVICE_ID					0x1C	//Not sure: should be [0b???11100]
#define ADS1299_6_DEVICE_ID					0x1D	//Not sure: should be [0b???11101]
#define ADS1299_DEVICE_ID						0x1E	//Not sure: should be [0b???11101]

/* CONFIG1 REGISTER ***************************************************************/

/**
 *  \brief Bit mask definitions for CONFIG1.SINGLE_SHOT (single-shot or continuous conversion setting).
 *
 */
#define ADS1291_2_REG_CONFIG1_CONTINUOUS_CONVERSION_MODE		(0<<7)
#define ADS1291_2_REG_CONFIG1_SINGLE_SHOT_MODE							(1<<7)

/**
 *  \brief Bit mask definitions for CONFIG1.DR (data rate).
 *
 * FMOD = 128 kHz (FCLK/4 for FCLK = 512 kHz, FCLK/16 for FCLK = 2.048 MHz).
 */
#define	ADS1291_2_REG_CONFIG1_FMOD_DIV_BY_1024			 	0		///< Data is output at FMOD/1024, or 125 SPS.
#define ADS1291_2_REG_CONFIG1_125_SPS									0
#define	ADS1291_2_REG_CONFIG1_FMOD_DIV_BY_512			 		1		///< Data is output at FMOD/512, or 250 SPS.
#define ADS1291_2_REG_CONFIG1_250_SPS									1
#define	ADS1291_2_REG_CONFIG1_FMOD_DIV_BY_256			 		2		///< Data is output at FMOD/256, or 500 SPS.
#define ADS1291_2_REG_CONFIG1_500_SPS									2
#define	ADS1291_2_REG_CONFIG1_FMOD_DIV_BY_128			 		3		///< Data is output at FMOD/128, or 1000 SPS.
#define ADS1291_2_REG_CONFIG1_1000_SPS								3
#define	ADS1291_2_REG_CONFIG1_FMOD_DIV_BY_64		 			4		///< Data is output at FMOD/64, or 2000 SPS.
#define ADS1291_2_REG_CONFIG1_2000_SPS								4
#define	ADS1291_2_REG_CONFIG1_FMOD_DIV_BY_32		 			5		///< Data is output at FMOD/32, or 4000 SPS.
#define ADS1291_2_REG_CONFIG1_4000_SPS								5
#define	ADS1291_2_REG_CONFIG1_FMOD_DIV_BY_16		 			6		///< Data is output at FMOD/16, or 8000 SPS.
#define ADS1291_2_REG_CONFIG1_8000_SPS								6

/**
 *  \brief Combined value of reserved bits in CONFIG1 register.
 *
 * Consult the ADS1291_2 datasheet, page 40, for more information.
 */
#define ADS1291_2_REG_CONFIG1_RESERVED_VALUE					0



/* CONFIG2 REGISTER ***************************************************************/

/**
 *  \brief Bit mask definitions for CONFIG2.TEST_FREQ (test signal frequency).
 */
#define	ADS1291_2_REG_CONFIG2_TEST_SIGNAL_DC					0		///< Test signal is not pulsed.
#define	ADS1291_2_REG_CONFIG2_TEST_SIGNAL_1_HZ				1		///< Test signal is pulsed at 1 Hz.


/**
 *  \brief Bit mask definitions for CONFIG2.INT_TEST (test signal on/off).
 */
#define	ADS1291_2_REG_CONFIG2_CAL_AMP_VREF_DIV_2_4_MV		(0<<1)		///< Calibration signal amplitude is 1 x (VREFP - VREFN)/(2.4 mV).
#define	ADS1291_2_REG_CONFIG2_CAL_AMP_2VREF_DIV_2_4_MV	(1<<1)		///< Calibration signal amplitude is 2 x (VREFP - VREFN)/(2.4 mV).

/**
 *  \brief Bit location and size definitions for CONFIG2.CLK_EN bit (oscillator output on CLK pin en/disabled).
 *
 * Consult the ADS1291_2 datasheet, page 40, for more information.
 */
#define ADS1291_2_REG_CONFIG2_CLOCK_OUTPUT_DISABLED	 (0<<3)
#define ADS1291_2_REG_CONFIG2_CLOCK_OUTPUT_ENABLED	 (1<<3)

/**
 *  \brief Bit mask definitions for CONFIG2.VREF_4V (reference voltage).
 */
#define	ADS1291_2_REG_CONFIG2_VREF_2V42						(0<<4)		///< Internal voltage reference is 2.42 V.
#define	ADS1291_2_REG_CONFIG2_VREF_4V033					(1<<4)		///< Internal voltage reference is 4.033 V (only for AVDD = 5V).

/**
 *  \brief Bit mask definitions for CONFIG2.PDB_REFBUF (internal voltage reference buffer enable/disable).
 *
 * Note that disabling the buffer for the internal voltage reference requires that a reference voltage
 * must be externally applied on VREFP for proper operation.
 */
#define	ADS1291_2_REG_CONFIG2_REFBUF_DISABLED			(0<<5)
#define	ADS1291_2_REG_CONFIG2_REFBUF_ENABLED			(1<<5)

/**
 *  \brief Bit mask definitions for CONFIG2.PDB_LOFF_COMP (power-down lead-off comparators).
 *
 */
#define ADS1291_2_REG_CONFIG2_LEAD_OFF_DISABLED				(0<<6)
#define ADS1291_2_REG_CONFIG2_LEAD_OFF_ENABLED				(1<<6)

/**
 *  \brief Combined value of reserved bits in CONFIG2 register.
 *
 * Consult the ADS1291/2 datasheet, page 41, for more information.
 */
#define	ADS1291_2_REG_CONFIG2_RESERVED_VALUE				(1<<7)



/* LOFF REGISTER ******************************************************************/

/**
 *  \brief Bit mask definitions for LOFF.COMP_TH (lead-off comparator threshold).
 *
 * Definition names are for the positive side (LOFFP). The corresponding LOFFN thresholds
 * are the difference between these thresholds and 100%. Default value is _95_PERCENT.
 */
#define	ADS1291_2_REG_LOFF_95_PERCENT					(0<<5)
#define	ADS1291_2_REG_LOFF_92_5_PERCENT				(1<<5)
#define	ADS1291_2_REG_LOFF_90_PERCENT					(2<<5)
#define	ADS1291_2_REG_LOFF_87_5_PERCENT				(3<<5)
#define	ADS1291_2_REG_LOFF_85_PERCENT					(4<<5)
#define	ADS1291_2_REG_LOFF_80_PERCENT					(5<<5)
#define	ADS1291_2_REG_LOFF_75_PERCENT					(6<<5)
#define	ADS1291_2_REG_LOFF_70_PERCENT					(7<<5)	

/**
 *  \brief Bit mask definitions for LOFF.ILEAD_OFF (lead-off current magnitude).
 *
 * This should be as small as possible for continuous lead-off detection, so as not to noticeably alter
 * the acquired signal. Default is _6_NA.
 */
#define	ADS1291_2_REG_LOFF_6_NA						(0<<2)			///< 6 nA lead-off current.
#define	ADS1291_2_REG_LOFF_22_NA					(1<<2)			///< 24 nA lead-off current.
#define	ADS1291_2_REG_LOFF_6_UA						(2<<2)			///< 6 uA lead-off current.
#define	ADS1291_2_REG_LOFF_22_UA					(3<<2)			///< 24 uA lead-off current.					

/**
 *  \brief Bit mask definitions for LOFF.FLEAD_OFF (lead-off current frequency).
 *
 * This should be as large as possible for continuous AC lead-off detection to ensure that it is out
 * of the frequency band of interest. The excitation signal can then be filtered out of the acquired
 * overall signal, and its voltage amplitude measured in order to determine the electrode impedance.
 * FCLK is the clock frequency of the ADS1291/2. This is normally 512 kHz.
 * FDR is the output data rate. With the default clock, this must be at least 1 kHz in order to use
 * continuous AC impedance monitoring, since the excitation frequency of FDR/4 = 250 Hz is the lowest
 * possible frequency outside of the EEG band. If only a specific band is needed and it is lower than
 * 62.5 Hz or 125 Hz, the 250/500 Hz settings may be used.
 */
#define	ADS1291_2_REG_LOFF_DC_LEAD_OFF							0		///< Lead-off current is at DC.
#define	ADS1291_2_REG_LOFF_AC_LEAD_OFF_FDR_DIV_4		1		///< Lead-off current is at FDR/4.

/**
 *  \brief Combined value of reserved bits in LOFF register.
 *
 */
#define ADS1291_2_REG_LOFF_RESERVED_VALUE				(1<<4)

/* CHnSET REGISTERS *************************************************************/

/**
 *  \brief Bit mask definitions for CHnSET.PD (channel power-down).
 */
#define	ADS1291_2_REG_CHNSET_CHANNEL_ON					(0<<7)
#define	ADS1291_2_REG_CHNSET_CHANNEL_OFF				(1<<7)

/**
 *  \brief Bit mask definitions for CHnSET.GAIN (channel PGA gain).
 *
 * Take care to ensure that the gain is appropriate for the common-mode level of the device inputs.
 * Higher gain settings have lower input-referred noise.
 * Consult the ADS1291/2 datasheet, pages 7-9 and 20-22, for more information.
 */
#define	ADS1291_2_REG_CHNSET_GAIN_1				(1<<4)			///< PGA gain = 1.
#define	ADS1291_2_REG_CHNSET_GAIN_2				(2<<4)			///< PGA gain = 2.
#define	ADS1291_2_REG_CHNSET_GAIN_3				(3<<4)			///< PGA gain = 4.
#define	ADS1291_2_REG_CHNSET_GAIN_4				(4<<4)			///< PGA gain = 6.
#define	ADS1291_2_REG_CHNSET_GAIN_6				(0<<4)			///< PGA gain = 8.
#define	ADS1291_2_REG_CHNSET_GAIN_8				(5<<4)			///< PGA gain = 12.
#define	ADS1291_2_REG_CHNSET_GAIN_12			(6<<4)			///< PGA gain = 24.


/**
 *  \brief Bit mask definitions for CHnSET.MUX (channel mux setting).
 *
 * Controls the channel multiplexing on the ADS1291/2.
 * Consult the ADS1291/2 datasheet, pages 17-19, for more information.
 */
#define	ADS1291_2_REG_CHNSET_NORMAL_ELECTRODE		0			///< Channel is connected to the corresponding positive and negative input pins.
#define	ADS1291_2_REG_CHNSET_INPUT_SHORTED			1			///< Channel inputs are shorted together. Used for offset and noise measurements.
#define	ADS1291_2_REG_CHNSET_RLD_MEASUREMENT		2			///< Used to measure RLD signal.
#define	ADS1291_2_REG_CHNSET_MVDD_SUPPLY				3			///< Used for measuring analog and digital supplies. See ADS1291/2 datasheet, p. 19.
#define	ADS1291_2_REG_CHNSET_TEMPERATURE_SENSOR	4			///< Measures device temperature. See ADS1291/2 datasheet, p. 19.
#define	ADS1291_2_REG_CHNSET_TEST_SIGNAL				5			///< Measures calibration signal. See ADS1291/2 datasheet, pp. 17 and 41.
#define	ADS1291_2_REG_CHNSET_RLD_DRP						6			///< Connects positive side of channel to RLD output.
#define	ADS1291_2_REG_CHNSET_RLD_DRM						7 		///< Connects negative side of channel to RLD output.
#define	ADS1291_2_REG_CHNSET_RLD_DRPM						8 		///< Connects both sides of channel to RLD output.
#define	ADS1291_2_REG_CHNSET_IN3								9 		///< Connects INxP/INxM of channel to IN3P/IN3M, respectively.

/**
 *  \brief Combined value of reserved bits in CHnSET registers.
 *
 */
#define ADS1291_2_REG_CHNSET_RESERVED_VALUE			0


/* RLD_SENS REGISTER **************************************************************/

/**
 *  \brief Bit mask definitions for RLD_SENS.CHOP (PGA chop frequency).
 */
#define ADS1291_2_REG_RLD_SENS_CHOP_FMOD_DIV_16			(0<<6)
#define ADS1291_2_REG_RLD_SENS_CHOP_FMOD_DIV_2			(2<<6)
#define ADS1291_2_REG_RLD_SENS_CHOP_FMOD_DIV_4			(3<<6)

/**
 *  \brief Bit mask definitions for RLD_SENS.PDB_RLD (power-down or enable RLD buffer amplifier).
 */
#define	ADS1291_2_REG_RLD_SENS_RLDBUF_DISABLED			(0<<5)
#define	ADS1291_2_REG_RLD_SENS_RLDBUF_ENABLED				(1<<5)

/**
 *  \brief Bit mask definitions for RLD_SENS.RLD_LOFF_SENS (detection of RLD lead-off en/disable).
 */
#define	ADS1291_2_REG_RLD_SENS_LOFF_SENSE_DISABLED	(0<<4)
#define ADS1291_2_REG_RLD_SENS_LOFF_SENSE_ENABLED		(1<<4)

/**
 *  \brief Bit mask definitions for RLD_SENS.RLDxy (channels for RLD derivation).
 *
 * Consult the ADS1291/2 datasheet, page 45, for more information.
 */
#define ADS1291_2_REG_RLD_SENS_RLD2N_ENABLED				(1<<3)
#define ADS1291_2_REG_RLD_SENS_RLD2N_DISABLED				(0<<3)
#define ADS1291_2_REG_RLD_SENS_RLD2P_ENABLED				(1<<2)
#define ADS1291_2_REG_RLD_SENS_RLD2P_DISABLED				(0<<2)
#define ADS1291_2_REG_RLD_SENS_RLD1N_ENABLED				(1<<1)
#define ADS1291_2_REG_RLD_SENS_RLD1N_DISABLED				(0<<1)
#define ADS1291_2_REG_RLD_SENS_RLD1P_ENABLED				(1<<0)
#define ADS1291_2_REG_RLD_SENS_RLD1P_DISABLED				(0<<0)

/**
 *  \brief Combined value of reserved bits in RLD_SENS register.
 *
 */
#define ADS1291_2_REG_RLD_SENS_RESERVED_VALUE				0



/* LOFF_SENS REGISTER *************************************************************/

/**
 *  \brief Bit mask definitions for LOFF_SENS.FLIP2 (current direction for CH2 LOFF).
 */
#define	ADS1291_2_REG_LOFF_SENS_CH2_PSOURCE_NSINK		(0<<5)
#define ADS1291_2_REG_LOFF_SENS_CH2_PSINK_NSOURCE		(1<<5)

/**
 *  \brief Bit mask definitions for LOFF_SENS.FLIP1 (current direction for CH1 LOFF).
 */
#define	ADS1291_2_REG_LOFF_SENS_CH1_PSOURCE_NSINK		(0<<4)
#define ADS1291_2_REG_LOFF_SENS_CH1_PSINK_NSOURCE		(1<<4)

/**
 *  \brief Bit mask definitions for LOFF_SENS register.
 *
 * Consult the ADS1291/2 datasheet, page 46, for more information.
 */
#define ADS1291_2_REG_LOFF_SENS_LOFF2N_ENABLED			(1<<3)
#define ADS1291_2_REG_LOFF_SENS_LOFF2N_DISABLED			(0<<3)
#define ADS1291_2_REG_LOFF_SENS_LOFF2P_ENABLED			(1<<2)
#define ADS1291_2_REG_LOFF_SENS_LOFF2P_DISABLED			(0<<2)
#define ADS1291_2_REG_LOFF_SENS_LOFF1N_ENABLED			(1<<1)
#define ADS1291_2_REG_LOFF_SENS_LOFF1N_DISABLED			(0<<1)
#define ADS1291_2_REG_LOFF_SENS_LOFF1P_ENABLED			(1<<0)
#define ADS1291_2_REG_LOFF_SENS_LOFF1P_DISABLED			(0<<0)

/**
 *  \brief Combined value of reserved bits in LOFF_SENS register.
 *
 */
#define ADS1291_2_REG_LOFF_SENS_RESERVED_VALUE				0



/* LOFF_STAT REGISTER *************************************************************/
/**
 *  \brief Bit mask definitions for LOFF_STAT.CLK_DIV (modulation clock division ratio).
 */
#define	ADS1291_2_REG_LOFF_STAT_FMOD_IS_FCLK_DIV_4	(0<<6)
#define ADS1291_2_REG_LOFF_STAT_FMOD_IS_FCLK_DIV_16	(1<<6)

/**
 *  \brief Bit mask definitions for LOFF_STAT (read-only).
 *
 * Consult the ADS1291/2 datasheet, page 47, for more information.
 */
#define	ADS1291_2_REG_LOFF_STAT_RLD_OFF							(1<<4)
#define ADS1291_2_REG_LOFF_STAT_IN2N_OFF						(1<<3)
#define	ADS1291_2_REG_LOFF_STAT_IN2P_OFF						(1<<2)
#define ADS1291_2_REG_LOFF_STAT_IN1N_OFF						(1<<1)
#define ADS1291_2_REG_LOFF_STAT_IN1P_OFF						(1<<0)

/**
 *  \brief Combined value of reserved bits in LOFF_STAT register.
 *
 */
#define ADS1291_2_REG_LOFF_STAT_RESERVED_VALUE				0



/* RESP1 REGISTER *****************************************************************/

/* RESP1 controls respiration functionality that is:
 * 				1) only present in the ADS1292R
 *				2) not supported by the current board design
 * Therefore this register is currently unimplemented by the present API.
 */

/**
 *  \brief Combined value of reserved bits in RESP1 register.
 *
 */
#define ADS1291_2_REG_RESP1_RESERVED_VALUE				0x02



/* RESP2 REGISTER *****************************************************************/

/**
 *  \brief Bit mask definitions for RESP2.CALIB_ON (offset calibration en/disable).
 */
#define ADS1291_2_REG_RESP2_OFFSET_CALIB_OFF			(0<<7)
#define ADS1291_2_REG_RESP2_OFFSET_CALIB_ON				(1<<7)

/**
 *  \brief Bit mask definitions for RESP2.RESP_FREQ (respiration modulation frequency, ADS1292R only).
 *
 * This is included only for completeness's sake. Current hardware doesn't support respiration monitoring.
 * This bit must be a '1' for ADS1291 and ADS1292 (64 kHz).
 */
#define ADS1291_2_REG_RESP2_RESP_FREQ_32_KHZ			(0<<2)
#define ADS1291_2_REG_RESP2_RESP_FREQ_64_KHZ			(1<<2)

/**
 *  \brief Bit mask definitions for RESP2.RLDREF_INT (RLD reference internally or externally generated).
 */
#define	ADS1291_2_REG_RESP2_RLDREF_EXT						(0<<1)
#define	ADS1291_2_REG_RESP2_RLDREF_INT						(1<<1)

/**
 *  \brief Combined value of reserved bits in RESP2 register.
 */
#define ADS1291_2_REG_RESP2_RESERVED_VALUE				0x01



/* GPIO REGISTER ******************************************************************/
/**
 *  \brief Bit mask definitions for GPIO.GPIODn (GPIO direction bits).
 *
 * The ADS1291/2 have 2 GPIO pins that can be manipulated via the SPI bus if there are not enough
 * GPIO pins available on the host.
 * GPIOD[2:1] controls the logic levels on GPIO pins 2:1.
 *
 * Consult the ADS1291/2 datasheet, page 49, for more information.
 */
#
#define ADS1291_2_REG_GPIO_GPIOD2_LOW			(0<<3)
#define ADS1291_2_REG_GPIO_GPIOD2_HIGH		(1<<3)
#define ADS1291_2_REG_GPIO_GPIOD1_LOW			(0<<2)
#define ADS1291_2_REG_GPIO_GPIOD1_HIGH		(1<<2)

/**
 *  \brief Bit mask definitions for GPIO.GPIOCn (GPIO level).
 *
 * The ADS1291/2 have 2 GPIO pins that can be manipulated via the SPI bus if there are not enough
 * GPIO pins available on the host.
 * GPIOC[2:1] controls the pin direction on GPIO pins 2:1.
 *
 * Consult the ADS1291/2 datasheet, page 49, for more information.
 */
#define ADS1291_2_REG_GPIO_GPIOC2_OUTPUT	(0<<1)
#define ADS1291_2_REG_GPIO_GPIOC2_INPUT		(1<<1)
#define ADS1291_2_REG_GPIO_GPIOC1_OUTPUT	(0<<0)
#define ADS1291_2_REG_GPIO_GPIOC1_INPUT		(1<<0)

/**
 *  \brief Combined value of reserved bits in GPIO register.
 *
 */
#define ADS1291_2_REG_GPIO_RESERVED_VALUE				0



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
#define	ADS1299_REGDEFAULT_BIAS_SENSP	 		0x00			///<
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


#endif // ADS1291_2_H__
