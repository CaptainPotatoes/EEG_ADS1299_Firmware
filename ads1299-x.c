/* Copyright (c) 2016 Musa Mahmood
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

/*#ifdef __cplusplus
extern "C" {
#endif*/


#include "ads1299-x.h"
#include "app_error.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "app_util_platform.h"
#include "nrf_log.h"
//#include "ble_bms.h"
#include "nrf_delay.h"
/*@stuff for delay:*/
#include <stdio.h> 
#include "compiler_abstraction.h"
#include "nrf.h"

/**@SPI STUFF*/
#define ADS1299_SPI_SCLK_PIN		10
#define ADS1299_SPI_CS_PIN			11
#define ADS1299_SPI_MOSI_PIN		14 //MASTER (nRF) OUT; SLAVE (ADS) DIN
#define ADS1299_SPI_MISO_PIN		 9 //MASTER (nRF) IN ; SLAVE (ADS) DOUT

/**@TX,RX Stuff: */
#define TX_RX_MSG_LENGTH         				7

/**@DEBUG STUFF */
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 
#define BYTE_TO_BINARY_PATTERN_16BIT "%c%c%c%c %c%c%c%c %c%c%c%c %c%c%c%c\r\n"
#define BYTE_TO_BINARY_16BIT(byte)  \
	(byte & 0x8000 ? '1' : '0'), \
  	(byte & 0x4000 ? '1' : '0'), \
  	(byte & 0x2000 ? '1' : '0'), \
  	(byte & 0x1000 ? '1' : '0'), \
  	(byte & 0x800 ? '1' : '0'), \
  	(byte & 0x400 ? '1' : '0'), \
  	(byte & 0x200 ? '1' : '0'), \
  	(byte & 0x100 ? '1' : '0'), \
  	(byte & 0x80 ? '1' : '0'), \
  	(byte & 0x40 ? '1' : '0'), \
  	(byte & 0x20 ? '1' : '0'), \
  	(byte & 0x10 ? '1' : '0'), \
  	(byte & 0x08 ? '1' : '0'), \
  	(byte & 0x04 ? '1' : '0'), \
  	(byte & 0x02 ? '1' : '0'), \
  	(byte & 0x01 ? '1' : '0')

/**
8 \FIX:
*/

uint8_t ads1299_default_registers[] = {
		ADS1299_REGDEFAULT_CONFIG1,
		ADS1299_REGDEFAULT_CONFIG2,
		ADS1299_REGDEFAULT_CONFIG3,
		ADS1299_REGDEFAULT_LOFF,
		ADS1299_REGDEFAULT_CH1SET,
		ADS1299_REGDEFAULT_CH2SET,
 		ADS1299_REGDEFAULT_CH3SET,
 		ADS1299_REGDEFAULT_CH4SET,
 		ADS1299_REGDEFAULT_CH5SET,
 		ADS1299_REGDEFAULT_CH6SET,
 		ADS1299_REGDEFAULT_CH7SET,
 		ADS1299_REGDEFAULT_CH8SET,
		ADS1299_REGDEFAULT_BIAS_SENSP,
		ADS1299_REGDEFAULT_BIAS_SENSN,
		ADS1299_REGDEFAULT_LOFF_SENSP,
		ADS1299_REGDEFAULT_LOFF_SENSN,
		ADS1299_REGDEFAULT_LOFF_FLIP,
		ADS1299_REGDEFAULT_LOFF_STATP,
		ADS1299_REGDEFAULT_LOFF_STATN,
		ADS1299_REGDEFAULT_GPIO,
		ADS1299_REGDEFAULT_MISC1,
		ADS1299_REGDEFAULT_MISC2,
		ADS1299_REGDEFAULT_CONFIG4
};

/**@SPI HANDLERS:
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
		/*switch (p_event->type) {
				case NRF_DRV_SPI_EVENT_DONE:
					break;
				default:
					break;
		}*/
    //NRF_LOG_PRINTF(" >>> Transfer completed.\r\n");
}

/**@INITIALIZE SPI INSTANCE */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(0); //SPI INSTANCE
void ads_spi_init(void) {
		nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG(0);
		spi_config.bit_order						= NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
		//SCLK = 1MHz is right speed because fCLK = (1/2)*SCLK, and fMOD = fCLK/4, and fMOD MUST BE 128kHz. Do the math.
		spi_config.frequency						= NRF_DRV_SPI_FREQ_1M;
		spi_config.irq_priority					= APP_IRQ_PRIORITY_LOW;
		spi_config.mode									= NRF_DRV_SPI_MODE_1; //CPOL = 0 (Active High); CPHA = TRAILING (1)
		spi_config.miso_pin 						= ADS1299_SPI_MISO_PIN;
		spi_config.sck_pin 							= ADS1299_SPI_SCLK_PIN;
		spi_config.mosi_pin 						= ADS1299_SPI_MOSI_PIN;
		spi_config.ss_pin								= ADS1299_SPI_CS_PIN;
		spi_config.orc									= 0x55;
		APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
		NRF_LOG_PRINTF(" SPI Initialized..\r\n");
}

/**@SPI-CLEARS BUFFER
 * @brief The function initializes TX buffer to values to be sent and clears RX buffer.
 *
 * @note Function clears RX and TX buffers.
 *
 * @param[out] p_tx_data    A pointer to a buffer TX.
 * @param[out] p_rx_data    A pointer to a buffer RX.
 * @param[in] len           A length of the data buffers.
 */
void init_buf(uint8_t * const p_tx_buffer,
                     uint8_t * const p_rx_buffer,
                     const uint16_t  len)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        p_tx_buffer[i] = 0;
        p_rx_buffer[i] = 0;
    }
		NRF_LOG_PRINTF(" SPI Buffer Cleared..\r\n");
}
/**************************************************************************************************************************************************
 *               Function Definitions                                                                                                              *
 **************************************************************************************************************************************************/

/*
 * ADS1299 CONTROL FUNCTIONS:
 */
void ads1299_init_regs(void) {
	uint8_t err_code;
	uint8_t num_registers = 23;
	uint8_t txrx_size = num_registers+2;
	uint8_t tx_data_spi[txrx_size]; //Size = 14 bytes
	uint8_t rx_data_spi[txrx_size]; //Size = 14 bytes
	uint8_t wreg_init_opcode = 0x41;
	for (int i = 0; i < txrx_size; ++i) {
		tx_data_spi[i] = 0;
		rx_data_spi[i] = 0;
	}
	tx_data_spi[0] = wreg_init_opcode;
	tx_data_spi[1] = num_registers - 1;
	for (int j = 0; j < num_registers; ++j) {
		tx_data_spi[j+2] = ads1299_default_registers[j];
	}
	err_code = nrf_drv_spi_transfer(&spi, tx_data_spi, num_registers+2, rx_data_spi, num_registers+2);
	nrf_delay_ms(150);
	NRF_LOG_PRINTF(" Power-on reset and initialization procedure.. EC: %d \r\n",err_code);
}

void ads1299_powerup_reset(void)
{
	nrf_gpio_pin_clear(ADS1299_PWDN_RST_PIN);
	nrf_delay_ms(50);
	NRF_LOG_PRINTF(" ADS1299-x POWERED UP AND RESET..\r\n");
}

void ads1299_powerdn(void)
{
	nrf_gpio_pin_clear(ADS1299_PWDN_RST_PIN);
	nrf_delay_us(20);
	NRF_LOG_PRINTF(" ADS1299-x POWERED DOWN..\r\n");
}

void ads1299_powerup(void)
{
	nrf_gpio_pin_set(ADS1299_PWDN_RST_PIN);
	nrf_delay_ms(1000);		// Allow time for power-on reset
	NRF_LOG_PRINTF(" ADS1299-x POWERED UP...\r\n");
}

void ads1299_standby(void) {
	uint8_t tx_data_spi;
	uint8_t rx_data_spi;

	tx_data_spi = ADS1299_OPC_STANDBY;

	nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1);
	NRF_LOG_PRINTF(" ADS1299-x placed in standby mode...\r\n");
}

void ads1299_wake(void) {
	uint8_t tx_data_spi;
	uint8_t rx_data_spi;

	tx_data_spi = ADS1299_OPC_WAKEUP;

	nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1);
	nrf_delay_ms(10);	// Allow time to wake up - 10ms
	NRF_LOG_PRINTF(" ADS1299-x Wakeup..\r\n");
}

void ads1299_soft_start_conversion(void) {
	uint8_t tx_data_spi;
	uint8_t rx_data_spi;

	tx_data_spi = ADS1299_OPC_START;

	nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1);
	NRF_LOG_PRINTF(" Start ADC conversion..\r\n");
}

void ads1299_stop_rdatac(void) {
	uint8_t tx_data_spi;
	uint8_t rx_data_spi;

	tx_data_spi = ADS1299_OPC_SDATAC;

	nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1);
	NRF_LOG_PRINTF(" Continuous Data Output Disabled..\r\n");
}

void ads1299_start_rdatac(void) {
	uint8_t tx_data_spi;
	uint8_t rx_data_spi;

	tx_data_spi = ADS1299_OPC_RDATAC;

	nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1);
	NRF_LOG_PRINTF(" Continuous Data Output Enabled..\r\n");
}

void ads1299_check_id(void) {
	uint8_t device_id_reg_value;
	uint8_t tx_data_spi[3];
	uint8_t rx_data_spi[7];
	tx_data_spi[0] = 0x20;	//Request Device ID
	tx_data_spi[1] = 0x01;	//Intend to read 1 byte
	tx_data_spi[2] = 0x00;	//This will be replaced by Reg Data
	nrf_drv_spi_transfer(&spi, tx_data_spi, 2+tx_data_spi[1], rx_data_spi, 2+tx_data_spi[1]);
	nrf_delay_ms(10); //Wait for response:
	device_id_reg_value = rx_data_spi[2];
	bool is_ads_1299_4 = (device_id_reg_value & 0x1F) == (ADS1299_4_DEVICE_ID); 
	bool is_ads_1299_6 = (device_id_reg_value & 0x1F) == (ADS1299_6_DEVICE_ID);
	bool is_ads_1299	 = (device_id_reg_value & 0x1F) == (ADS1299_DEVICE_ID); 
	uint8_t revisionVersion = (device_id_reg_value & 0xE0)>>5;
	if (is_ads_1299||is_ads_1299_6||is_ads_1299_4) {
		NRF_LOG_PRINTF("Device Address Matches!\r\n");
	} else {
		NRF_LOG_PRINTF("********SPI I/O Error, Device Not Detected! *********** \r\n");
		NRF_LOG_PRINTF("SPI Transfer Dump: \r\n");
		NRF_LOG_PRINTF("ID[b0->2]: [0x%x | 0x%x | 0x%x] \r\n", rx_data_spi[0],rx_data_spi[1],rx_data_spi[2]);
		NRF_LOG_PRINTF("ID[b3->6]: [0x%x | 0x%x | 0x%x | 0x%x] \r\n", rx_data_spi[3],rx_data_spi[4],rx_data_spi[5],rx_data_spi[6]);
	}
	if (is_ads_1299) {
		NRF_LOG_PRINTF("Device Name: ADS1299 \r\n");
	} else if (is_ads_1299_6) {
		NRF_LOG_PRINTF("Device Name: ADS1299-6 \r\n");
	} else if (is_ads_1299_4) {
		NRF_LOG_PRINTF("Device Name: ADS1299-4 \r\n");
	} 
	if (is_ads_1299||is_ads_1299_6||is_ads_1299_4) {
		NRF_LOG_PRINTF("Device Revision #%d\r\n",revisionVersion);
		NRF_LOG_PRINTF("Device ID: 0x%x \r\n",device_id_reg_value);
	}
}

/* DATA RETRIEVAL FUNCTIONS **********************************************************************************************************************/

/**@brief Function for acquiring a EEG Voltage Measurement samples.
 *
 * @details Uses SPI
 *          
 */
void get_eeg_voltage_samples (int32_t *eeg1, int32_t *eeg2, int32_t *eeg3, int32_t *eeg4) {
		uint8_t tx_rx_data[15] = {0x00, 0x00, 0x00,
														0x00, 0x00, 0x00,
														0x00, 0x00, 0x00,
														0x00, 0x00, 0x00,
														0x00, 0x00, 0x00};
		nrf_drv_spi_transfer(&spi, tx_rx_data, 15, tx_rx_data, 15);
		uint8_t cnt = 0;
		do {
			if(tx_rx_data[0]==0xC0){
				*eeg1 =  ( (tx_rx_data[3] << 16) | (tx_rx_data[4] << 8) | (tx_rx_data[5]) );					
				*eeg2 =  ( (tx_rx_data[6] << 16) | (tx_rx_data[7] << 8) | (tx_rx_data[8]) );			
				*eeg3 =  ( (tx_rx_data[9] << 16) | (tx_rx_data[10] << 8) | (tx_rx_data[11]) );
				*eeg4 =  ( (tx_rx_data[12] << 16) | (tx_rx_data[13] << 8) | (tx_rx_data[14]) );
				break;
			}
			cnt++;
			nrf_delay_us(1);
		} while(cnt<255);
		//NRF_LOG_PRINTF("B0-2 = [0x%x 0x%x 0x%x | cnt=%d]\r\n",tx_rx_data[0],tx_rx_data[1],tx_rx_data[2],cnt);
		//NRF_LOG_PRINTF("DATA:[0x%x 0x%x 0x%x 0x%x]\r\n",*eeg1,*eeg2,*eeg3,*eeg4);
}

// // // // // //
// End of File //
// // // // // // 
