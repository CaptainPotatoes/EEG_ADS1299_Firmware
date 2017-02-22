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
#include "ble_bms.h"
#include "nrf_delay.h"
/*@stuff for delay:*/
#include <stdio.h> 
#include "compiler_abstraction.h"
#include "nrf.h"

/**@SPI STUFF*/
#define SPIM0_SCK_PIN      	13    
#define SPIM0_MOSI_PIN      14    
#define SPIM0_MISO_PIN      12    
#define SPIM0_SS_PIN        15

/**@TX,RX Stuff: */
#define TX_RX_MSG_LENGTH         				7

uint8_t ads1291_2_default_regs[] = {
		ADS1291_2_REGDEFAULT_CONFIG1,
		ADS1291_2_REGDEFAULT_CONFIG2,
		ADS1291_2_REGDEFAULT_LOFF,
		ADS1291_2_REGDEFAULT_CH1SET,
		ADS1291_2_REGDEFAULT_CH2SET,
		ADS1291_2_REGDEFAULT_RLD_SENS,
		ADS1291_2_REGDEFAULT_LOFF_SENS,
		ADS1291_2_REGDEFAULT_LOFF_STAT,
		ADS1291_2_REGDEFAULT_RESP1,
		ADS1291_2_REGDEFAULT_RESP2,
		ADS1291_2_REGDEFAULT_GPIO 
};
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
	
		ADS1299_REGDEFAULT_GPIO 
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
		spi_config.frequency						=	NRF_DRV_SPI_FREQ_1M; 	
		//spi_config.irq_priority					= APP_IRQ_PRIORITY_LOW;
		spi_config.irq_priority					= APP_IRQ_PRIORITY_HIGHEST;
		spi_config.mode									= NRF_DRV_SPI_MODE_1; //CPOL = 0 (Active High); CPHA = TRAILING (1)
		spi_config.miso_pin 						= SPIM0_MISO_PIN;
		spi_config.sck_pin 							= SPIM0_SCK_PIN;
		spi_config.mosi_pin 						= SPIM0_MOSI_PIN;
		spi_config.ss_pin								= SPIM0_SS_PIN;
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

/* REGISTER READ/WRITE FUNCTIONS *****************************************************************************************************************/
void ads1291_2_rreg(uint8_t reg_addr, uint8_t num_to_read, uint8_t* read_reg_val_ptr){
		uint32_t i;
		uint8_t tx_data_spi[2+num_to_read];
						tx_data_spi[0] = ADS1291_2_OPC_RREG | reg_addr;
						tx_data_spi[1] = num_to_read - 1;
		for (i = 2; i < 2+num_to_read; i++)
		{
				tx_data_spi[i] = 0;
		}
		//NRF_LOG_PRINTF(" tx_data_spi: 0x%x \r\n", tx_data_spi);		
		nrf_drv_spi_transfer(&spi, tx_data_spi, 2+num_to_read, read_reg_val_ptr, 2+num_to_read);
		//NRF_LOG_PRINTF(" tx_data_spi (after): 0x%x \r\n", tx_data_spi);		
		NRF_LOG_PRINTF(" Single register read(*): 0x%x, %d \r\n", *read_reg_val_ptr, *read_reg_val_ptr);
		//NRF_LOG_PRINTF(" Single register read(1): 0x%x, %d \r\n", read_reg_val_ptr, read_reg_val_ptr);
		
}

void ads1291_2_wreg(uint8_t reg_addr, uint8_t num_to_write, uint8_t* write_reg_val_ptr){
		uint32_t i;
		uint8_t tx_data_spi[ADS1291_2_NUM_REGS+2];
		uint8_t rx_data_spi[ADS1291_2_NUM_REGS+2];
		
		if (num_to_write < ADS1291_2_NUM_REGS)
		{
			for (i = 0; i < ADS1291_2_NUM_REGS+2; i++)
			{
					tx_data_spi[i] = 0;
			}
		}
		
		tx_data_spi[0] = ADS1291_2_OPC_WREG | reg_addr;
		tx_data_spi[1] = num_to_write - 1;
		for (i = 0; i < num_to_write; i++)
		{
				tx_data_spi[i+2] = write_reg_val_ptr[i];
		}
		nrf_drv_spi_transfer(&spi,tx_data_spi,2+num_to_write, rx_data_spi,2+num_to_write);
		NRF_LOG_PRINTF(" Single register written:");
}


/* SYSTEM CONTROL FUNCTIONS **********************************************************************************************************************/

void ads1291_2_init_regs(void)
{	
	
	/**@TODO: REWRITE THIS FUNCTION. Not sure it works correctly.*/
	uint8_t i = 0;
	uint8_t num_registers = 12;
	uint8_t txrx_size = num_registers+2;
	uint8_t tx_data_spi[txrx_size]; //Size = 14 bytes
	uint8_t rx_data_spi[txrx_size]; //Size = 14 bytes
	uint8_t opcode_1 = 0x41;
	for(i=0;i<txrx_size;i++) {
		tx_data_spi[i] = 0;   // Set array to zero. 
		rx_data_spi[i] = 0;		// Set array to zero. 
	}
	// Set first byte to opcode WREG | Starting Address, which is = 0x41
	//tx_data_spi[0] = ADS1291_2_OPC_WREG | ADS1291_2_REGADDR_CONFIG1; 	
	tx_data_spi[0] = opcode_1;
	tx_data_spi[1] = num_registers-1;			//is the number of registers to write – 1. (OPCODE2)
	//fill remainder of tx with commands:
	for (i=0; i<num_registers; i++) {
		tx_data_spi[i+2] = ads1291_2_default_regs[i];
	}
	nrf_drv_spi_transfer(&spi, tx_data_spi, num_registers+2, rx_data_spi, num_registers+2);
	nrf_delay_ms(10);
	//ads1291_2_wreg(ADS1291_2_REGADDR_CONFIG1, ADS1291_2_NUM_REGS-1, ads1291_2_default_regs);
	NRF_LOG_PRINTF(" Power-on reset and initialization procedure..\r\n");
}

void ads1291_2_standby(void) {
		uint8_t tx_data_spi;
		uint8_t rx_data_spi;
	
		tx_data_spi = ADS1291_2_OPC_STANDBY;
	
		nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1);
		NRF_LOG_PRINTF(" ADS1291-2 placed in standby mode...\r\n");
}

void ads1291_2_wake(void) {
		uint8_t tx_data_spi;
		uint8_t rx_data_spi;
	
		tx_data_spi = ADS1291_2_OPC_WAKEUP;
	
		nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1);
		nrf_delay_ms(10);	// Allow time to wake up - 10ms
		NRF_LOG_PRINTF(" ADS1291-2 Wakeup..\r\n");
}

void ads1291_2_soft_start_conversion(void) {
		uint8_t tx_data_spi;
		uint8_t rx_data_spi;
	
		tx_data_spi = ADS1291_2_OPC_START;
	
		nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1);
		NRF_LOG_PRINTF(" Start ADC conversion..\r\n");
}

void ads1291_2_stop_rdatac(void) {
		uint8_t tx_data_spi;
		uint8_t rx_data_spi;
	
		tx_data_spi = ADS1291_2_OPC_SDATAC;
	
		nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1);
		NRF_LOG_PRINTF(" Continuous Data Output Disabled..\r\n");
}

void ads1291_2_start_rdatac(void) {
		uint8_t tx_data_spi;
		uint8_t rx_data_spi;
		tx_data_spi = ADS1291_2_OPC_RDATAC;
		nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1);
		NRF_LOG_PRINTF(" Continuous Data Output Enabled..\r\n");
}

void ads1299_powerdn(void)
{
	nrf_gpio_pin_clear(ADS1299_PWDN_PIN);
	nrf_delay_ms(10);
	NRF_LOG_PRINTF(" ADS POWERED DOWN..\r\n");
}

void ads1299_powerup(void)
{
		nrf_gpio_pin_set(ADS1299_PWDN_PIN);
		nrf_delay_ms(1000);		// Allow time for power-on reset
		NRF_LOG_PRINTF(" ADS POWERED UP...\r\n");
}

/* DATA RETRIEVAL FUNCTIONS **********************************************************************************************************************/

void ads1291_2_check_id(void)
{
		uint8_t device_id;
		#if defined(ADS1291)
		device_id = ADS1291_DEVICE_ID;
		#elif defined(ADS1292)
		device_id = ADS1292_DEVICE_ID;
		#endif
		uint8_t id_reg_val;
		
		uint8_t tx_data_spi[3];
		uint8_t rx_data_spi[7];
		tx_data_spi[0] = 0x20;	//Request Device ID
		tx_data_spi[1] = 0x01;	//Intend to read 1 byte
		tx_data_spi[2] = 0x00;	//This will be replaced by Reg Data
		nrf_drv_spi_transfer(&spi, tx_data_spi, 2+tx_data_spi[1], rx_data_spi, 2+tx_data_spi[1]);
		/*
		nrf_delay_ms(100);
		NRF_LOG_PRINTF("Check ID: 0x%x \r\n", rx_data_spi[0]);
		NRF_LOG_PRINTF("Check ID: 0x%x \r\n", rx_data_spi[1]);
		NRF_LOG_PRINTF("Check ID: 0x%x \r\n", rx_data_spi[2]);
		NRF_LOG_PRINTF("Check ID: 0x%x \r\n", rx_data_spi[3]);
		NRF_LOG_PRINTF("Check ID: 0x%x \r\n", rx_data_spi[4]);
		NRF_LOG_PRINTF("Check ID: 0x%x \r\n", rx_data_spi[5]);
		NRF_LOG_PRINTF("Check ID: 0x%x \r\n", rx_data_spi[6]);*/
		/**@NOTE: 0 & 1 contain nonsense information, only third byte we are interested in: **/
		nrf_delay_ms(1); //Wait for response:
		id_reg_val = rx_data_spi[2];
		if (id_reg_val == device_id)
		{
			NRF_LOG_PRINTF("Check ID (match): 0x%x \r\n", id_reg_val);
			//return 1;
		}
		else
		{
			NRF_LOG_PRINTF("Check ID (not match): 0x%x \r\n", id_reg_val);
			//return 0;
		}	
}

#define BYTE_TO_BINARY_PATTERN_16BIT "%c%c%c%c %c%c%c%c %c%c%c%c %c%c%c%c\r\n"
#define BYTE_TO_BINARY_16BIT(byte)  \
	(byte & 0x8000 ? '1' : '0'), 		\
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
/**@brief Function for acquiring a Body Voltage Measurement sample.
 *
 * @details If SPI is enabled, this function will use it. Otherwise, it will use the
 *          sensor simulator.
 */
 /**@FROM RDATAC:
 
void get_bvm_sample (body_voltage_t *body_voltage) {
		// Empty TX Buffer:
		uint8_t tx_rx_data[5] = {0x00, 0x00, 0x00, 0x00, 0x00}; //Allocate for 5 bytes of data
		//uint8_t tx_rx_data[6] = {0x00, 0x00, 0x00, 0x00, 0x00}; 	
		//99% Working: Make adjustments until it is 100%
		nrf_drv_spi_transfer(&spi, tx_rx_data, 0, tx_rx_data, 5);
		nrf_delay_us(40); //< = tSCLK * 5 bytes * 8 bits = 40µs
											//< = tSCLK (1µs) * 6 bytes * 8 bits = 48µs
											//< = if using 2 channels, delay = tSCLK * 9 bytes * 8 bits or 72µs
		*body_voltage = ((tx_rx_data[3] << 8) | tx_rx_data[4]);//(int16_t)
		//0,1,2 = 24-bit STAT
		//3,4,5 = 24-bit CH1 DATA
		//6,7,8 = 24-bit CH2 DATA
}*/
void get_bvm_sample (body_voltage_t *body_voltage) {
		uint8_t tx_rx_data[9] = {0x00, 0x00, 0x00,
														0x00, 0x00, 0x00,
														0x00, 0x00, 0x00};
		
		nrf_drv_spi_transfer(&spi, tx_rx_data, 9, tx_rx_data, 9);
		uint8_t cnt = 0;
		/**/do { 
			cnt++;
			if(tx_rx_data[6]==0xC0) {
				*body_voltage = ((tx_rx_data[3] << 8) | tx_rx_data[4]);
				//NRF_LOG_PRINTF("[cnt] = %d, bv = %d\r\n",cnt ,*body_voltage);
				break;
			}
			nrf_delay_us(1);
		} while (cnt<255);
		/*do {
			cnt++;
			*body_voltage = ((tx_rx_data[3] << 8) | tx_rx_data[4]);
			//NRF_LOG_PRINTF("[cnt] = %d, bv = %d\r\n",cnt ,*body_voltage);
			//}
		} while (cnt!=5);*/
}

void get_24bit_sample (eeg24_t *eeg) {
		uint8_t tx_rx_data[9] = {0x00, 0x00, 0x00,
														0x00, 0x00, 0x00,
														0x00, 0x00, 0x00};
		nrf_drv_spi_transfer(&spi, tx_rx_data, 9, tx_rx_data, 9);
		//uint8_t c = 0;
		int16_t gggg = ((tx_rx_data[3] << 8) | tx_rx_data[4]);
    *eeg = ((tx_rx_data[3] <<16) | (tx_rx_data[4] <<8) | (tx_rx_data[5]));
		nrf_delay_us(50);
		NRF_LOG_PRINTF("DATA[24]: 0x%x\r\n",*eeg);
		NRF_LOG_PRINTF("DATA[16]: 0x%x\r\n",gggg);														
}

//void get_24bit_samples (eeg24_t *eegch1, eeg24_t *eegch2, eeg24_t *eegch3, eeg24_t *eegch4) 

