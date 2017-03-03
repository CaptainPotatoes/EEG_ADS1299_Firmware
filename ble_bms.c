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

#include "ble_bms.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_error.h"
#include "ads1299-x.h"
#include "nrf_log.h"

#define MAX_BVM_LENGTH   		18	//20*3bytes																						 /**< Maximum size in bytes of a transmitted Body Voltage Measurement. */

void ble_bms_on_ble_evt(ble_bms_t * p_bms, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
						p_bms->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
						p_bms->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        default:
            break;
    }
}

/**@brief Function for encoding int16 Body Voltage Measurement buffer to a byte array.
 *
 * @param[in]   p_bms              Biopotential Measurement Service structure.
 * @param[in]   body_voltage       Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 
static uint8_t bvm_encode(ble_bms_t * p_bms, uint8_t * p_encoded_buffer)
{
    uint8_t len   = 0;
    int     i;

    // Encode body voltage measurement
    for (i = 0; i < p_bms->bvm_count; i++)
    {			
        if (len + sizeof(uint16_t) > MAX_BVM_LENGTH)
        {
            // Not all stored voltage values can fit into the packet, so
            // move the remaining values to the start of the buffer.
            memmove(&p_bms->bvm_buffer[0],
                    &p_bms->bvm_buffer[i],
                    (p_bms->bvm_count - i) * sizeof(uint16_t));
            break;
        }
        len += uint16_encode(p_bms->bvm_buffer[i], &p_encoded_buffer[len]);
    }
    p_bms->bvm_count -= i;
		

    return len;
}*/

static uint8_t bvm_encode_24(ble_bms_t * p_bms, uint8_t * p_encoded_buffer, int encodeChannel) {
    uint8_t len   = 0;
    int     i;
    // Encode body voltage measurement
		switch(encodeChannel) {
			case 1:
				for (i = 0; i < p_bms->bvm_count; i++) {			
						if (len + sizeof(uint16_t)+sizeof(uint8_t) > MAX_BVM_LENGTH) {
								// Not all stored voltage values can fit into the packet, so
								// move the remaining values to the start of the buffer.
								memmove(&p_bms->bvm_buffer[0],
												&p_bms->bvm_buffer[i],
												(p_bms->bvm_count - i) * sizeof(uint32_t));
								break;
						}
						len += uint24_encode(p_bms->bvm_buffer[i], &p_encoded_buffer[len]); //len+=3; 
				}
				p_bms->bvm_count -= i;
				break;
			case 2:
				for (i = 0; i < p_bms->eeg_ch2_count; i++) {			
						if (len + sizeof(uint16_t)+sizeof(uint8_t) > MAX_BVM_LENGTH) {
								// Not all stored voltage values can fit into the packet, so
								// move the remaining values to the start of the buffer.
								memmove(&p_bms->eeg_ch2_buffer[0],
												&p_bms->eeg_ch2_buffer[i],
												(p_bms->eeg_ch2_count - i) * sizeof(uint32_t));
								break;
						}
						len += uint24_encode(p_bms->eeg_ch2_buffer[i], &p_encoded_buffer[len]); //len+=3; 
				}
				p_bms->eeg_ch2_count -= i;
				break;
			case 3:
				for (i = 0; i < p_bms->eeg_ch3_count; i++) {			
						if (len + sizeof(uint16_t)+sizeof(uint8_t) > MAX_BVM_LENGTH) {
								// Not all stored voltage values can fit into the packet, so
								// move the remaining values to the start of the buffer.
								memmove(&p_bms->eeg_ch3_buffer[0],
												&p_bms->eeg_ch3_buffer[i],
												(p_bms->eeg_ch3_count - i) * sizeof(uint32_t));
								break;
						}
						len += uint24_encode(p_bms->eeg_ch3_buffer[i], &p_encoded_buffer[len]); //len+=3; 
				}
				p_bms->eeg_ch3_count -= i;
				break;
			case 4:
				for (i = 0; i < p_bms->eeg_ch4_count; i++) {			
						if (len + sizeof(uint16_t)+sizeof(uint8_t) > MAX_BVM_LENGTH) {
								// Not all stored voltage values can fit into the packet, so
								// move the remaining values to the start of the buffer.
								memmove(&p_bms->eeg_ch4_buffer[0],
												&p_bms->eeg_ch4_buffer[i],
												(p_bms->eeg_ch4_count - i) * sizeof(uint32_t));
								break;
						}
						len += uint24_encode(p_bms->eeg_ch4_buffer[i], &p_encoded_buffer[len]); //len+=3; 
				}
				p_bms->eeg_ch4_count -= i;
				break;
			default:
				break;
		}
    return len;
}


/**@brief Function for adding the Body Voltage Measurement characteristic.
 *
 * @param[in]   p_bms        Biopotential Measurement Service structure.
 * @param[in]   p_bms_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
 /**/
static uint32_t body_voltage_measurement_char_add(ble_bms_t * p_bms)
{
		uint32_t err_code = 0;
		ble_uuid_t	 						char_uuid;
		uint8_t             encoded_initial_bvm[MAX_BVM_LENGTH];
		BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_EEG_CH1_CHAR);
	
		ble_gatts_char_md_t char_md;
	
		memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write = 0;
		
		ble_gatts_attr_md_t cccd_md;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
		ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;    
    attr_md.vlen = 1;
    //TEEEEMP:
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
		
		ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
		attr_char_value.init_len		= bvm_encode_24(p_bms, encoded_initial_bvm, 1);
		attr_char_value.init_offs		= 0;
		attr_char_value.max_len			= MAX_BVM_LENGTH;
		attr_char_value.p_value   	= encoded_initial_bvm;
		err_code = sd_ble_gatts_characteristic_add(p_bms->service_handle,
																							&char_md,
																							&attr_char_value,
																							&p_bms->bvm_handles);
    APP_ERROR_CHECK(err_code);   

    return NRF_SUCCESS;
		//SET UP LIKE IN MPU EXAMPLE
}

static uint32_t eeg_ch2_char_add(ble_bms_t * p_bms)
{
		uint32_t err_code = 0;
		ble_uuid_t	 						char_uuid;
		uint8_t             encoded_initial_bvm[MAX_BVM_LENGTH];
		BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_EEG_CH2_CHAR);
	
		ble_gatts_char_md_t char_md;
	
		memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write = 0;
		
		ble_gatts_attr_md_t cccd_md;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
		ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;    
    attr_md.vlen = 1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
		
		ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
		attr_char_value.init_len		= bvm_encode_24(p_bms, encoded_initial_bvm, 2);
		attr_char_value.init_offs		= 0;
		attr_char_value.max_len			= MAX_BVM_LENGTH;
		attr_char_value.p_value   	= encoded_initial_bvm;
		err_code = sd_ble_gatts_characteristic_add(p_bms->service_handle,
																							&char_md,
																							&attr_char_value,
																							&p_bms->eeg_ch2_handles);
    APP_ERROR_CHECK(err_code);   

    return NRF_SUCCESS;
		//SET UP LIKE IN MPU EXAMPLE
}

static uint32_t eeg_ch3_char_add(ble_bms_t * p_bms)
{
		uint32_t err_code = 0;
		ble_uuid_t	 						char_uuid;
		uint8_t             encoded_initial_bvm[MAX_BVM_LENGTH];
		BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_EEG_CH3_CHAR);
	
		ble_gatts_char_md_t char_md;
	
		memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write = 0;
		
		ble_gatts_attr_md_t cccd_md;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
		ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;    
    attr_md.vlen = 1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
		
		ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
		attr_char_value.init_len		= bvm_encode_24(p_bms, encoded_initial_bvm, 3);
		attr_char_value.init_offs		= 0;
		attr_char_value.max_len			= MAX_BVM_LENGTH;
		attr_char_value.p_value   	= encoded_initial_bvm;
		err_code = sd_ble_gatts_characteristic_add(p_bms->service_handle,
																							&char_md,
																							&attr_char_value,
																							&p_bms->eeg_ch3_handles);
    APP_ERROR_CHECK(err_code);   

    return NRF_SUCCESS;
		//SET UP LIKE IN MPU EXAMPLE
}

static uint32_t eeg_ch4_char_add(ble_bms_t * p_bms)
{
		uint32_t err_code = 0;
		ble_uuid_t	 						char_uuid;
		uint8_t             encoded_initial_bvm[MAX_BVM_LENGTH];
		BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_EEG_CH4_CHAR);
	
		ble_gatts_char_md_t char_md;
	
		memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write = 0;
		
		ble_gatts_attr_md_t cccd_md;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
		ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;    
    attr_md.vlen = 1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
		
		ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
		attr_char_value.init_len		= bvm_encode_24(p_bms, encoded_initial_bvm, 4);
		attr_char_value.init_offs		= 0;
		attr_char_value.max_len			= MAX_BVM_LENGTH;
		attr_char_value.p_value   	= encoded_initial_bvm;
		err_code = sd_ble_gatts_characteristic_add(p_bms->service_handle,
																							&char_md,
																							&attr_char_value,
																							&p_bms->eeg_ch4_handles);
    APP_ERROR_CHECK(err_code);   

    return NRF_SUCCESS;
		//SET UP LIKE IN MPU EXAMPLE
}

/**@brief Function for initiating our new service.
 *
 * @param[in]   p_mpu        Our Service structure.
 *
 */
void ble_ecg_service_init(ble_bms_t *p_bms) {
		uint32_t   err_code; // Variable to hold return codes from library and softdevice functions

    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BMS_UUID_BASE;
    service_uuid.uuid = BLE_UUID_BIOPOTENTIAL_MEASUREMENT_SERVICE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);    

    p_bms->conn_handle = BLE_CONN_HANDLE_INVALID;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_bms->service_handle);
    
    APP_ERROR_CHECK(err_code);
		/*ADD CHARACTERISTIC(S)*/
		body_voltage_measurement_char_add(p_bms);
		eeg_ch2_char_add(p_bms);
		eeg_ch3_char_add(p_bms);
		eeg_ch4_char_add(p_bms);
}
#if (defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))
/**@Update adds single int16_t voltage value: */

void ble_bms_update_24(ble_bms_t *p_bms, int32_t *eeg, int32_t *eeg2, int32_t *eeg3, int32_t *eeg4) {
		// CH1
		ble_gatts_value_t gatts_value;
		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));
		gatts_value.len     = sizeof(uint16_t) + sizeof(uint8_t);
		gatts_value.offset  = 0;
		gatts_value.p_value = (uint8_t*)eeg;
		// Add new value to 32-bit buffer
		p_bms->bvm_buffer[p_bms->bvm_count++] = *eeg;
		if(p_bms->bvm_count == BLE_BMS_MAX_BUFFERED_MEASUREMENTS) {
				ble_bms_send_24_ch1(p_bms);
		}
		// CH2
		ble_gatts_value_t gatts_value_ch2;
		memset(&gatts_value_ch2, 0, sizeof(gatts_value_ch2));
		gatts_value_ch2.len     = sizeof(uint16_t) + sizeof(uint8_t);
		gatts_value_ch2.offset  = 0;
		gatts_value_ch2.p_value = (uint8_t*)eeg2;
		
		p_bms->eeg_ch2_buffer[p_bms->eeg_ch2_count++] = *eeg2;
		if(p_bms->eeg_ch2_count == BLE_BMS_MAX_BUFFERED_MEASUREMENTS) {
				ble_bms_send_24_ch2(p_bms);
		}
		//Ch3
		ble_gatts_value_t gatts_value_ch3;
		memset(&gatts_value_ch3, 0, sizeof(gatts_value_ch3));
		gatts_value_ch3.len     = sizeof(uint16_t) + sizeof(uint8_t);
		gatts_value_ch3.offset  = 0;
		gatts_value_ch3.p_value = (uint8_t*)eeg3;
		
		
		p_bms->eeg_ch3_buffer[p_bms->eeg_ch3_count++] = *eeg3;
		if(p_bms->eeg_ch3_count == BLE_BMS_MAX_BUFFERED_MEASUREMENTS) {
				ble_bms_send_24_ch3(p_bms);
		}
		
		//Ch4
		ble_gatts_value_t gatts_value_ch4;
		memset(&gatts_value_ch4, 0, sizeof(gatts_value_ch4));
		gatts_value_ch4.len     = sizeof(uint16_t) + sizeof(uint8_t);
		gatts_value_ch4.offset  = 0;
		gatts_value_ch4.p_value = (uint8_t*)eeg4;
		
		p_bms->eeg_ch4_buffer[p_bms->eeg_ch4_count++] = *eeg4;
		if(p_bms->eeg_ch4_count == BLE_BMS_MAX_BUFFERED_MEASUREMENTS) {
				ble_bms_send_24_ch4(p_bms);
		}
		
		
		//Initialize gatts buffer for each channel
		
		sd_ble_gatts_value_set(p_bms->conn_handle, p_bms->bvm_handles.value_handle, &gatts_value);
		sd_ble_gatts_value_set(p_bms->conn_handle, p_bms->eeg_ch2_handles.value_handle, &gatts_value_ch2);
		sd_ble_gatts_value_set(p_bms->conn_handle, p_bms->eeg_ch3_handles.value_handle, &gatts_value_ch3);
		sd_ble_gatts_value_set(p_bms->conn_handle, p_bms->eeg_ch4_handles.value_handle, &gatts_value_ch4);
}

uint32_t ble_bms_send_24_ch1 (ble_bms_t *p_bms) {
	uint32_t err_code;
	if (p_bms->conn_handle != BLE_CONN_HANDLE_INVALID) {
		uint8_t               	encoded_bvm[MAX_BVM_LENGTH];
		uint16_t      					len;
		uint16_t 								hvx_len;
		ble_gatts_hvx_params_t 	hvx_params;
		len 							= bvm_encode_24(p_bms, encoded_bvm, 1);
		hvx_len						= len;
		memset(&hvx_params, 0, sizeof(hvx_params));
		hvx_params.handle = p_bms->bvm_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = 0;
		hvx_params.p_len  = &hvx_len;
		hvx_params.p_data = encoded_bvm;
		err_code = sd_ble_gatts_hvx(p_bms->conn_handle, &hvx_params);
	}
	return err_code;
}

uint32_t ble_bms_send_24_ch2 (ble_bms_t *p_bms) {
	uint32_t err_code;
	if (p_bms->conn_handle != BLE_CONN_HANDLE_INVALID) {
		uint8_t               	encoded_bvm[MAX_BVM_LENGTH];
		uint16_t      					len;
		uint16_t 								hvx_len;
		ble_gatts_hvx_params_t 	hvx_params;
		len 							= bvm_encode_24(p_bms, encoded_bvm, 2);
		hvx_len						= len;
		memset(&hvx_params, 0, sizeof(hvx_params));
		hvx_params.handle = p_bms->eeg_ch2_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = 0;
		hvx_params.p_len  = &hvx_len;
		hvx_params.p_data = encoded_bvm;
		err_code = sd_ble_gatts_hvx(p_bms->conn_handle, &hvx_params);
	}
	return err_code;
}

uint32_t ble_bms_send_24_ch3 (ble_bms_t *p_bms) {
	uint32_t err_code;
	if (p_bms->conn_handle != BLE_CONN_HANDLE_INVALID) {
		uint8_t               	encoded_bvm[MAX_BVM_LENGTH];
		uint16_t      					len;
		uint16_t 								hvx_len;
		ble_gatts_hvx_params_t 	hvx_params;
		len 							= bvm_encode_24(p_bms, encoded_bvm, 3);
		hvx_len						= len;
		memset(&hvx_params, 0, sizeof(hvx_params));
		hvx_params.handle = p_bms->eeg_ch3_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = 0;
		hvx_params.p_len  = &hvx_len;
		hvx_params.p_data = encoded_bvm;
		err_code = sd_ble_gatts_hvx(p_bms->conn_handle, &hvx_params);
	}
	return err_code;
}

uint32_t ble_bms_send_24_ch4 (ble_bms_t *p_bms) {
	uint32_t err_code;
	if (p_bms->conn_handle != BLE_CONN_HANDLE_INVALID) {
		uint8_t               	encoded_bvm[MAX_BVM_LENGTH];
		uint16_t      					len;
		uint16_t 								hvx_len;
		ble_gatts_hvx_params_t 	hvx_params;
		len 							= bvm_encode_24(p_bms, encoded_bvm, 4);
		hvx_len						= len;
		memset(&hvx_params, 0, sizeof(hvx_params));
		hvx_params.handle = p_bms->eeg_ch4_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = 0;
		hvx_params.p_len  = &hvx_len;
		hvx_params.p_data = encoded_bvm;
		err_code = sd_ble_gatts_hvx(p_bms->conn_handle, &hvx_params);
	}
	return err_code;
}

#endif// (defined(ADS1291) || defined(ADS1292) || defined(ADS1292R))
