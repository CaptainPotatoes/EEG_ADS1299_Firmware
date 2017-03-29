/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "sensorsim.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_dis.h"
#include "ble_eeg.h"
#include "app_util_platform.h"
#include "nrf_log.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
/** ADS1299 Headers **/
#include "ads1299-x.h" /*< For the ADS1299 Chip */
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
/** @BAS: **/
#if (defined(BLE_BAS))
    #include "ble_bas.h"
    #include "nrf_adc.h"
#endif

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
#define CENTRAL_LINK_COUNT               0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
			/**@DEVICE INFO*/
#define DEVICE_NAME                      "EEG 250Hz"                      	    /**< Name of device. Will be included in the advertising data. */
#define DEVICE_NAME_500									 "EEG 500Hz" 
#define DEVICE_NAME_1000								 "EEG 1000Hz"
#define DEVICE_ERROR										 "EEG Other"
#define MANUFACTURER_NAME                "VCU-YEO-SeniorDesign"                      				/**< Manufacturer. Will be passed to Device Information Service. */
#define DEVICE_MODEL_NUMBERSTR					 "Version 3.1"
#define DEVICE_FIRMWARE_STRING					 "Version 2.0"
			/**@ADVERTISING INITIALIZATION: */
#define APP_ADV_INTERVAL                 300                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       0                                        /**< The advertising timeout in units of seconds. */
			/**@TIMER DETAILS:*/
#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */
			/**@GAP INITIALIZATION:*/
#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(16, UNIT_1_25_MS)//32        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(16, UNIT_1_25_MS)//32        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */
			/**@CONNPARAMS MODULE:*/
#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */
			/**@DEVICEMANAGER_INIT Definitions: */
#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */
			/**@ERROR MACRO:*/
#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
			/**@BLE HANDLES*/
static dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */
static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
/**@EEG BLE STRUCT */
ble_eeg_t 															 m_eeg;
//ble_bms_t m_eeg;
/**@BAS STUFF */
#if (defined(BLE_BAS))
ble_bas_t																 m_bas;
#endif
/**@GPIOTE */
#if defined(ADS1299)
	static bool															m_drdy = false;
#define DRDY_GPIO_PIN_IN 11
#endif //(defined(ADS1299)
/**@TIMER: -Timer Stuff- */
APP_TIMER_DEF(m_battery_timer_id);
#define BAS_TIMER_INTERVAL							APP_TIMER_TICKS(60000, APP_TIMER_PRESCALER)//every 60s
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS     1200                                     /**< Reference voltage (in millivolts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION      3                                        /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 255) * ADC_PRE_SCALING_COMPENSATION)
/**@Services declared under ble_###*/
static ble_uuid_t m_adv_uuids[] = 
{
		{BLE_UUID_BIOPOTENTIAL_EEG_MEASUREMENT_SERVICE, BLE_UUID_TYPE_BLE},
		#if defined(BLE_BAS)
			{BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
		#endif
		{BLE_UUID_DEVICE_INFORMATION_SERVICE, 			BLE_UUID_TYPE_BLE}
}; /**< Universally unique service identifiers. */

#if defined(BLE_BAS)
void ADC_IRQHandler(void)
{
    if (nrf_adc_conversion_finished())
    {
        uint8_t  adc_result;
        uint16_t batt_lvl_in_milli_volts;
        uint8_t  percentage_batt_lvl;
        uint32_t err_code;

        nrf_adc_conversion_event_clean();

        adc_result = nrf_adc_result_get();

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);
        percentage_batt_lvl     = battery_level_in_percent(batt_lvl_in_milli_volts) + 270;

        err_code = ble_bas_battery_level_update(&m_bas, percentage_batt_lvl);
        if (
            (err_code != NRF_SUCCESS)
            &&
            (err_code != NRF_ERROR_INVALID_STATE)
            &&
            (err_code != BLE_ERROR_NO_TX_PACKETS)
            &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
           )
        {
            APP_ERROR_HANDLER(err_code);
        }
    }
}
	#endif
 /**@ADC Configuration*/
#if defined(BLE_BAS)
static void adc_configure(void)
{
    uint32_t err_code;
    nrf_adc_config_t adc_config = NRF_ADC_CONFIG_DEFAULT;

    // Configure ADC
    adc_config.reference  = NRF_ADC_CONFIG_REF_VBG;
    adc_config.resolution = NRF_ADC_CONFIG_RES_8BIT;
    adc_config.scaling    = NRF_ADC_CONFIG_SCALING_SUPPLY_ONE_THIRD;
    nrf_adc_configure(&adc_config);

    // Enable ADC interrupt
    nrf_adc_int_enable(ADC_INTENSET_END_Msk);
    err_code = sd_nvic_ClearPendingIRQ(ADC_IRQn);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_SetPriority(ADC_IRQn, APP_IRQ_PRIORITY_LOW);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_EnableIRQ(ADC_IRQn);
    APP_ERROR_CHECK(err_code);
}
#endif
                                   
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/*static void timer_send_timeout_handler(void *p_context){
		UNUSED_PARAMETER(p_context);
}*/

#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))
static void mpu_send_timeout_handler(void *p_context) {
	
			//DEPENDSS ON SAMPLING RATE
			mpu_read_send_flag = true;	
	
}
#endif /**@(defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))*/

#if defined(BLE_BAS)
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    nrf_adc_start();
}
#endif

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
		#if defined(BLE_BAS)
    uint32_t err_code;
		err_code = app_timer_create(&m_battery_timer_id, APP_TIMER_MODE_REPEATED, battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
		#endif
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
		err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

#if defined(BLE_BAS)
static void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t *p_evt)
{
    uint32_t err_code;

    switch (p_evt->evt_type)
    {
        case BLE_BAS_EVT_NOTIFICATION_ENABLED:
            // Start battery timer
            err_code = app_timer_start(m_battery_timer_id, BAS_TIMER_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_BAS_EVT_NOTIFICATION_DISABLED:
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}
#endif

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
	
#if defined(BLE_BAS)
	ble_bas_init_t					m_bas_init;
	memset(&m_bas_init, 0, sizeof(m_bas_init));
	m_bas_init.evt_handler          = on_bas_evt;
    m_bas_init.support_notification = true;
    m_bas_init.p_report_ref         = NULL;
    m_bas_init.initial_batt_level   = 100;
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&m_bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&m_bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&m_bas_init.battery_level_char_attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&m_bas_init.battery_level_report_read_perm);
	ble_bas_init(&m_bas, &m_bas_init);
#endif
    ble_eeg_service_init(&m_eeg);
		//ble_ecg_service_init(&m_eeg);
		//ble_mpu_service_init(&m_mpu);
		/**@Device Information Service:*/
		uint32_t err_code;
		ble_dis_init_t dis_init;
		memset(&dis_init, 0, sizeof(dis_init));
		ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
		ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)DEVICE_MODEL_NUMBERSTR);
		ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)DEVICE_FIRMWARE_STRING);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);
		err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
*/
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.*/
    //uint32_t err_code;
    //err_code = app_timer_start(m_eeg_record_timer_id, TIMER_INTERVAL_UPDATE, NULL);
    //APP_ERROR_CHECK(err_code); 
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
		uint32_t err_code;
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    //uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id) {
				case BLE_EVT_TX_COMPLETE:
            break;
        case BLE_GAP_EVT_CONNECTED:
						#if defined(ADS1299)
                ads1299_wake();
						#endif
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
						#if defined(ADS1299)
                ads1299_standby();
						#endif
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
		ble_eeg_on_ble_evt(&m_eeg, p_ble_evt);
		#if defined(BLE_BAS)
		ble_bas_on_ble_evt(&m_bas, p_ble_evt);
		#endif
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    /** MAY NEED TO CHANGE THIS */
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
	
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);
    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}
#if defined(ADS1299) 
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
		UNUSED_PARAMETER(pin);
		UNUSED_PARAMETER(action);
    m_drdy = true;
}
#endif

#if defined(ADS1299)
static void ads1299_gpio_init(void) {
		nrf_gpio_pin_dir_set(ADS1299_DRDY_PIN, NRF_GPIO_PIN_DIR_INPUT); //sets 'direction' = input/output
		nrf_gpio_pin_dir_set(ADS1299_PWDN_RST_PIN, NRF_GPIO_PIN_DIR_OUTPUT);
		uint32_t err_code;
		if(!nrf_drv_gpiote_is_init())
		{
				err_code = nrf_drv_gpiote_init();
		}
		NRF_LOG_PRINTF("nrf_drv_gpiote_init: %d\r\n",err_code);
        APP_ERROR_CHECK(err_code);
		bool is_high_accuracy = true;
		nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(is_high_accuracy);
		in_config.is_watcher = true;
		in_config.pull = NRF_GPIO_PIN_NOPULL;
		err_code = nrf_drv_gpiote_in_init(ADS1299_DRDY_PIN, &in_config, in_pin_handler);
		NRF_LOG_PRINTF(" nrf_drv_gpiote_in_init: %d: \r\n",err_code);
		APP_ERROR_CHECK(err_code);
		nrf_drv_gpiote_in_event_enable(ADS1299_DRDY_PIN, true);
		ads1299_powerdn();
}
#endif /* defined(ADS1299) */

/**@brief Function for application main entry.
 */
int main(void)
{
    NRF_LOG_PRINTF(" BLE ECG WITH MPU - START..\r\n");
    uint32_t err_code;
    bool erase_bonds;
    // Initialize.
    timers_init();
    ble_stack_init();
		err_code = nrf_drv_clock_init();
		NRF_LOG_PRINTF("ERRCODE: DRV CLOCK: %d \r\n", err_code);
		APP_ERROR_CHECK(err_code);
    #if defined(ADS1299)
        ads1299_gpio_init();
    #endif
		
    device_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
    services_init();
    #if defined(BLE_BAS)
        adc_configure();
    #endif
    conn_params_init();
		
    #if defined(ADS1299)
				ads_spi_init();
				nrf_delay_ms(5);
				ads1299_powerdn();
        ads1299_powerup();
        //...
        ads1299_stop_rdatac();
        ads1299_init_regs();
        ads1299_soft_start_conversion();
        ads1299_check_id();
        ads1299_start_rdatac();
        ads1299_standby();
    #endif
    // Start execution.
    application_timers_start();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
		NRF_LOG_PRINTF(" BLE Advertising Start! %d \r\n",err_code);
		int32_t eeg1;
		int32_t eeg2;
		int32_t eeg3;
		int32_t eeg4;
    // Enter main loop.
    for (;;) {
				if(m_drdy) {
					m_drdy = false;
					//Acquire Data Samples
					get_eeg_voltage_samples(&eeg1, &eeg2, &eeg3, &eeg4);
					//Send 32-bit data samples to be organized into buffer 
					ble_eeg_update(&m_eeg, &eeg1, &eeg2, &eeg3, &eeg4);
				}
        power_manage();
    }
}

/**
 * @}
 */
