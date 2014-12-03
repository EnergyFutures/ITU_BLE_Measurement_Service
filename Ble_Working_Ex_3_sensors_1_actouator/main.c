/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
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
 * @defgroup ble_sdk_app_hts_main main.c
 * @{
 * @ingroup ble_sdk_app_hts
 * @brief Health Thermometer Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Health Thermometer service
 * It also includes the sample code for Battery and Device Information services.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "ble_bondmngr.h"
#include "nordic_common.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "sensor_service.h"
#include "simple_actuator_service.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "ble_stack_handler.h"
#include "app_timer.h"
#include "ble_error_log.h"
#include "ble_radio_notification.h"
#include "ble_flash.h"
#include "ble_debug_assert_handler.h"
#include "nrf_delay.h"
#include "dht22.h"
#include "app_gpiote.h"
#include "nrf_gpio.h"


//DIGITAL PINS
#define DHT22_PIN 2
#define SD_GATE_PIN_BIT BIT_3
#define SD_GATE_PIN 3
#define ACTUATOR_PIN 5
//ANALOG PINS
#define TMP36_PIN  ADC_CONFIG_PSEL_AnalogInput2
#define SD_AMP_PIN  ADC_CONFIG_PSEL_AnalogInput5

#define DEVICE_NAME                          "ITU SENSOR"			          /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                    "ITU 4D21"						                      /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                            "T1000"     													      /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                      0x1122334455                               /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                        0x667788                                   /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                     40                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS           180                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS                 5                                          /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE              6                                          /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(500, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.5 seconds) */
#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(1000, UNIT_1_25_MS)          /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                        0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                     MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of indication) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY        APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT         3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_TIMEOUT                    30                                         /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                       1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                       0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                        0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE               7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE               16                                         /**< Maximum encryption key size. */

#define FLASH_PAGE_SYS_ATTR                 (BLE_FLASH_PAGE_END - 3)                    /**< Flash page used for bond manager system attribute information. */
#define FLASH_PAGE_BOND                     (BLE_FLASH_PAGE_END - 1)                    /**< Flash page used for bond manager bonding information. */

#define DEAD_BEEF                            0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static uint16_t                              m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static ble_gap_sec_params_t                  m_sec_params;                              /**< Security requirements for this application. */
static ble_gap_adv_params_t                  m_adv_params;                              /**< Parameters to be passed to the stack when starting advertising. */
static iss_t						 										 iss_struct_1;															/**< Struct holding the ITU Sensor Service*/
static iss_t						 										 iss_struct_2;															/**< Struct holding the ITU Sensor Service*/
static iss_t						 										 iss_struct_3;															/**< Struct holding the ITU Sensor Service*/
static ias_t						 										 ias_struct_1;															/**< Struct holding the ITU Sensor Service*/
static uint8_t id =1;


/* Interrupt handler for ADC data ready event */
void ADC_IRQHandler(void)
{
	/* Clear dataready event */
  NRF_ADC->EVENTS_END = 0;	
	uint8_t pin = (NRF_ADC->CONFIG >> ADC_CONFIG_PSEL_Pos);
	

	if(pin == TMP36_PIN){
		// (result * (2/3 * 1,2vref) / 1024 (10 bit res))..  we multiply with 10 to avoid float
		int32_t temp = (((NRF_ADC->RESULT * 1800)/1024) - 500 ) * 10; 
		update_iss_measurement(&iss_struct_2, &temp);	
	}	else if (pin == SD_AMP_PIN){
		int32_t amplitude = ((NRF_ADC->RESULT * 3600)/1024) * 100; 
		update_iss_measurement(&iss_struct_3, &amplitude);	
	}
	
	//Use the STOP task to save current. Workaround for PAN_028 rev1.5 anomaly 1.
  NRF_ADC->TASKS_STOP = 1;
}	


/**@brief Function for error handling, which is called when an error has occurred. 
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
		
    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover on reset
    //NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the ADC.
 */static void adc_init(uint8_t pin,uint8_t scaling)
{		
	NRF_ADC->CONFIG	= (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) 										/* Analog reference inputs disabled. */
									| (pin << ADC_CONFIG_PSEL_Pos)																							/*!< Use PIN as analog input. */
									| (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)														/*!< Use internal 1.2V bandgap voltage as reference for conversion. */
									| (scaling << ADC_CONFIG_INPSEL_Pos)	/*!< Analog input specified by PSEL with 2/3 prescaler used as input for the conversion. */
									| (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos);																/*!< 10bit ADC resolution. */ 
	
	/* Enable ADC and set analog pin as input*/
	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
}

static void temp_adc_sampling_timeout_handler(void * p_context)
{        
	while(NRF_ADC->BUSY){;}
	adc_init(TMP36_PIN,ADC_CONFIG_INPSEL_AnalogInputTwoThirdsPrescaling);
	NRF_ADC->TASKS_START = 1;							//Start ADC sampling
}

static void take_sound_measurement(){
	while(NRF_ADC->BUSY){;}
	adc_init(SD_AMP_PIN,ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling);
	NRF_ADC->TASKS_START = 1;							//Start ADC sampling
}

static void sound_adc_sampling_timeout_handler(void * p_context)
{        
	take_sound_measurement();
}
/**@brief Will ask the DHT22 for a measurement
 *
 * @details Will ask the DHT22 component for a value.
 *
 * @warning Time between calls must be >2 sec
	*@param[in]   return_temp if true the temp will be returned, if no the humidity  
 * @return  The requested value multiplied with 100 to avoid float manipulation
 */
static int32_t get_dht22_measurement(bool return_temp){
		struct dht_data buffer;
		float t;
		uint32_t err_code = dht_read(DHT22_PIN, &buffer);
		if(err_code != 0){
			return INVALID_ITU_MEASUREMENT_VALUE;
		}
		if(return_temp){
			t = buffer.data[2] & 0x7F;
			t *= 256;
			t += buffer.data[3];
			if (buffer.data[2] & 0x80){
				t *= -10;
			}else{
				t *= 10;
			}	
			return (int32_t) t;
		}
		
		float h = buffer.data[0];
		h *= 256;
		h += buffer.data[1];
		h *= 10;
		return (int32_t) h;
}

static void start_measurement_timer(iss_t * iss_struct){
	uint32_t err_code = app_timer_start(iss_struct->meas_timer, APP_TIMER_TICKS(iss_struct->samp_freq_in_m_sec, APP_TIMER_PRESCALER), iss_struct);
	APP_ERROR_CHECK(err_code);
	iss_struct->timer_running = true;
}

static void update_measurement_samp_freq(iss_t * iss_struct){
	 if(iss_struct->timer_running){
		  uint32_t err_code;
			err_code = app_timer_stop(iss_struct->meas_timer);
			APP_ERROR_CHECK(err_code);
			start_measurement_timer(iss_struct);
	 }   
}

/**@brief Function for handling timer timeout for the ITS measurement.
 *
 * @details This function will be called each time the ITS measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void hum_value_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    int32_t humidity = 	get_dht22_measurement(false);
		update_iss_measurement(&iss_struct_1, &humidity);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;
    
    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timer to take temperature measurement every 10 sec
    err_code = app_timer_create(&iss_struct_1.meas_timer,APP_TIMER_MODE_REPEATED,hum_value_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&iss_struct_2.meas_timer,APP_TIMER_MODE_REPEATED,temp_adc_sampling_timeout_handler);
		APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&iss_struct_3.meas_timer,APP_TIMER_MODE_REPEATED,sound_adc_sampling_timeout_handler);
		APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_THERMOMETER);
    APP_ERROR_CHECK(err_code);
	    
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    
    ble_uuid_t adv_uuids[] = 
    {
        {BLE_UUID_ITU_MEASUREMENT_SERVICE, BLE_UUID_TYPE_BLE},  
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
    };
		
    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));
    
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;
		
    
    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

		
    // Initialize advertising parameters (used when starting advertising)
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    
    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;                           // Undirected advertisement
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
}

static void init_ISS_struct (iss_t * iss_struct, uint32_t * err_code){
		//Initialise ITU Temperature Service
		memset(iss_struct, 0, sizeof(*iss_struct));
		iss_struct->IEEE_exponent = ITU_IEEE_FLOAT32_DEFAULT_EXPONENT;
	
		iss_struct->curr_seq_nr = 0;
		iss_struct->coord = ITS_MEAS_LOCATION_NOT_SET;
		iss_struct->space_time_present = false;
	
		//org.bluetooth.unit.thermodynamic_temperature.degree_celsius
		//iss_struct->unit = 0x272F;
		iss_struct->unit = 0;
		iss_struct->unit_present = false;

		iss_struct->type = BLE_UUID_ITU_SENSOR_TYPE_NOT_SET;
		iss_struct->type = BLE_UUID_ITU_SENSOR_MAKE_NOT_SET;
		iss_struct->type_make_present = false;
	
		iss_struct->ID = id++;
		iss_struct->ID_present = false;
	
		//10 sec
		iss_struct->samp_freq_in_m_sec = 10000;
		iss_struct->samp_freq_present = false;

		iss_struct->p_update_samp_freq = update_measurement_samp_freq;
		iss_struct->gatt_db_needs_cleaning = false;
		
		*err_code = iss_initialize(iss_struct);
    
}

static void update_actuator(ias_t * ias_struct){
	if(ias_struct->status){
		nrf_gpio_pin_set(ACTUATOR_PIN);
	}else{
		nrf_gpio_pin_clear(ACTUATOR_PIN);
	}
}
/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the ITU Temperature Service and Device Information services.
 */
static void services_init(void)
{
    uint32_t         err_code;
    ble_dis_init_t   dis_init;
    ble_dis_sys_id_t sys_id;
    
		init_ISS_struct(&iss_struct_1,&err_code);
		APP_ERROR_CHECK(err_code);
	
		init_ISS_struct(&iss_struct_2,&err_code);
		APP_ERROR_CHECK(err_code);
	
		init_ISS_struct(&iss_struct_3,&err_code);
		APP_ERROR_CHECK(err_code);
		
		//Override default values
		iss_struct_3.IEEE_exponent = 0;
		iss_struct_3.samp_freq_in_m_sec = 1000; //1 sec
	
	
		ias_struct_1.status = false;
		ias_struct_1.p_update_status = update_actuator;
		ias_initialize(&ias_struct_1);
	
    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));
    
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str,     MODEL_NUM);

    sys_id.manufacturer_id            = MANUFACTURER_ID;
    sys_id.organizationally_unique_id = ORG_UNIQUE_ID;
    dis_init.p_sys_id                 = &sys_id;
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;  
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    // Start application timers
    start_measurement_timer(&iss_struct_1);
		nrf_delay_ms(100);
		start_measurement_timer(&iss_struct_2);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;    
    err_code = sd_ble_gap_adv_start(&m_adv_params);
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
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    //uint32_t err_code;
    
    //if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    //{
    //    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    //    APP_ERROR_CHECK(err_code);
    //}
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
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
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            
            m_conn_handle               = BLE_CONN_HANDLE_INVALID;
				
            // Since we are not in a connection and have not started advertising, store bonds
            err_code = ble_bondmngr_bonded_masters_store();
            APP_ERROR_CHECK(err_code);
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            if (p_ble_evt->evt.gatts_evt.params.timeout.src == BLE_GATT_TIMEOUT_SRC_PROTOCOL)
            {
                err_code = sd_ble_gap_disconnect(m_conn_handle,BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            }
            break;

        default:
            break;
    }

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
		iss_on_ble_evt(&iss_struct_1,p_ble_evt);
		iss_on_ble_evt(&iss_struct_2,p_ble_evt);
		iss_on_ble_evt(&iss_struct_3,p_ble_evt);
		ias_on_ble_evt(&ias_struct_1,p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_bondmngr_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
		//Modified to use the RFDUINO hf clk src to synth the lf clk src.
    BLE_STACK_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM,
                           BLE_L2CAP_MTU_DEF,
                           ble_evt_dispatch,
                           false);
}


/**@brief Function for handling a Bond Manager error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void bond_manager_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}



/**@brief Function for the Bond Manager initialization.
 */
static void bond_manager_init(void)
{
    uint32_t            err_code;
    ble_bondmngr_init_t bond_init_data;

    // Initialize the Bond Manager
    bond_init_data.flash_page_num_bond     = FLASH_PAGE_BOND;
    bond_init_data.flash_page_num_sys_attr = FLASH_PAGE_SYS_ATTR;
    bond_init_data.evt_handler             = NULL;
    bond_init_data.error_handler           = bond_manager_error_handler;
    bond_init_data.bonds_delete            = true;

    err_code = ble_bondmngr_init(&bond_init_data);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for initializing the Radio Notification events.
 */
static void radio_notification_init(void)
{
    uint32_t err_code;

    err_code = ble_radio_notification_init(NRF_APP_PRIORITY_HIGH,
                                           NRF_RADIO_NOTIFICATION_DISTANCE_4560US,
                                           ble_flash_on_radio_active_evt);
    APP_ERROR_CHECK(err_code);
}

static void sound_gate_handler(uint32_t pins_low_to_high, uint32_t pins_high_to_low){
		if(pins_high_to_low != 0){
			if(iss_struct_3.timer_running){
				app_timer_stop(iss_struct_3.meas_timer);
				iss_struct_3.timer_running = false;
			}
		} else if(pins_low_to_high != 0){
			if(!iss_struct_3.timer_running){
				take_sound_measurement();
				while(NRF_ADC->BUSY){;}
				start_measurement_timer(&iss_struct_3);
			}
		}
}

/**@brief Function for initializing the GPIOTE handler module.
 */
static void gpiote_init(void)
{
	//The GPIOTE users are responsible for configuring all their corresponding pins, except the SENSE field, which should be initialized to GPIO_PIN_CNF_SENSE_Disabled.
	//The SENSE field will be updated by the GPIOTE module when it is enabled or disabled, and also while it is enabled.
	GPIO_PIN_CONFIG(SD_GATE_PIN,                       \
                        GPIO_PIN_CNF_DIR_Input,       \
                        GPIO_PIN_CNF_INPUT_Connect,   \
                        GPIO_PIN_CNF_PULL_Pulldown,     \
                        GPIO_PIN_CNF_DRIVE_S0S1,      \
                        GPIO_PIN_CNF_SENSE_Disabled); 
		GPIO_LED_CONFIG(ACTUATOR_PIN);
    APP_GPIOTE_INIT(1);
		app_gpiote_user_id_t id;
		app_gpiote_user_register(&id,SD_GATE_PIN_BIT,SD_GATE_PIN_BIT,sound_gate_handler);
		app_gpiote_user_enable(id);
}

static void adc_first_time_init()
{	
	/* Enable interrupt on ADC sample ready event*/		
	NRF_ADC->INTENSET = ADC_INTENSET_END_Msk;   
	sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);  
	sd_nvic_EnableIRQ(ADC_IRQn);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
		dht_init(DHT22_PIN);		
    ble_stack_init();
		adc_first_time_init();
    bond_manager_init();
    gap_params_init();
    advertising_init();
    services_init();
		//Timer init function call must come after service init, or the timer id will be erased by memset
		timers_init();
    conn_params_init();
    sec_params_init();
    radio_notification_init();
    // Start execution.
    application_timers_start();
    advertising_start();	

		//Ask for temp just to warm up the device.. discard the result
		get_dht22_measurement(true);
		gpiote_init();
		// Enter main loop.
    for (;;)
    {
        uint32_t err_code = sd_app_event_wait();
				APP_ERROR_CHECK(err_code);
    }
}
