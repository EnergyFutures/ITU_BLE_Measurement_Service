/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "softdevice_handler.h"
#include "ble_error_log.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "ble_debug_assert_handler.h"
#include "nrf_delay.h"
#include "ble_dis.h"
#include "main.h"
#include "mote_config.h"
#include "read_all_sensor_service.h"

//Device Information Service
#define MANUFACTURER_NAME                    "DENMARK ITU 4D21"						               /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                            "BLEoT SENSOR"     												 /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                      ITU_COMPANY_ID                              /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                        ITU_COMPANY_ID                              /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */
#define DEAD_BEEF                       		 0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static uint16_t                         		 m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
bool do_measurements = true;

//STORAGE // CACHE
static pstorage_handle_t one_page_handle;
static uint8_t buffer_full = 0;

static storage_struct_t storage_struct = {
.pstorage_wait_flag = false,
.pstorage_clearing = false,
.pstorage_wait_handle = 0,
.current_block = 0,
.current_offset = 0,
.power_manage = power_manage,
.clear_cache = clear_cache
};



//BROADCAST / ADVERTISEMENT
#define MAX_BROADCAST_LENGTH 								 (31 - 3 - 2) 															 //31 = max bytes.. 3 = flags... 2= length + type for manuf_specific_data
static uint8_t encoded_broadcast[MAX_BROADCAST_LENGTH];
static uint8_t encoded_broadcast_length = 1;
static uint8_t broadcast_header = 0;
static uint8_t location_name_length;
static ble_advdata_t advdata;
static ble_gap_adv_params_t adv_params;
static uint8_t flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
static iss_broadc_t header = {
		.type_present = true,
		.misc_present = true,
		.value_present = true,
		.coor_present = true
};


// Persistent storage system event handler
void pstorage_sys_event_handler (uint32_t p_evt);

static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/* Interrupt handler for ADC data ready event */
void ADC_IRQHandler(void)
{
	/* Clear dataready event */
  NRF_ADC->EVENTS_END = 0;	
	uint8_t pin = (NRF_ADC->CONFIG >> ADC_CONFIG_PSEL_Pos);
	for(uint8_t i = 0; i < total_services_size; i++){
			if(all_services[i]->needs_adc && all_services[i]->adc_done != NULL){
					if(all_services[i]->adc_done(pin)){
						break;
					}
			}
	}
	//Use the STOP task to save current. Workaround for PAN_028 rev1.5 anomaly 1.
  NRF_ADC->TASKS_STOP = 1;
}	

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    //nrf_gpio_pin_set(ASSERT_LED_PIN_NO);

    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
   // ble_debug_assert_handler(error_code, line_num, p_file_name);
		while(true){;}
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




/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    for(uint8_t i = 0; i < total_services_size; i++){
				all_services[i]->timer_init();
		}
		irass_timer_init();
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

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void){
		uint8_t device_name_length = strlen(DEVICE_NAME);
		location_name_length = strlen(LOCATION_NAME);
		if((device_name_length + location_name_length) != 14){
			APP_ERROR_CHECK(777);	
		}
		if(location_name_length < 1){
			APP_ERROR_CHECK(777);
		}		
    		
    memset(&advdata, 0, sizeof(advdata));
		
    if (device_name_length > 0)
    {
        broadcast_header |= ITS_BROADC_FLAG_NAME_BIT;
				advdata.name_type               = BLE_ADVDATA_SHORT_NAME;
    }else{
				advdata.name_type               = BLE_ADVDATA_NO_NAME;
		}
		
		if (location_name_length > 0)
    {
        broadcast_header |= ITS_BROADC_FLAG_LOCATION_BIT;
				for(int y = 0; y < location_name_length; y++){
						encoded_broadcast[encoded_broadcast_length++] = ((uint8_t *)LOCATION_NAME)[y];
				}
    }
		
		if (header.type_present)
    {
        broadcast_header |= ITS_BROADC_FLAG_TYPE_BIT;
		}
		
    if (header.value_present)
    {
        broadcast_header |= ITS_BROADC_FLAG_VALUE_BIT;
    }    
		
    if (header.coor_present)
    {
        broadcast_header |= ITS_BROADC_FLAG_COORDINATE_BIT;
    }
	
    if (header.misc_present)
    {
				broadcast_header |= ITS_BROADC_FLAG_MISC_BIT;
		}			
		encoded_broadcast[0] = broadcast_header;
		
		advdata.flags.size              = sizeof(flags);
		advdata.flags.p_data            = &flags;
		advdata.include_appearance      = false;
		
		memset(&adv_params, 0, sizeof(adv_params));
		
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
		

		uint8_t APP_ADV_TIMEOUT_IN_SECONDS;
		if(total_services_size == 1){
			//THIS VALUE MAY BE OVERIDDEN IN services_init()
			APP_ADV_TIMEOUT_IN_SECONDS = 0;
		}else{
			APP_ADV_TIMEOUT_IN_SECONDS = 1;
		}   
    adv_params.interval    = MSEC_TO_UNITS(1100, UNIT_0_625_MS);
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
}

static void its_broadcast_encode_and_set(void * p_itu_service_struct, uint8_t type){
		
		encoded_broadcast_length = 1 + location_name_length;
		bool is_actuator = type;
    if (header.type_present)
    {
			encoded_broadcast[encoded_broadcast_length++] = is_actuator ? ((ias_t *)p_itu_service_struct)->type : ((iss_t *)p_itu_service_struct)->type;
		}
		
    if (header.value_present)
    {
				encoded_broadcast_length += uint32_encode(is_actuator ? ((ias_t *)p_itu_service_struct)->value : ((iss_t *)p_itu_service_struct)->last_measurement, &encoded_broadcast[encoded_broadcast_length]);
    }    
		
    if (header.coor_present)
    {
				encoded_broadcast[encoded_broadcast_length++] = is_actuator ? ((ias_t *)p_itu_service_struct)->coord : ((iss_t *)p_itu_service_struct)->coord;
    }
		
		if(storage_struct.current_block > BLOCK_COUNT_PROCENT && buffer_full != 1){
			buffer_full = 1;
			sd_ble_gap_tx_power_set(CONNECTABLE_TRANSMIT_POWER_DB);
		}
	
		//MISC (xxxxyyyz) = xxxx for battery procentage in increments of 10, yyy for service type (sensor or actuator or mix), z for buffer full
    if (header.misc_present)
    {
				uint8_t misc = 0x00;
							
				misc |= buffer_full;
				
				misc |= (is_actuator << 1);
				
				//Hardcoded 90 procent battery
				misc |= (0x09 << 4);
				encoded_broadcast[encoded_broadcast_length++] = misc;
		}			
		
		uint8_array_t data;
		data.size = encoded_broadcast_length;
		data.p_data = encoded_broadcast;
		
		ble_advdata_manuf_data_t ble_data;
		memset(&ble_data, 0, sizeof(data));
		ble_data.company_identifier = ITU_COMPANY_ID;
		ble_data.data = data;		
    advdata.p_manuf_specific_data = &ble_data;
		
		if(buffer_full || actuators_size > 0){
				adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
		}else{
				adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
		}
		
		uint32_t      err_code = 0;		
		err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
}

static void advertising_start(void){
	static uint8_t current_service_nr = 0;
	if(current_service_nr == total_services_size ){
			current_service_nr = 0;
	}	
	its_broadcast_encode_and_set(all_services[current_service_nr]->service,all_services[current_service_nr]->service_type);
	current_service_nr++;
}

static void advertising_start_sche(void *data, uint16_t size){
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
	advertising_start();
}



static void clear_cache_sche1(void *data, uint16_t size){
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);	
		storage_struct.current_block = 0;
		storage_struct.current_offset = 0;
		storage_struct.pstorage_clearing = true;
		pstorage_clear(&storage_struct.pstorage_handle,(BLOCK_SIZE*BLOCK_COUNT)/2);		
}

static void clear_cache_sche2(void *data, uint16_t size){
		UNUSED_PARAMETER(data);
		UNUSED_PARAMETER(size);
		pstorage_block_identifier_get(&storage_struct.pstorage_handle,BLOCK_COUNT/2, &one_page_handle);
		storage_struct.pstorage_wait_flag = true;
		storage_struct.pstorage_wait_handle = one_page_handle.block_id;
		storage_struct.pstorage_clearing = true;
		pstorage_clear(&one_page_handle,(BLOCK_SIZE*BLOCK_COUNT)/2);
}

static void clear_cache(void){
		app_sched_event_put(NULL, 0, clear_cache_sche1);			
		app_sched_event_put(NULL, 0, clear_cache_sche2);
		sd_ble_gap_tx_power_set(NONCONNECTABLE_TRANSMIT_POWER_DB);
		buffer_full = false;
}

static void persist_measurement(iss_t * iss_struct){
	//only update if we are not connected
	if((m_conn_handle == BLE_CONN_HANDLE_INVALID) && (!storage_struct.pstorage_clearing) && (storage_struct.current_block < BLOCK_COUNT)){
			uint32_t error_code;	
			pstorage_handle_t block_handle;
			error_code = pstorage_block_identifier_get(&storage_struct.pstorage_handle, storage_struct.current_block, &block_handle);
			APP_ERROR_CHECK(error_code);
			uint8_t cache_val[8];
			uint32_encode(iss_struct->last_measurement,cache_val);
			uint16_encode(iss_struct->curr_seq_nr,&(cache_val[4]));
			uint16_encode(iss_struct->ID,&(cache_val[6]));
			while(storage_struct.pstorage_wait_flag){
				power_manage();
			}
			storage_struct.pstorage_wait_flag = true;
			storage_struct.pstorage_wait_handle = block_handle.block_id;			
			error_code = pstorage_store(&block_handle, cache_val, 8, storage_struct.current_offset);
			APP_ERROR_CHECK(error_code);
			if(storage_struct.current_offset > 0){
					storage_struct.current_block++;
					storage_struct.current_offset = 0;
			}else{
					storage_struct.current_offset = 8;
			}
	}
}

static void init_ISS_struct (itu_service_t * iss_struct, uint32_t * err_code){
		//Initialise ITU Temperature Service
		iss_t * service = iss_struct->service;
		memset(service, 0, sizeof(iss_t));
		service->IEEE_exponent = ITU_IEEE_FLOAT32_DEFAULT_EXPONENT;
	
		service->curr_seq_nr = 0;
		service->coord = ITS_MEAS_LOCATION_NOT_SET;
		service->space_time_present = false;
	
		//org.bluetooth.unit.thermodynamic_temperature.degree_celsius
		//iss_struct->unit = 0x272F;
		service->unit = 0;
		service->unit_present = false;

		service->type_make_present = false;
	
		service->ID = 4;
		service->ID_present = false;
	
		service->samp_freq_in_m_sec = DEFAULT_SAMPLING_FREQUENCY_IN_MS;
		service->samp_freq_present = false;

		// EACH SENSOR SHOULD CONFIGURE ITS SAMPLING FREQUENCY UPDATE FUNCTION IN THE INIT HOOK
		//iss_struct->p_update_samp_freq = update_measurement_samp_freq;
		
		service->gatt_db_needs_cleaning = false;
		service->p_persist_measurement = persist_measurement;
		
		*err_code = iss_initialize(service);
    
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
	
		for(uint8_t i = 0; i < sensors_size; i++){
			init_ISS_struct(sensors[i],&err_code);
			APP_ERROR_CHECK(err_code);
		}
		
		//FIXME what if we change the freq in the future
		if(total_services_size == 1 && sensors_size == 1){
			iss_t * service = sensors[0]->service;
			uint16_t sam_freq = service->samp_freq_in_m_sec / 900; //900 ensures that we time out after the samp freq
			adv_params.timeout    = sam_freq > 0x3fff ? 0x3fff : sam_freq;  // Advertising timeout between 0x0001 and 0x3FFF in seconds, 0x0000 disables timeout.
		}
		
		for(uint8_t i = 0; i < actuators_size; i++){
			ias_t * service = actuators[i]->service;
			memset(service, 0, sizeof(ias_t));
			err_code =  ias_initialize(service);
			APP_ERROR_CHECK(err_code);
		}
		
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
		
		err_code = irass_initialize(&storage_struct);
		APP_ERROR_CHECK(err_code);
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
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
		for(uint8_t i = 0; i < total_services_size; i++){
				all_services[i]->timer_start();
				nrf_delay_ms(50);
		}		
}



/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    //static ble_gap_evt_auth_status_t m_auth_status;
    //ble_gap_enc_info_t *             p_enc_info;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
						m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle               = BLE_CONN_HANDLE_INVALID;
            app_sched_event_put(NULL, 0, advertising_start_sche);
            break;

        /*case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &m_sec_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;

        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            p_enc_info = &m_auth_status.periph_keys.enc_info;
            if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;*/

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gatts_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {
                app_sched_event_put(NULL, 0, advertising_start_sche);
            }
            break;
				case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server and Client timeout events.
            err_code = sd_ble_gap_disconnect(m_conn_handle,BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
						app_sched_event_put(NULL, 0, advertising_start_sche);
            break;
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
		for(uint8_t i = 0; i < total_services_size; i++){
				all_services[i]->ble_evt(p_ble_evt);
		}
		irass_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_4000MS_CALIBRATION, true);

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = 0;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void){
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

static void adc_first_time_init(){	
	for(uint8_t i = 0; i < total_services_size; i++){
		if(all_services[i]->needs_adc){
			NRF_ADC->INTENSET = ADC_INTENSET_END_Msk;   
			sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);  
			sd_nvic_EnableIRQ(ADC_IRQn);
			break;
		}
	}	
}

static void init_sensors_and_actuators(void){
	for(uint8_t i = 0; i < total_services_size; i++){
		all_services[i]->init();
	}
}

static void init_timer_module(void){
		APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);
}

static void storage_handler(pstorage_handle_t  * handle, uint8_t op_code, uint32_t result, uint8_t * p_data, uint32_t data_len){

		//If we are waiting for this callback, clear the wait flag.	
		if(handle->block_id == storage_struct.pstorage_wait_handle) {
				storage_struct.pstorage_wait_flag = false;
		}  
		if(PSTORAGE_CLEAR_OP_CODE == op_code && handle->block_id == one_page_handle.block_id){
			storage_struct.pstorage_clearing = false;
			//WE ONLY ENABLE MEASUREMENTS ONCE ALL CACHE IS CLEARED
			do_measurements	= true;
		}
		if(result != NRF_SUCCESS){
			APP_ERROR_CHECK(result);
		}
}


static void initStorage(void){
	uint32_t err_code;
	err_code = pstorage_init();
	APP_ERROR_CHECK(err_code);
	pstorage_module_param_t param;
	param.block_size = BLOCK_SIZE;
	param.block_count = BLOCK_COUNT;
	param.cb = storage_handler;
	err_code = pstorage_register(&param, &storage_struct.pstorage_handle);
	APP_ERROR_CHECK(err_code);
	clear_cache();
}

static void registerServices(void){
	registerSensors();
	registerActuators();
	uint8_t i = 0;
	for(; i < (total_services_size - actuators_size); i++){
		all_services[i] = sensors[i];
	}
	for(uint8_t j = 0; j < (total_services_size - sensors_size); j++){
		all_services[i++] = actuators[j];
	}
}

void gpiote_event_handler (uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
	for(uint8_t i = 0; i < total_services_size; i++){
		if(all_services[i]->needs_gpiote){
			all_services[i]->on_gpiote_event(&event_pins_low_to_high,&event_pins_high_to_low);
		}
	}
}

static void init_gpiote(){
	uint32_t low_to_high_bitmask = 0;
	uint32_t high_to_low_bitmask = 0;
	app_gpiote_user_id_t m_example_user_id;
	for(uint8_t i = 0; i < total_services_size; i++){
		if(all_services[i]->needs_gpiote){
			all_services[i]->gpiote_init(&low_to_high_bitmask,&high_to_low_bitmask);
		}
	}
	
	if(low_to_high_bitmask != 0 || high_to_low_bitmask != 0){
		APP_GPIOTE_INIT(1);
		uint32_t retval;
		retval = app_gpiote_user_register(&m_example_user_id,low_to_high_bitmask,high_to_low_bitmask,gpiote_event_handler);
		APP_ERROR_CHECK(retval);
		retval = app_gpiote_user_enable(m_example_user_id);
		APP_ERROR_CHECK(retval);
	}
}

int main(void)
{
		
		registerServices();
		init_timer_module();
    ble_stack_init();
		scheduler_init();
		initStorage();
		gap_params_init();
    advertising_init();
		conn_params_init();
		services_init();
		//Timer init function call must come after service init, or the timer id will be erased by memset
		timers_init();
    adc_first_time_init();
		init_sensors_and_actuators();
		
    // Start execution.
		application_timers_start();				
    advertising_start();	
		init_gpiote();
    for (;;)
    {
				app_sched_execute();
        power_manage();
    }
}
