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
#include <ble_types.h>
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
#include "mote_service.h"

#define DEAD_BEEF                       		 0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
bool do_measurements = true;
static uint16_t                         		 m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

//STORAGE // CACHE
static uint8_t cache_val[HALF_BLOCK_SIZE] __attribute__((aligned(4)));
#if(SENSORS_SIZE > 0)
static uint8_t config_cache_val[CONFIG_PERSIST_BLOCK_SIZE] __attribute__((aligned(4)));
#endif
static uint8_t buffer_full = 0;
static uint8_t battery_level = 10;
static uint16_t batt_counter = 0;


static storage_struct_t storage_struct = {
	.pstorage_wait_flag = false,
	.pstorage_clearing = false,
	.pstorage_wait_handle = 0,
	.current_block = 0,
	.current_offset = 0,
	.current_off_loading_block = 0,
	.clear_cache = clear_cache,
	.persist_config = persist_config
};

static mote_config_struct_t mote_config = {
	.device_name = DEVICE_NAME,
	.location_name = LOCATION_NAME,
	.adv_freq_sec = DEFAULT_ADVERTISEMENT_FREQUENCY_IN_SEC,
	.block_count_percent_for_buffer_full = BLOCK_COUNT_PROCENT,
	.non_conn_trans_power = NONCONNECTABLE_TRANSMIT_POWER_DB,
	.conn_trans_power = CONNECTABLE_TRANSMIT_POWER_DB
};


//BROADCAST / ADVERTISEMENT
#define MAX_BROADCAST_LENGTH 								 (31 - 3 - 2) 															 //31 = max bytes.. 3 = flags... 2= length + type for manuf_specific_data
static uint8_t encoded_broadcast[MAX_BROADCAST_LENGTH];
static uint8_t location_name_length;
static uint8_t device_name_length;
static ble_advdata_t advdata;
static ble_gap_adv_params_t adv_params;


// Persistent storage system event handler
void pstorage_sys_event_handler (uint32_t p_evt);

static void power_manage(void)
{
		app_sched_execute();
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/* Interrupt handler for ADC data ready event */
void ADC_IRQHandler(void)
{
	/* Clear dataready event */
  NRF_ADC->EVENTS_END = 0;	
	uint8_t pin = (NRF_ADC->CONFIG >> ADC_CONFIG_PSEL_Pos);
	if(pin == 0){
		battery_level = (NRF_ADC->RESULT / 1024.0) * 100.0; 
		battery_level = 100 - battery_level;
	}else{
		for(uint8_t i = 0; i < total_services_size; i++){
			if(all_services[i]->needs_adc && all_services[i]->adc_done != NULL){
					if(all_services[i]->adc_done(pin)){
						break;
					}
			}
		}
	}
	
	//Use the STOP task to save current. Workaround for PAN_028 rev1.5 anomaly 1.
  NRF_ADC->TASKS_STOP = 1;
	//Disable ADC to save current
	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;
}	

static void battery_level_get(void)
{
	adc_init(ADC_CONFIG_PSEL_Disabled,ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling);
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
		ble_debug_assert_handler(error_code, line_num, p_file_name);
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
		ims_timer_init();
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
    
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
		sd_ble_gap_tx_power_set(mote_config.non_conn_trans_power);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
void advertising_init(){
		device_name_length = strlen(mote_config.device_name);
		uint8_t device_name_bytes = device_name_length ? 2 : 0;
		location_name_length = strlen(mote_config.location_name);
		if((device_name_length + device_name_bytes + location_name_length) > 13){
			APP_ERROR_CHECK(777);	
		}
		if(location_name_length < 1){
			APP_ERROR_CHECK(777);
		}		
    		
    memset(&advdata, 0, sizeof(advdata));
		
    if (device_name_length > 0)
    {
				advdata.name_type               = BLE_ADVDATA_SHORT_NAME;
				ble_gap_conn_sec_mode_t sec_mode;
				BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
				uint32_t err_code = sd_ble_gap_device_name_set(&sec_mode,(const uint8_t *)mote_config.device_name,device_name_length);
				APP_ERROR_CHECK(err_code);
    }else{
				advdata.name_type               = BLE_ADVDATA_NO_NAME;
		}
		
		for(int i = 0; i < location_name_length; i++){
				encoded_broadcast[i] = mote_config.location_name[i];
		}
		encoded_broadcast[location_name_length] = '\0';
			
		advdata.include_appearance      = false;
		
		memset(&adv_params, 0, sizeof(adv_params));
		
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
		

		uint16_t timeout_in_sec = mote_config.adv_freq_sec;
		uint16_t interval = mote_config.adv_freq_sec * 1050;
		if(SENSORS_SIZE == 1 && total_services_size == 1){
			#if(SENSORS_SIZE > 0)
			iss_t * service = sensors[0]->service;
			uint16_t sam_freq = service->samp_freq_in_m_sec / 1000;
			timeout_in_sec  = sam_freq >= 0x3fff ? 0x3fff : sam_freq+1;  // Advertising timeout between 0x0001 and 0x3FFF in seconds, 0x0000 disables timeout.
			#endif
		} 
		if(ACTUATORS_SIZE > 0){
			interval = 250;
			if(ACTUATORS_SIZE == 1 && total_services_size == 1){
				timeout_in_sec = 0;
			}
		}
    adv_params.interval    = MSEC_TO_UNITS(interval, UNIT_0_625_MS);
    adv_params.timeout     = timeout_in_sec;
}

static void its_broadcast_encode_and_set(void * p_itu_service_struct, uint8_t type){
		uint8_t encoded_broadcast_length = location_name_length+1;
		bool is_actuator = type;
    if(storage_struct.current_block > ((BLOCK_COUNT / 100 ) * mote_config.block_count_percent_for_buffer_full) && buffer_full != 1){
			buffer_full = 1;
			sd_ble_gap_tx_power_set(mote_config.conn_trans_power);
		}
		
		//BUFFER LEVEL
		encoded_broadcast[encoded_broadcast_length++] = (storage_struct.current_block * 100 ) / BLOCK_COUNT;
	
		//MISC (zzzzzzxy) = zzzzzz for battery diff (100 - level), x for buffer-full, y for service type (sensor or actuator)
    uint8_t misc = 0x00;
		misc |= is_actuator;
		misc |= buffer_full << 1;
				
		if(batt_counter++ == NUMBER_OF_SEC_FOR_BATTERY_CHECK){
			batt_counter = 0;
			battery_level_get();
		}
		misc |= (battery_level << 2);
		encoded_broadcast[encoded_broadcast_length++] = misc;	
		if(is_actuator){
			ias_t * pointer = (ias_t *)p_itu_service_struct;
			encoded_broadcast[encoded_broadcast_length++] = pointer->type ;
			encoded_broadcast_length += uint32_encode(pointer->value, &encoded_broadcast[encoded_broadcast_length]);
			encoded_broadcast[encoded_broadcast_length++] = pointer->coord ;
			encoded_broadcast_length += uint16_encode(0, &encoded_broadcast[encoded_broadcast_length]);
		}else{
			iss_t * pointer = (iss_t *)p_itu_service_struct;
			encoded_broadcast[encoded_broadcast_length++] = pointer->type;
			encoded_broadcast_length += uint32_encode(pointer->last_measurement, &encoded_broadcast[encoded_broadcast_length]);
			encoded_broadcast[encoded_broadcast_length++] = pointer->coord;
			encoded_broadcast_length += uint16_encode(pointer->ID, &encoded_broadcast[encoded_broadcast_length]);
		}
		uint8_array_t data;
		data.size = encoded_broadcast_length;
		data.p_data = encoded_broadcast;
		
		ble_advdata_manuf_data_t ble_data;
		memset(&ble_data, 0, sizeof(data));
		ble_data.company_identifier = ITU_COMPANY_ID;
		ble_data.data = data;		
    advdata.p_manuf_specific_data = &ble_data;
		
		if(buffer_full || ACTUATORS_SIZE > 0 || (strcmp(mote_config.device_name,DEVICE_NAME) == 0)){
				adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
		}else{
				adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
		}
		
		uint32_t      err_code = 0;	
		uint8_t flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
		advdata.flags.size              = sizeof(flags);
		advdata.flags.p_data            = &flags;		
		err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
}

static void advertising_start(void *data, uint16_t size){
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
	static uint8_t current_service_nr = 0;
	if(current_service_nr == total_services_size ){
			current_service_nr = 0;
	}	
	its_broadcast_encode_and_set(all_services[current_service_nr]->service,all_services[current_service_nr]->service_type);
	current_service_nr++;
}

static void init_ISS_struct (itu_service_t * iss_struct, uint32_t * err_code, uint16_t id){
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
	
		service->ID = id;
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
		uint16_t id = START_ID;

		#if(SENSORS_SIZE > 0)
		for(uint8_t i = 0; i < SENSORS_SIZE; i++){
			init_ISS_struct(sensors[i],&err_code,id++);
			APP_ERROR_CHECK(err_code);
		}
		#endif
		#if(ACTUATORS_SIZE > 0)
		for(uint8_t i = 0; i < ACTUATORS_SIZE; i++){
			ias_t * service = actuators[i]->service;
			memset(service, 0, sizeof(ias_t));
			err_code =  ias_initialize(service);
			APP_ERROR_CHECK(err_code);
		}
		#endif
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
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void *data, uint16_t use_offset)
{
		UNUSED_PARAMETER(data);
		if(use_offset){
			//If we are in this state, then we just cleaned the cache and wish to continue as fast as possible
			//But to avoid to have to many consecutive time-outs we let the timers start with an offset
			for(uint8_t i = 0; i < total_services_size; i++){
				all_services[i]->timer_start((i+1) * 100);			
			}		
		}else{
		for(uint8_t i = 0; i < total_services_size; i++){
				all_services[i]->timer_start(0);
				nrf_delay_ms(200); // so we don't get to many time outs in one go
			}		
		}		
}

static void application_timers_stop(void *data, uint16_t size)
{
		UNUSED_PARAMETER(data);
		UNUSED_PARAMETER(size);
		for(uint8_t i = 0; i < total_services_size; i++){
				all_services[i]->timer_stop();
		}		
}

static void reestablish_system(void *data, uint16_t size){
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
	uint32_t err_code = app_sched_event_put(NULL, 0, advertising_start);
	APP_ERROR_CHECK(err_code);
	if(!storage_struct.pstorage_clearing){
		do_measurements = true; //has to come before we start timers
		err_code = app_sched_event_put(NULL, 1, application_timers_start);
		APP_ERROR_CHECK(err_code);
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
						do_measurements = false;
						m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						err_code = app_sched_event_put(NULL, 0, application_timers_stop);
						APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle               = BLE_CONN_HANDLE_INVALID;
						err_code = app_sched_event_put(NULL, 0, reestablish_system);
						APP_ERROR_CHECK(err_code);
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
               err_code = app_sched_event_put(NULL, 0, advertising_start);
							 APP_ERROR_CHECK(err_code);
            }
            break;
				case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server and Client timeout events.
            err_code = sd_ble_gap_disconnect(m_conn_handle,BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
						err_code = app_sched_event_put(NULL, 0, advertising_start);
						APP_ERROR_CHECK(err_code);
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
		ims_on_ble_evt(p_ble_evt);
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
		
		//ble_uuid128_t ITU_BASE_UUID = {0x21, 0x4E, 0x49, 0x57, 0x45, 0x48, 0x54, 0x34, 0x31, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		//err_code = sd_ble_uuid_vs_add(&ITU_BASE_UUID, &ITU_UUID_TYPE);
		//APP_ERROR_CHECK(err_code);
		
		// WE HAVE TO USE STANDARD UUID OR WE GET PROBLEMS WITH MEM
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

static void clear_cache(void *data, uint16_t startup_clean){
		UNUSED_PARAMETER(data);	
		buffer_full = false;
		storage_struct.current_off_loading_block = 0;
		storage_struct.current_block = 0;
		storage_struct.current_offset = 0;	
		storage_struct.pstorage_clearing = !startup_clean;
		pstorage_clear(&storage_struct.pstorage_handle,BLOCK_COUNT*BLOCK_SIZE);
}

static void persist_measurement(iss_t * iss_struct){
	//only update if we are not connected
	if((m_conn_handle == BLE_CONN_HANDLE_INVALID) && (!storage_struct.pstorage_clearing) && (storage_struct.current_block < BLOCK_COUNT)){
		while(storage_struct.pstorage_wait_flag){
			power_manage();
		}
		uint32_t error_code;	
		pstorage_handle_t block_handle;
		error_code = pstorage_block_identifier_get(&storage_struct.pstorage_handle, storage_struct.current_block, &block_handle);
		APP_ERROR_CHECK(error_code);
		
		uint32_encode(iss_struct->last_measurement,cache_val);
		uint16_encode(iss_struct->curr_seq_nr-1,&(cache_val[SEQUENCE_NUMBER_OFFSET]));
		uint16_encode(iss_struct->ID,&(cache_val[ID_OFFSET]));
		
		storage_struct.pstorage_wait_flag = true;
		storage_struct.pstorage_wait_handle = block_handle.block_id;			
		error_code = pstorage_store(&block_handle, cache_val, HALF_BLOCK_SIZE, storage_struct.current_offset);
		APP_ERROR_CHECK(error_code);
		if(storage_struct.current_offset > 0){
				storage_struct.current_block++;
				storage_struct.current_offset = 0;
		}else{
				storage_struct.current_offset = HALF_BLOCK_SIZE;
		}
	}
}

#if(SENSORS_SIZE > 0)
static void read_config(void){
	uint32_t error_code;	
	pstorage_handle_t block_handle;			
	error_code = pstorage_block_identifier_get(&storage_struct.config_handle, 0, &block_handle);		
	APP_ERROR_CHECK(error_code);
	error_code = pstorage_load(config_cache_val, &block_handle, CONFIG_PERSIST_BLOCK_SIZE, 0);
	APP_ERROR_CHECK(error_code);
	if(config_cache_val[0] == CONFIG_MAGIC_1 && config_cache_val[1] == CONFIG_MAGIC_2){
		uint8_t i = 2;
		for(int j = 0; j <=MAXIMUM_LOCATION_DEVICE_NAME_LENGTH; j++){
			mote_config.device_name[j] = config_cache_val[i+j];
			if(mote_config.device_name[j] == '\0'){
				i += j+1;
				break;
			}
		}
		for(int j = 0; j <=MAXIMUM_LOCATION_DEVICE_NAME_LENGTH; j++){
			mote_config.location_name[j] = config_cache_val[i+j];
			if(mote_config.location_name[j] == '\0'){
				i += j+1;
				break;
			}
		}
		mote_config.adv_freq_sec = config_cache_val[i++];
		mote_config.block_count_percent_for_buffer_full = config_cache_val[i++];
		mote_config.conn_trans_power = config_cache_val[i++];
		mote_config.non_conn_trans_power = config_cache_val[i++];
		for(int j = 0; j <SENSORS_SIZE; j++){
			iss_t* service = (iss_t*) sensors[j]->service;
			service->ID = uint16_decode(&config_cache_val[i]);
			i += 2;
			service->coord = config_cache_val[i++];
			service->samp_freq_in_m_sec = uint32_decode(&config_cache_val[i]);
			i += 4;
			iss_update_config(service);
		}
		sd_ble_gap_tx_power_set(mote_config.non_conn_trans_power);
	}
}
#endif

static void persist_config(void){
	uint32_t error_code;	
	storage_struct.config_clearing = true;
	error_code = pstorage_clear(&storage_struct.config_handle,CONFIG_PERSIST_BLOCK_SIZE);
	APP_ERROR_CHECK(error_code);
}

static void storage_handler(pstorage_handle_t  * handle, uint8_t op_code, uint32_t result, uint8_t * p_data, uint32_t data_len){

	//If we are waiting for this callback, clear the wait flag.	
	if(handle->block_id == storage_struct.pstorage_wait_handle) {
			storage_struct.pstorage_wait_flag = false;
	}else if(PSTORAGE_CLEAR_OP_CODE == op_code && storage_struct.pstorage_clearing){
		storage_struct.pstorage_clearing = false;
		//WE ONLY ENABLE MEASUREMENTS ONCE ALL CACHE IS CLEARED
		sd_ble_gap_tx_power_set(mote_config.non_conn_trans_power);
		for(uint8_t i = 0; i < total_services_size; i++){
			itu_service_t *service = all_services[i];
			if(service->service_type == 0){
				iss_t * iss = (iss_t *) service->service;
				iss->curr_seq_nr = 0;
			}
		}
		do_measurements	= true;
		uint32_t err_code = app_sched_event_put(NULL, 1, application_timers_start);
		APP_ERROR_CHECK(err_code);
	}
	APP_ERROR_CHECK(result);
}

static void reset_system_sche(void *data, uint16_t size){
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
	uint32_t count;
	pstorage_access_status_get(&count);
	if(count != 0){
		app_sched_event_put(NULL, 0, reset_system_sche);
		return;
	}
	sd_nvic_SystemReset();
}

static void continue_config_persist(void *data, uint16_t size){
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
	storage_struct.config_clearing = false;
	pstorage_handle_t block_handle;
	uint32_t error_code = pstorage_block_identifier_get(&storage_struct.config_handle, 0, &block_handle);
	APP_ERROR_CHECK(error_code);
	uint8_t index   = 0;
	config_cache_val[index++] = CONFIG_MAGIC_1;
	config_cache_val[index++] = CONFIG_MAGIC_2;
	for(int i = 0; i < strlen(mote_config.device_name);i++){
		config_cache_val[index++] = mote_config.device_name[i];
	}
	config_cache_val[index++] = '\0';
	for(int i = 0; i < strlen(mote_config.location_name);i++){
		config_cache_val[index++] = mote_config.location_name[i];
	}
	config_cache_val[index++] = '\0';
	config_cache_val[index++] = mote_config.adv_freq_sec;
	config_cache_val[index++] = mote_config.block_count_percent_for_buffer_full;
	config_cache_val[index++] = mote_config.conn_trans_power;
	config_cache_val[index++] = mote_config.non_conn_trans_power;
	for(int j = 0; j <SENSORS_SIZE; j++){
			iss_t* service = (iss_t*) sensors[j]->service;
			index += uint16_encode(service->ID,&(config_cache_val[index]));
			config_cache_val[index++] = service->coord;
			index += uint32_encode(service->samp_freq_in_m_sec,&(config_cache_val[index]));
	}
	for(; index <CONFIG_PERSIST_BLOCK_SIZE; index++){
		config_cache_val[index] = 0;
	}
	storage_struct.config_wait_handle = block_handle.block_id;
	error_code = pstorage_store(&block_handle, config_cache_val, CONFIG_PERSIST_BLOCK_SIZE, 0);
	APP_ERROR_CHECK(error_code);	
}

#if(SENSORS_SIZE > 0)
static void config_storage_handler(pstorage_handle_t  * handle, uint8_t op_code, uint32_t result, uint8_t * p_data, uint32_t data_len){
	if(PSTORAGE_CLEAR_OP_CODE == op_code && storage_struct.config_clearing){
		app_sched_event_put(NULL, 0, continue_config_persist);
	}else if(handle->block_id == storage_struct.config_wait_handle) {
		app_sched_event_put(NULL, 0, reset_system_sche);
	}	
	APP_ERROR_CHECK(result);
}
#endif

static void initStorage(void){
	uint32_t err_code;
	err_code = pstorage_init();
	APP_ERROR_CHECK(err_code);
	pstorage_module_param_t param1;
	param1.block_size = BLOCK_SIZE;
	param1.block_count = BLOCK_COUNT;
	param1.cb = storage_handler;
	err_code = pstorage_register(&param1, &storage_struct.pstorage_handle);
	APP_ERROR_CHECK(err_code);
	#if(SENSORS_SIZE > 0)
	pstorage_module_param_t param2;
	param2.block_size = CONFIG_PERSIST_BLOCK_SIZE;
	param2.block_count = 1;
	param2.cb = config_storage_handler;
	err_code = pstorage_register(&param2, &storage_struct.config_handle);
	APP_ERROR_CHECK(err_code);
	#endif
	for(int i = 0; i < CONFIG_PERSIST_BLOCK_SIZE; i++){
		config_cache_val[i] = 0;
	}
	clear_cache(NULL,1);
}

static void registerServices(void){
	#if(SENSORS_SIZE > 0)
	registerSensors();
	#endif
	#if(ACTUATORS_SIZE > 0)
	registerActuators();
	#endif
	uint8_t i = 0;
	#if(SENSORS_SIZE > 0)
	for(; i < (total_services_size - ACTUATORS_SIZE); i++){
		all_services[i] = sensors[i];
	}
	#endif
	#if(ACTUATORS_SIZE > 0)
	for(uint8_t j = 0; j < (total_services_size - SENSORS_SIZE); j++){
		all_services[i++] = actuators[j];
	}
	#endif
}
void handle_config_pin_push(void *data, uint16_t size)
{
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
	strcpy(mote_config.location_name,LOCATION_NAME_CONFIG);
	uint32_t err_code;
	err_code = sd_ble_gap_adv_stop();
	APP_ERROR_CHECK(err_code);
	advertising_init();
	err_code = app_sched_event_put(NULL, 0, advertising_start);
	APP_ERROR_CHECK(err_code);
}

void gpiote_event_handler (uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low){
	//
	if((event_pins_high_to_low & (1 << BUTTON_PIN)) && (strcmp(mote_config.device_name,DEVICE_NAME) == 0) ){
		if(strcmp(mote_config.location_name,LOCATION_NAME_CONFIG) == 0){
			return;
		}
		uint32_t err_code = app_sched_event_put(NULL, 0, handle_config_pin_push);
		APP_ERROR_CHECK(err_code);
	}
	if(do_measurements){
		for(uint8_t i = 0; i < total_services_size; i++){
			if(all_services[i]->needs_gpiote){
				all_services[i]->on_gpiote_event(&event_pins_low_to_high,&event_pins_high_to_low);
			}
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
	if(strcmp(mote_config.device_name,DEVICE_NAME) == 0){
		NRF_GPIO->PIN_CNF[BUTTON_PIN] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                        | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                        | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)
                                        | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                        | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
		high_to_low_bitmask |= (1 << BUTTON_PIN);
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

static void init_mote_service(void){
	APP_ERROR_CHECK(ims_initialize(&storage_struct,&mote_config));
}

int main(void)
{
		
		registerServices();
		init_timer_module();
    ble_stack_init();
		scheduler_init();
		initStorage();
		conn_params_init();
		services_init();
		gap_params_init();
		//Timer init function call must come after service init, or the timer id will be erased by memset
		timers_init();
    adc_first_time_init();
		init_sensors_and_actuators();
		#if(SENSORS_SIZE > 0)
		//must come after services_init()
		read_config();
		#endif
		//must come after read_config()
		init_mote_service();
		advertising_init();
    // Start execution.
		battery_level_get();
		application_timers_start(NULL,0);				
    advertising_start(NULL,0);	
		init_gpiote();
    while(true)
    {
				power_manage();
    }
}
