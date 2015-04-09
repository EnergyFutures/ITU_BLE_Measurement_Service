#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "pstorage.h"
#include "mote_service.h"

#define MAX_HTM_LEN   16 
#define ONE_MEASUREMENT   8 
#define TWO_MEASUREMENTS   16 
#define CLOCK_TIME_OUT 800

#define ITS_MOTE_CONFIG_DEVICE_NAME_BIT_PLACEMENT       											0             
#define ITS_MOTE_CONFIG_LOCATION_NAME_BIT_PLACEMENT    												1             
#define ITS_MOTE_CONFIG_ADVERTISEMENT_FREQUENCY_BIT_PLACEMENT           			2             
#define ITS_MOTE_CONFIG_BLOCK_COUNT_PERCENTAGE_BIT_PLACEMENT             			3                        


static volatile uint16_t conn_handle;
static volatile bool is_indication_supported = false;
static ble_gatts_char_handles_t     read_all_char_handles;
static ble_gatts_char_handles_t     mote_config_handles;
static app_timer_id_t one_shoot_timer;
static storage_struct_t *storage_struct;
static mote_config_struct_t *mote_config_struct;
static volatile bool transmit_flag = false;
static uint8_t operation_size;

static void continue_cache_loop(void * p_context)
{        
	UNUSED_PARAMETER(p_context);	
	if(transmit_flag){
		uint32_t error_code = app_timer_start(one_shoot_timer, CLOCK_TIME_OUT, NULL);
		APP_ERROR_CHECK(error_code);	
	}else{
		if(conn_handle != BLE_CONN_HANDLE_INVALID){
			if(operation_size == ONE_MEASUREMENT){
				storage_struct->current_offset = 0;
			}else{
				storage_struct->current_off_loading_block += 1;
			}
			process_all_cache_measurements(NULL,0);
		}
	}
}

static void get_measurements_from_cache(){
		uint32_t error_code;	
		pstorage_handle_t block_handle;			
		uint8_t measurements_arr[operation_size];
		error_code = pstorage_block_identifier_get(&storage_struct->pstorage_handle, storage_struct->current_off_loading_block, &block_handle);		
		APP_ERROR_CHECK(error_code);
		pstorage_load(measurements_arr, &block_handle, operation_size, 0);
		error_code = update_ims_measurement(measurements_arr, operation_size);
		APP_ERROR_CHECK(error_code);	
		transmit_flag = true;
		error_code = app_timer_start(one_shoot_timer, CLOCK_TIME_OUT, NULL);
		APP_ERROR_CHECK(error_code);	
}

static void process_all_cache_measurements(void *data, uint16_t size){
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
	if(conn_handle != BLE_CONN_HANDLE_INVALID){
		if(storage_struct->current_off_loading_block < storage_struct->current_block){
			operation_size = TWO_MEASUREMENTS;
			get_measurements_from_cache();
			return;
		}
		if(storage_struct->current_offset){
			operation_size = ONE_MEASUREMENT;
			get_measurements_from_cache();
			return;	
		}
		if(storage_struct->current_offset == 0 && storage_struct->current_off_loading_block == storage_struct->current_block){
			uint8_t measurements_arr[1] = {0};
			uint32_t error_code = update_ims_measurement(measurements_arr, 1 );	
			APP_ERROR_CHECK(error_code);
		}
		//THIS WILL CLEAR CACHE AND TURN ON MEASUREMENTS
		storage_struct->clear_cache(NULL,0);			
	}
}


void ims_timer_init(void){
	APP_ERROR_CHECK(app_timer_create(&one_shoot_timer,APP_TIMER_MODE_SINGLE_SHOT,continue_cache_loop));
}

static uint8_t conf_encode(uint8_t * p_encoded_buffer)
{
    uint8_t len   = 0;
    for(int i = 0; i < strlen(mote_config_struct->device_name);i++){
			p_encoded_buffer[len++] = mote_config_struct->device_name[i];
		}
		p_encoded_buffer[len++] = '\0';
		for(int i = 0; i < strlen(mote_config_struct->location_name);i++){
			p_encoded_buffer[len++] = mote_config_struct->location_name[i];
		}
		p_encoded_buffer[len++] = '\0';
		p_encoded_buffer[len++] = mote_config_struct->adv_freq_sec;
		p_encoded_buffer[len++] = mote_config_struct->block_count_percent_for_buffer_full;
    return len;
}

static void reset_adv_sche(void *data, uint16_t size){
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
	uint32_t error;
	uint8_t encoded_conf[MAX_HTM_LEN];
	uint16_t len = MAX_HTM_LEN;
	memset(encoded_conf, 0, sizeof(encoded_conf));
	//First clean up with all zeros
	error = sd_ble_gatts_value_set(mote_config_handles.value_handle,0,&len,encoded_conf);
	APP_ERROR_CHECK(error);
	
	//Now save the new value
	len = conf_encode(encoded_conf);
	error = sd_ble_gatts_value_set(mote_config_handles.value_handle,0,&len,encoded_conf);
	APP_ERROR_CHECK(error);
	
	//Reset advertisement data
	advertising_init();
}

static void conf_decode(uint16_t data_length, uint8_t * data)
{
	if(data_length > 0){		
		uint8_t i = 0;
		uint8_t data_bits = data[i++];
		if(BIT_VALUE(data_bits,ITS_MOTE_CONFIG_DEVICE_NAME_BIT_PLACEMENT)){
			for(int j = 0; j <=MAXIMUM_LOCATION_DEVICE_NAME_LENGTH; j++){
				mote_config_struct->device_name[j] = data[i+j];
				if(mote_config_struct->device_name[j] == '\0'){
					i += j+1;
					break;
				}
			}
		}
		if(BIT_VALUE(data_bits,ITS_MOTE_CONFIG_LOCATION_NAME_BIT_PLACEMENT)){
			for(int j = 0; j <=MAXIMUM_LOCATION_DEVICE_NAME_LENGTH; j++){
				mote_config_struct->location_name[j] = data[i+j];
				if(mote_config_struct->location_name[j] == '\0'){
					i += j+1;
					break;
				}
			}
		}
		if(BIT_VALUE(data_bits,ITS_MOTE_CONFIG_ADVERTISEMENT_FREQUENCY_BIT_PLACEMENT)){
			mote_config_struct->adv_freq_sec = data[i++];
		}
		if(BIT_VALUE(data_bits,ITS_MOTE_CONFIG_BLOCK_COUNT_PERCENTAGE_BIT_PLACEMENT)){
			mote_config_struct->block_count_percent_for_buffer_full = data[i++];
		}
		uint32_t error = app_sched_event_put(NULL, 0, reset_adv_sche);
		APP_ERROR_CHECK(error);
	}
}


static uint32_t add_conf_char(uint16_t service_handle)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
		uint8_t             encoded_conf[MAX_HTM_LEN];

    
		//Now we set the Characteristic metadata
	  memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write_wo_resp = 1;
    char_md.p_char_pf         = NULL; //No presentation format structure
    char_md.p_user_desc_md    = NULL; //User Description descriptor, NULL for default values
    char_md.p_cccd_md         = NULL; //The config attribute do not need indication or notify
    char_md.p_sccd_md         = NULL;	//We use no global settings, Server Characteristic Configuration Descriptor
		
		//Now we set the Characteristic User Description Descriptor
		char user_desc[] = "Configure the mote: Name, location, adv-freq and connection-state-level";
		char_md.p_char_user_desc  = (uint8_t *) user_desc;
    char_md.char_user_desc_size = strlen(user_desc);
    char_md.char_user_desc_max_size = strlen(user_desc);
    
    ble_uuid.type = BLE_UUID_TYPE_BLE;
    ble_uuid.uuid = BLE_UUID_ITU_CONFIG_MOTE_CHAR;
		
    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = conf_encode(encoded_conf);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = MAX_HTM_LEN;
    attr_char_value.p_value      = encoded_conf;
    
    err_code = sd_ble_gatts_characteristic_add(service_handle, &char_md,
                                               &attr_char_value,
                                               &mote_config_handles);
    
    return err_code;
}

static uint32_t add_read_all_measurements_char(uint16_t service_handle)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
		//Now we set the Client Characteristic Configuration Descriptor
    memset(&cccd_md, 0, sizeof(cccd_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	
		//Now we set the Characteristic metadata
	  memset(&char_md, 0, sizeof(char_md)); 
		char_md.char_props.write_wo_resp = 0;
		char_md.char_props.read   = 0;
    char_md.char_props.notify = 0;
		char_md.char_props.indicate = 1;
    char_md.p_char_pf         = NULL; //No presentation format structure
    char_md.p_user_desc_md    = NULL; //User Description descriptor, NULL for default values
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;		// We use no global settings, Server Characteristic Configuration Descriptor
		
		//Now we set the Characteristic User Description Descriptor
		char user_desc[] = "Read all cached values";
		char_md.p_char_user_desc  = (uint8_t *) user_desc;
    char_md.char_user_desc_size = strlen(user_desc);
    char_md.char_user_desc_max_size = strlen(user_desc);
    
		//BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ITU_MEASUREMENT_VALUE_CHAR);
		//Expanded the above macro to the code below for readability
    ble_uuid.type = BLE_UUID_TYPE_BLE; \
    ble_uuid.uuid = BLE_UUID_ITU_READ_ALL_MEASUREMENTS_VALUE_CHAR;
		
		
    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;
    
    uint8_t arr[MAX_HTM_LEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = MAX_HTM_LEN;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = MAX_HTM_LEN;
    attr_char_value.p_value      = arr;
    
    err_code = sd_ble_gatts_characteristic_add(service_handle, &char_md,
                                               &attr_char_value,
                                               &read_all_char_handles);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		return add_conf_char(service_handle);
}


uint32_t ims_initialize(storage_struct_t * p_storage_struct, mote_config_struct_t * p_mote_config_struct)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
		
		//Initialize service structure
    conn_handle     	= BLE_CONN_HANDLE_INVALID;
    
    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ITU_MOTE_SERVICE);
		uint16_t service_handle;
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		storage_struct = p_storage_struct;
		mote_config_struct = p_mote_config_struct;
    
    // Add measurement characteristic
    return add_read_all_measurements_char(service_handle);
}


static uint32_t update_ims_measurement(uint8_t * p_new_meas, uint16_t len)
{
    uint32_t err_code = NRF_SUCCESS;
				
		// Send value if connected and notifying
		if ((conn_handle != BLE_CONN_HANDLE_INVALID) && is_indication_supported)
		{
				ble_gatts_hvx_params_t hvx_params;
				
				memset(&hvx_params, 0, sizeof(hvx_params));
				
				hvx_params.handle   = read_all_char_handles.value_handle;
				hvx_params.type     = BLE_GATT_HVX_INDICATION;
				hvx_params.offset   = 0;
				hvx_params.p_len    = &len;
				hvx_params.p_data   = p_new_meas;
				
				err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);
		}		
    return err_code;
}


static void on_connect(ble_evt_t * p_ble_evt)
{
    conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}



static void on_disconnect(ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    conn_handle = BLE_CONN_HANDLE_INVALID;
}


static void on_write( ble_evt_t * p_ble_evt)
{
		ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
		if ((p_evt_write->handle == read_all_char_handles.cccd_handle) && (p_evt_write->len == 2))
		{
				is_indication_supported = ble_srv_is_indication_enabled(p_evt_write->data);
				if(is_indication_supported){
						uint32_t error = app_sched_event_put(NULL, 0, process_all_cache_measurements);
						APP_ERROR_CHECK(error);
				}
		}else if (p_evt_write->handle == mote_config_handles.value_handle)
		{
				conf_decode(p_evt_write->len,p_evt_write->data);
		}
}


void ims_on_ble_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect( p_ble_evt);
						transmit_flag = false;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ble_evt);
						transmit_flag = false;
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_ble_evt);
            break;
				
        case BLE_EVT_TX_COMPLETE:
            transmit_flag = false;
            break;
				
				case BLE_GATTS_EVT_HVC:
						transmit_flag = false;
            break;
				
        default:
            break;
    }
}
