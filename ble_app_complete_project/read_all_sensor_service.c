#include "sensor_service.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "pstorage.h"
#include "read_all_sensor_service.h"

#define MAX_HTM_LEN   16 
static  uint16_t conn_handle;
static bool is_indication_supported = false;
static ble_gatts_char_handles_t     meas_val_handles;
static uint16_t                     service_handle;
static app_timer_id_t one_shoot_timer;
static storage_struct_t * storage_struct;
static bool transmit_flag = false;


static void continue_cache_loop(void * p_context)
{        
	UNUSED_PARAMETER(p_context);	
	if(transmit_flag){
			app_timer_start(one_shoot_timer, APP_TIMER_TICKS(10, APP_TIMER_PRESCALER), NULL);
	}else{
			app_sched_event_put(NULL, 0, get_all_cached_measurement_sche);
	}	
}


static void clear_cache_offset_sche(void *data, uint16_t size){
		UNUSED_PARAMETER(data);
		UNUSED_PARAMETER(size);	
		uint32_t error_code;	
		pstorage_handle_t block_handle;	
		uint8_t measurements_arr[8] = {0,0,0,0,0,0,0,0};
		error_code = pstorage_block_identifier_get(&storage_struct->pstorage_handle, storage_struct->current_block, &block_handle);
		APP_ERROR_CHECK(error_code);
		storage_struct->pstorage_wait_flag = true;
		storage_struct->pstorage_wait_handle = block_handle.block_id;
		pstorage_load(measurements_arr, &block_handle, 8 , 0);
		while(storage_struct->pstorage_wait_flag){
			storage_struct->power_manage();
		}				
		error_code = update_irass_measurement(measurements_arr, 8);	
		if(error_code != NRF_SUCCESS){
			APP_ERROR_CHECK(error_code);
		}
		storage_struct->current_offset = 0;
		transmit_flag = true;
		app_timer_start(one_shoot_timer, APP_TIMER_TICKS(20, APP_TIMER_PRESCALER), NULL);
}

static void clear_cache_loop_sche(void *data, uint16_t size){
		UNUSED_PARAMETER(data);
		UNUSED_PARAMETER(size);	
		uint32_t error_code;	
		pstorage_handle_t block_handle;			
		uint8_t measurements_arr[BLOCK_SIZE] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		error_code = pstorage_block_identifier_get(&storage_struct->pstorage_handle, --(storage_struct->current_block), &block_handle);		
		if(error_code != NRF_SUCCESS){
			APP_ERROR_CHECK(error_code);
		}					
		storage_struct->pstorage_wait_flag = true;
		storage_struct->pstorage_wait_handle = block_handle.block_id;
		pstorage_load(measurements_arr, &block_handle, BLOCK_SIZE, 0);
		while(storage_struct->pstorage_wait_flag){
			storage_struct->power_manage();
		}	
		error_code = update_irass_measurement(measurements_arr, BLOCK_SIZE);
		if(error_code != NRF_SUCCESS){
			APP_ERROR_CHECK(error_code);
		}		
		transmit_flag = true;
		app_timer_start(one_shoot_timer, APP_TIMER_TICKS(10, APP_TIMER_PRESCALER), NULL);
}

static void get_all_cached_measurement_sche(void *data, uint16_t size){
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
	//WE DISABLE MEASUREMENTS... REMBER THIS IS TURNED ON ONCE THE CACHE IS CLEARED COMPLETELY
	do_measurements = false;
	if(conn_handle != BLE_CONN_HANDLE_INVALID){
			if(storage_struct->current_offset > 0){
					app_sched_event_put(NULL, 0, clear_cache_offset_sche);	
					return;				
			}
			if(storage_struct->current_block > 0){
					app_sched_event_put(NULL, 0, clear_cache_loop_sche);	
					return;				
			}
			//THIS WILL CLEAR CACHE AND TURN ON MEASUREMENTS
			storage_struct->clear_cache();						
	}
}


void irass_timer_init(void){
	APP_ERROR_CHECK(app_timer_create(&one_shoot_timer,APP_TIMER_MODE_SINGLE_SHOT,continue_cache_loop));
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
		if ((p_evt_write->handle == meas_val_handles.cccd_handle) && (p_evt_write->len == 2))
		{
				is_indication_supported = ble_srv_is_indication_enabled(p_evt_write->data);
				if(is_indication_supported){
						app_sched_event_put(NULL, 0, get_all_cached_measurement_sche);
				}
		}
}


void irass_on_ble_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect( p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ble_evt);
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

static uint32_t add_meas_char()
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
    ble_uuid.uuid = BLE_UUID_ITU_READ_ALL_MEASUREMENT_VALUE_CHAR;
		
		
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
                                               &meas_val_handles);
    
    return err_code;
}


uint32_t irass_initialize(storage_struct_t * p_storage_struct)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
		
		//Initialize service structure
    conn_handle     	= BLE_CONN_HANDLE_INVALID;
    
    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ITU_READ_ALL_MEASUREMENT_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		storage_struct = p_storage_struct;
    
    // Add measurement characteristic
    return add_meas_char();
}


uint32_t update_irass_measurement(uint8_t * p_new_meas, uint16_t len)
{
    uint32_t err_code = NRF_SUCCESS;
				
		// Send value if connected and notifying
		if ((conn_handle != BLE_CONN_HANDLE_INVALID) && is_indication_supported)
		{
				ble_gatts_hvx_params_t hvx_params;
				
				memset(&hvx_params, 0, sizeof(hvx_params));
				
				hvx_params.handle   = meas_val_handles.value_handle;
				hvx_params.type     = BLE_GATT_HVX_INDICATION;
				hvx_params.offset   = 0;
				hvx_params.p_len    = &len;
				hvx_params.p_data   = p_new_meas;
				
				err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);
		}		
    return err_code;
}
