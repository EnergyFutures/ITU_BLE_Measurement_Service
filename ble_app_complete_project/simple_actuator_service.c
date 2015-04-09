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

#include "simple_actuator_service.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"


#define OPCODE_LENGTH  1                                                    /**< Length of opcode inside Health Thermometer Measurement packet. */
#define HANDLE_LENGTH  2                                                    /**< Length of handle inside Health Thermometer Measurement packet. */
#define MAX_HTM_LEN   (OPCODE_LENGTH + HANDLE_LENGTH + 1)   /**< Maximum size of a transmitted Health Thermometer Measurement. */


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_iss       ITU Sensor Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ias_t * p_ias, ble_evt_t * p_ble_evt)
{
    p_ias->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_iss       ITU Sensor Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ias_t * p_ias, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_ias->conn_handle = BLE_CONN_HANDLE_INVALID;
}



/**@brief Function for handling the Write event.
 *
 * @param[in]   p_iss       ITU Sensor Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ias_t * p_ias, ble_evt_t * p_ble_evt)
{
		ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
		if (p_evt_write->handle == p_ias->actuator_handles.value_handle)
		{
			p_ias->value = p_evt_write->data[0] ;
			p_ias->p_update_status(p_ias);
		}
}


void ias_on_ble_evt(ias_t * p_ias, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_ias, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ias, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_ias, p_ble_evt);
            break;
            
        default:
            break;
    }
}

static uint32_t add_conf_char(ias_t * p_ias)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    
		//Now we set the Characteristic metadata
	  memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write_wo_resp = 1;
    char_md.p_char_pf         = NULL; //No presentation format structure
    char_md.p_user_desc_md    = NULL; //User Description descriptor, NULL for default values
    char_md.p_cccd_md         = NULL; //The config attribute do not need indication or notify
    char_md.p_sccd_md         = NULL;	//We use no global settings, Server Characteristic Configuration Descriptor
		
		//Now we set the Characteristic User Description Descriptor
		char user_desc[] = "Actuator on/off";
		char_md.p_char_user_desc  = (uint8_t *) user_desc;
    char_md.char_user_desc_size = strlen(user_desc);
    char_md.char_user_desc_max_size = strlen(user_desc);
    
		//BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ITU_MEASUREMENT_VALUE_CHAR);
		//Expanded the above macro to the code below for readability
    ble_uuid.type = BLE_UUID_TYPE_BLE; 
    ble_uuid.uuid = BLE_UUID_ITU_ACTUATOR_COMMAND_CHAR;
		 
		
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
    attr_char_value.init_len     = 1;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = MAX_HTM_LEN;
    attr_char_value.p_value      = &p_ias->value;
    
    err_code = sd_ble_gatts_characteristic_add(p_ias->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_ias->actuator_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    return NRF_SUCCESS;
}




uint32_t ias_initialize(ias_t * p_ias)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
		
		//Initialize service structure
    p_ias->conn_handle     	= BLE_CONN_HANDLE_INVALID;
    
		ble_uuid.type = BLE_UUID_TYPE_BLE; 
    ble_uuid.uuid = BLE_UUID_ITU_ACTUATOR_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_ias->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Add measurement characteristic
    return add_conf_char(p_ias);
}
