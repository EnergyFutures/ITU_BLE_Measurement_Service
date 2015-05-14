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

#include "sensor_service.h"
#include <string.h>
#include "nordic_common.h"
#include "app_util.h"


#define OPCODE_LENGTH  1                                                    /**< Length of opcode inside Health Thermometer Measurement packet. */
#define HANDLE_LENGTH  2                                                    /**< Length of handle inside Health Thermometer Measurement packet. */
#define MAX_HTM_LEN   (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH)   /**< Maximum size of a transmitted Health Thermometer Measurement. */

// Presentation flag bits
#define ITS_MEAS_FLAG_COORDINATE_BIT             		BIT_0             /**< Coordinate flag */
#define ITS_MEAS_FLAG_SEQUENCE_NUMBER_BIT        		BIT_1             /**< Coordinate flag */
#define ITS_MEAS_FLAG_UNIT_BIT                			BIT_2             /**< Units flag. */
#define ITS_MEAS_FLAG_TYPE_MAKE_BIT                 BIT_3             /**< Type flag. */
#define ITS_MEAS_FLAG_ID_BIT             						BIT_4             /**< ID flag */
#define ITS_MEAS_FLAG_SAMP_FREQ_BIT             		BIT_5             /**< Sampling Frequency flag */

#define ITS_MEAS_FLAG_COORDINATE_BIT_PLACEMENT       		0             /**< Coordinate placement in header */
#define ITS_MEAS_FLAG_SEQUENCE_NUMBER_BIT_PLACEMENT     1             /**< Coordinate placement in header */
#define ITS_MEAS_FLAG_UNIT_BIT_PLACEMENT    						2             /**< Units placement in header */
#define ITS_MEAS_FLAG_TYPE_MAKE_BIT_PLACEMENT           3             /**< Type placement in header*/
#define ITS_MEAS_FLAG_ID_BIT_PLACEMENT             			4             /**< ID placement in header*/
#define ITS_MEAS_FLAG_SAMP_FREQ_BIT_PLACEMENT         	5             /**< Sampling Frequency placement in header */

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_iss       ITU Sensor Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(iss_t * p_iss, ble_evt_t * p_ble_evt)
{
    p_iss->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_iss       ITU Sensor Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(iss_t * p_iss, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_iss->conn_handle = BLE_CONN_HANDLE_INVALID;
}



/**@brief Function for encoding a ITU Sensor Service Configuration.
 *
 * @param[in]   p_iss			         ITU Sensor Service structure.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t its_conf_encode(iss_t * p_iss, uint8_t * p_encoded_buffer)
{
		uint8_t pres_flags = 0;
    uint8_t len   = 1;
		
		pres_flags |= ITS_MEAS_FLAG_COORDINATE_BIT;
		pres_flags |= ITS_MEAS_FLAG_SEQUENCE_NUMBER_BIT;
		pres_flags |= ITS_MEAS_FLAG_UNIT_BIT;
		pres_flags |= ITS_MEAS_FLAG_TYPE_MAKE_BIT;
		pres_flags |= ITS_MEAS_FLAG_ID_BIT;
		pres_flags |= ITS_MEAS_FLAG_SAMP_FREQ_BIT;

    // Presentation Flag field
    p_encoded_buffer[0] = pres_flags;		
		p_encoded_buffer[len++]  = p_iss->coord;
		len += uint16_encode(p_iss->curr_seq_nr, &p_encoded_buffer[len]);
		len += uint16_encode(p_iss->unit, &p_encoded_buffer[len]);
		p_encoded_buffer[len++]  = p_iss->type;
		p_encoded_buffer[len++]  = p_iss->make;
		len += uint16_encode(p_iss->ID, &p_encoded_buffer[len]);
		len += uint32_encode(p_iss->samp_freq_in_m_sec, &p_encoded_buffer[len]);

    return len;
}


/**@brief Function for decoding a ITU Sensor Service Configuration.
 *
 * @param[in]   p_iss			         ITU Sensor Service structure
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static void its_conf_decode(iss_t * p_iss,uint16_t data_length, uint8_t * data)
{
		if(data_length > 0){		
			uint8_t data_bits = data[0];
				if(data_length > 1){
					
					uint8_t i = 1;
					
					//Check if Coordinate is set	
					if(BIT_VALUE(data_bits,ITS_MEAS_FLAG_COORDINATE_BIT_PLACEMENT)){
						p_iss->coord = data[i];	
						i++;
					}
					
					//Check if Seq. nr. is set	
					if(BIT_VALUE(data_bits,ITS_MEAS_FLAG_SEQUENCE_NUMBER_BIT_PLACEMENT)){
						p_iss->curr_seq_nr = uint16_decode(&data[i]);	
						i += 2;
					}
				
					//Check if Unit is set	
					if(BIT_VALUE(data_bits,ITS_MEAS_FLAG_UNIT_BIT_PLACEMENT)){
							p_iss->unit = uint16_decode(&data[i]);
							i += 2;
					}
					
					//Check if Type is set	
					if(BIT_VALUE(data_bits,ITS_MEAS_FLAG_TYPE_MAKE_BIT_PLACEMENT)){
							p_iss->type = data[i];
							i++;
							p_iss->make = data[i];
							i++;
					}
										
					//Check if ID is set	
					if(BIT_VALUE(data_bits,ITS_MEAS_FLAG_ID_BIT_PLACEMENT)){
							p_iss->ID = uint16_decode(&data[i]);
							i += 2;
					}
					
					//Check if Sampling freq. is set	
					if(BIT_VALUE(data_bits,ITS_MEAS_FLAG_SAMP_FREQ_BIT_PLACEMENT)){
							p_iss->samp_freq_in_m_sec = uint32_decode(&data[i]);
							p_iss->p_update_samp_freq();
					}
					iss_update_config(p_iss);
				}					
		}
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_iss       ITU Sensor Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(iss_t * p_iss, ble_evt_t * p_ble_evt)
{
		ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
		if ((p_evt_write->handle == p_iss->meas_val_handles.cccd_handle) && (p_evt_write->len == 2))
		{
				p_iss->is_notification_supported = ble_srv_is_notification_enabled(p_evt_write->data);
		}else if (p_evt_write->handle == p_iss->meas_conf_handles.value_handle)
		{
				its_conf_decode(p_iss,p_evt_write->len,p_evt_write->data);
				iss_update_config(p_iss);
		}
}


void iss_on_ble_evt(iss_t * p_iss, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_iss, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_iss, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_iss, p_ble_evt);
            break;
            
        default:
            break;
    }
}

/**@brief Function for adding the ITU Sensor Service config characteristic.
 *
 * @param[in]   p_iss        ITU Sensor Service structure.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */

static uint32_t add_conf_char(iss_t * p_iss)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
		uint8_t             encoded_meas_conf[MAX_HTM_LEN];

    
		//Now we set the Characteristic metadata
	  memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write_wo_resp = 1;
    char_md.p_char_pf         = NULL; //No presentation format structure
    char_md.p_user_desc_md    = NULL; //User Description descriptor, NULL for default values
    char_md.p_cccd_md         = NULL; //The config attribute do not need indication or notify
    char_md.p_sccd_md         = NULL;	//We use no global settings, Server Characteristic Configuration Descriptor
		
		//Now we set the Characteristic User Description Descriptor
		char user_desc[] = "Configure the temp sensor e.g. ID, Seq nr, Sampl freq ect.";
		char_md.p_char_user_desc  = (uint8_t *) user_desc;
    char_md.char_user_desc_size = strlen(user_desc);
    char_md.char_user_desc_max_size = strlen(user_desc);
    
		//BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ITU_MEASUREMENT_VALUE_CHAR);
		//Expanded the above macro to the code below for readability
    ble_uuid.type = BLE_UUID_TYPE_BLE;
    ble_uuid.uuid = BLE_UUID_ITU_MEASUREMENT_CONFIG_CHAR;
		
		
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
    attr_char_value.init_len     = its_conf_encode(p_iss,encoded_meas_conf);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = MAX_HTM_LEN;
    attr_char_value.p_value      = encoded_meas_conf;
    
    err_code = sd_ble_gatts_characteristic_add(p_iss->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_iss->meas_conf_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    return NRF_SUCCESS;
}

uint32_t iss_initialize(iss_t * p_iss)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
		
		//Initialize service structure
    p_iss->conn_handle     	= BLE_CONN_HANDLE_INVALID;
		p_iss->last_measurement = 0;
    
    // Add service
		ble_uuid.type = BLE_UUID_TYPE_BLE;
    ble_uuid.uuid = BLE_UUID_ITU_MEASUREMENT_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_iss->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Add config characteristic
    return add_conf_char(p_iss);
}

uint32_t iss_update_config(iss_t * p_iss){
	uint8_t             encoded_meas_conf[MAX_HTM_LEN];
	uint16_t len = its_conf_encode(p_iss, encoded_meas_conf);
	return sd_ble_gatts_value_set(p_iss->meas_conf_handles.value_handle,
																			0,
																			&len,
																			encoded_meas_conf);
}

uint32_t update_iss_measurement(iss_t * p_iss, int32_t * p_new_meas)
{
		p_iss->last_measurement = (((p_iss->IEEE_exponent << 24) & 0xFF000000) | (*p_new_meas & 0x00FFFFFF));
		++(p_iss->curr_seq_nr);
		p_iss->p_persist_measurement(p_iss);
    return NRF_SUCCESS;
}

