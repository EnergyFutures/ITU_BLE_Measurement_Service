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
#define ITS_MEAS_FLAG_SPACE_TIME_BIT             		BIT_0             /**< Space + Time flag */
#define ITS_MEAS_FLAG_UNIT_BIT                			BIT_1             /**< Units flag. */
#define ITS_MEAS_FLAG_TYPE_MAKE_BIT                 BIT_2             /**< Type flag. */
#define ITS_MEAS_FLAG_ID_BIT             						BIT_3             /**< ID flag */
#define ITS_MEAS_FLAG_SAMP_FREQ_BIT             		BIT_4             /**< Sampling Frequency flag */

#define ITS_MEAS_FLAG_SPACE_TIME_BIT_PLACEMENT       		0             /**< Space + Time placement in header */
#define ITS_MEAS_FLAG_UNIT_BIT_PLACEMENT    						1             /**< Units placement in header */
#define ITS_MEAS_FLAG_TYPE_MAKE_BIT_PLACEMENT           2             /**< Type placement in header*/
#define ITS_MEAS_FLAG_ID_BIT_PLACEMENT             			3             /**< ID placement in header*/
#define ITS_MEAS_FLAG_SAMP_FREQ_BIT_PLACEMENT         	4             /**< Sampling Frequency placement in header */
#define ITS_MEAS_FLAG_OVERRIDE_BIT_PLACEMENT          	7             /**< When configuation is received by the client, this bit indicate whether we are setting the presentation or overriding values */

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
		
	/*
    // Sequence number field
    if (p_iss->space_time_present)
    {
        pres_flags |= ITS_MEAS_FLAG_SPACE_TIME_BIT;
    }
	
		// Unit value
    if (p_iss->unit_present)
    {
        pres_flags |= ITS_MEAS_FLAG_UNIT_BIT;
		}
		
		// Type value
    if (p_iss->type_make_present)
    {
        pres_flags |= ITS_MEAS_FLAG_TYPE_MAKE_BIT;
		}
		
		// ID field
    if (p_iss->ID_present)
    {
        pres_flags |= ITS_MEAS_FLAG_ID_BIT;
    }
		
		// Sampling Frequency field
    if (p_iss->samp_freq_present)
    {
        pres_flags |= ITS_MEAS_FLAG_SAMP_FREQ_BIT;
    }*/
		
		pres_flags |= ITS_MEAS_FLAG_SPACE_TIME_BIT;
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
			//p_iss->gatt_db_needs_cleaning = true;
			uint8_t data_bits = data[0];
			//if(BIT_VALUE(data_bits,ITS_MEAS_FLAG_OVERRIDE_BIT_PLACEMENT)){
					//OVERRIDE.. Client wants to set new values
					if(data_length > 1){
						
						uint8_t i = 1;
						
						//Check if Seq. nr. is set	
						if(BIT_VALUE(data_bits,ITS_MEAS_FLAG_SPACE_TIME_BIT_PLACEMENT)){
							p_iss->coord = data[i];	
							i++;
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
					}					
			/*}else{
				//set presentation			
				
				// SET Space + Time
				p_iss->space_time_present = BIT_VALUE(data_bits,ITS_MEAS_FLAG_SPACE_TIME_BIT_PLACEMENT);
				
				//SET Unit	
				p_iss->unit_present = BIT_VALUE(data_bits,ITS_MEAS_FLAG_UNIT_BIT_PLACEMENT);
				
				//SET Type	
				p_iss->type_make_present = BIT_VALUE(data_bits,ITS_MEAS_FLAG_TYPE_MAKE_BIT_PLACEMENT);
				
				//SET ID	
				p_iss->ID_present = BIT_VALUE(data_bits,ITS_MEAS_FLAG_ID_BIT_PLACEMENT);
				
				//SET Sampling freq.	
				p_iss->samp_freq_present = BIT_VALUE(data_bits,ITS_MEAS_FLAG_SAMP_FREQ_BIT_PLACEMENT);
			}
			
			uint16_t data_len = MAX_HTM_LEN;
			uint8_t data_buf[MAX_HTM_LEN];
			memset(data_buf, 0, sizeof(data_buf));
			data_len = its_conf_encode(p_iss, data_buf);
			sd_ble_gatts_value_set(p_iss->meas_conf_handles.value_handle,
																			0,
																			&data_len,
																			data_buf);*/
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

/**@brief Function for encoding a ITU Sensor Service Measurement.
 *
 * @param[in]   p_iss			         ITU Sensor Service structure.
 * @param[in]   measurement        Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t iss_measurement_encode(iss_t * p_iss, int32_t * measurement,uint8_t * p_encoded_buffer,uint32_t * value)
{
    uint8_t flags = 0;
    uint8_t len   = 1;
	
		//Value always present
		*value = (((p_iss->IEEE_exponent << 24) & 0xFF000000) | (*measurement & 0x00FFFFFF));
		len += uint32_encode(*value, &p_encoded_buffer[len]);
		
		//Remember, to increament the seq nr so we have a continuous flow
		++(p_iss->curr_seq_nr);
	
		// Space + Time value
    if (p_iss->space_time_present)
    {
        flags |= ITS_MEAS_FLAG_SPACE_TIME_BIT;
				p_encoded_buffer[len++]  = p_iss->coord;
        len   += uint16_encode(p_iss->curr_seq_nr, &p_encoded_buffer[len]);
    }
	
    // Unit value
    if (p_iss->unit_present)
    {
        flags |= ITS_MEAS_FLAG_UNIT_BIT;
        len   += uint16_encode(p_iss->unit, &p_encoded_buffer[len]);
		}
		
		// Type value
    if (p_iss->type_make_present)
    {
        flags |= ITS_MEAS_FLAG_TYPE_MAKE_BIT;
        p_encoded_buffer[len++]  = p_iss->type;
				p_encoded_buffer[len++]  = p_iss->make;
		}
		
		// ID field
    if (p_iss->ID_present)
    {
        flags |= ITS_MEAS_FLAG_ID_BIT;
        len   += uint16_encode(p_iss->ID, &p_encoded_buffer[len]);
    }
		
		// Sampling Frequency field
    if (p_iss->samp_freq_present)
    {
        flags |= ITS_MEAS_FLAG_SAMP_FREQ_BIT;
        len   += uint32_encode(p_iss->samp_freq_in_m_sec, &p_encoded_buffer[len]);
    }

    // Flags field
    p_encoded_buffer[0] = flags;

    return len;
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

/**@brief Function for adding the ITU Sensor Service measurement characteristic.
 *
 * @param[in]   p_iss        ITU Sensor Service structure.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
/*
static uint32_t add_meas_char(iss_t * p_iss)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
		uint8_t             encoded_measurement[MAX_HTM_LEN];
    
		//Now we set the Client Characteristic Configuration Descriptor
    memset(&cccd_md, 0, sizeof(cccd_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
		//Now we set the Characteristic metadata
	  memset(&char_md, 0, sizeof(char_md)); 
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_pf         = NULL; //No presentation format structure
    char_md.p_user_desc_md    = NULL; //User Description descriptor, NULL for default values
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;		// We use no global settings, Server Characteristic Configuration Descriptor
		
		//Now we set the Characteristic User Description Descriptor
		char user_desc[] = "Temp in C multiplied with 100";
		char_md.p_char_user_desc  = (uint8_t *) user_desc;
    char_md.char_user_desc_size = strlen(user_desc);
    char_md.char_user_desc_max_size = strlen(user_desc);
    
		//BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ITU_MEASUREMENT_VALUE_CHAR);
		//Expanded the above macro to the code below for readability
    ble_uuid.type = BLE_UUID_TYPE_BLE;
    ble_uuid.uuid = BLE_UUID_ITU_MEASUREMENT_VALUE_CHAR;
		
		
    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    int32_t init_value = INVALID_ITU_MEASUREMENT_VALUE;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = iss_measurement_encode(p_iss,&init_value, encoded_measurement, &(p_iss->last_measurement));
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = MAX_HTM_LEN;
    attr_char_value.p_value      = encoded_measurement;
    
    err_code = sd_ble_gatts_characteristic_add(p_iss->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_iss->meas_val_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Add config characteristic
    return add_conf_char(p_iss);
}*/


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


uint32_t update_iss_measurement(iss_t * p_iss, int32_t * p_new_meas)
{
    uint32_t err_code = NRF_SUCCESS;
		uint8_t encoded_measurement[MAX_HTM_LEN];
		uint16_t len = MAX_HTM_LEN;
	
		memset(encoded_measurement, 0, sizeof(encoded_measurement));
		/*if(p_iss->gatt_db_needs_cleaning){
			p_iss->gatt_db_needs_cleaning = false;
			// Update database with all zeros... to remove old values
			err_code = sd_ble_gatts_value_set(p_iss->meas_val_handles.value_handle,
																			0,
																			&len,
																			encoded_measurement);
			if (err_code != NRF_SUCCESS)
			{
					return err_code;
			}
		}*/
		len = iss_measurement_encode(p_iss,p_new_meas, encoded_measurement, &(p_iss ->last_measurement));
		
		// Update database with the new measurement
		/*err_code = sd_ble_gatts_value_set(p_iss->meas_val_handles.value_handle,
																			0,
																			&len,
																			encoded_measurement);*/
		if (err_code != NRF_SUCCESS)
		{
				return err_code;
		}
		
		//Persist before using the radio
		p_iss->p_persist_measurement(p_iss);
		
		// Send value if connected and notifying
		if ((p_iss->conn_handle != BLE_CONN_HANDLE_INVALID) && p_iss->is_notification_supported)
		{
				ble_gatts_hvx_params_t hvx_params;
				
				memset(&hvx_params, 0, sizeof(hvx_params));
				
				hvx_params.handle   = p_iss->meas_val_handles.value_handle;
				hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
				hvx_params.offset   = 0;
				hvx_params.p_len    = &len;
				hvx_params.p_data   = encoded_measurement;
				
				err_code = sd_ble_gatts_hvx(p_iss->conn_handle, &hvx_params);
		}		
    return err_code;
}
