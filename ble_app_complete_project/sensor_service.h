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
 * @defgroup ble_sdk_srv_bas ITU Temperature Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief ITU Sensor Service module.
 *
 * @details This module implements the ITU Sensor Service with the Battery Level characteristic.
 *          During initialization it adds the ITU Sensor Service and Battery Level characteristic
 *          to the BLE stack database. Optionally it can also add a Report Reference descriptor
 *          to the Battery Level characteristic (used when including the ITU Sensor Service in
 *          the HID service).
 *
 *          If specified, the module will support notification of the Battery Level characteristic
 *          through the ble_bas_battery_level_update() function.
 *          If an event handler is supplied by the application, the ITU Sensor Service will
 *          generate ITU Sensor Service events to the application.
 *
 * @note The application must propagate BLE stack events to the ITU Sensor Service module by calling
 *       ble_bas_on_ble_evt() from the from the @ref ble_stack_handler callback.
 */

#ifndef ITU_sensor_service_H__
#define ITU_sensor_service_H__
#include "itu_service.h"
#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "app_timer.h"


//struct iss;

/**@brief Temperature Service structure. This contains various status information for the service. */
typedef struct iss
{
    uint16_t                     service_handle;               /**< Handle of the Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     meas_val_handles;      			 /**< Handles related to the measurement value characteristic. */
		ble_gatts_char_handles_t     meas_conf_handles;      			 /**< Handles related to the measurement config characteristic. */
    uint16_t                     conn_handle;                  /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
		bool                         space_time_present;       		 /**< True if coordinte + sequence number is present. */
		its_measurement_location_t   coord;                   		 /**< New measurement Coordinate. */
		uint16_t										 curr_seq_nr;									 /**< Current Sequence Number*/
		bool                         unit_present;     		  	  	 /**< True if Unit is present. */
		uint16_t                     unit;      	   							 /**< Unit https://developer.bluetooth.org/gatt/units/Pages/default.aspx */
	  bool                         type_make_present;     		   /**< True if sensor type is present. */
		uint8_t                      type;      	   							 /**< ITU type */
		uint8_t                      make;      	   							 /**< ITU make */
		bool                         ID_present;      	           /**< True if ID should is included*/
		uint16_t										 ID;													 /**< New ITU allocated ID */
    bool                         samp_freq_present;        		 /**< True if Sampling Frequence is present. */
		uint32_t										 samp_freq_in_m_sec;					 /**< New Sampling Frequency in miliseconds*/		
		int8_t											 IEEE_exponent;								 /**< Exponent used for encoding the IEEE float*/
		bool												 is_notification_supported;		 /**< Flag to indicate if client has enabled notifications*/
		void (*p_update_samp_freq)(void);									 				 /**< Function to update the sampling frequency if client asks for it*/
		bool gatt_db_needs_cleaning;															 /**< Flag to indicate that presentation config has been changed, and we need to clean the gatt db*/
		app_timer_id_t							 meas_timer;									 /**< Timer id used to start/stop*/
		bool												 timer_running;								 /**< True if running, false otherwise */		
		uint32_t										 last_measurement;
		bool												 is_sensor;
		void (*p_persist_measurement)(struct iss *);
} iss_t;




/**@brief Function for initializing the Service.
 *
 * @param[out]  p_iss       ITU Sensor Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t iss_initialize(iss_t * p_iss);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the ITU Temperature Service.
 *
 * @param[in]   p_iss      ITU Sensor Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void iss_on_ble_evt(iss_t * p_iss, ble_evt_t * p_ble_evt);

/**@brief Function for updating the measurement value.
 *
 * @details The application calls this function when a new measurement is ready. If
 *          notification has been enabled, the measurement characteristic is sent to the client.
 *
 * @param[in]   p_iss          		ITU Sensor Service structure.
 * @param[in]   value_provider  	The function to call to get the measurement
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t update_iss_measurement(iss_t * p_iss, int32_t * p_new_meas);

#endif // ITU_sensor_service_H__

/** @} */
