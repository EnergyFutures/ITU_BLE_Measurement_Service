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

#ifndef ITU_actuator_service_H__
#define ITU_actuator_service_H__
#include <itu_service.h>
#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "app_timer.h"

struct ias;

/**@brief Temperature Service structure. This contains various status information for the service. */
typedef struct ias
{
    uint16_t                     service_handle;               
		ble_gatts_char_handles_t     actuator_handles;      			 
    uint16_t                     conn_handle;  
		void (*p_update_status)(struct ias *);	
		uint8_t   coord;     
		uint8_t                     type;
		uint8_t                     make;
		uint8_t 										value;
} ias_t;

/**@brief Function for initializing the Service.
 *
 * @param[out]  p_iss       ITU Sensor Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ias_initialize(ias_t * p_ias);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the ITU Temperature Service.
 *
 * @param[in]   p_iss      ITU Sensor Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ias_on_ble_evt(ias_t * p_ias, ble_evt_t * p_ble_evt);

#endif // ITU_actuator_service_H__

/** @} */
