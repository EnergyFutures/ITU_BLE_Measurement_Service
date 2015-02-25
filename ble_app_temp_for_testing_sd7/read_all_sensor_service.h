#ifndef ITU_read_all_sensor_service_H__
#define ITU_read_all_sensor_service_H__
#include "itu_service.h"
#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "app_timer.h"
#include "main.h"


uint32_t irass_initialize(storage_struct_t * p_storage_struct);

void irass_on_ble_evt(ble_evt_t * p_ble_evt);

uint32_t update_irass_measurement(uint8_t * p_new_meas, uint16_t len);

void irass_timer_init(void);

#endif // ITU_read_all_sensor_service_H__

/** @} */
