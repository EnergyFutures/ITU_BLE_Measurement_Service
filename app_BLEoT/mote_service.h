#ifndef ITU_mote_service_H__
#define ITU_mote_service_H__
#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "app_timer.h"
#include "itu_service.h"


uint32_t ims_initialize(storage_struct_t * p_storage_struct, mote_config_struct_t * p_mote_config_struct);

void ims_on_ble_evt(ble_evt_t * p_ble_evt);

void ims_timer_init(void);

static uint32_t update_ims_measurement(uint8_t * p_new_meas, uint16_t len);

static void process_all_cache_measurements(void *data, uint16_t size);

#endif // ITU_mote_service_H__

/** @} */
