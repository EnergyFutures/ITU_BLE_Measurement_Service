#ifndef ITU_window_actuator_H__
#define ITU_window_actuator_H__
#include <stdint.h>
#include "itu_service.h"
#define WINDOW_TYPE BLE_UUID_ITU_ACTUATOR_TYPE_WINDOW
static void actuator_timer_init(void);
static void actuator_timer_start(uint16_t offset);
static void actuator_timer_stop(void);
static void actuator_ble_evt(ble_evt_t * p_ble_evt);
static void init(void);
itu_service_t * get_window_actuator(void);

#endif
