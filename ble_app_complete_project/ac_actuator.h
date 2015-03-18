#ifndef ITU_ac_actuator_H__
#define ITU_ac_actuator_H__
#include <stdint.h>
#include "main.h"
#define AC_TYPE BLE_UUID_ITU_ACTUATOR_TYPE_AC

static void actuator_timer_init(void);
static void actuator_timer_start(void);
static void actuator_ble_evt(ble_evt_t * p_ble_evt);
static void init(void);
itu_service_t * get_ac_actuator(void);

#endif
