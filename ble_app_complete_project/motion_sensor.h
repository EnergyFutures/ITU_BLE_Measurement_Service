#ifndef ITU_motion_sensor_H__
#define ITU_motion_sensor_H__
#include <stdint.h>
#include "main.h"
#include "sensor_service.h"
static void sensor_timer_init(void);
static void services_init(void);
static void sensor_timer_start(void);
static void sensor_ble_evt(ble_evt_t * p_ble_evt);
static void init(void);
static void gpiote_init(uint32_t *low_to_high,uint32_t *high_to_low);
static void on_gpiote_event(uint32_t *low_to_high,uint32_t *high_to_low);
itu_service_t * getMotionSensor(void);

#endif
