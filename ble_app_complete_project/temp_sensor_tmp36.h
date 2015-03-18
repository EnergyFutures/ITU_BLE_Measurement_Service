#ifndef ITU_temp_sensor_tmp36_H__
#define ITU_temp_sensor_tmp36_H__
#include <stdint.h>
#include "main.h"
#include "sensor_service.h"
static void sensor_timer_init(void);
static void services_init(void);
static void sensor_timer_start(void);
static void sensor_ble_evt(ble_evt_t * p_ble_evt);
static void init(void);
itu_service_t * getTempSensorTmp36(void);
static bool adc_done(uint8_t pin);

#endif
