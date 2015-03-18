#ifndef ITU_light_sensor_TSL2561_H__
#define ITU_light_sensor_TSL2561_H__
#include <stdint.h>
#include <itu_service.h>
#include "main.h"
#include "sensor_service.h"


static uint32_t calc_lux(uint16_t ch0, uint16_t ch1);
static void twi_light_start_measuring(void * p_context);
static void twi_light_read_measurement(void * p_context);
static void sensor_timer_init(void);
static void services_init(void);
static void sensor_timer_start(void);
static void sensor_ble_evt(ble_evt_t * p_ble_evt);
static void init(void);
itu_service_t * getLightSensorTSL2561(void);

#endif
