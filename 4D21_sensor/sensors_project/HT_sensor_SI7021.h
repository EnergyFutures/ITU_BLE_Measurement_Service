#ifndef ITU_HT_SENSOR_SI7021_H__
#define ITU_HT_SENSOR_SI7021_H__
#include <stdint.h>
#include <itu_service.h>
#include "sensor_service.h"

#define SI7021_MEASURE_RH_COM 0xE5
#define SI7021_GET_TEMP_COM 	0xE0 // this needs to be called AFTER Relative Humidity measurement has been called
#define SI7021_WRITE_USER_REG	0xE6
#define SI7021_USER_REG_CONFIG 	0xBB
#define SI7021_ADR 	0x40

static void twi_ht_start_measuring(void * p_context);
static void twi_ht_read_measurement(void * p_context);
static void sensor_timer_init(void);
static void services_init(void);
static void sensor_timer_start(uint16_t offset);
static void sensor_timer_stop(void);
static void sensor_ble_evt(ble_evt_t * p_ble_evt);
static void init(void);
itu_service_t * getHumiditySensorSI7021(void);
itu_service_t * getTempSensorSI7021(void);
#endif
