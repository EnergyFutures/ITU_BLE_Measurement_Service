#ifndef ITU_temp_sensor_lmt85_H__
#define ITU_temp_sensor_lmt85_H__
#include <stdint.h>
#include "main.h"
#include "sensor_service.h"
#define LMT85_VCC_PIN 12
#define LMT85_ADC_PIN  ADC_CONFIG_PSEL_AnalogInput0

static void sensor_timer_init(void);
static void services_init(void);
static void sensor_timer_start(void);
static void sensor_ble_evt(ble_evt_t * p_ble_evt);
static void init(void);
itu_service_t * getTempSensorLmt85(void);
static bool adc_done(uint8_t pin);

#endif
