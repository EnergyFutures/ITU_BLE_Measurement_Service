#include "temp_sensor_lmt85.h"
#include "app_timer.h"
#include "sensor_service.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "nrf_gpio.h"
#include "nrf_soc.h"
#include <math.h>
#include "itu_service.h"

static uint32_t adc_result;
static iss_t iss_struct; 
static app_timer_id_t one_shoot_timer;
static itu_sensor_t temp_sensor1 =
 {.sensor_timer_init = sensor_timer_init,
	.sensor_timer_start = sensor_timer_start,
	.init = init,
	.sensor_ble_evt = sensor_ble_evt,
	.service = &iss_struct,
	.needs_adc = true,
	.adc_done = adc_done,
	.type = BLE_UUID_ITU_SENSOR_TYPE_TEMPERATURE,
	.make = BLE_UUID_ITU_SENSOR_MAKE_LMT85};

itu_sensor_t * getTempSensorLmt85(void){
	return &temp_sensor1;
}


static void updateTempValue(void *data, uint16_t size){
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
	double a = adc_result * 1.7578125;
	a = (1324 - a);
	a = sqrt(67.141636  + 0.01048 * a);
	a = (8.194 - a);
	a = (( a / -0.00524) + 30.0);
	int32_t temp_value = a * 100.0;
	update_iss_measurement(&iss_struct, &temp_value);
}

static bool adc_done(uint8_t pin){
	if(pin == LMT85_ADC_PIN){		
		adc_result = NRF_ADC->RESULT;		
		nrf_gpio_pin_clear(LMT85_VCC_PIN);
		app_sched_event_put(NULL, 0, updateTempValue);
		return true;
	}	
	return false;
}

static void temp_adc_sampling_set_pin_sche(void *data, uint16_t size)
{        
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
		nrf_gpio_pin_set(LMT85_VCC_PIN);	
		app_timer_start(one_shoot_timer, APP_TIMER_TICKS(50, APP_TIMER_PRESCALER), NULL);
}

static void temp_adc_sampling_start_sche(void *data, uint16_t size)
{        
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
	
	while(NRF_ADC->BUSY){;}
	adc_init(LMT85_ADC_PIN,ADC_CONFIG_INPSEL_AnalogInputTwoThirdsPrescaling);
	NRF_ADC->TASKS_START = 1;							//Start ADC sampling
}

static void temp_adc_sampling_start(void * p_context)
{        
	UNUSED_PARAMETER(p_context);	
	app_sched_event_put(NULL, 0, temp_adc_sampling_start_sche);
}


static void temp_adc_sampling_set_pin(void * p_context)
{        
	UNUSED_PARAMETER(p_context);
	if(do_measurements)
	app_sched_event_put(NULL, 0, temp_adc_sampling_set_pin_sche);
}

static void sensor_timer_init(void)
{
		uint32_t err_code;	
		err_code = app_timer_create(&iss_struct.meas_timer,APP_TIMER_MODE_REPEATED,temp_adc_sampling_set_pin);
		APP_ERROR_CHECK(err_code);
		err_code = app_timer_create(&one_shoot_timer,APP_TIMER_MODE_SINGLE_SHOT,temp_adc_sampling_start);
		APP_ERROR_CHECK(err_code);
}



static void sensor_timer_start(void)
{
		uint32_t err_code = app_timer_start(iss_struct.meas_timer, APP_TIMER_TICKS(iss_struct.samp_freq_in_m_sec, APP_TIMER_PRESCALER), &iss_struct);
		APP_ERROR_CHECK(err_code);
		iss_struct.timer_running = true;
}


static void sensor_ble_evt(ble_evt_t * p_ble_evt)
{
		iss_on_ble_evt(&iss_struct,p_ble_evt);
}

static void update_measurement_samp_freq(){
	 if(iss_struct.timer_running){
		  uint32_t err_code;
			err_code = app_timer_stop(iss_struct.meas_timer);
			APP_ERROR_CHECK(err_code);
			err_code = app_timer_start(iss_struct.meas_timer, APP_TIMER_TICKS(iss_struct.samp_freq_in_m_sec, APP_TIMER_PRESCALER), &iss_struct);
			APP_ERROR_CHECK(err_code);
	 }   
}

static void init(void){
	//The analog input pin (27) is configred in adc_init() function
	nrf_gpio_cfg_output(LMT85_VCC_PIN);
		
	iss_struct.p_update_samp_freq = update_measurement_samp_freq;	
}
