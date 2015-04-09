#include "temp_sensor_tmp36.h"
#include "temp_sensor_tmp36_config.h"
#include "app_timer.h"
#include "sensor_service.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "nrf_gpio.h"
#include "nrf_soc.h"
#include "itu_service.h"

static uint32_t adc_result;
static iss_t iss_struct; 
//static app_timer_id_t one_shoot_timer;
static itu_service_t temp_sensor1 ={.timer_init = sensor_timer_init,
																		.timer_start = sensor_timer_start,
																		.timer_stop = sensor_timer_stop,
																		.init = init,
																		.ble_evt = sensor_ble_evt,
																		.service = &iss_struct,
																		.service_type = 0,
																		.needs_adc = true,
																		.adc_done = adc_done,
																		.needs_gpiote = false,
																		.gpiote_init = NULL,
																		.on_gpiote_event = NULL
																		};

itu_service_t * getTempSensorTmp36(void){
	return &temp_sensor1;
}

static void updateTempValue(void *data, uint16_t size){
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
	// (result * (3/2 * 1,2vref) / 1024 (10 bit res))..  we multiply with 100 to avoid float.. the result is already multiplied with 10... we multiply with 10
	int32_t temp_value = (((adc_result * 1800)/1024) - 500 ) * 10; 
	uint32_t err_code;	
	err_code = update_iss_measurement(&iss_struct, &temp_value);
	APP_ERROR_CHECK(err_code);
	err_code = app_timer_start(iss_struct.meas_timer, APP_TIMER_TICKS(iss_struct.samp_freq_in_m_sec, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);
}

static bool adc_done(uint8_t pin){
	if(pin == TMP36_ADC_PIN){		
		adc_result = NRF_ADC->RESULT;		
		nrf_gpio_pin_clear(TMP36_VCC_PIN);
		uint32_t err_code;	
		err_code = app_sched_event_put(NULL, 0, updateTempValue);
		APP_ERROR_CHECK(err_code);
		return true;
	}	
	return false;
}

static void temp_adc_sampling(void * p_context)
{        
	UNUSED_PARAMETER(p_context);
	uint32_t err_code;	
	static bool set_pin = false;
	if(do_measurements){
		set_pin = !set_pin;
		if(set_pin){
			nrf_gpio_pin_set(TMP36_VCC_PIN);	
			err_code = app_timer_start(iss_struct.meas_timer, APP_TIMER_TICKS(150, APP_TIMER_PRESCALER), NULL);
			APP_ERROR_CHECK(err_code);
		}else{
			adc_init(TMP36_ADC_PIN,ADC_CONFIG_INPSEL_AnalogInputTwoThirdsPrescaling);
		}	
	}
}

static void sensor_timer_init(void)
{
		uint32_t err_code;	
		err_code = app_timer_create(&iss_struct.meas_timer,APP_TIMER_MODE_SINGLE_SHOT,temp_adc_sampling);
		APP_ERROR_CHECK(err_code);
}


static void sensor_timer_start(uint16_t offset)
{
		uint32_t err_code = app_timer_start(iss_struct.meas_timer, APP_TIMER_TICKS(offset ? offset : iss_struct.samp_freq_in_m_sec, APP_TIMER_PRESCALER), &iss_struct);
		APP_ERROR_CHECK(err_code);
		iss_struct.timer_running = true;
}

static void sensor_timer_stop(void)
{
		uint32_t err_code = app_timer_stop(iss_struct.meas_timer);
		APP_ERROR_CHECK(err_code);
		iss_struct.timer_running = false;
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
		  advertising_init();
	 }   
}

static void init(void){
	//The analog input pin (26) is configred in adc_init() function
	nrf_gpio_cfg_output(TMP36_VCC_PIN);
		
	iss_struct.p_update_samp_freq = update_measurement_samp_freq;	
	iss_struct.coord = TMP36_COORDINATE;
	iss_struct.type = BLE_UUID_ITU_SENSOR_TYPE_TEMPERATURE;
	iss_struct.make = BLE_UUID_ITU_SENSOR_MAKE_TMP36;
}
