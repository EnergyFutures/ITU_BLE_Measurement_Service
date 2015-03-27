#include "motion_sensor.h"
#include "motion_sensor_config.h"
#include "app_timer.h"
#include "sensor_service.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "nrf_gpio.h"
#include "nrf_soc.h"
#include "itu_service.h"

uint8_t motion_detected = 0;
static iss_t iss_struct; 
static itu_service_t motion_sensor ={.timer_init = sensor_timer_init,
																		.timer_start = sensor_timer_start,
																		.init = init,
																		.ble_evt = sensor_ble_evt,
																		.service = &iss_struct,
																		.service_type = 0,
																		.needs_adc = false,
																		.adc_done = NULL,
																		.needs_gpiote = true,
																		.gpiote_init = gpiote_init,
																		.on_gpiote_event = on_gpiote_event
																		};

itu_service_t * getMotionSensor(void){
	return &motion_sensor;
}


static void gpiote_init(uint32_t *low_to_high,uint32_t *high_to_low){
	*low_to_high |= (1 << MOTION_SENSE_PIN);
}

static void on_gpiote_event(uint32_t *low_to_high,uint32_t *high_to_low){
	if(*low_to_high & (1 << MOTION_SENSE_PIN)){
		motion_detected++;
	}	
}

static void update_value_timer_handler_sche(void *data, uint16_t size){
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
	int32_t value = motion_detected*100; 
	update_iss_measurement(&iss_struct, &value);
	motion_detected = 0;	
}

static void update_value_timer_handler(void * p_context)
{        
	UNUSED_PARAMETER(p_context);	
	if(do_measurements){
		app_sched_event_put(NULL, 0, update_value_timer_handler_sche);
	}
}


static void sensor_timer_init(void)
{
		uint32_t err_code;	
		err_code = app_timer_create(&iss_struct.meas_timer,APP_TIMER_MODE_REPEATED,update_value_timer_handler);
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
	NRF_GPIO->PIN_CNF[MOTION_SENSE_PIN] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                        | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                        | (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)
                                        | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                        | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
	
	nrf_gpio_cfg_output(MOTION_VCC_PIN);
	nrf_gpio_pin_set(MOTION_VCC_PIN);	
		
	iss_struct.p_update_samp_freq = update_measurement_samp_freq;	
	iss_struct.coord = MOTION_COORDINATE;
	iss_struct.type = BLE_UUID_ITU_SENSOR_TYPE_MOTION;
	iss_struct.make = BLE_UUID_ITU_SENSOR_MAKE_EKMB1303112;
}
