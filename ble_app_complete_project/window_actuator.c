#include "window_actuator.h"
#include "window_actuator_config.h"
#include "app_timer.h"
#include "simple_actuator_service.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "nrf_gpio.h"
#include "nrf_soc.h"
#include <math.h>
#include "nrf_delay.h"

static ias_t ias_struct; 
static uint8_t pin = WINDOW_STOP_PIN;
static itu_service_t window_actuator =
 {.timer_init = actuator_timer_init,
	.timer_start = actuator_timer_start,
	 .timer_stop = actuator_timer_stop,
	.init = init,
	.ble_evt = actuator_ble_evt,
	.service = &ias_struct,
	.service_type = 1,
	.needs_adc = false,
	.adc_done = NULL,
	.needs_gpiote = false,
	.gpiote_init = NULL,
	.on_gpiote_event = NULL
};

itu_service_t * get_window_actuator(void){
	return &window_actuator;
}

static void actuator_timer_init(void){}

static void actuator_timer_start(uint16_t offset){}

static void actuator_timer_stop(void){}

static void actuator_ble_evt(ble_evt_t * p_ble_evt)
{
		ias_on_ble_evt(&ias_struct,p_ble_evt);
}

static void update_actuator(ias_t * ias_struct){
	if(ias_struct->value  == 1){
		nrf_gpio_pin_set(WINDOW_OPEN_PIN);
		pin = WINDOW_OPEN_PIN;
	}else if(ias_struct->value  == 0){
		nrf_gpio_pin_set(WINDOW_CLOSE_PIN);
		pin = WINDOW_CLOSE_PIN;
	}else{
		nrf_gpio_pin_set(WINDOW_STOP_PIN);
		pin = WINDOW_STOP_PIN;
	}
	nrf_delay_ms(100);
	nrf_gpio_pin_clear(pin);
}
	
static void init(void){
	nrf_gpio_cfg_output(WINDOW_OPEN_PIN);
	nrf_gpio_cfg_output(WINDOW_CLOSE_PIN);
	nrf_gpio_cfg_output(WINDOW_STOP_PIN);
	ias_struct.p_update_status = update_actuator;
	ias_struct.coord = WINDOW_COORDINATE;
	ias_struct.type = WINDOW_TYPE;
	ias_struct.make = WINDOW_MAKE;
}
