#include "HT_sensor_SI7021_config.h"
#include "twi_master_int.h"
#include "HT_sensor_SI7021.h"
#include "app_timer.h"
#include "sensor_service.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include <math.h>
#include "nordic_common.h"
#include "nrf_gpio.h"
#include "itu_service.h"

#define HT_sens_adr TSL2561_ADDR_FLOAT

static uint8_t twi_data_tx[2] = {0,0};
static uint8_t twi_data_rx[2] = {0,0};
static twi_config_t my_twi_config;
static app_timer_id_t twi_timer;
static iss_t iss_struct_h; 
static iss_t iss_struct_t; 
static bool first_run = true;
static uint16_t last_temp_value = 0;
static itu_service_t humidity_sensor ={.timer_init = sensor_timer_init,
																		.timer_start = sensor_timer_start,
																		.init = init,
																		.ble_evt = sensor_ble_evt,
																		.service = &iss_struct_h,
																		.service_type = 0,
																		.needs_adc = false,
																		.adc_done = NULL
																		};
static itu_service_t temp_sensor ={.timer_init = sensor_timer_init,
																		.timer_start = sensor_timer_start,
																		.init = init,
																		.ble_evt = sensor_ble_evt,
																		.service = &iss_struct_t,
																		.service_type = 0,
																		.needs_adc = false,
																		.adc_done = NULL
																		};

itu_service_t * getHumiditySensorSI7021(void){
	return &humidity_sensor;
}

itu_service_t * getTempSensorSI7021(void){
	return &temp_sensor;
}


void twi_ht_start_measuring_sche(void *data, uint16_t size){
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
	app_timer_start(twi_timer, APP_TIMER_TICKS(50, APP_TIMER_PRESCALER), NULL);
}

static void twi_ht_start_measuring(void * p_context){
	UNUSED_PARAMETER(p_context);	
	if(do_measurements)
	app_sched_event_put(NULL, 0, twi_ht_start_measuring_sche);
}


void twi_ht_read_measurement_sche(void *data, uint16_t size){
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
	
	uint16_t hum = 0;
	if(first_run){
		first_run = false;
		twi_data_tx[0] = SI7021_WRITE_USER_REG;	
		twi_data_tx[1] = SI7021_USER_REG_CONFIG;
		twi_master_write(SI7021_ADR, twi_data_tx, 2,&my_twi_config);
	}
	twi_data_tx[0] = SI7021_MEASURE_RH_COM;	
	twi_master_write_read(SI7021_ADR, twi_data_tx, 1, twi_data_rx, 2,&my_twi_config);
	hum = twi_data_rx[0]<< 8 | twi_data_rx[1];
	
	twi_data_tx[0] = SI7021_GET_TEMP_COM;	
	twi_master_write_read(SI7021_ADR, twi_data_tx, 1, twi_data_rx, 2,&my_twi_config);
	last_temp_value = (twi_data_rx[0] << 8) | twi_data_rx[1];
	
	int32_t humidity = (((125 * hum)/65536)-6);
	humidity *= 100;
	update_iss_measurement(&iss_struct_h, &humidity);
}


static void twi_ht_read_measurement(void * p_context){
	UNUSED_PARAMETER(p_context);	
	app_sched_event_put(NULL, 0, twi_ht_read_measurement_sche);
}

void send_temp_update_sche(void *data, uint16_t size){
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(size);
	int32_t temperature = ((175.72 * last_temp_value)/65536)-46.85;
	temperature *= 100;
	update_iss_measurement(&iss_struct_t, &temperature);
}

static void send_temp_update(void * p_context){
	UNUSED_PARAMETER(p_context);	
	app_sched_event_put(NULL, 0, send_temp_update_sche);
}

static void sensor_timer_init(void)
{
		uint32_t err_code;	
		err_code = app_timer_create(&iss_struct_h.meas_timer,APP_TIMER_MODE_REPEATED,twi_ht_start_measuring);
		APP_ERROR_CHECK(err_code);
		err_code = app_timer_create(&iss_struct_t.meas_timer,APP_TIMER_MODE_REPEATED,send_temp_update);
		APP_ERROR_CHECK(err_code);
		err_code = app_timer_create(&twi_timer,APP_TIMER_MODE_SINGLE_SHOT,twi_ht_read_measurement);
		APP_ERROR_CHECK(err_code);
}


static void sensor_timer_start(void)
{
		uint32_t err_code = app_timer_start(iss_struct_h.meas_timer, APP_TIMER_TICKS(iss_struct_h.samp_freq_in_m_sec, APP_TIMER_PRESCALER), &iss_struct_h);
		APP_ERROR_CHECK(err_code);	
		err_code = app_timer_start(iss_struct_t.meas_timer, APP_TIMER_TICKS(iss_struct_t.samp_freq_in_m_sec, APP_TIMER_PRESCALER), &iss_struct_t);
		APP_ERROR_CHECK(err_code);
		iss_struct_h.timer_running = true;
		iss_struct_t.timer_running = true;
}


static void sensor_ble_evt(ble_evt_t * p_ble_evt)
{
		iss_on_ble_evt(&iss_struct_h,p_ble_evt);
		iss_on_ble_evt(&iss_struct_t,p_ble_evt);
}

static void update_measurement_samp_freq_h(){
	 if(iss_struct_h.timer_running){
		  uint32_t err_code;
			err_code = app_timer_stop(iss_struct_h.meas_timer);
			APP_ERROR_CHECK(err_code);
			err_code = app_timer_start(iss_struct_h.meas_timer, APP_TIMER_TICKS(iss_struct_h.samp_freq_in_m_sec, APP_TIMER_PRESCALER), &iss_struct_h);
			APP_ERROR_CHECK(err_code);
	 }   
}

static void update_measurement_samp_freq_t(){
	 if(iss_struct_t.timer_running){
		  uint32_t err_code;
			err_code = app_timer_stop(iss_struct_t.meas_timer);
			APP_ERROR_CHECK(err_code);
			err_code = app_timer_start(iss_struct_t.meas_timer, APP_TIMER_TICKS(iss_struct_t.samp_freq_in_m_sec, APP_TIMER_PRESCALER), &iss_struct_t);
			APP_ERROR_CHECK(err_code);
	 }   
}


static void init(void){
	NRF_GPIO->PIN_CNF[HT_VCC_PIN] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                            | (GPIO_PIN_CNF_DRIVE_S0H1 << GPIO_PIN_CNF_DRIVE_Pos)
                                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                            | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
	nrf_gpio_pin_set(HT_VCC_PIN);
	
	my_twi_config.twi_pinselect_scl = HT_TWI_SCL;
	my_twi_config.twi_pinselect_sda = HT_TWI_SDA;
	my_twi_config.frequency     = TWI_FREQ_400KHZ;
	my_twi_config.twi_ppi_ch = 1;
	my_twi_config.twi_interrupt_no = SPI1_TWI1_IRQn;
	my_twi_config.twi = NRF_TWI1;
	if(!twi_master_init(&my_twi_config))
	{
			APP_ERROR_CHECK(666);
	}
	
	iss_struct_h.p_update_samp_freq = update_measurement_samp_freq_h;
	iss_struct_h.coord = HT_COORDINATE;
	iss_struct_h.type = BLE_UUID_ITU_SENSOR_TYPE_HUMIDITY;
	iss_struct_h.make = BLE_UUID_ITU_SENSOR_MAKE_SI7021;
	
	iss_struct_t.p_update_samp_freq = update_measurement_samp_freq_t;
	iss_struct_t.coord = HT_COORDINATE;
	iss_struct_t.type = BLE_UUID_ITU_SENSOR_TYPE_TEMPERATURE;
	iss_struct_t.make = BLE_UUID_ITU_SENSOR_MAKE_SI7021;
}
