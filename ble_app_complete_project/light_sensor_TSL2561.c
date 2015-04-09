#include "twi_master_int.h"
#include "light_sensor_TSL2561_config.h"
#include "app_timer.h"
#include "sensor_service.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include <math.h>
#include "nordic_common.h"
#include "nrf_gpio.h"
#include "itu_service.h"

static twi_config_t my_twi_config;
static iss_t iss_struct; 
static itu_service_t light_sensor1 ={.timer_init = sensor_timer_init,
																		.timer_start = sensor_timer_start,
																		.timer_stop = sensor_timer_stop,
																		.init = init,
																		.ble_evt = sensor_ble_evt,
																		.service = &iss_struct,
																		.service_type = 0,
																		.needs_adc = false,
																		.adc_done = NULL,
																		.needs_gpiote = false,
																		.gpiote_init = NULL,
																		.on_gpiote_event = NULL
																		};

itu_service_t * getLightSensorTSL2561(void){
	return &light_sensor1;
}


static void twi_light_sampling(void * p_context){
	UNUSED_PARAMETER(p_context);	
	static bool startup = true;
	static bool set_pin = true;
	uint32_t err_code;
	if(do_measurements){
		if(set_pin){
			set_pin = false;
			//TURN ON THE TRANSISTOR AND WAIT FOR IT TO STABILIZE
			nrf_gpio_pin_set(TSL2561_BASE_PIN);
			err_code = app_timer_start(iss_struct.meas_timer, APP_TIMER_TICKS(50, APP_TIMER_PRESCALER), NULL);
			APP_ERROR_CHECK(err_code);
		}else{
			uint8_t twi_data_tx[2] = {0,0};
			uint8_t twi_data_rx[2] = {0,0};
			uint8_t light_twi_command = TSL2561_COMMAND_BIT | TSL2561_CLEAR_BIT | TSL2561_WORD_BIT;
			if(startup){
				// POWER ON and wait 450 ms
				startup = false;
				twi_data_tx[0] = light_twi_command | TSL2561_REGISTER_CONTROL;
				twi_data_tx[1] = TSL2561_CONTROL_POWERON;
				err_code = !twi_master_write(light_sens_adr, twi_data_tx, 2,&my_twi_config);
				APP_ERROR_CHECK(err_code);
				err_code = app_timer_start(iss_struct.meas_timer, APP_TIMER_TICKS(450, APP_TIMER_PRESCALER), NULL);
				APP_ERROR_CHECK(err_code);
			}else{
				uint16_t ch0,ch1 = 0;
				twi_data_tx[0] = light_twi_command | TSL2561_REGISTER_CHAN0_LOW;	
				err_code = !twi_master_write_read(light_sens_adr, twi_data_tx, 1, twi_data_rx, 2,&my_twi_config);
				APP_ERROR_CHECK(err_code);
				ch0 = twi_data_rx[0];
				ch0 = ch0 | (((uint16_t)(twi_data_rx[1]))<<8);
				
				twi_data_tx[0] = light_twi_command | TSL2561_REGISTER_CHAN1_LOW;
				err_code = !twi_master_write_read(light_sens_adr, twi_data_tx, 1, twi_data_rx, 2,&my_twi_config);
				APP_ERROR_CHECK(err_code);
				ch1 = twi_data_rx[0];
				ch1 = ch1 | (((uint16_t)(twi_data_rx[1]))<<8);
				
				nrf_gpio_pin_clear(TSL2561_BASE_PIN);

				int32_t lux = calc_lux(ch0,ch1);
				err_code = update_iss_measurement(&iss_struct, &lux);
				APP_ERROR_CHECK(err_code);
				startup = true;
				set_pin = true;
				err_code = app_timer_start(iss_struct.meas_timer, APP_TIMER_TICKS(iss_struct.samp_freq_in_m_sec, APP_TIMER_PRESCALER), &iss_struct);
				APP_ERROR_CHECK(err_code);
			}	
		}		
	}
}

static void sensor_timer_init(void)
{
		uint32_t err_code;	
		err_code = app_timer_create(&iss_struct.meas_timer,APP_TIMER_MODE_SINGLE_SHOT,twi_light_sampling);
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
	my_twi_config.twi_ppi_ch = 0;
  my_twi_config.twi = NRF_TWI0;
	
	twi_init_config_t init_config;
	init_config.frequency = TWI_FREQ_400KHZ;
	init_config.twi_pinselect_scl = light_TWI_SCL;
	init_config.twi_pinselect_sda = light_TWI_SDA;
	init_config.twi_interrupt_no = SPI0_TWI0_IRQn;
	if(!twi_master_init(&init_config,&my_twi_config))
	{
			APP_ERROR_CHECK(666);
	}
	
	iss_struct.p_update_samp_freq = update_measurement_samp_freq;
	
	NRF_GPIO->PIN_CNF[TSL2561_BASE_PIN] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                            | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                            | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
	
	iss_struct.coord = TSL2561_COORDINATE;
	iss_struct.type = BLE_UUID_ITU_SENSOR_TYPE_LIGHT;
	iss_struct.make = BLE_UUID_ITU_SENSOR_MAKE_TSL2561;
	iss_struct.IEEE_exponent = 0;
}


static uint32_t calc_lux(uint16_t ch0, uint16_t ch1)
{
  unsigned long chScale;
	unsigned long channel1;
	unsigned long channel0;
	
	chScale = (1 << TSL2561_LUX_CHSCALE);
	//if GAIN == 0X
	chScale = chScale << 4;
	// scale the channel values
	channel0 = (ch0 * chScale) >> TSL2561_LUX_CHSCALE;
	channel1 = (ch1 * chScale) >> TSL2561_LUX_CHSCALE;
	// find the ratio of the channel values (Channel1/Channel0)
	unsigned long ratio1 = 0;
	if (channel0 != 0) ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE+1)) / channel0;
	// round the ratio value
	unsigned long ratio = (ratio1 + 1) >> 1;
	unsigned int b, m;
	
	if ((ratio > 0) && (ratio <= TSL2561_LUX_K1T))
	{b=TSL2561_LUX_B1T; m=TSL2561_LUX_M1T;}
	else if (ratio <= TSL2561_LUX_K2T)
	{b=TSL2561_LUX_B2T; m=TSL2561_LUX_M2T;}
	else if (ratio <= TSL2561_LUX_K3T)
	{b=TSL2561_LUX_B3T; m=TSL2561_LUX_M3T;}
	else if (ratio <= TSL2561_LUX_K4T)
	{b=TSL2561_LUX_B4T; m=TSL2561_LUX_M4T;}
	else if (ratio <= TSL2561_LUX_K5T)
	{b=TSL2561_LUX_B5T; m=TSL2561_LUX_M5T;}
	else if (ratio <= TSL2561_LUX_K6T)
	{b=TSL2561_LUX_B6T; m=TSL2561_LUX_M6T;}
	else if (ratio <= TSL2561_LUX_K7T)
	{b=TSL2561_LUX_B7T; m=TSL2561_LUX_M7T;}
	else if (ratio > TSL2561_LUX_K8T)
	{b=TSL2561_LUX_B8T; m=TSL2561_LUX_M8T;}
	
	unsigned long temp;
	temp = ((channel0 * b) - (channel1 * m));
	// do not allow negative lux value
	if (temp < 0) temp = 0;
	// round lsb (2^(LUX_SCALE-1))
	temp += (1 << (TSL2561_LUX_LUXSCALE-1));
	// strip off fractional portion
	uint32_t lux = temp >> TSL2561_LUX_LUXSCALE;
	// Signal I2C had no errors
	return lux;
}
