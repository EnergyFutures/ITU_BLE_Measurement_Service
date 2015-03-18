static uint32_t adc_result;
static iss_t iss_struct; 
static app_timer_id_t one_shoot_timer;
static itu_sensor_t temp_sensor1 ={.sensor_timer_init = sensor_timer_init,
																		.sensor_timer_start = sensor_timer_start,
																		.init = init,
																		.sensor_ble_evt = sensor_ble_evt,
																		.service = &iss_struct,
																		.needs_adc = true,
																		.adc_done = adc_done};


static void temp_adc_sampling_start(void * p_context)
{        
	UNUSED_PARAMETER(p_context);	
	app_sched_event_put(NULL, 0, temp_adc_sampling_start_sche);
}


static void temp_adc_sampling_set_pin(void * p_context)
{        
	UNUSED_PARAMETER(p_context);	
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