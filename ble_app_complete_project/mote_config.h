#ifndef ITU_mote_config_H__
#define ITU_mote_config_H__
#include "main.h"

#define DEVICE_NAME                          "_ACTUATOR_"			                          
#define LOCATION_NAME                        "4D21"

//REMEMBER TO INCLUDE THE SENSORS
#include "light_sensor_TSL2561.h"
#include "temp_sensor_tmp36.h"
#include "HT_sensor_SI7021.h"

//REMEMBER TO REGISTER YOUR SENSORS HERE AND COUNT UP
#define sensors_size 4  // if no sensors, then set to 0
itu_service_t *sensors[4]; // if no sensors, then set to 1
static void registerSensors(void){		
	sensors[0] = getLightSensorTSL2561();
	sensors[1] = getTempSensorTmp36();
	sensors[2] = getHumiditySensorSI7021();
	sensors[3] = getTempSensorSI7021();
}


//REMEMBER TO INCLUDE THE ACTUATORS
#include "window_actuator.h"
#include "ac_actuator.h"

//REMEMBER TO REGISTER YOUR ACTUATORS HERE AND COUNT UP
itu_service_t *actuators[2]; // if no actuators, then set to 1
#define actuators_size 2 // if no actuators, then set to 0
static void registerActuators(void){
	actuators[0] = get_window_actuator();
	actuators[1] = get_ac_actuator();
}

#define total_services_size (actuators_size + sensors_size)
static itu_service_t *all_services[total_services_size];
#endif
