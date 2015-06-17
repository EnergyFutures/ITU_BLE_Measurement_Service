#ifndef ITU_mote_config_H__
#define ITU_mote_config_H__

#define DEVICE_NAME                          "NEWBORN"			                          
#define LOCATION_NAME                        "XXX"
#define LOCATION_NAME_CONFIG                 "YYY"
#define CONNECTABLE_TRANSMIT_POWER_DB 4
#define NONCONNECTABLE_TRANSMIT_POWER_DB -12

#define DEFAULT_SAMPLING_FREQUENCY_IN_MS 60000
#define DEFAULT_ADVERTISEMENT_FREQUENCY_IN_SEC 2
//Old boards
//#define BUTTON_PIN 11
//White boards
#define BUTTON_PIN 30
//Compact boards
//#define BUTTON_PIN 29

#define START_ID 0


//REMEMBER TO REGISTER YOUR SENSORS HERE AND COUNT UP
#define SENSORS_SIZE 2 // if no sensors, then set to 0
#if(SENSORS_SIZE > 0)
//REMEMBER TO INCLUDE THE SENSORS
#include "light_sensor_TSL2561.h"
#include "HT_sensor_SI7021.h"
itu_service_t *sensors[SENSORS_SIZE];
static void registerSensors(void){		
	sensors[0] = getHumiditySensorSI7021();
	sensors[1] = getTempSensorSI7021();
	//sensors[0] = getLightSensorTSL2561();
}
#endif

//REMEMBER TO INCLUDE THE ACTUATORS
#define ACTUATORS_SIZE 0 // if no actuators, then set to 0

#define total_services_size (ACTUATORS_SIZE + SENSORS_SIZE)
static itu_service_t *all_services[total_services_size];

//STORAGE // CACHE
#define BLOCK_SIZE PSTORAGE_MIN_BLOCK_SIZE //16 bytes = 2 values... 4 bytes for value, 2 for sequence nr and 2 for id
#define BLOCK_COUNT 3200 // a page is 1024 bytes... 3200 * 16 = 51200 = 50 pages... 1 sensor with sampling freq of 1 min will give us around 106,6 hours of data (4,4 days)
#define BLOCK_COUNT_PROCENT 50
#define HALF_BLOCK_SIZE (BLOCK_SIZE/2)
#define SEQUENCE_NUMBER_OFFSET 4
#define ID_OFFSET 6
#define CONFIG_PERSIST_BLOCK_SIZE ((SENSORS_SIZE * 8) + 16 + 4)
#define CONFIG_MAGIC_1 0xF0
#define CONFIG_MAGIC_2 0x55

#define NUMBER_OF_SEC_FOR_BATTERY_CHECK 64800 // 18 hours
#endif
