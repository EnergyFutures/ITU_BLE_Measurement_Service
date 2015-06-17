#ifndef itu_service_H__
#define itu_service_H__
#include <stdint.h>
#include <stdbool.h>
#include "pstorage.h"
#include "ble.h"
#include "nrf.h"

/**< to avoid float we multiply the temperature with 100, hence the exponent is always 2 */
#define ITU_IEEE_FLOAT32_DEFAULT_EXPONENT 						-2    
#define INVALID_ITU_MEASUREMENT_VALUE 								0x00FFFFFF

#define BLE_UUID_ITU_MEASUREMENT_VALUE_CHAR 					0xFF00
#define BLE_UUID_ITU_MEASUREMENT_CONFIG_CHAR					0xFF01
#define BLE_UUID_ITU_ACTUATOR_COMMAND_CHAR						0xFF02
#define BLE_UUID_ITU_ACTUATOR_JSON_CHAR								0xFF03
#define BLE_UUID_ITU_READ_ALL_MEASUREMENTS_VALUE_CHAR	0xFF04
#define BLE_UUID_ITU_CONFIG_MOTE_CHAR									0xFF05

#define BLE_UUID_ITU_MEASUREMENT_SERVICE							0xFFA0
#define BLE_UUID_ITU_ACTUATOR_SERVICE									0xFFA1
#define BLE_UUID_ITU_MOTE_SERVICE											0xFFA2

#define BLE_UUID_ITU_SENSOR_TYPE_NOT_SET							0x00
#define BLE_UUID_ITU_SENSOR_TYPE_TEMPERATURE					0x01
#define BLE_UUID_ITU_SENSOR_TYPE_LIGHT								0x02
#define BLE_UUID_ITU_SENSOR_TYPE_SOUND								0x03
#define BLE_UUID_ITU_SENSOR_TYPE_HUMIDITY							0x04
#define BLE_UUID_ITU_SENSOR_TYPE_MOTION								0x05
#define BLE_UUID_ITU_ACTUATOR_TYPE_NOT_SET						0x06
#define BLE_UUID_ITU_ACTUATOR_TYPE_WINDOW							0x07
#define BLE_UUID_ITU_ACTUATOR_TYPE_AC									0x08
#define BLE_UUID_ITU_SENSOR_TYPE_AMPERE								0x09

#define BLE_UUID_ITU_SENSOR_MAKE_NOT_SET							0x00
#define BLE_UUID_ITU_SENSOR_MAKE_DHT22								0x01
#define BLE_UUID_ITU_SENSOR_MAKE_TMP36								0x02
#define BLE_UUID_ITU_SENSOR_MAKE_LMT85								0x03
#define BLE_UUID_ITU_SENSOR_MAKE_TSL2561							0x04
#define BLE_UUID_ITU_SENSOR_MAKE_SI7021								0x05
#define BLE_UUID_ITU_SENSOR_MAKE_EKMB1303112					0X06
#define BLE_UUID_ITU_ACTUATOR_MAKE_NOT_SET						0x07
#define BLE_UUID_ITU_ACTUATOR_MAKE_SSD_RELAY					0x08
#define BLE_UUID_ITU_ACTUATOR_MAKE_BISTABLE_RELAY			0x09
#define BLE_UUID_ITU_ACTUATOR_MAKE_MECH_RELAY					0x0A
#define BLE_UUID_ITU_SENSOR_MAKE_ACS712_5A						0x0B

// Broadcast flag bits
#define ITS_BROADC_FLAG_NAME_BIT											BIT_0   
#define ITS_BROADC_FLAG_LOCATION_BIT									BIT_1
#define ITS_BROADC_FLAG_TYPE_BIT											BIT_2             
#define ITS_BROADC_FLAG_VALUE_BIT											BIT_3
#define ITS_BROADC_FLAG_COORDINATE_BIT								BIT_4
#define ITS_BROADC_FLAG_MISC_BIT											BIT_5 



#define ITU_COMPANY_ID																0xDDDD

#define ITS_MEAS_LOCATION_NOT_SET											0x00
#define	ITS_MEAS_LOCATION_IN_SOMEWHERE								0x01
#define	ITS_MEAS_LOCATION_IN_FLOOR										0x02
#define	ITS_MEAS_LOCATION_IN_MIDDLE										0x03
#define	ITS_MEAS_LOCATION_IN_CEILING									0x04
#define	ITS_MEAS_LOCATION_OUT													0x05


#define MAXIMUM_LOCATION_DEVICE_NAME_LENGTH 					12


#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            7                                         	/**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         1                                           /**< Size of timer operation queues. */
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                5                                         	/**< Maximum number of events in the scheduler queue. */


//Should give about 1 sec conn_interval and at least 6 tries before the conn is abonded
//#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(45, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (0.200 seconds) */
//#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(65, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (0,250 second). */
//#define SLAVE_LATENCY                        19                                          /**< Slave latency. */
//#define CONN_SUP_TIMEOUT                     MSEC_TO_UNITS(10000, UNIT_10_MS)            /**< Connection supervisory timeout (6,3 seconds). */

//FOR NOW, THE CONNECTION STATE SHOULD BE AS SHORT AS POSSIBLE
#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(7.5, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (0.200 seconds) */
#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(30, UNIT_1_25_MS)             	/**< Maximum acceptable connection interval (0,250 second). */
#define SLAVE_LATENCY                        0                                          	/**< Slave latency. */
#define CONN_SUP_TIMEOUT                     MSEC_TO_UNITS(250, UNIT_10_MS)            		/**< Connection supervisory timeout (6,3 seconds). */

//Master controls conn_interval... we ask to upgrade the conn as soon as possible
#define FIRST_CONN_PARAMS_UPDATE_DELAY  		 APP_TIMER_TICKS(1, APP_TIMER_PRESCALER)  		/**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   		 APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) 	/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    		 2                                           	/**< Number of attempts before giving up the connection parameter negotiation. */


//Extract one bit from a number
#define BIT_VALUE(n,bv) ((n >> bv) & 1)



/**@brief FLOAT format (IEEE-11073 32-bit FLOAT, defined as a 32-bit value with a 24-bit mantissa
 *        and an 8-bit exponent. */
typedef struct
{
  int8_t  exponent;                                                         /**< Base 10 exponent */
  int32_t mantissa;                                                         /**< Mantissa, should be using only 24 bits */
} ieee_float32_t;


typedef struct storage_struct
{ 
	pstorage_handle_t pstorage_handle;
	pstorage_handle_t config_handle;
	uint16_t current_block;
	uint8_t current_offset;
	uint16_t current_off_loading_block;	
	bool pstorage_wait_flag;
	bool pstorage_clearing;	
	bool config_clearing;
	pstorage_block_t pstorage_wait_handle;
	pstorage_block_t config_wait_handle;
	void (*clear_cache)(void *data, uint16_t size);
	void (*persist_config)(void);
} storage_struct_t;

typedef struct mote_config_struct
{ 
	char device_name [13];
	char location_name [13];
	uint8_t adv_freq_sec;
	uint8_t block_count_percent_for_buffer_full;	
	int8_t conn_trans_power;
	int8_t non_conn_trans_power;
	bool force_connectable;
} mote_config_struct_t;


typedef struct itu_service
{
    void (*timer_init)(void);									 
		void (*timer_start)(uint16_t offset);
		void (*timer_stop)(void);	
		void (*init)(void);									 					 		
		void (*ble_evt)(ble_evt_t * p_ble_evt);		 
		void *service;	/**< The service*/
		bool (*adc_done)(uint8_t pin);
		void (*gpiote_init)(uint32_t *low_to_high,uint32_t *high_to_low);
		void (*on_gpiote_event)(uint32_t *low_to_high,uint32_t *high_to_low);
		uint8_t service_type: 1;
		bool needs_adc : 1;
		bool needs_gpiote : 1;
} itu_service_t;

/**@brief Function for initializing the ADC.
 */
inline void adc_init(uint8_t pin,uint8_t scaling)
{		
	while(NRF_ADC->BUSY){;}
	NRF_ADC->CONFIG	= (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) 										/* Analog reference inputs disabled. */
									| (pin << ADC_CONFIG_PSEL_Pos)																								/*!< Use PIN as analog input. */
									| (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)														/*!< Use internal 1.2V bandgap voltage as reference for conversion. */
									| (scaling << ADC_CONFIG_INPSEL_Pos)																					/*!< Analog input specified by PSEL with 2/3 prescaler used as input for the conversion. */
									| (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos);																/*!< 10bit ADC resolution. */ 
	/* Enable ADC and set analog pin as input*/
	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
	NRF_ADC->TASKS_START = 1;							//Start ADC sampling
}

void advertising_init(void);
extern bool do_measurements;
#endif // itu_service_H__
