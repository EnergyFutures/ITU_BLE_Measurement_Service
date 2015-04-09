#ifndef ITU_main_H__
#define ITU_main_H__
#include "ble.h"
#include "sensor_service.h"
#include "simple_actuator_service.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "pstorage.h"

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

extern bool do_measurements;
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

typedef struct storage_struct
{ 
	pstorage_handle_t pstorage_handle;
	uint16_t current_block;
	uint8_t current_offset;
	uint16_t current_off_loading_block;	
	bool pstorage_wait_flag;
	bool pstorage_clearing;	
	pstorage_block_t pstorage_wait_handle;
	void (*clear_cache)(void *data, uint16_t size);
} storage_struct_t;

typedef struct mote_config_struct
{ 
	char device_name [13];
	char location_name [13];
	uint8_t adv_freq_sec;
	uint8_t block_count_percent_for_buffer_full;	
} mote_config_struct_t;


static void power_manage(void);
static void clear_cache(void *data, uint16_t size);
static void persist_measurement(iss_t * iss_struct);
void advertising_init(void);
#endif
