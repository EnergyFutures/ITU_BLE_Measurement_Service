#ifndef ITU_main_H__
#define ITU_main_H__
#include "ble.h"
#include "sensor_service.h"
#include "simple_actuator_service.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "pstorage.h"

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            10                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         20                                           /**< Size of timer operation queues. */
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                20                                         /**< Maximum number of events in the scheduler queue. */
//STORAGE // CACHE
#define BLOCK_SIZE PSTORAGE_MIN_BLOCK_SIZE //16 bytes = 2 values... 4 bytes for value, 2 for sequence nr and 2 for id
#define BLOCK_COUNT 128 // a page is 1024 bytes... 128 * 16 = 2048 = 2 pages... 1 sensor with sampling freq of 1 min will give us around 4.3 hours of data
#define BLOCK_COUNT_PROCENT (BLOCK_COUNT * 0.5)

//Should give about 1 sec conn_interval and at least 6 tries before the conn is abonded
#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(45, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (0.200 seconds) */
#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(65, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (0,250 second). */
#define SLAVE_LATENCY                        19                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                     MSEC_TO_UNITS(10000, UNIT_10_MS)            /**< Connection supervisory timeout (6,3 seconds). */

//Master controls conn_interval... we ask to upgrade the conn as soon as possible
#define FIRST_CONN_PARAMS_UPDATE_DELAY  		 APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   		 APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    		 3                                           /**< Number of attempts before giving up the connection parameter negotiation. */


extern bool do_measurements;
typedef struct itu_service
{
    void (*timer_init)(void);									 
		void (*timer_start)(void);									 
		void (*init)(void);									 					 		
		void (*ble_evt)(ble_evt_t * p_ble_evt);		 
		void *service;	/**< The service*/
		uint8_t service_type;
		bool needs_adc;
		bool (*adc_done)(uint8_t pin);	
} itu_service_t;

/**@brief Function for initializing the ADC.
 */
inline void adc_init(uint8_t pin,uint8_t scaling)
{		
	NRF_ADC->CONFIG	= (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) 										/* Analog reference inputs disabled. */
									| (pin << ADC_CONFIG_PSEL_Pos)																								/*!< Use PIN as analog input. */
									| (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)														/*!< Use internal 1.2V bandgap voltage as reference for conversion. */
									| (scaling << ADC_CONFIG_INPSEL_Pos)																					/*!< Analog input specified by PSEL with 2/3 prescaler used as input for the conversion. */
									| (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos);																/*!< 10bit ADC resolution. */ 
	
	/* Enable ADC and set analog pin as input*/
	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
}


typedef struct iss_broadc
{ 
		bool                         type_present;   		  
		bool                         misc_present;      	          
		bool                         value_present;
		bool												 coor_present;			
} iss_broadc_t;

typedef struct storage_struct
{ 
	pstorage_handle_t pstorage_handle;
	uint16_t current_block;
	uint8_t current_offset;	
	bool pstorage_wait_flag;
	bool pstorage_clearing;	
	pstorage_block_t pstorage_wait_handle;
	void (*power_manage)(void);
	void (*clear_cache)(void);
} storage_struct_t;

static void get_all_cached_measurement_sche(void *data, uint16_t size);
static void power_manage(void);
static void clear_cache(void);
#endif
