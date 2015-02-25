#ifndef itu_service_H__
#define itu_service_H__
#include <stdint.h>
#include <stdbool.h>

/**< to avoid float we multiply the temperature with 100, hence the exponent is always 2 */
#define ITU_IEEE_FLOAT32_DEFAULT_EXPONENT  															 -2    
#define INVALID_ITU_MEASUREMENT_VALUE  																	 0x00FFFFFF

#define BLE_UUID_ITU_MEASUREMENT_VALUE_CHAR															 0xFF00
#define BLE_UUID_ITU_MEASUREMENT_CONFIG_CHAR														 0xFF01
#define BLE_UUID_ITU_ACTUATOR_COMMAND_CHAR															 0xFF02
#define BLE_UUID_ITU_ACTUATOR_JSON_CHAR																	 0xFF03
#define BLE_UUID_ITU_READ_ALL_MEASUREMENT_VALUE_CHAR										 0xFF04

#define BLE_UUID_ITU_MEASUREMENT_SERVICE                                 0xFFA0
#define BLE_UUID_ITU_ACTUATOR_SERVICE    		                             0xFFA1
#define BLE_UUID_ITU_READ_ALL_MEASUREMENT_SERVICE                        0xFFA2

#define BLE_UUID_ITU_SENSOR_TYPE_NOT_SET														 		 0x00
#define BLE_UUID_ITU_SENSOR_TYPE_TEMPERATURE														 0x01
#define BLE_UUID_ITU_SENSOR_TYPE_LIGHT													 				 0x02
#define BLE_UUID_ITU_SENSOR_TYPE_SOUND                                	 0x03
#define BLE_UUID_ITU_SENSOR_MULTI_TYPE															 		 0x00

#define BLE_UUID_ITU_SENSOR_MAKE_NOT_SET														 		 0x00
#define BLE_UUID_ITU_SENSOR_MAKE_DHT22																	 0x01
#define BLE_UUID_ITU_SENSOR_MAKE_TMP36													 				 0x02
#define BLE_UUID_ITU_SENSOR_MAKE_LMT85													 				 0x03
#define BLE_UUID_ITU_SENSOR_MAKE_TSL2561													 			 0x04

// Broadcast flag bits
#define ITS_BROADC_FLAG_NAME_BIT             		BIT_0   
#define ITS_BROADC_FLAG_LOCATION_BIT            BIT_1
#define ITS_BROADC_FLAG_TYPE_BIT             		BIT_2             
#define ITS_BROADC_FLAG_VALUE_BIT             	BIT_3
#define ITS_BROADC_FLAG_COORDINATE_BIT          BIT_4
#define ITS_BROADC_FLAG_MISC_BIT     			      BIT_5             


#define ITU_COMPANY_ID 0xDDDD



/**@brief Temperature Location types. */
typedef enum
{
    ITS_MEAS_LOCATION_NOT_SET,
		ITS_MEAS_LOCATION_IN_SOMEWHERE,
		ITS_MEAS_LOCATION_IN_FLOOR,
		ITS_MEAS_LOCATION_IN_MIDDLE,
		ITS_MEAS_LOCATION_IN_CEILING,
		ITS_MEAS_LOCATION_OUT
} its_measurement_location_t;


/**@brief FLOAT format (IEEE-11073 32-bit FLOAT, defined as a 32-bit value with a 24-bit mantissa
 *        and an 8-bit exponent. */
typedef struct
{
  int8_t  exponent;                                                         /**< Base 10 exponent */
  int32_t mantissa;                                                         /**< Mantissa, should be using only 24 bits */
} ieee_float32_t;
#endif // itu_service_H__

/** @} */
