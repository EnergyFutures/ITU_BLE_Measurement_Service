#ifndef itu_service_H__
#define itu_service_H__
#include <stdint.h>
#include <stdbool.h>

/**< to avoid float we multiply the temperature with 100, hence the exponent is always 2 */
#define ITU_IEEE_FLOAT32_DEFAULT_EXPONENT  															 -2    
#define INVALID_ITU_MEASUREMENT_VALUE  																	 0x00FFFFFF

#define BLE_UUID_ITU_MEASUREMENT_VALUE_CHAR															 0xFF00
#define BLE_UUID_ITU_MEASUREMENT_CONFIG_CHAR														 0xFF01
#define BLE_UUID_ITU_ACTUATOR_CHAR																			 0xFF02

#define BLE_UUID_ITU_MEASUREMENT_SERVICE                                 0xFFA0
#define BLE_UUID_ITU_ACTUATOR_SERVICE    		                             0xFFA1

#define BLE_UUID_ITU_SENSOR_TYPE_NOT_SET														 		 0x00
#define BLE_UUID_ITU_SENSOR_TYPE_TEMPERATURE														 0x01
#define BLE_UUID_ITU_SENSOR_TYPE_LIGHT													 				 0x02
#define BLE_UUID_ITU_SENSOR_TYPE_SOUND                                	 0x03

#define BLE_UUID_ITU_SENSOR_MAKE_NOT_SET														 		 0x00
#define BLE_UUID_ITU_SENSOR_MAKE_DHT22																	 0x01
#define BLE_UUID_ITU_SENSOR_MAKE_TMP36													 				 0x02


/**@brief Temperature Location types. */
typedef enum
{
    ITS_MEAS_LOCATION_NOT_SET,
		ITS_MEAS_LOCATION_IN_END_END_TOP, 				 /**< 3 dimension axis, always starting from the corner closest to the door */
    ITS_MEAS_LOCATION_IN_END_END_MIDDLE,
		ITS_MEAS_LOCATION_IN_END_END_FLOOR,
		ITS_MEAS_LOCATION_IN_END_MIDDLE_TOP,
		ITS_MEAS_LOCATION_IN_END_BEGIN_TOP,
		ITS_MEAS_LOCATION_IN_MIDDLE_END_TOP,
		ITS_MEAS_LOCATION_IN_BEGIN_END_TOP,
		ITS_MEAS_LOCATION_IN_BEGIN_END_MIDDLE,
		ITS_MEAS_LOCATION_OUT_WINDOW_1						/**< Always count from left to right*/
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
