#ifndef ITU_main_H__
#define ITU_main_H__
#include "sensor_service.h"
#include "simple_actuator_service.h"

static void power_manage(void);
static void clear_cache(void *data, uint16_t size);
static void persist_measurement(iss_t * iss_struct);
#endif
