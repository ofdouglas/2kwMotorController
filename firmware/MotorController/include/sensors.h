/*
 * sensors.h
 *
 *  Created on: Oct 20, 2017
 *      Author: odougs
 */

#ifndef INCLUDE_SENSORS_H_
#define INCLUDE_SENSORS_H_

#include "stdinclude.h"


float sensor_get_motor_current_amps(void);
float sensor_get_vbus_volts(void);
float sensor_get_vbatt_volts(void);
float sensor_get_motor_temp_celsius(void);
float sensor_get_hbridge_temp_celsius(void);


#endif /* INCLUDE_SENSORS_H_ */
