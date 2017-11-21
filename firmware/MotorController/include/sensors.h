/*
 * sensors.h
 *
 *  Created on: Oct 20, 2017
 *      Author: odougs
 */

#ifndef INCLUDE_SENSORS_H_
#define INCLUDE_SENSORS_H_

#include "stdinclude.h"


float sensor_get_motor_current(void);
float sensor_get_bus_voltage(void);
float sensor_get_battery_voltage(void);
float sensor_get_motor_temperature(void);
float sensor_get_hbridge_temperature(void);
void sensors_setup(void);

#endif /* INCLUDE_SENSORS_H_ */
