/*
 * encoder.h
 *
 *  Created on: Nov 2, 2017
 *      Author: Oliver Douglas
 */

#ifndef INCLUDE_ENCODER_H_
#define INCLUDE_ENCODER_H_

#include "stdinclude.h"

float encoder_get_motor_position_rads(void);
float encoder_get_motor_velocity_rads(void);

#define ENCODER_GPIO_BASE   GPIO_PORTL_BASE
#define ENCODER_GPIO_PINA   GPIO_PIN_1
#define ENCODER_GPIO_INT    INT_GPIOL

#endif /* INCLUDE_ENCODER_H_ */
