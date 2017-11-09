/*
 * encoder.h - Read velocity and position from a quadrature encoder
 *
 *  Created on: Nov 2, 2017
 *      Author: Oliver Douglas
 */

#ifndef INCLUDE_ENCODER_H_
#define INCLUDE_ENCODER_H_

#include "stdinclude.h"

// TODO:
// Get the motor's current position, in radians
float encoder_get_motor_position_rads(void);

// Get the motor's current velocity, in radians per second
float encoder_get_motor_velocity_rads(void);

// Get the motor's current velocity, and indicate if it has been updated
// since this function was last called (for use by control task only)
bool encoder_poll_motor_velocity_rads(float * velocity);

#define ENCODER_GPIO_BASE   GPIO_PORTL_BASE
#define ENCODER_GPIO_PINA   GPIO_PIN_1
#define ENCODER_GPIO_INT    INT_GPIOL

#endif /* INCLUDE_ENCODER_H_ */
