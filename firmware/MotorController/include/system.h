/*
 * system.h
 *
 *  Created on: Oct 3, 2017
 *      Author: Oliver Douglas
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "stdinclude.h"

/******************************************************************************
 * System state definitions
 *****************************************************************************/
/* System state defines legal operations:
 *  STATE_CONFIG - Motor is shut down. Configuration changes may be performed.
 *  STATE_RUNNING - Motor is active. Motion commands may be performed.
 *  STATE_FAULTED - Motor is shut down. No commands may be performed.
 */
enum { STATE_CONFIG, STATE_RUNNING, STATE_FAULTED };

// System state is STATE_FAULTED while any fault flag is asserted

enum { FAULT_OVERCURRENT, FAULT_MOTOR_TEMP, FAULT_BRIDGE_TEMP,
       FAULT_CONTROL_BATT, FAULT_CAN_BUS, FAULT_ETHERNET };

// Motor control mode
enum { CTRL_OPEN_LOOP, CTRL_CURRENT, CTRL_VELOCITY, CTRL_POSITION };

// H-bridge drive mode
// Lock-antiphase drive will probably not be implemented.
enum { DRIVE_SIGN_MAG, DRIVE_ASYNC_SIGN_MAG, DRIVE_LOCK_ANTIPHASE };

// Sensor update mode
//enum { };


/******************************************************************************
 * System default parameters
 *****************************************************************************/



/******************************************************************************
 * System state functions
 *****************************************************************************/
int system_get_state(void);
int system_get_control_mode(void);
int system_get_drive_mode(void);
float system_get_control_target(void);
int system_get_update_mode(void);

/******************************************************************************
 * System configuration functions
 *****************************************************************************/
void system_init_clocks(void);
uint32_t system_get_sysclk_freq(void);


#endif /* SYSTEM_H_ */
