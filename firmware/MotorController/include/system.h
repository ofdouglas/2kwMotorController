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
 * Register enumerations
 *****************************************************************************/

// Config registers can only be changed while system_state == STATE_CONFIG
enum config_registers {
    // PID controller gains
    REG_CURRENT_KP,
    REG_CURRENT_KI,
    REG_CURRENT_KD,
    REG_VELOCITY_KI,
    REG_VELOCITY_KP,
    REG_VELOCITY_KD,
    REG_POSITION_KP,
    REG_POSITION_KI,
    REG_POSITION_KD,

    // PID controller deadbands
    REG_CURRENT_DEADBAND,
    REG_VELOCITY_DEADBAND,
    REG_POSITION_DEADBAND,

    // Motor limit values
    REG_MAX_CURRENT,
    REG_MAX_VOLTAGE,
    REG_MAX_MOTOR_TEMP,
    REG_MAX_HBRIDGE_TEMP,
    //REG_MAX_ACCEL_RATE,

    // Comm. interface params
    REG_NODE_ID,
    REG_CAN_BAUD_RATE,
    REG_UART_BAUD_RATE,
    REG_SENSOR_LOG_ENABLES,

    NUM_CONFIG_REGISTERS
};

//  These registers can be modified at any time
enum state_registers {
    // System state
    REG_SYSTEM_STATE,
    REG_FAULT_FLAGS,            // read only ?

    // Motion control state
    REG_CONTROL_MODE,
    REG_CONTROL_TARGET,
    REG_DRIVE_MODE,

    NUM_STATE_REGISTERS
};


/******************************************************************************
 * State Register definitions
 *****************************************************************************/
// System state defines legal operations:
enum system_state {
    STATE_CONFIG,       // Motor inactive. Config registers can be changed.
    STATE_RUNNING,      // Motor active. Motion commands can be issued.
    STATE_FAULTED       // Motor shut down. No motion command accepted.
};

// System state is STATE_FAULTED while any fault flag is asserted
enum faults {
    FAULT_OVERCURRENT,
    FAULT_MOTOR_TEMP,
    FAULT_BRIDGE_TEMP,
    FAULT_CONTROL_BATT,
    FAULT_CAN_BUS,
};

// Motor control modes
enum control_mode {
    CTRL_OPEN_LOOP,         // control_target == duty cycle in [-100, 100]
    CTRL_CURRENT,           // control_target == current in amperes
    CTRL_VELOCITY,          // control_target == velocity in radians / sec
    CTRL_POSITION           // control target == position in radians
};

// H-bridge drive modes
enum drive_mode {
    DRIVE_FREEWHEEL,        // Drive is disabled
    DRIVE_SIGN_MAG,         // Sign-magnitude (4-quadrant PWM)
    DRIVE_ASYNC_SIGN_MAG,   // Asynchronous sign-magnitude (2-quadrant PWM)
    DRIVE_LOCK_ANTIPHASE    // will not be implemented
};


/******************************************************************************
 * Commands
 *****************************************************************************/
enum cmd {
    CMD_FAULT,          // uint8_t fault_flags
    CMD_SET_REF,        // float reference
    CMD_STOP,           // (none)
    CMD_FREEWHEEL,      // (none)

    CMD_SET_STATE,      // uint8_t state_reg, uint32_t/float value
    CMD_GET_STATE,      // uint8_t state_reg

    CMD_SENSOR_DATA,    // uint8_t sensor enum, float data
    CMD_REQUEST_DATA,   // uint8_t sensor enum

    CMD_SET_CONFIG,     // uint8_t config_reg, uint32_t / float value
    CMD_GET_CONFIG,     // uint8_t config_reg

    CMD_NODE_ONLINE     // (none)
};


enum sensor_data {
    SENSOR_CURRENT,
    SENSOR_VELOCITY,
    SENSOR_POSITION,
    SENSOR_BUS_VOLTAGE,
    SENSOR_BATTERY_VOLTAGE,
    SENSOR_MOTOR_TEMP,
    SENSOR_HBRIDGE_TEMP
};


/******************************************************************************
 * System state functions
 *****************************************************************************/
int system_get_state(void);
int system_get_control_mode(void);
int system_get_drive_mode(void);
float system_get_control_target(void);
int system_get_update_mode(void);

int system_get_node_id(void);

/******************************************************************************
 * System configuration functions
 *****************************************************************************/
void system_init_clocks(void);
uint32_t system_get_sysclk_freq(void);


#endif /* SYSTEM_H_ */
