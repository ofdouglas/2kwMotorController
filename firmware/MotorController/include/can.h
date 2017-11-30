/*
 * can.h
 *
 *  Created on: Nov 16, 2017
 *      Author: Oliver Douglas
 */

struct can_msg {
    uint8_t can_cmd;
    uint8_t data_len;
    uint8_t data[8];
};

bool can_send(struct can_msg * msg);
bool can_recv(struct can_msg * msg, bool blocking);

/*
enum CAN_CMD {
    CMD_FAULT = 1,      // enum fault_code
    CMD_SET_REF,        // float reference
    CMD_ESTOP,          //
    CMD_FREEWHEEL,      //
    CMD_CLEAR_FAULT,    // enum fault_code

    CMD_SET_CTRL_MODE,  // enum conrol_mode
    CMD_SET_DRV_MODE,   // enum drive_mode

    //CMD_GET_STATUS,     // ctrl_mode, drive_mode, ref, time?

    CMD_SET_PARAM,      // uint8_t index, float value
    CMD_GET_PARAM,      // uint8_t index
    CMD_RET_PARAM,      // uint8_t index, float value

    CMD_TEMP_DATA,      // float hbridge_temp, float motor_temp
    CMD_VOLT_DATA,      // float vbatt, float vbus
    CMD_MOTOR_DATA,     // float current, float rpm

    CMD_REQUEST_DATA,   // enum data_type {temp/volt/motor}

    CMD_NODE_ONLINE,
};
*/



/* Possible alternative to representing all sensor data as floats. This would
 * allow {temp/volt/motor} data to be condensed into two message types: {rt/aux}.
 *  motor temp    - uint8_t (1 bit = 1 deg C)
 *  hbridge temp  - uint8_t (1 bit = 1 deg C)
 *
 *  vbatt         - uint16_t (1 bit = 1mV...
 *  vbus          - uint16_t (1 bit = 1mV... up to 65.535V)
 *
 *  motor current  - int16_t (1 bit = 10mA... up to 327 A)
 *  motor velocity - int16_t (1 bit = 1 rpm... up to 65,535 RPM)
 *
 */
