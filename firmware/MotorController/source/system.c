/*
 * system.c - Top-level management of controller configuration, status and faults
 *
 *  Created on: Jun 26, 2017
 *      Author: Oliver Douglas
 */

#include "system.h"
#include "can.h"
#include "debug.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"


// File index for ASSERT() macro
FILENUM(2)


typedef union {
    int i;
    float f;
} union32;

/*
 * TODO: read these from EEPROM instead of using default initializers.
 * (When making that change, these tables can become flash-resident backups
 * that are recovered when factory reset is activated.
 */
static union32 config_registers[NUM_CONFIG_REGISTERS] =
{
 [REG_CURRENT_KP].f = 1.00,
 [REG_CURRENT_KI].f = 0.10,
 [REG_CURRENT_KD].f = 0.10,
 [REG_VELOCITY_KI].f = 0.01,
 [REG_VELOCITY_KP].f = 0.01,
 [REG_VELOCITY_KD].f = 0.00,
 [REG_POSITION_KP].f = 0,
 [REG_POSITION_KI].f = 0,
 [REG_POSITION_KD].f = 0,

 [REG_CURRENT_DEADBAND].f = 0,
 [REG_VELOCITY_DEADBAND].f = 0,
 [REG_POSITION_DEADBAND].f = 0,

 [REG_MAX_CURRENT].f = 10,
 [REG_MAX_VOLTAGE].f = 24,
 [REG_MAX_MOTOR_TEMP].f = 100,
 [REG_MAX_HBRIDGE_TEMP].f = 100,
 //[REG_MAX_ACCEL_RATE].f = 0

 [REG_NODE_ID].i = 1,
 [REG_CAN_BAUD_RATE].i = 125000,
 [REG_UART_BAUD_RATE].i = 921600,
 [REG_SENSOR_LOG_ENABLES].i = 0x0,
};

static union32 state_registers[NUM_STATE_REGISTERS] =
{
 [REG_SYSTEM_STATE].i = STATE_CONFIG,
 [REG_FAULT_FLAGS].i = 0x0,
 [REG_CONTROL_MODE].i = CTRL_OPEN_LOOP,
 [REG_CONTROL_TARGET].f = 0,
 [REG_DRIVE_MODE].i = DRIVE_FREEWHEEL
};


/******************************************************************************
 * System fault management
 *****************************************************************************/
/*
void system_raise_fault(int fault)
{
  system_fault_flags |= 1 << fault;

  motor_shutdown();
  system_state = STATE_FAULTED;
  transmit_msg(MSG_FAULT, system_fault_flags);
}

void system_lower_fault(int fault)
{
  system_fault_flags &= ~(1 << fault);

  if (system_fault_flags == 0) {
    motor_power_up();
    system_state = STATE_RUNNING;
  }

  transmit_msg(MSG_OK);
}
*/

int system_get_state(void)
{
  return state_registers[REG_SYSTEM_STATE].i;
}

void system_task_code(void * arg)
{
    struct can_msg msg;

    while (1) {
        if (can_recv(&msg)) {
            can_send(&msg);
         //   memcpy(&arg, can.data, 4);
            //log_msg("received %d\n", arg, 0, 0);
        }
        vTaskDelay(50);
    }
}


/******************************************************************************
 * System configuration
 *****************************************************************************/

static uint32_t sysclk_freq;

#define SYSCTL_CLOCK_CONFIG (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN \
                             | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480)

void system_init_clocks(void)
{
    sysclk_freq = MAP_SysCtlClockFreqSet(SYSCTL_CLOCK_CONFIG, 120000000);
}

uint32_t system_get_sysclk_freq(void)
{
    return sysclk_freq;
}

int system_get_node_id(void)
{
    return 1;
}

/******************************************************************************
 * Motor control commands
 *****************************************************************************/
int system_get_control_mode(void)
{
    return state_registers[REG_CONTROL_MODE].i;
}

int system_get_drive_mode(void)
{
    return state_registers[REG_DRIVE_MODE].i;
}

float system_get_control_target(void)
{
    return state_registers[REG_CONTROL_TARGET].f;
}

int system_get_update_mode(void)
{
    return config_registers[REG_SENSOR_LOG_ENABLES].i;
}

/* TODO: This is supposed to be the 'normal' bus voltage, e.g. the average bus
 * voltage when regeneration is not occurring. However, the 'normal' bus
 * voltage changes with current supplied (battery resistance) and battery state
 * of charge, so this value must be updated periodically from average bus
 * voltage... but we should ignore samples of VBUS taken while in regen mode!
 *
 */
float system_get_bus_voltage(void)
{
    return 0;
}






/* Temporary crap - get rid of this!
 *
 *

#define CMD_BUF_LEN 40
#include "driverlib/uart.h"

char cmdbuf[CMD_BUF_LEN];


int atoi(const char * s)
{
    int sign = 1;
    int res = 0;

    if (*s == '-') {
        sign = -1;
        s++;
    }

    while (*s) {
        res = res*10 + *s - '0';
        s++;
    }

    return res * sign;
}


void process_cmd(void)
{
    if (cmdbuf[0] == 0 || cmdbuf[1] == 0)
        return;

    uint16_t cmd = cmdbuf[0] * 256 + cmdbuf[1];
    int arg = atoi(cmdbuf + 2);

    switch(cmd) {
    case 's' * 256 + 't':   // set target
    if (motor_control_mode == CTRL_VELOCITY)
        motor_target_value = arg * RPM_TO_RADS;
    else
        motor_target_value = arg;
    break;

    case 'c' * 256 + 'm':   // change mode
        if (arg == CTRL_OPEN_LOOP)
            motor_control_mode = CTRL_OPEN_LOOP;
        else if (arg == CTRL_CURRENT)
            motor_control_mode = CTRL_CURRENT;
        else if (arg == CTRL_VELOCITY)
            motor_control_mode = CTRL_VELOCITY;
        break;

    case 'h' * 256 + 'l':   // halt
        motor_target_value = 0;
    break;
    }
}

void system_task_code(void * arg)
{
    int index = 0;
    int c;

    while (1) {
        while ((c = UARTCharGetNonBlocking(UART3_BASE)) == -1)
            vTaskDelay(10);

        if (c == '\r') {
            cmdbuf[index] = '\0';
            process_cmd();
            index = 0;
        } else if (c == '\n') {
            ;
        } else {
            cmdbuf[index] = c;
            index++;
            if (index == CMD_BUF_LEN)
                index = 0;
        }
    }
*/
