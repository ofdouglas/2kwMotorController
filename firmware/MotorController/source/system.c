/*
 * system.c - Top-level management of controller configuration, status and faults
 *
 *  Created on: Jun 26, 2017
 *      Author: Oliver Douglas
 */

#include "system.h"
#include "debug.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"


// File index for ASSERT() macro
FILENUM(2)

/******************************************************************************
 * System state variables
 *****************************************************************************/
//static int system_state = STATE_CONFIG;
static int system_state = STATE_RUNNING;
static int system_fault_flags = 0;

static int motor_control_mode = CTRL_OPEN_LOOP;
static int hbridge_drive_mode = DRIVE_ASYNC_SIGN_MAG;
static float motor_target_value = 0;

static int sensor_update_mode = 0;


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
  return system_state;
}

void system_task_code(void * arg)
{
    vTaskDelay(2000);

    /*
    int duty[5] = {5, 10, 20, 30, 40};
    while (1) {
        for (int i = 0; i <= 5; i++) {
            motor_target_value = duty[i];
            vTaskDelay(1000);
        }
        motor_target_value = 0;
        vTaskDelay(1000);
        for (int i = 0; i <= 5; i++) {
            motor_target_value = -duty[i];
            vTaskDelay(1000);
        }
        motor_target_value = 0;
        vTaskDelay(1000);
    }
    */

    motor_target_value = 15;
    while (1)
        vTaskDelay(1000);
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


/******************************************************************************
 * Motor control commands
 *****************************************************************************/
int system_get_control_mode(void)
{
    return motor_control_mode;
}

int system_get_drive_mode(void)
{
    return hbridge_drive_mode;
}

float system_get_control_target(void)
{
    return motor_target_value;
}

int system_get_update_mode(void)
{
    return sensor_update_mode;
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


