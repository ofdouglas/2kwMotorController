/*  system.c - Top-level management of controller configuration, status and faults
 */

#include "system.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

/* System state defines legal operations:
 *  STATE_CONFIG - Motor is shut down. Configuration changes may be performed.
 *  STATE_RUNNING - Motor is active. Motion commands may be performed.
 *  STATE_FAULTED - Motor is shut down. No commands may be performed.
 */
static int system_state;
enum { STATE_CONFIG, STATE_RUNNING, STATE_FAULTED };

// System state is STATE_FAULTED while any fault flag is asserted
static int system_fault_flags;
enum { FAULT_OVERCURRENT, FAULT_MOTOR_TEMP, FAULT_BRIDGE_TEMP,
       FAULT_CONTROL_BATT, FAULT_CAN_BUS, FAULT_ETHERNET };

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

void system_thread(void * arg)
{







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

/*
void assertion_failed(const char *file, int line)
{
  motor_shutdown();
  log_error_blocking("Assertion failed: %s:%d\n", file, line);
  reboot();
}
*/
