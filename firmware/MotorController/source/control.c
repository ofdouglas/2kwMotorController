/*
 * control.c
 *
 *  Created on: Oct 19, 2017
 *      Author: Oliver Douglas
 */


#include "stdinclude.h"
#include "system.h"
#include "sensors.h"
#include "encoder.h"

#include "driverlib/pwm.h"
#include "driverlib/gpio.h"


// File index for ASSERT() macro
FILENUM(4)

/* PWM Pins:
 *  M0PWM0 = PF0 (42): VBUS_DIS_MCU
 *  M0PWM2 = PF2 (44): PWM1_MCU
 *  M0PWM3 = PF3 (45): PWM2_MCU
 *  M0PWM4 = PG0 (49): PWM4_MCU
 *  M0PWM5 = PG1 (50): PWM3_MCU
 *
 *    _______
 *   |       |
 * --P1      P3--
 *   |       |
 *   |---M---|
 *   |       |
 * --P2      P4--
 *   |       |
 *   ---------
 *
 */

/* Define the timeout for sensor data to be received. We add 2 to the timeout
 * because the time until the next tick is unknown (and possibly very short).
 */
#define CONTROL_RATE_HZ         10000
#define CONTROL_PERIOD_SECONDS  (1.0 / CONTROL_RATE_HZ)
#define CONTROL_PERIOD_TICKS    (configTICK_RATE_HZ / CONTROL_RATE_HZ)
#define CONTROL_TICK_TIMEOUT    (CONTROL_PERIOD_TICKS + 2)

#define PWM_RATE_HZ             20000
#define DUTY_CYCLE_MAX          98.0

#define BLEEDER_PWM_BIT PWM_OUT_0_BIT
#define HBRIDGE_PWM_BITS        \
    (PWM_OUT_2_BIT | PWM_OUT_3_BIT | PWM_OUT_4_BIT | PWM_OUT_5_BIT)


// Bus voltage hysteresis for use of bleeder resistor
#define VBUS_HYSTERESIS         1.0
#define CURRENT_MAX_AMPS        5.0

/* Ampflow E30-400 parameters used to calculate the maximum duty cycle
 * that will not cause overcurrent. This is a temporary fix and should
 * be removed when the cascade (velocity + current) controller works.
 */
#define AMPFLOW_KV_RADS         24.87   // radians per volt-second
#define AMPFLOW_RESISTANCE      0.089   // ohms


/******************************************************************************
 * H-bridge hardware layer
 *****************************************************************************/

// Number of clock ticks per PWM period. Set at run-time because the result of
// SysCtlClockFreqSet(...) isn't guaranteed to be exact requested frequency.
uint32_t pwm_period_ticks;

void pwm_duty_update(float duty)
{
    int mode = system_get_drive_mode();
    int p1_duty, p2_duty, p3_duty, p4_duty;

    bool positive = duty >= 0;
    float abs_duty = positive ? duty : -duty;

    switch (mode) {
    case DRIVE_ASYNC_SIGN_MAG:
        if (positive) {
            p1_duty = (pwm_period_ticks * abs_duty) / 100;
            p2_duty = 1;
            p3_duty = 1;
            p4_duty = pwm_period_ticks - 1;
        } else {
            p1_duty = 1;
            p2_duty = pwm_period_ticks - 1;
            p3_duty = (pwm_period_ticks * abs_duty) / 100;
            p4_duty = 1;
        }
        if (abs_duty < 0.01) {
            p1_duty = 1;
            p2_duty = 1;
            p3_duty = 1;
            p4_duty = 1;
        }

        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, p1_duty);
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, p2_duty);
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, p4_duty);
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, p3_duty);
        break;

    // TODO: implement this
    case DRIVE_SIGN_MAG:
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 1);
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 1);
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 1);
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 1);
        break;

    // No plans to implement this drive mode yet.
    case DRIVE_LOCK_ANTIPHASE:
        ASSERT(0);
        break;

    default:
        ASSERT (0);
    }

    MAP_PWMSyncUpdate(PWM0_BASE, PWM_GEN_0_BIT | PWM_GEN_1_BIT | PWM_GEN_2_BIT);
}

// Freewheel the motor if enabled == false
static void hbridge_enabled(bool enabled)
{
    MAP_PWMOutputState(PWM0_BASE, HBRIDGE_PWM_BITS, enabled);
}

// Disconnect from the battery if connected == false
//  PA6 (40): VBUS_EN_MCU
static void hbridge_connected(bool connected)
{
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, connected ? GPIO_PIN_6: 0);
}

static void hbridge_set_bleeder_duty(float duty)
{
    ASSERT(duty >= 0. && duty <= 100.0);
    duty = duty > 98.0 ? 98.0 : duty;

    uint16_t pulse_width = (duty / 100.0) * pwm_period_ticks;
    MAP_PWMPulseWidthSet(PWM0_BASE, BLEEDER_PWM_BIT, pulse_width);
}

// Bypass soft-start resistor (used once vbus has stabilized)
//  PA7 (41): VBUS_ST_MCU
static void hbridge_soft_start_bypass(void)
{
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
}

void pwm_setup(void)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    // Count down mode, duty cycles update at end of each period
#define PWM_MODE (PWM_GEN_MODE_DOWN | PWM_GEN_MODE_SYNC | PWM_GEN_MODE_DBG_RUN)
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_MODE);
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_MODE);
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_MODE);
#undef PWM_MODE

    // Set PWM clock rate
    pwm_period_ticks = system_get_sysclk_freq() / PWM_RATE_HZ;
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, pwm_period_ticks - 1);
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, pwm_period_ticks - 1);
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, pwm_period_ticks - 1);

    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1);
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 1);
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 1);
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 1);
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 1);

    // Start timers and enable outputs
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_2);

    hbridge_enabled(true);

    // TODO: sketchy fix for mysterious bug causing H-bridge PWM pins to
    // go high inbetween configuration and first run of control loop
    pwm_duty_update(0);
}


/******************************************************************************
 * Math functions for control task
 *****************************************************************************/
// PID state. This is cleared by pid_state_reset when control modes are changed
static float current_accumulator;
static float velocity_accumulator;
static float position_accumulator;

// PID gains
// TODO: these can be modified by configuration commands
#define CURRENT_KI  (10.0 * CONTROL_PERIOD_SECONDS)
#define CURRENT_KP  (100.0 * CONTROL_PERIOD_SECONDS)
#define VELOCITY_KI 0.01
#define VELOCITY_KP 0.01

float pid_current_loop(float error_signal)
{
    current_accumulator += error_signal;
    float i_term = CURRENT_KI * current_accumulator;
    float p_term = CURRENT_KP * error_signal;
    float d_term = 0;
    return i_term + p_term + d_term;
}

float pid_velocity_loop(float error_signal)
{
    velocity_accumulator += error_signal;
    float i_term = VELOCITY_KI * velocity_accumulator;
    float p_term = VELOCITY_KP * error_signal;
    float d_term = 0;
    return i_term + p_term + d_term;
}

void pid_state_reset(void)
{
    current_accumulator = 0;
    velocity_accumulator = 0;
    position_accumulator = 0;
}

// Clamp abs(duty) to DUTY_CYCLE_MAX
static float duty_cycle_clamp(float duty, float max)
{
    if (max > DUTY_CYCLE_MAX)
        max = DUTY_CYCLE_MAX;

    if (duty > 0)
        duty = duty > max ? max : duty;
    else
        duty = duty < -max ? -max : duty;

    return duty;
}


/******************************************************************************
 * Control task
 *****************************************************************************/

/* Prevent current from flowing back into the h-bridge supply during
 * regenerative braking, by disconnecting the h-bridge from the supply.
 * Keep VBUS from overcharging with hysteresis-based PWM control of the
 * h-bridge bleeder resistor. After a regen event, wait until VBUS is at a
 * safe level before reconnecting the h-hbridge to the supply.
 */
static void hbridge_regen_control(float duty, float current, float vbus)
{
    //TODO: vbus_nominal should be determined from sensor data.
    float vbus_nominal = system_get_bus_voltage();

    if ((duty > 0 && current < 0) || (duty < 0 && current > 0))
        hbridge_connected(false);
    else if (vbus < vbus_nominal + VBUS_HYSTERESIS)
        hbridge_connected(true);

    if (vbus > vbus_nominal + VBUS_HYSTERESIS)
        hbridge_set_bleeder_duty(100.0);
    else if (vbus < vbus_nominal - VBUS_HYSTERESIS)
        hbridge_set_bleeder_duty(0.0);
}

// TODO: implement this
static void hbridge_safety_control(float current, float hbridge_temp,
                                   float motor_temp)
{
    if (current > CURRENT_MAX_AMPS || current < -CURRENT_MAX_AMPS) {
        hbridge_enabled(false);
        // system_raise_fault(FAULT_OVERCURRENT);
    }
}

// Get control mode from system module. Clear PID state upon entry to
// closed-loop modes to avoid glitches from residual PID state.
static int update_control_mode(void)
{
    static int prev_control_mode = CTRL_OPEN_LOOP;
    int control_mode = system_get_control_mode();

    if (prev_control_mode == CTRL_OPEN_LOOP &&
            control_mode != CTRL_OPEN_LOOP)
        pid_state_reset();
    prev_control_mode = control_mode;

    return control_mode;
}

void hbridge_soft_start(void)
{
    // Crude soft start mechanism.
    // TODO: check VBUS sensor to confirm VBUS stabilized before proceeding
    hbridge_connected(true);
    vTaskDelay(1000);
    hbridge_soft_start_bypass();
}

void control_task_code(void * arg)
{
    pwm_setup();
    hbridge_soft_start();

    while (1) {
        bool timeout = !ulTaskNotifyTake(true, CONTROL_TICK_TIMEOUT);
        ASSERT(timeout == false);
        debug_pins_set(0x04, 0x04);

        // read sensor data
        float motor_current = sensor_get_motor_current_amps();
        float bus_voltage = sensor_get_vbus_volts();
        float motor_velocity;
        bool new_velocity = encoder_poll_motor_velocity_rads(&motor_velocity);

        // hbridge_safety_control();
        // hbridge_regen_control(duty_cycle, motor_current, bus_voltage);

        static float duty_cycle;
        float control_target = system_get_control_target();
        int control_mode = update_control_mode();

        switch(control_mode) {
        case CTRL_OPEN_LOOP:
            duty_cycle = control_target;
            break;

        case CTRL_CURRENT:
            duty_cycle = pid_current_loop(control_target - motor_current);
            break;

        case CTRL_VELOCITY:
            if (new_velocity)
                duty_cycle = pid_velocity_loop(control_target - motor_velocity);
            break;

        case CTRL_POSITION:
            //duty_cycle = pid_position_loop(control_target - motor_position);
            break;

        default:
            ASSERT(0);
        }

        /*
        float duty_max = (CURRENT_MAX_AMPS * AMPFLOW_RESISTANCE +
                motor_velocity * AMPFLOW_KV_RADS) / bus_voltage;
         */

        duty_cycle = duty_cycle_clamp(duty_cycle, DUTY_CYCLE_MAX);
        if (system_get_state() == STATE_RUNNING)
            pwm_duty_update(duty_cycle);
        if (system_get_state() == STATE_FAULTED)
            hbridge_enabled(false);

        debug_pins_set(0x04, 0x00);
    }
}
