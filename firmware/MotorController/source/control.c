/*
 * control.c - Motor control loop
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

/* PWM Map:
 *  M0PWM0 = PF0 (42): VBUS_DIS_MCU     // bleeder PWM
 *  M0PWM2 = PF2 (44): PWM1_MCU         // motor-drive PWM
 *  M0PWM3 = PF3 (45): PWM2_MCU         // motor-drive PWM
 *  M0PWM4 = PG0 (49): PWM4_MCU         // motor-drive PWM
 *  M0PWM5 = PG1 (50): PWM3_MCU         // motor-drive PWM
 *
 *    __VBUS__
 *   |       |
 * --P1      P3--
 *   |       |
 *   |-Motor-|
 *   |       |
 * --P2      P4--
 *   |       |
 *   -PWR_GND-
 *
 */

/******************************************************************************
 *
 * Motor control definitions
 *
 *****************************************************************************/
#define PWM_RATE_HZ             20000
#define DUTY_CYCLE_MAX          98.0        // TODO: this value is a guess
#define DUTY_CYCLE_DEADBAND     0.50        // TODO: this value is a guess
#define CONTROL_RATE_HZ         10000
#define CONTROL_PERIOD_SECONDS  (1.0 / CONTROL_RATE_HZ)
#define CONTROL_PERIOD_TICKS    (configTICK_RATE_HZ / CONTROL_RATE_HZ)
#define CONTROL_TICK_TIMEOUT    (CONTROL_PERIOD_TICKS + 2)
// Defines the timeout for sensor data to be received. We add 2 to the timeout
// because the time until the next tick is unknown (and possibly very short).

#define BLEEDER_PWM_BIT PWM_OUT_0_BIT
#define HBRIDGE_PWM_BITS        \
    (PWM_OUT_2_BIT | PWM_OUT_3_BIT | PWM_OUT_4_BIT | PWM_OUT_5_BIT)

// Bus voltage hysteresis for use of bleeder resistor
// Not yet implemented
#define VBUS_HYSTERESIS         1.0

#define FAN_HYSTERESIS          10

struct pid {
    float accumulator;
    float prev_error;
    float kp_gain;
    float ki_gain;
    float kd_gain;
    float deadband;
};

/******************************************************************************
 *
 * H-bridge hardware layer
 *
 *****************************************************************************/
// Number of clock ticks per PWM period. Set at run-time because the result of
// SysCtlClockFreqSet(...) isn't guaranteed to be exact requested frequency.
uint32_t pwm_period_ticks;

// This defines the PWM scheme used for the motor-drive PWM generators
static int drive_mode;

/* Sets the motor-drive PWM generators to produce the requested duty cycle,
 * according to the current drive mode. The duty cycle must be clamped to the
 * specified range before calling this function.
 *
 * duty: signed duty cycle, in range [-DUTY_CYCLE_MAX, DUTY_CYCLE_MAX]
 */
void pwm_duty_update(float duty)
{
    int p1_duty, p2_duty, p3_duty, p4_duty;
    p1_duty = p2_duty = p3_duty = p4_duty = 0;

    bool positive = duty >= 0;
    float abs_duty = positive ? duty : -duty;
    ASSERT(abs_duty <= DUTY_CYCLE_MAX);

    switch (drive_mode) {
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
        if (abs_duty < DUTY_CYCLE_DEADBAND) {
            p1_duty = 1;
            p2_duty = 1;
            p3_duty = 1;
            p4_duty = 1;
        }
        break;

    // TODO: implement this
    case DRIVE_SIGN_MAG:
        p1_duty = 1;
        p2_duty = 1;
        p3_duty = 1;
        p4_duty = 1;
        break;

    default:
        ASSERT (0);
    }

    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, p1_duty);
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, p2_duty);
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, p4_duty);
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, p3_duty);
    MAP_PWMSyncUpdate(PWM0_BASE, PWM_GEN_0_BIT | PWM_GEN_1_BIT | PWM_GEN_2_BIT);
}


/* Put PWM outputs in active or inactive (Hi-Z) state. The inactive state is
 * used to freewheel the motor, particularly during emergency shutdowns.
 *
 * enabled: 'true' to generate PWM outputs, 'false' to freewheel the motor.
 */
static void hbridge_set_enabled(bool enabled)
{
    MAP_PWMOutputState(PWM0_BASE, HBRIDGE_PWM_BITS, enabled);
}


/* Switch the VBUS_EN transistor to make the H-Bridge power rail connected or
 * disconnected from the motor's battery. The disconnected state is used to
 * protect the battery from excessive regenerative current during braking.
 *
 * connected: 'true' connects the h-bridge to the battery, 'false' disconnects it.
 *
 * Note: pin PA6 (40) == VBUS_EN_MCU
 */
static void hbridge_set_connected(bool connected)
{
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, connected ? GPIO_PIN_6: 0);
}


/* Sets the duty cycle for the 'bleeder' resistor PWM. This is used to drain
 * excess energy from the H-bridge bulk capacitors during regenerative braking.
 *
 * duty: duty cycle between [0, 100]
 *
 * TODO: this function has not been tested yet.
 */
static void hbridge_set_bleeder_duty(float duty)
{
    ASSERT(duty >= 0. && duty <= 100.0);
    duty = duty > 98.0 ? 98.0 : duty;

    uint16_t pulse_width = (duty / 100.0) * pwm_period_ticks;
    MAP_PWMPulseWidthSet(PWM0_BASE, BLEEDER_PWM_BIT, pulse_width);
}


/* Switches out the h-bridge bulk capacitor soft start resistor. This function
 * is called once during system startup, after the H-bridge VBUS sensor
 * indicates a stable bus voltage. The motor should not be driven before then.
 *
 * NOTE: Pin PA7 (41) == VBUS_ST_MCU
 */
static void hbridge_soft_start_bypass(void)
{
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
}


/* Enable and configure the PWM generator peripheral. This covers both the
 * motor-drive PWM and the bleeder resistor PWM.
 */
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

    hbridge_set_enabled(true);

    // TODO: sketchy fix for mysterious bug causing H-bridge PWM pins to
    // go high inbetween configuration and first run of control loop
    pwm_duty_update(0);
}


/******************************************************************************
 *
 * Math functions for control task
 *
 *****************************************************************************/
static struct pid current_pid;
static struct pid velocity_pid;
static struct pid position_pid;     // not implemented yet


/* Run one iteration of a PID controller.
 *
 * pid: PID controller parameters and state
 * error_signal: Current difference between plant output and setpoint.
 */
float pid_loop(struct pid * pid, float error_signal)
{
    pid->accumulator += error_signal;
    float p_term = pid->kp_gain * error_signal;
    float i_term = pid->ki_gain * pid->accumulator;
    float d_term = pid->kd_gain * (error_signal - pid->prev_error);
    pid->prev_error = error_signal;

    return i_term + p_term + d_term;
}


/* Clear a PID's state to zero. This function is used when starting up a PID
 * controller (after changing configuration, or changing the controlled variable)
 * in order to avoid glitches from residual state.
 *
 * pid: The PID controller to be reset.
 */
void pid_state_reset(struct pid * pid)
{
    pid->accumulator = 0;
    pid->prev_error = 0;
}


/* Clamp the duty cycle to the range [-max, max].
 *
 * duty: unclamped duty cycle
 * max:  desired maximum absolute value. Generally this will be DUTY_CYCLE_MAX.
 *
 * returns: clamped duty cycle
 */
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
 *
 * Control task
 *
 *****************************************************************************/
static float max_avg_current;
static float max_peak_current;
static float motor_shutdown_temp;
static float hbridge_fan_temp;
static float hbridge_shutdown_temp;
//static float max_accel_rate;            // Not used at the moment.


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
        hbridge_set_connected(false);
    else if (vbus < vbus_nominal + VBUS_HYSTERESIS)
        hbridge_set_connected(true);

    if (vbus > vbus_nominal + VBUS_HYSTERESIS)
        hbridge_set_bleeder_duty(100.0);
    else if (vbus < vbus_nominal - VBUS_HYSTERESIS)
        hbridge_set_bleeder_duty(0.0);
}

/* Turn the hbridge cooling fan on or off.
 *
 * enabled: When true, the fan is turned on.
 *
 * NOTE: pin B4 = PWM_FAN, but it isn't used as a PWM. Most BLDC fans don't
 * like to be PWM'd.
 */
static void hbridge_fan_set_enabled(bool enabled)
{
    MAP_GPIOPinWrite(GPIO_PORTB_BASE, 0x04, enabled ? 0x04 : 0);
}

static void hbridge_safety_control(float current, float hbridge_temp,
                                   float motor_temp)
{
    // This IIR filter will reach 95% of a step input in about 20 samples.
    // For our 10 kHz sample rate, this is about 2 milliseconds.
    static float avg_current;
    avg_current = avg_current * 0.85 + current * 0.15;

    if (hbridge_temp > hbridge_fan_temp) {
        hbridge_fan_set_enabled(true);
    } else if (hbridge_temp < hbridge_fan_temp - FAN_HYSTERESIS)
        hbridge_fan_set_enabled(false);

    // TODO: lower faults when they are no longer active!
    if (ABS(current) > max_peak_current ||
        ABS(avg_current) > max_avg_current) {
        hbridge_set_enabled(false);
        system_raise_fault(FAULT_OVERCURRENT);
    }

    if (motor_temp > motor_shutdown_temp) {
        hbridge_set_enabled(false);
        system_raise_fault(FAULT_MOTOR_TEMP);
    }

    if (hbridge_temp > hbridge_shutdown_temp) {
        hbridge_set_enabled(false);
        system_raise_fault(FAULT_MOTOR_TEMP);
    }
}

/* Startup routine that ensures soft-start of the H-bridge bulk capacitors. To
 * avoid excessive inrush currents, the H-bridge bulk capacitors are initially
 * charged through a resistor. When the bus voltage stabilizes, the resistor
 * is switched out.
 */
void hbridge_soft_start(void)
{
    /* Crude soft start mechanism.
     * TODO: check VBUS sensor to confirm VBUS stabilized before proceeding:
     *  Low dv/dt
     *  Vbus above minimum threshold (10V)
     */
    hbridge_set_connected(true);
    vTaskDelay(1000);
    hbridge_soft_start_bypass();
}

/* Load motion control configuration registers from the system module.
 */
void load_config_regs(void)
{
    drive_mode = system_read_config_reg(REG_DRIVE_MODE).i;

    max_avg_current = system_read_config_reg(REG_MAX_AVG_CURRENT).f;
    max_peak_current = system_read_config_reg(REG_MAX_PEAK_CURRENT).f;
    motor_shutdown_temp= system_read_config_reg(REG_MOTOR_SHUTDOWN_TEMP).f;
    hbridge_fan_temp = system_read_config_reg(REG_HBRIDGE_FAN_TEMP).f;
    hbridge_shutdown_temp = system_read_config_reg(REG_HBRIDGE_SHUTDOWN_TEMP).f;

    current_pid.kp_gain = system_read_config_reg(REG_CURRENT_KP).f;
    current_pid.ki_gain = system_read_config_reg(REG_CURRENT_KI).f;
    current_pid.kd_gain = system_read_config_reg(REG_CURRENT_KD).f;
    current_pid.deadband = system_read_config_reg(REG_CURRENT_DEADBAND).f;

    velocity_pid.kp_gain = system_read_config_reg(REG_VELOCITY_KP).f;
    velocity_pid.ki_gain = system_read_config_reg(REG_VELOCITY_KI).f;
    velocity_pid.kd_gain = system_read_config_reg(REG_VELOCITY_KD).f;
    velocity_pid.deadband = system_read_config_reg(REG_VELOCITY_DEADBAND).f;

    position_pid.kp_gain = system_read_config_reg(REG_POSITION_KP).f;
    position_pid.ki_gain = system_read_config_reg(REG_POSITION_KI).f;
    position_pid.kd_gain = system_read_config_reg(REG_POSITION_KD).f;
    position_pid.deadband = system_read_config_reg(REG_POSITION_DEADBAND).f;

    pid_state_reset(&current_pid);
    pid_state_reset(&velocity_pid);
    pid_state_reset(&position_pid);
}


/* Get control mode from system module. Clear PID state upon entry to
 * closed-loop modes to avoid glitches from residual PID state.
 *
 * returns: the current system control mode.
 */
static int update_control_mode(void)
{
    static int prev_control_mode = CTRL_OPEN_LOOP;
    int control_mode = system_get_control_mode();

    if (prev_control_mode != control_mode) {
        pid_state_reset(&current_pid);
        pid_state_reset(&velocity_pid);
        pid_state_reset(&position_pid);
    }
    prev_control_mode = control_mode;

    return control_mode;
}


/* Get system state from system module. Clear PID state upon entry to run mode
 * to avoid glitches from residual PID state.
 *
 * returns: the current system state.
 */
static int update_system_state(void)
{
    static int prev_system_state = STATE_CONFIG;
    int system_state = system_get_state();

    if (system_state == STATE_RUNNING && prev_system_state != STATE_RUNNING) {
        load_config_regs();
        pid_state_reset(&current_pid);
        pid_state_reset(&velocity_pid);
        pid_state_reset(&position_pid);
    }

    prev_system_state = system_state;
    return system_state;
}


static float current_controller(float motor_current)
{
    float target_current = system_get_control_target();
    float duty_cycle = 0;

    if (ABS(target_current) > current_pid.deadband ||
        ABS(motor_current) > current_pid.deadband) {
        float error = target_current - motor_current;
        duty_cycle = pid_loop(&current_pid, error);
    }

    return duty_cycle;
}

static float velocity_controller(float motor_current)
{
    float target_velocity = system_get_control_target();
    static float target_current = 0;
    float duty_cycle = 0;
    float error;
    float motor_velocity;
    bool new_velocity = encoder_poll_motor_velocity_rads(&motor_velocity);

    if (ABS(target_velocity) > velocity_pid.deadband ||
        ABS(motor_velocity) > velocity_pid.deadband) {
        if (new_velocity) {
            // Run velocity controller
            error = target_velocity - motor_velocity;
            target_current = pid_loop(&velocity_pid, error);
            target_current = duty_cycle_clamp(target_current, max_avg_current);
        }
        // Always run current controller
        error = target_current - motor_current;
        duty_cycle = pid_loop(&current_pid, error);
    }

    return duty_cycle;
}

/* Motor controller control loop.
 *
 */
void control_task_code(void * arg)
{
    pwm_setup();
    sensors_setup();
    load_config_regs();
    hbridge_soft_start();

    while (1) {
        bool timeout = !ulTaskNotifyTake(true, CONTROL_TICK_TIMEOUT);
        ASSERT(timeout == false);
        debug_pins_set(0x04, 0x04);

        //
        sensors_rt_current_update();

        // Read sensor data for safety check
        float motor_current = sensor_get_motor_current();
        float bus_voltage = sensor_get_bus_voltage();
        float motor_temp = sensor_get_motor_temperature();
        float hbridge_temp = sensor_get_hbridge_temperature();

        // Run safety checks and update fault flags
        hbridge_safety_control(motor_current, hbridge_temp, motor_temp);

        // This isn't used yet because the only drive mode available now
        // (async_sign_mag) can't cause regenerative braking to happen.
        // hbridge_regen_control(duty_cycle, motor_current, bus_voltage);

        // Check system status and control; update if necessary
        int control_mode = update_control_mode();
        int system_state = update_system_state();

        float duty_cycle = 0;

        switch(control_mode) {
        case CTRL_OPEN_LOOP:
            duty_cycle = system_get_control_target();
            break;

        case CTRL_CURRENT:
            duty_cycle = current_controller(motor_current);
            break;

        case CTRL_VELOCITY:
            duty_cycle = velocity_controller(motor_current);
            break;

        default:
            ASSERT(0);
        }

        duty_cycle = duty_cycle_clamp(duty_cycle, DUTY_CYCLE_MAX);

        if (system_state == STATE_RUNNING) {
            pwm_duty_update(duty_cycle);
            hbridge_set_enabled(system_get_drive_enabled());
        } else {
            pwm_duty_update(0);
            hbridge_set_enabled(false);
        }
        debug_pins_set(0x04, 0x00);
    }
}
