/*
 * encoder.c - Read velocity and position from a quadrature encoder
 *
 *  Created on: Nov 2, 2017
 *      Author: Oliver Douglas
 */

#include "encoder.h"
#include "system.h"

#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"
#include "inc/hw_qei.h"
#include "inc/hw_types.h"

// File index for ASSERT() macro
FILENUM(9)

/******************************************************************************
 * Encoder definitions
 *****************************************************************************/
#define ENCODER_IRQ_RATE_HZ     1000.0
#define ENCODER_TIMEOUT         (2 * configTICK_RATE_HZ / ENCODER_IRQ_RATE_HZ)


/******************************************************************************
 * Encoder output data
 *****************************************************************************/
static float motor_velocity_rads;      // Motor velocity (radians/sec)
static float motor_position_rads;      // Motor position (radians)


/******************************************************************************
 * Encoder configuration
 *****************************************************************************/
static unsigned encoder_ppr;
static float encoder_gear_ratio = (97.716 / 30.0);
extern TaskHandle_t encoder_task_handle;

void qei_ISR(void)
{
    QEIIntClear(QEI0_BASE, QEI_INTTIMER);

    int position_change = QEIVelocityGet(QEI0_BASE) * QEIDirectionGet(QEI0_BASE);

    BaseType_t task_woken;
    xTaskNotifyFromISR(encoder_task_handle, position_change,
                       eSetValueWithOverwrite, &task_woken);
    portYIELD_FROM_ISR(task_woken);
}

static void encoder_setup(void)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

    // Signswise encoder has 600 ppr on one channel; multiply by 4 for quadrature.
    // TODO: read this from system module's config registers
    encoder_ppr = 2400;

    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A | QEI_CONFIG_NO_RESET |
                 QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP), encoder_ppr - 1);
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1,
                         system_get_sysclk_freq() / ENCODER_IRQ_RATE_HZ);

    QEIEnable(QEI0_BASE);
    QEIVelocityEnable(QEI0_BASE);
    QEIIntEnable(QEI0_BASE, QEI_INTTIMER);
    IntEnable(INT_QEI0);
}

#define N_MAX 10

static volatile bool velocity_updated = false;

void encoder_task_code(void * arg)
{
    encoder_setup();

    // circular buffer of signed position changes gathered by qei_ISR
    static int buffer[N_MAX];
    static int index = 0;

    while (1) {
        int32_t position_change;
        bool timeout = !xTaskNotifyWait(0, 0, (uint32_t *)&position_change,
                                       ENCODER_TIMEOUT);
        ASSERT(timeout == false);
        buffer[index] = position_change;

        // Search backwards through buffer until 100 encoder pulses or 10
        // sample points have been examined.
        int sum = 0;
        int abs_sum = 0;
        int N = 0;
        int i = index;

        while (N < 10 && abs_sum < 50) {
            sum += buffer[i];
            abs_sum += ABS(buffer[i]);
            N++;

            if (--i < 0)
                i = N_MAX - 1;
        }

        index++;
        if (index == N_MAX)
            index = 0;

        // Calculate velocity from accumulated data
        float pulses_per_sec = sum * encoder_gear_ratio * ENCODER_IRQ_RATE_HZ / N;
        float rads = pulses_per_sec * 60.0 / encoder_ppr * RPM_TO_RADS;
        motor_velocity_rads = motor_velocity_rads * 0.9 + rads * 0.1;
        //motor_velocity_rads = rads;

        velocity_updated = true;

        log_msg("%d\n", motor_velocity_rads * RADS_TO_RPM, 0, 0);
    }
}


/******************************************************************************
 * Public functions
 *****************************************************************************/

float encoder_get_motor_position_rads(void)
{
    return motor_position_rads;
}

float encoder_get_motor_velocity_rads(void)
{
    return motor_velocity_rads;
}

/* Stores the current motor velocity in *velocity. Returns true if the velocity
 * was updated since the last time this function was called, else false.
 *
 * This function should only be called by the control task. Other tasks should
 * use encoder_get_motor_velocity_rads.
 */
bool encoder_poll_motor_velocity_rads(float * velocity)
{
    *velocity = motor_velocity_rads;

    if (velocity_updated == true) {
        velocity_updated = false;
        return true;
    }
    return false;
}
