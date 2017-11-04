/*
 * encoder.c
 *
 *  Created on: Nov 2, 2017
 *      Author: odougs
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

#define RPM_TO_RADS             (60.0 / (2*PI))

#define ENCODER_IRQ_RATE_HZ     100.0
#define ENCODER_TIMEOUT         (2 * configTICK_RATE_HZ / ENCODER_IRQ_RATE_HZ)

#define BUFFER_LENGTH           100

/******************************************************************************
 * Encoder output data
 *****************************************************************************/
static float motor_velocity_rads;      // Motor velocity (radians/sec)
static float motor_position_rads;      // Motor position (radians)


/******************************************************************************
 * Encoder configuration
 *****************************************************************************/
static unsigned encoder_ppr;
static float encoder_gear_ratio = 77.0 / 30.0;
extern TaskHandle_t encoder_task_handle;


static float rads_from_pps(uint32_t pulses_per_second)
{
    return pulses_per_second * 60.0 / encoder_ppr * RPM_TO_RADS;
}

static void velocity_update(float velocity_rads)
{
    // Noise filter: delta_v guestimated to be less than 1000 rads/s
    if (ABS(motor_velocity_rads - velocity_rads) < 10000)
        // low pass IIR, alpha = 15/16
        motor_velocity_rads = ((motor_velocity_rads * 15) + velocity_rads) / 16;
 }

void qei_ISR(void)
{
    QEIIntClear(QEI0_BASE, QEI_INTTIMER);

    int position_change = QEIVelocityGet(QEI0_BASE) * encoder_gear_ratio *
            QEIDirectionGet(QEI0_BASE);

    BaseType_t task_woken;
    xTaskNotifyFromISR(encoder_task_handle, position_change,
                       eSetValueWithOverwrite, &task_woken);
    portYIELD_FROM_ISR(task_woken);
}


static void encoder_setup(void)
{
    // Signswise encoder has 600 ppr on one channel; multiply by 4 for quadrature.
    // TODO: read this from system module's config registers
    encoder_ppr = 2400;

    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A | QEI_CONFIG_NO_RESET |
                 QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), encoder_ppr - 1);
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1,
                         system_get_sysclk_freq() / ENCODER_IRQ_RATE_HZ);

    QEIEnable(QEI0_BASE);
    QEIVelocityEnable(QEI0_BASE);
    QEIIntEnable(QEI0_BASE, QEI_INTTIMER);
    IntEnable(INT_QEI0);
}


void encoder_task_code(void * arg)
{
    encoder_setup();

    // circular buffer of signed position changes gathered by qei_ISR
    static int buffer[BUFFER_LENGTH];
    static int index = 0;

    while (1) {
        int position_change;
        bool timeout = !xTaskNotifyWait(0, 0, (uint32_t *)&position_change,
                                       ENCODER_TIMEOUT);
        ASSERT(timeout == false);
        buffer[index] = position_change;

        // Search backwards through buffer until 100 encoder pulses or 10
        // sample points have been examined.
        int total_position_change = 0;
        int abs_position_change = 0;
        int num_periods = 0;
        int i = index;

        while (num_periods < 10 && abs_position_change < 200) {
            total_position_change += buffer[i];
            abs_position_change += ABS(buffer[i]);
            num_periods++;

            if (--i < 0)
                i = BUFFER_LENGTH - 1;
        }

        index++;
        if (index == BUFFER_LENGTH)
            index = 0;

        // Calculate velocity from accumulated data
        float total_time = num_periods / ENCODER_IRQ_RATE_HZ;
        int pulses_per_second = total_position_change / total_time;
        velocity_update(rads_from_pps(pulses_per_second));
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
