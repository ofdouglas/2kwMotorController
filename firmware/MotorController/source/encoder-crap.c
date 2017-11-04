/*
 * encoder.c
 *
 *  Created on: Nov 2, 2017
 *      Author: odougs
 */

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
#define PI                      3.1415926
#define RPM_TO_RADS             (60.0 / (2*PI))


#define ENCODER_SAMPLE_RATE_HZ  100.0
#define MAX_PULSES_PER_SECOND   10000
#define MAX_PULSES_HYSTERESIS   1000


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


/******************************************************************************
 * Encoder driver state
 *****************************************************************************/
#define FLAG_EDGE_COUNT_MODE    0x01
#define FLAG_EDGE_SEEN          0x02
#define FLAG_EDGE_SKIP          0x04

static int driver_flags = FLAG_EDGE_COUNT_MODE | FLAG_EDGE_SKIP;

static void rads_from_pps(uint32_t pulses_per_second)
{
    return pulses_per_second * 60.0 / encoder_ppr * RPM_TO_RADS
}

static void velocity_update(float velocity_rads)
{
    // magic number: delta_v guestimated to be less than 1000 rads/s
    if (ABS(motor_velocity_rads - velocity_rads) < 10000)
        motor_velocity_rads = ((motor_velocity_rads * 15) + velocity_rads) / 16;
    // low pass IIR, alpha = 15/16
    }
}

void qei_gpio_ISR(void)
{
    GPIOIntClear(GPIO_PORTL_BASE, GPIO_PIN_1);

    static uint32_t prev_time = 0;
    static uint32_t new_time = 0;

    new_time = HWREG(QEI0_BASE + QEI_O_TIME);
    uint32_t time_diff = new_time - prev_time;
    prev_time = new_time;

    int pulses_per_second = system_get_sysclk_freq() / time_diff;

    if (driver_flags & FLAG_EDGE_SKIP == false)
        velocity_update(rads_from_pps(pulses_per_second));

    driver_flags &= ~FLAG_EDGE_SKIP;
    driver_flags |= FLAG_EDGE_SEEN;
}

void qei_ISR(void)
{
    QEIIntClear(QEI0_BASE, QEI_INTTIMER);

    int pulses_per_second = QEIVelocityGet(QEI0_BASE) * ENCODER_SAMPLE_RATE_HZ *
            encoder_gear_ratio * QEIDirectionGet(QEI0_BASE);
    velocity_update(rads_from_pps(pulses_per_second));

}


static void encoder_setup(void)
{
    // Signswise encoder has 600 ppr on one channel; multiply by 4 for quadrature.
    // TODO: read this from system module's config registers
    encoder_ppr = 2400;

    // A GPIO IRQ on rising edges of the encoder A pin.
    GPIOIntTypeSet(ENCODER_GPIO_BASE, ENCODER_GPIO_PIN, GPIO_RISING_EDGE);
    GPIOIntEnable(ENCODER_GPIO_BASE, ENCODER_GPIO_PIN);
    IntEnable(ENCODER_GPIO_INT);

    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A | QEI_CONFIG_NO_RESET |
                 QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), encoder_ppr - 1);
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1,
                         system_get_sysclk_freq() / ENCODER_SAMPLE_RATE_HZ);

    // Enable the QEI and the velocity capture.
    QEIEnable(QEI0_BASE);
    QEIVelocityEnable(QEI0_BASE);

    // Enable the QEI velocity interrupt.
    QEIIntEnable(QEI0_BASE, QEI_INTTIMER);
    IntEnable(INT_QEI0);
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
