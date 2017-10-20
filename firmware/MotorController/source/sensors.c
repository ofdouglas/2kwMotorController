/*
 * sensors.c - ADC and encoder code to collect sensor data.
 *
 *  Created on: Oct 8, 2017
 *      Author: Oliver Douglas
 */

#include "sensors.h"

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/udma.h"
#include "inc/hw_adc.h"

// File index for ASSERT() macro
FILENUM(3)

/******************************************************************************
 * ADC pin/channel map
 *****************************************************************************/
#define BRIDGE_TEMP_PIN     ADC_CTL_CH12
#define MOTOR_TEMP_PIN      ADC_CTL_CH13
#define VBAT_CTRL_PIN       ADC_CTL_CH15
#define VBUS_MOTOR_PIN      ADC_CTL_CH17
#define I_SENSOR_PIN        ADC_CTL_CH19


/******************************************************************************
 * Definitions for sample rates and timers
 *****************************************************************************/
// Real-time sensor trigger (ADC0)
#define RT_SAMPLE_RATE_HZ   40000
#define RT_TIMER_SYSCTL     SYSCTL_PERIPH_TIMER0
#define RT_TIMER_BASE       TIMER0_BASE

// Low-speed sensor trigger (ADC1)
#define LS_SAMPLE_RATE_HZ   100
#define LS_TIMER_SYSCTL     SYSCTL_PERIPH_TIMER1
#define LS_TIMER_BASE       TIMER1_BASE


/******************************************************************************
 * DMA control structure and RT sensor data destination buffer
 *****************************************************************************/
uint8_t uDMA_control_table[1024] __attribute__ ((aligned(1024)));
float rt_adc_buffer[8];

void * adc_src = (void *)(ADC0_BASE + ADC_O_SSFIFO0);
void * adc_dest = (void *)rt_adc_buffer;


/******************************************************************************
 * Most recent sensor data
 *
 * The sensor ISRs post their data here so that irregular reads of the sensor
 * data can be done using the public 'getter' functions. IPC for control
 * purposes is done using other mechanisms (FreeRTOS queues, TaskNotify, etc)
 *
 * TODO: decimated sensor data from the control task should actually be used
 * for irregular reads!
 *****************************************************************************/
// Encoder data
static float motor_velocity_rads;      // Motor velocity (radians/sec)
static float motor_position_rads;      // Motor position (radians)

// Real-time ADC data
static float motor_current_amps;       // Motor current (amperes)
static float vbus_volts;               // H-bridge bus voltage (volts)

// Low-speed ADC data
static float vbatt_volts;              // Controller battery voltage (volts)
static float motor_temp_celsius;       // Motor case temperature (celsius)
static float hbridge_temp_celsius;     // H-bridge heatsink temperature (celsius)



/******************************************************************************
 * Encoder functions
 *****************************************************************************/

static void encoder_setup(void)
{

}


/******************************************************************************
 * Real-time ADC functions
 *
 * Motor current and bus voltage are sensed together at 40 kHz on ADC0.
 * When four samples of each have been stored by DMA, the control task is
 * woken to process these samples (at 10 kHz).
 *****************************************************************************/

// TODO: move this elsewhere!
extern TaskHandle_t control_task_handle;

// This ISR runs when DMA has filled the RT sensor buffer. It resets the DMA
// for the next sequence of samples, and wakes up the control task.
void adc0_seq0_ISR(void)
{
    BaseType_t task_woken;

    MAP_ADCIntClear(ADC0_BASE, 0);
    MAP_ADCIntClearEx(ADC0_BASE, ADC_INT_DMA_SS0);

    MAP_uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC, adc_src, adc_dest, 8);
    MAP_uDMAChannelEnable(UDMA_CHANNEL_ADC0);

    vTaskNotifyGiveFromISR(control_task_handle, &task_woken);
    portYIELD_FROM_ISR(task_woken);
}

// Configure RT_TIMER_BASE to trigger ADC0 at RT_SAMPLE_RATE_HZ
static void rt_trigger_timer_setup()
{
    uint32_t rt_timer_period = system_get_sysclk_freq() / RT_SAMPLE_RATE_HZ;

    MAP_SysCtlPeripheralEnable(RT_TIMER_SYSCTL);
    MAP_TimerConfigure(RT_TIMER_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(RT_TIMER_BASE, TIMER_BOTH, rt_timer_period);
    MAP_TimerEnable(RT_TIMER_BASE, TIMER_BOTH);
    MAP_TimerControlTrigger(RT_TIMER_BASE, TIMER_BOTH, true);
}

// Configure ADC0 to collect the RT sensor data
static void rt_adc_setup(void)
{
    // Set up DMA
    MAP_uDMAEnable();
    MAP_uDMAControlBaseSet(uDMA_control_table);
    MAP_uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                              UDMA_SIZE_32 | UDMA_SRC_INC_NONE |
                              UDMA_DST_INC_32 | UDMA_ARB_1);
    MAP_uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC, adc_src, adc_dest, 8);
    MAP_uDMAChannelEnable(UDMA_CHANNEL_ADC0);

    // Configure sample sequencer 0 with GPTM trigger and priority 0.
    MAP_ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);
    MAP_ADCSequenceStepConfigure(ADC0_BASE, 0, 0, I_SENSOR_PIN);
    MAP_ADCSequenceStepConfigure(ADC0_BASE, 0, 1, VBUS_MOTOR_PIN |
                                 ADC_CTL_IE | ADC_CTL_END);
    MAP_ADCSequenceDMAEnable(ADC0_BASE, 0);

    // Enable ADC0 DMA interrupts
    MAP_ADCIntEnableEx(ADC0_BASE, ADC_INT_DMA_SS0);
    MAP_IntEnable(INT_ADC0SS0);
    MAP_IntPrioritySet(INT_ADC0SS0, 0xE0);

    MAP_ADCSequenceEnable(ADC0_BASE, 0);
    rt_trigger_timer_setup();
}


/******************************************************************************
 * Low-speed ADC functions
 *****************************************************************************/
// TODO: implement this
static float celsius_from_adc_raw(uint32_t raw)
{
    return 0;
}

// TODO: implement this
static float voltage_from_adc_raw(uint32_t raw)
{
    return 0;
}

// This ISR runs when the low-speed sensor data has been collected. It checks
// for motor and h-bridge overheating, and control battery undervoltage
void adc1_seq0_ISR(void)
{
    MAP_ADCIntClear(ADC1_BASE, 0);

    // TODO: can we use a digital comparator instead?
    // logic to handle temp/battery errors shouldn't be here...

    // Pull data from ADC and convert
    uint32_t raw_data[3];
    MAP_ADCSequenceDataGet(ADC1_BASE, 0, raw_data);

    // TODO: implement these conversion functions
    hbridge_temp_celsius = celsius_from_adc_raw(raw_data[0]);
    motor_temp_celsius = celsius_from_adc_raw(raw_data[1]);
    vbatt_volts = voltage_from_adc_raw(raw_data[2]);
}

// Configure LS_TIMER_BASE to trigger ADC1 at LS_SAMPLE_RATE_HZ
static void ls_trigger_timer_setup()
{
    uint32_t ls_timer_period = system_get_sysclk_freq() / LS_SAMPLE_RATE_HZ;

    MAP_SysCtlPeripheralEnable(LS_TIMER_SYSCTL);
    MAP_TimerConfigure(LS_TIMER_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(LS_TIMER_BASE, TIMER_BOTH, ls_timer_period);
    MAP_TimerEnable(LS_TIMER_BASE, TIMER_BOTH);
    MAP_TimerControlTrigger(LS_TIMER_BASE, TIMER_BOTH, true);
}

// Configure ADC1 to collect the LS sensor data
static void ls_adc_setup(void)
{
    // Configure sample sequencer 0 with GPTM trigger and priority 0.
    MAP_ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_TIMER, 0);
    MAP_ADCSequenceStepConfigure(ADC1_BASE, 0, 0, BRIDGE_TEMP_PIN);
    MAP_ADCSequenceStepConfigure(ADC1_BASE, 0, 1, MOTOR_TEMP_PIN);
    MAP_ADCSequenceStepConfigure(ADC1_BASE, 0, 2, VBAT_CTRL_PIN |
                                 ADC_CTL_IE | ADC_CTL_END);

    MAP_ADCSequenceEnable(ADC1_BASE, 0);
    ls_trigger_timer_setup();
}


/******************************************************************************
 * Public functions
 *****************************************************************************/

float sensor_get_motor_position_rads(void)
{
    return motor_position_rads;
}

float sensor_get_motor_velocity_rads(void)
{
    return motor_velocity_rads;
}

float sensor_get_motor_current_amps(void)
{
    return motor_current_amps;
}

float sensor_get_vbus_volts(void)
{
    return vbus_volts;
}

float sensor_get_vbatt_volts(void)
{
    return vbatt_volts;
}

float sensor_get_motor_temp_celsius(void)
{
    return motor_temp_celsius;
}

float sensor_get_hbridge_temp_celsius(void)
{
    return hbridge_temp_celsius;
}

void sensor_setup(void)
{
    // Enable clocks to sensor hardware
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

    // Configure each group of sensors
    encoder_setup();
    rt_adc_setup();
    ls_adc_setup();
}
