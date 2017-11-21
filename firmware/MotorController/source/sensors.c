/*
 * sensors.c - ADC code to collect sensor data.
 *
 *  Created on: Oct 8, 2017
 *      Author: Oliver Douglas
 */

#include "sensors.h"
#include "system.h"
#include "fir.h"

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/udma.h"
#include "inc/hw_adc.h"


// File index for ASSERT() macro
FILENUM(3)

/******************************************************************************
 * ADC and sensor conversion definitions
 *****************************************************************************/
#define ADC_VOLTS_PER_BIT   (3.0 / 4096)


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
#define RT_BUFFER_LEN       4

// Low-speed sensor trigger (ADC1)
#define LS_SAMPLE_RATE_HZ   100 //100
#define LS_TIMER_SYSCTL     SYSCTL_PERIPH_TIMER1
#define LS_TIMER_BASE       TIMER1_BASE
#define LS_TIMER_INT        INT_TIMER1A
#define LS_BUFFER_LEN       4


/******************************************************************************
 * DMA control structure and RT sensor data destination buffer
 *****************************************************************************/
uint8_t uDMA_control_table[1024] __attribute__ ((aligned(1024)));
uint32_t rt_adc_buffer[RT_BUFFER_LEN];  // 4 samples of current sensor
uint32_t ls_adc_buffer[LS_BUFFER_LEN];  // hbrige_temp, motor_temp, vbatt, vbus

void * adc_src = (void *)(ADC0_BASE + ADC_O_SSFIFO0);
void * adc_dest = (void *)rt_adc_buffer;


/******************************************************************************
 * Task handles for IPC and synchronization
 *****************************************************************************/
extern TaskHandle_t control_task_handle;


/******************************************************************************
 * Most recent sensor data
 *
 * The sensor ISRs post their data here so that irregular reads of the sensor
 * data can be done using the public 'getter' functions. IPC for control
 * purposes is done using other mechanisms (FreeRTOS queues, TaskNotify, etc)
 *****************************************************************************/
// Real-time ADC data
static float motor_current;         // Motor current (amperes)

// Low-speed ADC data
static float bus_voltage;           // H-bridge bus voltage (volts)
static float battery_voltage;       // Controller battery voltage (volts)
static float motor_temperature;     // Motor case temperature (celsius)
static float hbridge_temperature;   // H-bridge heatsink temperature (celsius)


/******************************************************************************
 * Real-time ADC functions
 *
 * Motor current and is sensed at 40 kHz on ADC0. When four samples have been
 * stored by DMA, the control task is woken to process the samples (at 10 kHz).
 *****************************************************************************/

// This ISR runs when DMA has filled the RT sensor buffer. It resets the DMA
// for the next sequence of samples, and wakes up the control task.
void adc0_seq0_ISR(void)
{
    BaseType_t task_woken;

    MAP_ADCIntClear(ADC0_BASE, 0);
    MAP_ADCIntClearEx(ADC0_BASE, ADC_INT_DMA_SS0);

    MAP_uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC, adc_src, adc_dest,
                                   RT_BUFFER_LEN);
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
                               UDMA_MODE_BASIC, adc_src, adc_dest,
                               RT_BUFFER_LEN);
    MAP_uDMAChannelEnable(UDMA_CHANNEL_ADC0);

    // Configure sample sequencer 0 with GPTM trigger and priority 0.
    MAP_ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);
    MAP_ADCSequenceStepConfigure(ADC0_BASE, 0, 0, I_SENSOR_PIN |
                                 ADC_CTL_IE | ADC_CTL_END);
    MAP_ADCSequenceDMAEnable(ADC0_BASE, 0);

    MAP_ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);

    // Enable ADC0 DMA interrupts
    MAP_ADCIntEnableEx(ADC0_BASE, ADC_INT_DMA_SS0);
    MAP_IntEnable(INT_ADC0SS0);

    MAP_ADCSequenceEnable(ADC0_BASE, 0);
}


/******************************************************************************
 * Low-speed ADC functions
 *****************************************************************************/
extern float celsius_from_adc_raw(uint32_t raw);

static float bus_voltage_from_adc_raw(uint32_t raw)
{
    // TODO: eliminate magic numbers
    return raw * ADC_VOLTS_PER_BIT * 19.58;
}

static float battery_voltage_from_adc_raw(uint32_t raw)
{
    // TODO: eliminate magic numbers
    return raw * ADC_VOLTS_PER_BIT * 3.7;
}

// This ISR runs when the low-speed sensor data has been collected.
void adc1_seq0_ISR(void)
{
    // spurious interrupts are happening
    MAP_ADCIntClear(ADC1_BASE, 0);
    MAP_ADCIntClearEx(ADC0_BASE, ADC_INT_SS0);

    // Pull data from ADC and convert
    MAP_ADCSequenceDataGet(ADC1_BASE, 0, ls_adc_buffer);

    hbridge_temperature = celsius_from_adc_raw(ls_adc_buffer[0]);
    motor_temperature = celsius_from_adc_raw(ls_adc_buffer[1]);
    battery_voltage = battery_voltage_from_adc_raw(ls_adc_buffer[2]);
    bus_voltage = bus_voltage_from_adc_raw(ls_adc_buffer[3]);
}

void ls_timer_ISR(void)
{
    MAP_TimerIntClear(LS_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    ADCProcessorTrigger(ADC1_BASE, 0);
}

// Configure LS_TIMER_BASE to trigger ADC1 at LS_SAMPLE_RATE_HZ
static void ls_trigger_timer_setup()
{
    uint32_t ls_timer_period = system_get_sysclk_freq() / LS_SAMPLE_RATE_HZ;

    MAP_SysCtlPeripheralEnable(LS_TIMER_SYSCTL);
    MAP_TimerConfigure(LS_TIMER_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(LS_TIMER_BASE, TIMER_BOTH, ls_timer_period);
    MAP_TimerIntEnable(LS_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    MAP_IntEnable(LS_TIMER_INT);
    MAP_TimerEnable(LS_TIMER_BASE, TIMER_BOTH);
}

// Configure ADC1 to collect the LS sensor data
static void ls_adc_setup(void)
{
    // Configure sample sequencer 0 with GPTM trigger and priority 0.
    MAP_ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    MAP_ADCSequenceStepConfigure(ADC1_BASE, 0, 0, BRIDGE_TEMP_PIN);
    MAP_ADCSequenceStepConfigure(ADC1_BASE, 0, 1, MOTOR_TEMP_PIN);
    MAP_ADCSequenceStepConfigure(ADC1_BASE, 0, 2, VBAT_CTRL_PIN);
    MAP_ADCSequenceStepConfigure(ADC1_BASE, 0, 3, VBUS_MOTOR_PIN |
                                 ADC_CTL_IE | ADC_CTL_END);

    MAP_ADCReferenceSet(ADC1_BASE, ADC_REF_EXT_3V);

    // Enable ADC0 interrupts
    MAP_ADCIntEnableEx(ADC1_BASE, ADC_INT_SS0);
    MAP_IntEnable(INT_ADC1SS0);

    MAP_ADCSequenceEnable(ADC1_BASE, 0);
}


/******************************************************************************
 * Current sensor processing
 *****************************************************************************/

#define DECIMATOR_LENGTH  (sizeof(h_decimator) / sizeof(h_decimator[0]))
static struct fir_filter * current_decimator;

/* Impulse response for the 40 kHz -> 10 kHz current sensor decimator.
 * Produced by Octave command 'fir1(20, 0.2)'
 */
static const float h_decimator[] = {
  -0.000078, -0.002202, -0.006373,
  -0.011527, -0.012039, 0.000525,
  0.032310, 0.081681, 0.137227,
  0.181370, 0.198213, 0.181370,
  0.137227, 0.081681, 0.032310,
  0.000525, -0.012039, -0.011527,
  -0.006373, -0.002202, -0.000078
};

/* Decimate the current signal (again) from 10 kHz to 100 Hz.
 *
 * TODO: This is going to produce awful aliasing; the roll-off is
 * way too slow
 *
 */
static float current_iir_decimate(float new_current)
{
    static float filt1_out;
    float filt2_out;

    // Cascade of single-pole IIR low pass filters.
    // -3dB frequency is approximately 100 Hz
    filt1_out = filt1_out * 0.95 + new_current * 0.05;
    filt2_out = motor_current * 0.95 + filt1_out * 0.05;

    return filt2_out;
}

/* Decimate the 40 kHz raw current signal to 10 kHz to remove PWM harmonics.
 * Decimate this new 10 kHz signal to 100 Hz for other tasks to use in logging.
 * Return the full-rate (10 kHz) motor current, in amperes.
 *
 * This function is called once at the start of each control period, by the
 * control task. It should NOT be used anywhere else.
 */
float sensors_rt_current_update(void)
{
    float new_current[RT_BUFFER_LEN] =
    {
     rt_adc_buffer[0],
     rt_adc_buffer[1],
     rt_adc_buffer[2],
     rt_adc_buffer[3],
    };

    float rt_current = fir_do_filter(current_decimator, new_current);
    log_msg("%d\n", (int)rt_current, 0, 0);

    // New 10 kHz current sample
    rt_current = (rt_current * ADC_VOLTS_PER_BIT - 1.5) * -67.8;

    // New 100 Hz current sample
    motor_current = current_iir_decimate(rt_current);

    return rt_current;
}


/******************************************************************************
 * Other Public functions
 *****************************************************************************/

/* Returns the decimated (100 Hz) motor current, in amperes.
 *
 * This current sensor data has been decimated to 100 Hz for logging purposes.
 * The control task instead uses 'sensors_rt_current_update' to access the
 * full 10 kHz current signal.
 */
float sensor_get_motor_current(void)
{
    return motor_current;
}

/* Returns the voltage of the H-bridge bus, in volts.
 * This sensor is sampled at 100 Hz.
 */
float sensor_get_bus_voltage(void)
{
    return bus_voltage;
}

/* Returns the voltage of the controller battery, in volts.
 * This sensor is sampled at 100 Hz.
 */
float sensor_get_battery_voltage(void)
{
    return battery_voltage;
}
/* Returns the temperature of the motor case, in celsius.
 * This sensor is sampled at 100 Hz.
 */
float sensor_get_motor_temperature(void)
{
    return motor_temperature;
}

/* Returns the temperature of the H-bridge heatsink, in celsius.
 * This sensor is sampled at 100 Hz.
 */
float sensor_get_hbridge_temperature(void)
{
    return hbridge_temperature;
}

/* Configure all sensors at system startup.
 */
void sensors_setup(void)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

    current_decimator = fir_create(h_decimator, DECIMATOR_LENGTH, 4);

    rt_adc_setup();
    ls_adc_setup();
    rt_trigger_timer_setup();
    ls_trigger_timer_setup();
}
