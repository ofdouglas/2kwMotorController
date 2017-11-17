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

// Low-speed sensor trigger (ADC1)
#define LS_SAMPLE_RATE_HZ   100 //100
#define LS_TIMER_SYSCTL     SYSCTL_PERIPH_TIMER1
#define LS_TIMER_BASE       TIMER1_BASE


/******************************************************************************
 * DMA control structure and RT sensor data destination buffer
 *****************************************************************************/
uint8_t uDMA_control_table[1024] __attribute__ ((aligned(1024)));
uint32_t rt_adc_buffer[8];
uint32_t ls_adc_buffer[3];

void * adc_src = (void *)(ADC0_BASE + ADC_O_SSFIFO0);
void * adc_dest = (void *)rt_adc_buffer;


/******************************************************************************
 * Task handles for IPC and synchronization
 *****************************************************************************/
extern TaskHandle_t rt_sensor_task_handle;
extern TaskHandle_t ls_sensor_task_handle;
extern TaskHandle_t control_task_handle;


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
// Real-time ADC data
static float motor_current_amps;       // Motor current (amperes)
static float vbus_volts;               // H-bridge bus voltage (volts)

// Low-speed ADC data
static float vbatt_volts;              // Controller battery voltage (volts)
static float motor_temp_celsius;       // Motor case temperature (celsius)
static float hbridge_temp_celsius;     // H-bridge heatsink temperature (celsius)


/******************************************************************************
 * Real-time ADC functions
 *
 * Motor current and bus voltage are sensed together at 40 kHz on ADC0.
 * When four samples of each have been stored by DMA, the control task is
 * woken to process these samples (at 10 kHz).
 *****************************************************************************/

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

    vTaskNotifyGiveFromISR(rt_sensor_task_handle, &task_woken);
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

    MAP_ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);

    // Enable ADC0 DMA interrupts
    MAP_ADCIntEnableEx(ADC0_BASE, ADC_INT_DMA_SS0);
    MAP_IntEnable(INT_ADC0SS0);

    MAP_ADCSequenceEnable(ADC0_BASE, 0);
    rt_trigger_timer_setup();
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

// This ISR runs when the low-speed sensor data has been collected. It checks
// for motor and h-bridge overheating, and control battery undervoltage
void adc1_seq0_ISR(void)
{
    MAP_ADCIntClear(ADC1_BASE, 0);

    // TODO: can we use a digital comparator instead?
    // logic to handle temp/battery errors shouldn't be here...

    // Pull data from ADC and convert
    /*
    uint32_t raw_data[3];
    MAP_ADCSequenceDataGet(ADC1_BASE, 0, raw_data);

    hbridge_temp_celsius = celsius_from_adc_raw(raw_data[0]);
    motor_temp_celsius = celsius_from_adc_raw(raw_data[1]);
    vbatt_volts = voltage_from_adc_raw(raw_data[2]);
    */

    MAP_ADCSequenceDataGet(ADC1_BASE, 0, ls_adc_buffer);

    BaseType_t task_woken;
    vTaskNotifyGiveFromISR(ls_sensor_task_handle, &task_woken);
    portYIELD_FROM_ISR(task_woken);
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

    MAP_ADCReferenceSet(ADC1_BASE, ADC_REF_EXT_3V);

    // Enable ADC0 interrupts
    MAP_ADCIntEnableEx(ADC1_BASE, ADC_INT_SS0);
    MAP_IntEnable(INT_ADC1SS0);

    MAP_ADCSequenceEnable(ADC1_BASE, 0);
    ls_trigger_timer_setup();
}


/******************************************************************************
 * Sensor task
 *
 * TODO: mechanism for periodically sending updates over CAN / UART, at
 * configurable rate(s)
 *
 * TODO: synchronize timers for ADC0 and ADC1; sensor task will do processing
 * of ADC1 data at divided frequency, but zero phase shift, relative to ADC0.
 *****************************************************************************/

/* Impulse response for the 40 kHz -> 10 kHz decimator.
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

/* Process real-time sensor data.
 */
void rt_sensor_task_code(void * arg)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    rt_adc_setup();

    unsigned h_len = sizeof(h_decimator) / sizeof(h_decimator[0]);
    struct fir_filter * current_decimator = fir_create(h_decimator, h_len, 4);
    struct fir_filter * voltage_decimator = fir_create(h_decimator, h_len, 4);

    while (1) {
        ulTaskNotifyTake(true, portMAX_DELAY);
        debug_pins_set(0x01, 0x01);

        // Retrieve newest samples from the DMA buffer
        float new_motor_current[4], new_bus_voltage[4];
        for (int i = 0; i < 4; i++) {
            new_motor_current[i] = rt_adc_buffer[i * 2];
            new_bus_voltage[i] = rt_adc_buffer[i * 2 + 1];
        }

        // Decimate to the primary control rate (10 kHz) and send the results
        // to the control task.
        debug_pins_set(0x02, 0x02);
        float current = fir_do_filter(current_decimator, new_motor_current);
        log_msg("%d\n", (int)current, 0, 0);
        current = (current * ADC_VOLTS_PER_BIT - 1.5) * -67.8;
        debug_pins_set(0x02, 0x00);

        float voltage = *new_bus_voltage;
        //float voltage = fir_do_filter(voltage_decimator, new_bus_voltage);
        //voltage = (voltage * ADC_VOLTS_PER_BIT * 19.58);

        // TODO: decimate to secondary rate for periodic update

        // Tasks that infrequently read these variables will implicitly
        // downsample, so we lowpass filter then to reduce aliasing
        vbus_volts = vbus_volts * 0.995 + voltage * 0.005;
        motor_current_amps = motor_current_amps * 0.995 + current * 0.005;

        debug_pins_set(0x01, 0x00);
        xTaskNotifyGive(control_task_handle);
    }
}

void ls_sensor_task_code(void * arg)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    ls_adc_setup();

    while (1) {
        ulTaskNotifyTake(true, portMAX_DELAY);
    }
}



/******************************************************************************
 * Public functions
 *****************************************************************************/
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

float array_average(float *ar, int n)
{
    float sum = 0;
    for (int i = 0; i < n; i++)
        sum += ar[i];
    return sum / n;
}
