/*
 * main.c
 *
 *  Created on: Jun 26, 2017
 *      Author: Oliver Douglas
 */

#include "stdinclude.h"
#include "encoder.h"
#include "system.h"
#include "pinconfig.h"
#include "sensors.h"
#include "can.h"

#include <driverlib/interrupt.h>
#include <driverlib/rom_map.h>
#include <driverlib/gpio.h>
#include <inc/hw_nvic.h>
#include <inc/tm4c1294kcpdt.h>
#include <sys/_stdint.h>

#include <string.h>


#ifndef HWREG
#define HWREG(x) (*((volatile uint32_t *)(x)))
#endif


// File index for ASSERT() macro
FILENUM(1)


/******************************************************************************
 * Task handles and task code declarations
 *****************************************************************************/
TaskHandle_t led_task_handle;

TaskHandle_t poll_task_handle;
TaskHandle_t rt_sensor_task_handle;
TaskHandle_t control_task_handle;
TaskHandle_t logger_task_handle;
TaskHandle_t system_task_handle;
TaskHandle_t encoder_task_handle;
TaskHandle_t can_task_handle;

extern void rt_sensor_task_code(void * arg);
extern void control_task_code(void * arg);
extern void logger_task_code(void * arg);
extern void system_task_code(void * arg);
extern void encoder_task_code(void * arg);
extern void can_task_code(void * arg);


/******************************************************************************
 * RTOS objects declared elsewhere, but initialized in main
 *****************************************************************************/
extern QueueHandle_t log_message_queue;
extern QueueHandle_t can_tx_queue;
extern QueueHandle_t can_rx_queue;

// TODO: implement this
void vApplicationStackOverflowHook(void)
{
    while (1)
        ;
}

inline void assertion_failed(int filenum, int linenum)
{
    IntMasterDisable();
    log_msg_panic("FATAL ERROR: Assertion failed: %d:%d\n", filenum, linenum, 0);
    while (1)
        ;
}


#define RED_LED   0
#define GREEN_LED 1
#define BLUE_LED  2

void poll_task_code(void * foo)
{
    struct can_msg msg;
    msg.can_cmd = CMD_SENSOR_DATA;
    msg.data_len = 5;

    float sensor_data[NUM_SENSORS];

    while (1) {
        vTaskDelay(10);

        switch (system_get_state()) {
        case STATE_CONFIG:
            led_set(RED_LED, true);
            led_set(GREEN_LED, true);
            led_set(BLUE_LED, false);
            break;
        case STATE_RUNNING:
            led_set(RED_LED, false);
            led_set(GREEN_LED, true);
            led_set(BLUE_LED, false);
            break;
        case STATE_FAULTED:
            led_set(RED_LED, true);
            led_set(GREEN_LED, false);
            led_set(BLUE_LED, false);
        }

        sensor_data[SENSOR_CURRENT] = sensor_get_motor_current();
        sensor_data[SENSOR_VELOCITY] = encoder_get_motor_velocity_rads();
        sensor_data[SENSOR_POSITION] = encoder_get_motor_position_rads();
        sensor_data[SENSOR_BUS_VOLTAGE] = sensor_get_bus_voltage();
        sensor_data[SENSOR_BATTERY_VOLTAGE] = sensor_get_battery_voltage();
        sensor_data[SENSOR_MOTOR_TEMP] = sensor_get_motor_temperature();
        sensor_data[SENSOR_HBRIDGE_TEMP] = sensor_get_hbridge_temperature();

        int enable_bits = system_read_config_reg(REG_SENSOR_LOG_ENABLES).i;
        for (int i = 0; i < NUM_SENSORS; i++) {
            if ((1 << i) & enable_bits) {
                msg.data[0] = i;
                memcpy(msg.data + 1, sensor_data + i, 4);
                can_send(&msg);
            }
        }
    }
}

int main (void)
 {
    // Disable write buffering to make all bus faults precise. This reduces
    // performance and should only be enabled when debugging faults.
    HWREG(NVIC_ACTLR) |= NVIC_ACTLR_DISWBUF;

    system_init_clocks();
    pinconfig();

    log_message_queue = xQueueCreate(100, sizeof(struct log_message));
    can_tx_queue = xQueueCreate(100, sizeof(struct can_msg));
    can_rx_queue = xQueueCreate(100, sizeof(struct can_msg));

    xTaskCreate(poll_task_code, "poll_task", configMINIMAL_STACK_SIZE,
                NULL, 2, &poll_task_handle);

    xTaskCreate(logger_task_code, "log_task", 500,
                NULL, 3, &logger_task_handle);

    xTaskCreate(can_task_code, "can_task", 500,
                NULL, 3, &can_task_handle);

    xTaskCreate(encoder_task_code, "encoder_task", 500,
                NULL, 5, &encoder_task_handle);

    xTaskCreate(system_task_code, "system_task", 500,
                NULL, 6, &system_task_handle);

    xTaskCreate(control_task_code, "control_task", 1000,
                NULL, 8, &control_task_handle);

    MAP_IntPrioritySet(INT_ADC0SS0, 0xE0);
    MAP_IntPrioritySet(INT_ADC1SS0, 0xE0);
    MAP_IntPrioritySet(INT_QEI0, 0xE0);
    MAP_IntPrioritySet(INT_CAN1, 0xE0);

    vTaskStartScheduler();
    ASSERT(0); // The scheduler does not return
}
