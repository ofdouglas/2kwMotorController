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

#include <driverlib/interrupt.h>
#include <driverlib/rom_map.h>
#include <inc/hw_nvic.h>
#include <inc/tm4c1294kcpdt.h>
#include <sys/_stdint.h>


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
TaskHandle_t ls_sensor_task_handle;
TaskHandle_t rt_sensor_task_handle;
TaskHandle_t control_task_handle;
TaskHandle_t logger_task_handle;
TaskHandle_t system_task_handle;
TaskHandle_t encoder_task_handle;

extern void ls_sensor_task_code(void * arg);
extern void rt_sensor_task_code(void * arg);
extern void control_task_code(void * arg);
extern void logger_task_code(void * arg);
extern void system_task_code(void * arg);
extern void encoder_task_code(void * arg);


/******************************************************************************
 * RTOS objects declared elsewhere, but initialized in main
 *****************************************************************************/
extern QueueHandle_t log_message_queue;

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

void led_task_code(void * foo)
{
    (void) foo;

    // power-on test LEDs
    for (int i = 0; i < 4; i++) {
        led_set(i, true);
        vTaskDelay(500);
        led_set(i, false);
    }

    // flash LED0 indefinitely
    while (1) {
        led_set(0, false);
        vTaskDelay(400);
        led_set(0, true);
        vTaskDelay(100);
        // log_message(".", 0, 0, 0);
    }
}

void poll_task_code(void * foo)
{
    struct real_int ri;

    while (1) {
        //vTaskDelay(5);
        vTaskDelay(1000);
/*
        uint32_t tick_count = xTaskGetTickCount();
        ri = real_int_from_float(tick_count/1000.0, 2);

        log_msg("Uptime              = %d.%02d s\n", ri.whole, ri.fract, 0);

        log_msg("Motor current       = %d mA\n",
                sensor_get_motor_current_amps() * 1000, 0, 0);

        ri = real_int_from_float(sensor_get_vbus_volts(), 2);
        log_msg("Bus voltage         = %d.%02d V\n", ri.whole, ri.fract, 0);

        int rpm = encoder_get_motor_velocity_rads() * (30.0 / PI);
        log_msg("Motor speed         = %d RPM\n", rpm, 0, 0);
        log_msg("\n", 0, 0, 0);
        */
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

    xTaskCreate(poll_task_code, "poll_task", configMINIMAL_STACK_SIZE,
                NULL, 2, &poll_task_handle);

    xTaskCreate(led_task_code, "led_task", configMINIMAL_STACK_SIZE,
                NULL, 2, &led_task_handle);

    xTaskCreate(logger_task_code, "log_task", 500,
                NULL, 3, &logger_task_handle);

    xTaskCreate(ls_sensor_task_code, "sensor_task", 500,
                NULL, 4, &ls_sensor_task_handle);

    xTaskCreate(encoder_task_code, "encoder_task", 500,
                NULL, 5, &encoder_task_handle);

    xTaskCreate(system_task_code, "system_task", 500,
                NULL, 6, &system_task_handle);

    xTaskCreate(rt_sensor_task_code, "sensor_task", 500,
                NULL, 7, &rt_sensor_task_handle);

    xTaskCreate(control_task_code, "control_task", 500,
                NULL, 8, &control_task_handle);

    MAP_IntPrioritySet(INT_ADC0SS0, 0xE0);
    MAP_IntPrioritySet(INT_ADC1SS0, 0xE0);
    MAP_IntPrioritySet(INT_QEI0, 0xE0);

    vTaskStartScheduler();
    ASSERT(0); // The scheduler does not return
}
