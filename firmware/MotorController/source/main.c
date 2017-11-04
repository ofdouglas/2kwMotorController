/*
 * main.c
 *
 *  Created on: Jun 26, 2017
 *      Author: Oliver Douglas
 */


#include <assert.h>
#include <debug.h>
#include <driverlib/interrupt.h>
#include <driverlib/rom_map.h>
#include <inc/hw_nvic.h>
#include <inc/tm4c1294kcpdt.h>
#include <logger.h>
#include <pinconfig.h>
#include <queue.h>
#include <sys/_stdint.h>
#include <system.h>

#ifndef HWREG
#define HWREG(x) (*((volatile uint32_t *)(x)))
#endif


#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


// File index for ASSERT() macro
FILENUM(1)


/******************************************************************************
 * Task handles and task code declarations
 *****************************************************************************/
TaskHandle_t led_task_handle;

TaskHandle_t sensor_task_handle;
TaskHandle_t control_task_handle;
TaskHandle_t logger_task_handle;
TaskHandle_t system_task_handle;
TaskHandle_t encoder_task_handle;

extern void sensor_task_code(void * arg);
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


int main (void)
 {
    // Disable write buffering to make all bus faults precise. This reduces
    // performance and should only be enabled when debugging faults.
    HWREG(NVIC_ACTLR) |= NVIC_ACTLR_DISWBUF;

    system_init_clocks();
    pinconfig();

    log_message_queue = xQueueCreate(100, sizeof(struct log_message));

    xTaskCreate(led_task_code, "led_task", configMINIMAL_STACK_SIZE,
                NULL, 2, &led_task_handle);

    xTaskCreate(logger_task_code, "log_task", 500,
                NULL, 3, &logger_task_handle);

    xTaskCreate(encoder_task_code, "encoder_task", 500,
                NULL, 4, &encoder_task_handle);

    xTaskCreate(system_task_code, "system_task", 500,
                NULL, 5, &system_task_handle);

    xTaskCreate(sensor_task_code, "sensor_task", 500,
                NULL, 6, &sensor_task_handle);

    xTaskCreate(control_task_code, "control_task", 500,
                NULL, 7, &control_task_handle);

    MAP_IntPrioritySet(INT_ADC0SS0, 0xE0);
    MAP_IntPrioritySet(INT_ADC1SS0, 0xE0);
    MAP_IntPrioritySet(INT_QEI0, 0xE0);

    vTaskStartScheduler();
    ASSERT(0); // The scheduler does not return
}
