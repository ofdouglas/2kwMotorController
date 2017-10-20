/*
 * main.c
 *
 *  Created on: Jun 26, 2017
 *      Author: Oliver Douglas
 */


#include "stdinclude.h"
#include "system.h"
#include "pinconfig.h"
#include "debug.h"

#include "driverlib/systick.h"
#include "driverlib/gpio.h"

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

extern void sensor_task_code(void * arg);
extern void control_task_code(void * arg);
extern void logger_task_code(void * arg);


/******************************************************************************
 * RTOS objects declared elsewhere, but initialized in main
 *****************************************************************************/
extern QueueHandle_t log_msg_queue;

// TODO: implement this
void vApplicationStackOverflowHook(void)
{
    while (1)
        ;
}

void assertion_failed(int filenum, int linenum)
{
    log_message("Assertion failed: %d:%d\n", filenum, linenum, 0);
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
        log_message(".", 0, 0, 0);
    }
}


int main (void)
{
    system_init_clocks();
    pinconfig();

    log_msg_queue = xQueueCreate(100, sizeof(struct log_msg));

    xTaskCreate(led_task_code, "led_task", configMINIMAL_STACK_SIZE,
                NULL, 2, &led_task_handle);

    xTaskCreate(logger_task_code, "log_task", configMINIMAL_STACK_SIZE,
                NULL, 3, &logger_task_handle);

    vTaskStartScheduler();
    ASSERT(0); // The scheduler does not return
}
