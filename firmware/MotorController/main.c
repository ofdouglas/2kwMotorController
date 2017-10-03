/*
 * main.c
 *
 *  Created on: Jun 26, 2017
 *      Author: Oliver Douglas
 */


#include "stdinclude.h"
#include "logger.h"
#include "system.h"

#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


void vApplicationStackOverflowHook(void)
{
    while (1)
        ;
}


#define LED_PIN     ((uint8_t)0x01)

void led_task_code(void * foo)
{
    (void) foo;

    // setup LED
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    for (int i = 0; i < 2; i++)
      ; // delay to avoid bus fault (access unpowered GPIO module)
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, LED_PIN);

    uint32_t pinVal = LED_PIN;
    int i = 0;
    while (1) {
        GPIOPinWrite(GPIO_PORTN_BASE, LED_PIN, (pinVal));
        pinVal ^= LED_PIN;
        log_message("Hello %d\n", i++, 0, 0);
        vTaskDelay(250);
    }
}

extern QueueHandle_t log_msg_queue;

TaskHandle_t led_task_handle;
TaskHandle_t logger_task_handle;

extern void logger_task_code(void * arg);

int main (void)
{
    system_init_clocks();

    log_msg_queue = xQueueCreate(100, sizeof(struct log_msg));

    xTaskCreate(led_task_code,
                "ledTask",
                200,
                NULL,
                3,
                &led_task_handle);

    xTaskCreate(logger_task_code,
                "log_task",
                200,
                NULL,
                2,
                &logger_task_handle);

    // TODO: replace this with pinout.c
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_1);

    vTaskStartScheduler();

    while (1)
        ; // The scheduler does not return

}
