/*
 * debug.c - control status LEDs and debug pins
 *
 *  Created on: Oct 18, 2017
 *      Author: Oliver Douglas
 */

#include "debug.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"

// File index for ASSERT() macro
FILENUM(6)

struct gpio_lookup {
    uint32_t gpio_port;
    int pin_num;
};


#if BUILD_TARGET == BUILD_TARGET_LAUNCHPAD

struct gpio_lookup leds_table[] = {
 {GPIO_PORTN_BASE, GPIO_PIN_1},
 {GPIO_PORTN_BASE, GPIO_PIN_0},
 {GPIO_PORTF_BASE, GPIO_PIN_4},
 {GPIO_PORTF_BASE, GPIO_PIN_0}
};

#elif BUILD_TARGET == BUILD_TARGET_2KWMC_R0

struct gpio_lookup leds_table[] = {
 {GPIO_PORTP_BASE, GPIO_PIN_2},
 {GPIO_PORTP_BASE, GPIO_PIN_3},
 {GPIO_PORTP_BASE, GPIO_PIN_4},
 {GPIO_PORTP_BASE, GPIO_PIN_5}
};

/*
struct gpio_lookup debug_table[] = {
 {GPIO_PORTN_BASE, GPIO_PIN_0},
 {GPIO_PORTN_BASE, GPIO_PIN_1},
 {GPIO_PORTN_BASE, GPIO_PIN_2},
 {GPIO_PORTN_BASE, GPIO_PIN_3},
 {GPIO_PORTP_BASE, GPIO_PIN_4},
 {GPIO_PORTP_BASE, GPIO_PIN_5},
 {GPIO_PORTP_BASE, GPIO_PIN_6},
 {GPIO_PORTP_BASE, GPIO_PIN_7},
};
*/

#endif


bool led_get(int index)
{
    ASSERT(index >= LED_INDEX_MIN && index <= LED_INDEX_MAX);

    struct gpio_lookup led = leds_table[index];
    return MAP_GPIOPinRead(led.gpio_port, led.pin_num) ? 1 : 0;
}

void led_set(int index, bool on)
{
    ASSERT(index >= LED_INDEX_MIN && index <= LED_INDEX_MAX);

    struct gpio_lookup led = leds_table[index];
    MAP_GPIOPinWrite(led.gpio_port, led.pin_num, on ? led.pin_num : 0);
}

void debug_pins_set(int mask, int on_bits)
{
#if BUILD_TARGET == BUILD_TARGET_2KWMC_R0
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, mask & 0x0F, on_bits);
    MAP_GPIOPinWrite(GPIO_PORTP_BASE, mask & 0xF0, on_bits);
#endif
}
