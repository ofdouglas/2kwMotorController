/*
 * debug.h
 *
 *  Created on: Oct 18, 2017
 *      Author: odougs
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "stdinclude.h"

bool led_get(int index);
void led_set(int index, bool on);

//int debug_pins_get(void);
void debug_pins_set(int mask, int on_bits);

#define LED_INDEX_MIN 0
#define LED_INDEX_MAX 3


#endif /* DEBUG_H_ */
