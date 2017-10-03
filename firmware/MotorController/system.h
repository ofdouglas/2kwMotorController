/*
 * system.h
 *
 *  Created on: Oct 3, 2017
 *      Author: odougs
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "stdinclude.h"

/******************************************************************************
 * System configuration
 *****************************************************************************/
void system_init_clocks(void);
uint32_t system_get_sysclk_freq(void);


#endif /* SYSTEM_H_ */
