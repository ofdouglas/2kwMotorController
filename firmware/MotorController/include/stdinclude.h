/*
 * stdinclude.h
 *
 *  Created on: Oct 3, 2017
 *      Author: Oliver Douglas
 */

#ifndef STDINCLUDE_H_
#define STDINCLUDE_H_

// libc includes
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// tivaware / driverlib includes
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "FreeRTOSConfig.h"

// project includes
#include "build.h"
#include "logger.h"
#include "assert.h"
#include "debug.h"

#define ABS(x)          ((x) > 0 ? (x) : -(x))
#define PI              3.1415926
#define RPM_TO_RADS     (PI / 30.0)
#define RADS_TO_RPM     (30.0 / PI)

typedef union {
    int i;
    float f;
} union32;

#endif /* STDINCLUDE_H_ */
