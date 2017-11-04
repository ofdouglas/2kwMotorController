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

#define ABS(x)  ((x) > 0 ? (x) : -(x))
#define PI      3.1415926

#endif /* STDINCLUDE_H_ */
