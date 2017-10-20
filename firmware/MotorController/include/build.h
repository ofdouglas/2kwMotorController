/*
 * build.h - Top-level configuration
 *
 *  Created on: Oct 3, 2017
 *      Author: Oliver Douglas
 */

#ifndef BUILD_H_
#define BUILD_H_

//enum { BUILD_TARGET_NONE, BUILD_TARGET_LAUNCHPAD, BUILD_TARGET_2KWMC_R0 };
#define BUILD_TARGET_NONE           0
#define BUILD_TARGET_LAUNCHPAD      1
#define BUILD_TARGET_2KWMC_R0       2

// Change this when building for a different target platform
//#define BUILD_TARGET BUILD_TARGET_LAUNCHPAD
#define BUILD_TARGET BUILD_TARGET_2KWMC_R0

#if BUILD_TARGET == FOO

#elif BUILD_TARGET == BUILD_TARGET_LAUNCHPAD
#include "inc/tm4c1294ncpdt.h"

#elif BUILD_TARGET == BUILD_TARGET_2KWMC_R0
#include "inc/tm4c1294kcpdt.h"

#else
#error "No build target specified!"
#endif

#endif /* BUILD_H_ */
