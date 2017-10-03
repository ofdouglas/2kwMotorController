/*
 * build.h - Top-level configuration
 *
 *  Created on: Oct 3, 2017
 *      Author: Oliver Douglas
 */

#ifndef BUILD_H_
#define BUILD_H_

enum { BUILD_TARGET_NONE, BUILD_TARGET_LAUNCHPAD, BUILD_TARGET_2KWMC };

// Change this when building for a different target platform
#define BUILD_TARGET BUILD_TARGET_LAUNCHPAD

#endif /* BUILD_H_ */
