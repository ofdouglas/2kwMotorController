/*
 * assert.h
 *
 *  Created on: Oct 18, 2017
 *      Author: Oliver Douglas
 */

#ifndef ASSERT_H_
#define ASSERT_H_

/* Enumerate files to avoid using strings in assertion_failed().
 * Credit to Niall Murphy "How to Define Your Own assert() Macro..."
 *
 * startup.c    0
 * main.c       1
 * system.c     2
 * sensors.c    3
 * control.c    4
 * logger.c     5
 * debug.c      6
 * can.c        7
 * pinconfig.c  8
 * encoder.c    9
 */
#define FILENUM(num) \
    enum { F_NUM=num }; \
    void _dummy##num(void) {}


void assertion_failed(int, int);


#define ASSERT(x)   \
        do { if (!(x)) assertion_failed(F_NUM, __LINE__); } while (0)


#endif /* ASSERT_H_ */
