/*
 * logger.h
 *
 *  Created on: Oct 3, 2017
 *      Author: Oliver Douglas
 */

#ifndef LOGGER_H_
#define LOGGER_H_

// This is made public so that main() can allocate a log_message queue.
struct log_message {
    const char * str;
    int32_t args[3];
};

bool log_msg(const char * str, int32_t arg0, int32_t arg1, int32_t arg2);
int log_msg_ISR(const char * str, int32_t arg0, int32_t arg1, int32_t arg2);
void log_msg_panic(const char * str, int32_t arg0, int32_t arg1, int32_t arg2);

struct real_int {
    int whole;
    int fract;
};

struct real_int real_int_from_float(float x, int precision);


#endif /* LOGGER_H_ */
