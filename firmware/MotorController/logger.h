/*
 * logger.h
 *
 *  Created on: Oct 3, 2017
 *      Author: Oliver Douglas
 */

#ifndef LOGGER_H_
#define LOGGER_H_



struct log_msg {
    const char * str;
    int32_t args[3];
};
bool log_message(const char * str, int32_t arg0, int32_t arg1, int32_t arg2);
int log_message_from_ISR(const char * str, int32_t arg0,
                         int32_t arg1, int32_t arg2);


#endif /* LOGGER_H_ */
