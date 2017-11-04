/*
 * fir.h - Finite Impulse Response (FIR) filter.
 *
 *  Created on: Nov 3, 2017
 *      Author: odougs
 */

#ifndef INCLUDE_FIR_H_
#define INCLUDE_FIR_H_


struct fir_filter;

/* Allocate and set up an FIR object.
 *  h      : Impulse response, h[n]
 *  h_len  : Length of h[n]
 *  stride : Decimation factor (set to 1 for a regular FIR filter)
 */
struct fir_filter * fir_create(float * h, unsigned h_len, unsigned stride);

/* Process the next input sample(s). The length of the vector pointed by x must
 * be equal to the FIR object's stride (1 for a regular FIR filter).
 */
float fir_do_filter(struct fir_filter * fir, const float * x);


#endif /* INCLUDE_FIR_H_ */
