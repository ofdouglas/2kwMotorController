/*
 * fir.c - Finite Impulse Response (FIR) filter.
 *
 *  Created on: Nov 3, 2017
 *      Author: Oliver Douglas
 */

#include "stdinclude.h"
#include <stdlib.h>
#include <string.h>

struct fir_filter {
  float * h;        // Impulse response, h[n]
  float * x;        // Circular buffer of previous filter inputs, x[n]
  unsigned h_len;   // Length of h[n]
  unsigned index;   // Index into the buffer x[n]. Points to most recent input.
  unsigned stride;  // Decimation factor (set to 1 for a regular FIR filter)
};


struct fir_filter * fir_create(const float * h, unsigned h_len, unsigned stride)
{
  struct fir_filter * fir = malloc(sizeof(struct fir_filter));
  fir->h = malloc(h_len * sizeof(float));
  fir->x = malloc(h_len * sizeof(float));
  fir->h_len = h_len;
  fir->index = 0;
  fir->stride = stride;

  memcpy(fir->h, h, h_len * sizeof(float));
  memset(fir->x, 0, h_len * sizeof(float));

  return fir;
}

float fir_do_filter(struct fir_filter * fir, const float * x)
{
  float y = 0;

  for (int i = 0; i < fir->stride; i++) {
    fir->index = (fir->index + 1) % fir->h_len;
    (fir->x)[fir->index] = *x++;
  }

  for (int i = 0, j = fir->index; i < fir->h_len; i++) {
    y += (fir->x)[j] * fir->h[i];
    j = (j + 1) % fir->h_len;
  }

  return y;
}



