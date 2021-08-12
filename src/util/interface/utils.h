#ifndef __STEP_STABILIZER_UTILS_H__
#define __STEP_STABILIZER_UTILS_H__

#include <stdint.h>

/* --------------- MACROS --------------- */

#define abs( x )                       ((x) < 0 ? -(x) : (x))

/* -------- FUNCTION PROTOTYPES --------- */

float sumbuffer(float* array, uint32_t start_index, uint32_t end_index, uint32_t buffer_length);

#endif // __STEP_STABILIZER_UTILS_H__