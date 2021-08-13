/*
 * utils.h
 * Philip Wiese <wiesep@student.ethz.ch>
 * Luca Rufer <lrufer@student.ethz.ch>
 *
 * Copyright (C) 2021 ETH Zurich
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License. 
 */

#ifndef __STEP_STABILIZER_UTILS_H__
#define __STEP_STABILIZER_UTILS_H__

#include <stdint.h>

/* --------------- MACROS --------------- */

#define abs( x )                       ((x) < 0 ? -(x) : (x))

/* -------- FUNCTION PROTOTYPES --------- */

float sumbuffer(float* array, uint32_t start_index, uint32_t end_index, uint32_t buffer_length);

#endif // __STEP_STABILIZER_UTILS_H__