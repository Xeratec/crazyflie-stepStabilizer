/*
 * machinelearning.h
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

#ifndef __STEP_STABILIZER_ML_H__
#define __STEP_STABILIZER_ML_H__

#include "tfmicro_models.h"

#ifndef TFMICRO_MODEL
#define TFMICRO_MODEL NN_SSE_RMS_10_v2
#endif

// type of model. uint8_t if quantized, usually float if not.
#define model_type float

#ifdef __cplusplus
extern "C" {
#endif

int machine_learning_test();

struct CTfLiteModel; // An opaque type that we'll use as a handle
typedef struct CTfLiteModel CTfLiteModel;

const CTfLiteModel* CTfLiteModel_create(const void *);
void CTfLiteModel_destroy(CTfLiteModel* c_model);
int CTfLiteModel_version(CTfLiteModel* c_model);

struct CTfLiteInterpreter; // An opaque type that we'll use as a handle
typedef struct CTfLiteInterpreter CTfLiteInterpreter;

CTfLiteInterpreter* CTFLiteInterpreter_create(const CTfLiteModel* c_model);
void CTFLiteInterpreter_destroy(CTfLiteInterpreter* c_interpreter);

// Actual inference functions
int CTfLiteInterpreter_run(CTfLiteInterpreter* c_interpreter, model_type* input, size_t input_size, model_type* result, size_t result_size);
#ifdef __cplusplus
}
#endif

#endif // __STEP_STABILIZER_ML_H__
