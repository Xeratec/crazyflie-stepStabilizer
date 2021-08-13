/*
 * app_stepStabilizer_ml.h
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

#ifndef __STEP_STABILIZER_APP_ML_H__
#define __STEP_STABILIZER_APP_ML_H__

#include "app_stepStabilizer_main.h"
#include "machinelearning.h"

#define NUM_DATAPOINT 10
#define NUM_SENSORS 3

#define MIN_DHDT 0.05f

typedef struct __PACKED stepStabilizer_ml_sensorData_s {
    model_type acc_z_buffer[NUM_DATAPOINT];
    model_type posCtrl_z_buffer[NUM_DATAPOINT];
    model_type tof_buffer[NUM_DATAPOINT];
} stepStabilizer_ml_sensorData_t;

typedef struct stepStabilizer_ml_s {
    stepStabilizer_ml_sensorData_t *sensor_data;
    float dhdt;
    float step_height_estimation;
    uint32_t buffer_idx;
} stepStabilizer_ml_t;

void stepStabilizer_machine_learning_init(void);
void stepStabilizer_machine_learning_reset(void);
void stepStabilizer_machine_learning_test(void);
float stepStabilizer_machine_learning_run(tofMeasurement_t *tofData, float acc_z, float posCtrl_z);

#endif //__STEP_STABILIZER_APP_ML_H__