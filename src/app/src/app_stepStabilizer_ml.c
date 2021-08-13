/*
 * app_stepStabilizer_ml.c
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

#define DEBUG_MODULE "APP"

// C Libraries
#include <math.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"

// Crazyflie Firmware
#include "log.h"
#include "param.h"

// User files
#include "app_stepStabilizer_ml.h"

/* --------------- MACROS --------------- */
#define OUTPUT_SCALE 5

float scale[] = {0.0201552701, 0.153594448, 0.187429917};
float mean[] = {1.002914, 0.60478177, 0.54083201351};
// float scale[] = {1, 1, 1};
// float mean[] = {0, 0, 0};


/* ---------- PRIVATE VARIABLES --------- */

const CTfLiteModel* tfl_model;
CTfLiteInterpreter* tfl_interpreter;

stepStabilizer_ml_t stepStabilizer_ml;
stepStabilizer_ml_sensorData_t sensor_data;
model_type sensor_data_temp[NUM_DATAPOINT*NUM_SENSORS];

model_type acc_z_tmp;
model_type posCtrl_z_tmp;
model_type tof_distance_tmp;

/* ------------- FUNCTIONS -------------- */

void stepStabilizer_machine_learning_init()
{
  
  tfl_model = CTfLiteModel_create(TFMICRO_MODEL);
  tfl_interpreter = CTFLiteInterpreter_create(tfl_model);

  stepStabilizer_ml = (stepStabilizer_ml_t)
  {
    .sensor_data = &sensor_data,
    .dhdt = 0,
    .step_height_estimation = 0,
    .buffer_idx = 0
  };

  memset(stepStabilizer_ml.sensor_data, 0, sizeof(stepStabilizer_ml_sensorData_t));
}

void stepStabilizer_machine_learning_reset() {
  stepStabilizer_ml.dhdt = 0;
  stepStabilizer_ml.step_height_estimation = 0;
  stepStabilizer_ml.buffer_idx = 0;
  memset(stepStabilizer_ml.sensor_data, 0, sizeof(stepStabilizer_ml_sensorData_t));
}

void stepStabilizer_machine_learning_test() {
    machine_learning_test();
}

float stepStabilizer_machine_learning_run(tofMeasurement_t *tofData, float acc_z, float posCtrl_z)
{ 
  acc_z_tmp = (acc_z - mean[0] ) / scale[0];
  posCtrl_z_tmp = (posCtrl_z- mean[1]) / scale[1];
  tof_distance_tmp = (tofData->distance - mean[2]) / scale[2];

  // Save new data into ringbuffer
  stepStabilizer_ml.sensor_data->tof_buffer[(stepStabilizer_ml.buffer_idx + NUM_DATAPOINT -1 ) % NUM_DATAPOINT] = tof_distance_tmp;
  stepStabilizer_ml.sensor_data->acc_z_buffer[(stepStabilizer_ml.buffer_idx + NUM_DATAPOINT -1 ) % NUM_DATAPOINT] = acc_z_tmp;
  stepStabilizer_ml.sensor_data->posCtrl_z_buffer[(stepStabilizer_ml.buffer_idx + NUM_DATAPOINT -1 ) % NUM_DATAPOINT] = posCtrl_z_tmp;

  for (int j=0; j<NUM_DATAPOINT; ++j) {
    sensor_data_temp[j*NUM_SENSORS+0] = stepStabilizer_ml.sensor_data->acc_z_buffer[(stepStabilizer_ml.buffer_idx+j ) % NUM_DATAPOINT];
    sensor_data_temp[j*NUM_SENSORS+1] = stepStabilizer_ml.sensor_data->posCtrl_z_buffer[(stepStabilizer_ml.buffer_idx+j ) % NUM_DATAPOINT];
    sensor_data_temp[j*NUM_SENSORS+2] = stepStabilizer_ml.sensor_data->tof_buffer[(stepStabilizer_ml.buffer_idx+j ) % NUM_DATAPOINT];
  }

  // Run empty inference
  uint32_t inference_cycles = CTfLiteInterpreter_run(tfl_interpreter, (float *) &sensor_data_temp, sizeof(stepStabilizer_ml_sensorData_t), &stepStabilizer_ml.dhdt, 1);
  if ( stepStabilizer_getConfig()->print_data ) {
    double inference_time = inference_cycles / (1.0f * configCPU_CLOCK_HZ) * 1000.0f;
    DEBUG_PRINT("%.3f ms (%lu CPU cycles).\r\n", inference_time, inference_cycles);
  }

  stepStabilizer_ml.buffer_idx = (stepStabilizer_ml.buffer_idx + 1) % NUM_DATAPOINT;

  if (fabsf(stepStabilizer_ml.dhdt) >= MIN_DHDT ) {
    stepStabilizer_ml.step_height_estimation += 0.025f * stepStabilizer_ml.dhdt * OUTPUT_SCALE;
  }

  if ( stepStabilizer_ml.step_height_estimation >  STEP_LIMIT) stepStabilizer_ml.step_height_estimation =  STEP_LIMIT;
  if ( stepStabilizer_ml.step_height_estimation < -STEP_LIMIT) stepStabilizer_ml.step_height_estimation = -STEP_LIMIT;

  return stepStabilizer_ml.step_height_estimation;
}

LOG_GROUP_START(ssm)
LOG_ADD(LOG_FLOAT, dhdt, &(stepStabilizer_ml.dhdt))
LOG_ADD(LOG_FLOAT, step_height_est, &(stepStabilizer_ml.step_height_estimation))
LOG_ADD(LOG_UINT32, buffer_idx, &(stepStabilizer_ml.buffer_idx))
LOG_ADD(LOG_FLOAT, acc_z, &acc_z_tmp)
LOG_ADD(LOG_FLOAT, posCtrl_z, &posCtrl_z_tmp)
LOG_ADD(LOG_FLOAT, tof_distance_tmp, &tof_distance_tmp)
LOG_GROUP_STOP(ssm)

