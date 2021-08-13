/*
 * app_stepStabilizer_filter.c
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

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// Crazyflie Firmware
#include "log.h"
#include "param.h"

// User files
#include "app_stepStabilizer_filter.h"

/* --------------- MACROS --------------- */

#define ESTIMATOR_BUFFER_SIZE         20

/* ---------- PRIVATE VARIABLES --------- */

// data used for the estimation algorithm
stepStabilizer_estimation_t stepStabilizer_estimation;
stepStabilizer_estimation_parameters_t stepStabilizer_estimation_parameters;

/* ------------- FUNCTIONS -------------- */

void stepStabilizer_estimation_init()
{
  if (stepStabilizer_estimation.acc_z_dt_buffer != NULL) vPortFree(stepStabilizer_estimation.acc_z_dt_buffer);
  if (stepStabilizer_estimation.v_tof_buffer != NULL) vPortFree(stepStabilizer_estimation.v_tof_buffer);

  stepStabilizer_estimation_parameters = (stepStabilizer_estimation_parameters_t)
  {
    .max_step_duration = 10,
    .step_dv_hist_low = 0.2f,
    .step_dv_hist_high = 0.8f,
    .after_step_cooldown = 5,
  };

  stepStabilizer_estimation = (stepStabilizer_estimation_t)
  {
    .last_valid_v_tof_index = 0,
    .step_detected = false,
    .step_height_estimation = 0.0,
    .step_duration = 0,
    .after_step_cooldown = stepStabilizer_estimation_parameters.after_step_cooldown,
    .vel_z_estimated = 0,
    .prev_tof = (tofMeasurement_t) {.distance = 0, .stdDev = 0, .timestamp = xTaskGetTickCount()},
    .buffer_idx = ESTIMATOR_BUFFER_SIZE - 1,
    .acc_z_dt_buffer = (float*) pvPortMalloc(ESTIMATOR_BUFFER_SIZE * sizeof(float)),
    .v_tof_buffer = (float*) pvPortMalloc(ESTIMATOR_BUFFER_SIZE * sizeof(float)),
  };

  memset(stepStabilizer_estimation.acc_z_dt_buffer, 0, ESTIMATOR_BUFFER_SIZE * sizeof(float));
  memset(stepStabilizer_estimation.v_tof_buffer, 0, ESTIMATOR_BUFFER_SIZE * sizeof(float));
}

void stepStabilizer_estimation_reset() {
   stepStabilizer_estimation_init();
}

void stepStabilizer_estimation_test() {
    return;
}

float stepStabilizer_estimation_run(tofMeasurement_t *tofData, float acc_z)
{
  // make sure we are initialized
  if ( acc_z == 0.f) return 0;

  // make the code easier to read
  stepStabilizer_estimation_t* sse = &stepStabilizer_estimation;
  stepStabilizer_estimation_parameters_t* ssep = &stepStabilizer_estimation_parameters;

  float dt = T2M(tofData->timestamp - sse->prev_tof.timestamp) / 1000.f; // s
  float dh = tofData->distance - sse->prev_tof.distance; // m

  // 2 different estimations of the current velocity
  sse->dhdt = dh / dt; // m/s
  sse->acc_z_dt = (acc_z - 1) * 9.81f * dt; // m/s^2 * s = m/s, only current "difference" in velocity

  // store values into the buffer
  sse->buffer_idx = (sse->buffer_idx + 1) % ESTIMATOR_BUFFER_SIZE;
  sse->v_tof_buffer[sse->buffer_idx] = sse->dhdt;
  sse->acc_z_dt_buffer[sse->buffer_idx] = sse->acc_z_dt;

  // calculate the current estimated velocity by selecting a trusted reference point in the past, 
  // and integrating over the accelerometer data since that point in time until now
  sse->vel_reference = sse->v_tof_buffer[sse->last_valid_v_tof_index];
  sse->vel_integrated = sumbuffer(sse->acc_z_dt_buffer, 
                                  sse->last_valid_v_tof_index, 
                                  sse->buffer_idx,
                                  ESTIMATOR_BUFFER_SIZE);
  sse->vel_z_estimated = sse->vel_reference + sse->vel_integrated;

  // calculate the velocity difference between the estimated velocity and the differentiation of the TOF data
  // This is an indicater if a step is happening
  sse->vel_difference = abs(sse->vel_z_estimated - sse->dhdt);

  if( sse->step_detected)
  {
    sse->step_duration++;
    if ((sse->step_duration >= ssep->max_step_duration) || (sse->vel_difference < ssep->step_dv_hist_low))
    {
      sse->step_detected = false; // no step anymore
      sse->last_valid_v_tof_index = sse->buffer_idx; // latest
      sse->step_duration = 0; // reset step duration
      sse->after_step_cooldown = ssep->after_step_cooldown;
      //DEBUG_PRINT("End of step detected!\n");
    }
  }
  else
  {
    if ( sse->vel_difference > ssep->step_dv_hist_high && !sse->after_step_cooldown)
    {
      sse->step_detected = true;
      //DEBUG_PRINT("Start of step detected!\n");
    }
    if ( sse->after_step_cooldown ) sse->after_step_cooldown--;
    else sse->last_valid_v_tof_index = (sse->last_valid_v_tof_index + 1) % ESTIMATOR_BUFFER_SIZE; // current - n_estimate
  }

  if(sse->step_detected)
  {
    // update the step height estimation
    sse->step_height_estimation += dt * (sse->vel_z_estimated - sse->dhdt); // s * m/s = m

    // apply limits
    if ( sse->step_height_estimation >  STEP_LIMIT) sse->step_height_estimation =  STEP_LIMIT;
    if ( sse->step_height_estimation < -STEP_LIMIT) sse->step_height_estimation = -STEP_LIMIT;
  }

  // store the last "raw" tof data
  sse->prev_tof = *tofData;

  return sse->step_height_estimation;
}

LOG_GROUP_START(sse)
LOG_ADD(LOG_UINT8, step_det, &(stepStabilizer_estimation.step_detected))
LOG_ADD(LOG_FLOAT, dhdt, &(stepStabilizer_estimation.dhdt))
LOG_ADD(LOG_FLOAT, acc_z_dt, &(stepStabilizer_estimation.acc_z_dt))
LOG_ADD(LOG_FLOAT, vel_ref, &(stepStabilizer_estimation.vel_reference))
LOG_ADD(LOG_FLOAT, vel_int, &(stepStabilizer_estimation.vel_integrated))
LOG_ADD(LOG_FLOAT, vel_z_est, &(stepStabilizer_estimation.vel_z_estimated))
LOG_ADD(LOG_FLOAT, vel_diff, &(stepStabilizer_estimation.vel_difference))
LOG_ADD(LOG_UINT32, cooldown, &(stepStabilizer_estimation.after_step_cooldown))
LOG_ADD(LOG_UINT32, step_dur, &(stepStabilizer_estimation.step_duration))
LOG_ADD(LOG_FLOAT, step_height_est, &(stepStabilizer_estimation.step_height_estimation))
LOG_ADD(LOG_UINT32, buffer_idx, &(stepStabilizer_estimation.buffer_idx))
LOG_ADD(LOG_UINT32, v_ref_idx, &(stepStabilizer_estimation.last_valid_v_tof_index))
LOG_GROUP_STOP(sse)

PARAM_GROUP_START(ssep)
PARAM_ADD(PARAM_UINT32, max_step_duration, &(stepStabilizer_estimation_parameters.max_step_duration))
PARAM_ADD(PARAM_FLOAT, step_dv_hist_low, &(stepStabilizer_estimation_parameters.step_dv_hist_low))
PARAM_ADD(PARAM_FLOAT, step_dv_hist_high, &(stepStabilizer_estimation_parameters.step_dv_hist_high))
PARAM_ADD(PARAM_UINT32, after_step_cooldown , &(stepStabilizer_estimation_parameters.after_step_cooldown))
PARAM_GROUP_STOP(ssep)
