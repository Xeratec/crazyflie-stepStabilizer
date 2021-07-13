/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * internal_log_param_api.c - App layer application of the internal log
 *  and param api  
 */


#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"

#include "estimator.h"
#include "stabilizer_types.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#include "app_stepStabilizer_main.h"
#include "buffered_linear_regression.h"

#define DEBUG_MODULE "INTERNLOGPARAM"

/* --------------- MACROS --------------- */

#define STABILIZER_UPDATE_RATE    100   // ms

#define NUM_TOF_EDGE_DETECT            5   // number of TOF measurements used for edge detection
#define NUM_ACC_EDGE_DETECT            5   // number of ACC z-axis measurements used for edge detection

#define ESTIMATOR_BUFFER_SIZE         20

#define STEP_LIMIT                     3  // maximum (cumulative) step height in meters

#define abs( x ) ((x) < 0 ? -(x) : (x))

/* ---------- PRIVATE VARIABLES --------- */

typedef enum {
  SLOPE_TOF = 0,
  SLOPE_ACC_Z = 1,
} slope_t;


// TOF Data buffer for linear regression (edge detection)
buffered_linear_regression_float_t tof_buffer;
float tof_time_buffer[NUM_TOF_EDGE_DETECT];
float tof_data_buffer[NUM_TOF_EDGE_DETECT];

// accelerometer Data buffer for linear regression (edge detection)
buffered_linear_regression_float_t acc_buffer;
float acc_time_buffer[NUM_ACC_EDGE_DETECT];
float acc_data_buffer[NUM_ACC_EDGE_DETECT];

// Measurements of TOF from laser sensor
static xQueueHandle tofUnfilteredDataQueue;
STATIC_MEM_QUEUE_ALLOC(tofUnfilteredDataQueue, 1, sizeof(tofMeasurement_t));

// slopes from linear regression
float slopes[2];

// algorithm that is to be used for step detection and stabilization
stepStabilizerAlgorithm_t step_detection_approach = SSALGORITHM_NONE;
uint8_t step_detection_reset = 0;
uint8_t step_detection_print_data = 0;

// data used for the estimation algorithm
stepStabilizer_estimation_t stepStabilizer_estimation;
stepStabilizer_estimation_parameters_t stepStabilizer_estimation_parameters;

/* ------------- FUNCTIONS -------------- */

void appMain()
{
  DEBUG_PRINT("Starting Application\n");

  // initialize the buffers
  buffered_linear_regression_init_float( &tof_buffer, tof_time_buffer, tof_data_buffer, NUM_TOF_EDGE_DETECT);
  buffered_linear_regression_init_float( &acc_buffer, acc_time_buffer, acc_data_buffer, NUM_ACC_EDGE_DETECT);

  // initialize the queue
  tofUnfilteredDataQueue = STATIC_MEM_QUEUE_CREATE(tofUnfilteredDataQueue);
  xQueueReset(tofUnfilteredDataQueue);

  //logVarId_t idTOF = logGetVarId("range", "zrange");
  logVarId_t idAcc_z = logGetVarId("acc", "z");

  tofMeasurement_t tofData;

  // initialize the estimation algorithm
  stepStabilizer_estimation_init();

  while(1)
  {
    // wait for a new TOF measurement
    if( xQueueReceive(tofUnfilteredDataQueue, &tofData, M2T(100)) == pdTRUE )
    {
      // check for reset
      if ( step_detection_reset )
      {
        DEBUG_PRINT("Reset Application State\n");
        stepStabilizer_estimation_init();
        step_detection_reset = 0;
      }

      // collect data
      uint32_t new_time = T2M(xTaskGetTickCount()); // ms
      //float tof_new_data = logGetUint(idTOF);
      float tof_new_data = tofData.distance; //m 
      float acc_new_data = logGetFloat(idAcc_z);

      buffered_linear_regression_add_new_float_data( &tof_buffer, new_time/1000.f, tof_new_data);
      buffered_linear_regression_add_new_float_data( &acc_buffer, new_time, acc_new_data);
      buffered_linear_regression_result_t tof_reg_res = buffered_linear_regression_calculate_float_fit(&tof_buffer);
      buffered_linear_regression_result_t acc_reg_res = buffered_linear_regression_calculate_float_fit(&acc_buffer);

      // store the slopes into an array to use them as log parameters
      slopes[SLOPE_TOF] = tof_reg_res.a; // m/s
      slopes[SLOPE_ACC_Z] = acc_reg_res.a; // g/ms

      // do the magic with the step detection and estimation
      switch(step_detection_approach)
      {
        case SSALGORITHM_NONE:
          break;
        case SSALGORITHM_MACHINE_LEARNING:
          break;
        case SSALGORITHM_ESTIMATION:
          stepStabilizer_estimation_run(&tofData, acc_new_data);
          break;
      }

      if ( step_detection_print_data )
      {
        DEBUG_PRINT("%lu,%f,%f,%f,%f,%f,%f,%f,%lu,%f,%lu,%f,%lu,%lu,%lu\n",
                      new_time,
                      (double) tof_new_data, 
                      (double) acc_new_data,
                      (double) slopes[SLOPE_TOF],
                      (double) stepStabilizer_estimation.acc_z_dt,
                      (double) stepStabilizer_estimation.vel_reference,
                      (double) stepStabilizer_estimation.vel_integrated,
                      (double) stepStabilizer_estimation.vel_difference,
                      (uint32_t)(stepStabilizer_estimation.step_detected ? 1 : 0),
                      (double) stepStabilizer_estimation.step_height_estimation,
                      stepStabilizer_estimation.after_step_cooldown,
                      (double) stepStabilizer_estimation.vel_z_estimated,
                      stepStabilizer_estimation.step_duration,
                      stepStabilizer_estimation.buffer_idx,
                      stepStabilizer_estimation.last_valid_v_tof_index);
      } 

      // enqueue another height estimation for the controller
      // Note: Even though it is called TOF data, the value actually encodes the estimated down range
      // of the drone relative to the liftoff point and not the current distance the drone has to the floor
      estimatorEnqueueTOF(&tofData);
    }
    else 
    { 
      DEBUG_PRINT("TOF Timeout!\n");
    }
  }
}

void stepStabilizerEnqueueTOF(tofMeasurement_t *tofData)
{
  xQueueOverwrite( tofUnfilteredDataQueue, tofData );
}

void stepStabilizer_estimation_init()
{
  if (stepStabilizer_estimation.acc_z_dt_buffer != NULL) free(stepStabilizer_estimation.acc_z_dt_buffer);
  if (stepStabilizer_estimation.v_tof_buffer != NULL) free(stepStabilizer_estimation.v_tof_buffer);

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
    .acc_z_dt_buffer = (float*) malloc(ESTIMATOR_BUFFER_SIZE * sizeof(float)),
    .v_tof_buffer = (float*) malloc(ESTIMATOR_BUFFER_SIZE * sizeof(float)),
  };

  memset(stepStabilizer_estimation.acc_z_dt_buffer, 0, ESTIMATOR_BUFFER_SIZE * sizeof(float));
  memset(stepStabilizer_estimation.v_tof_buffer, 0, ESTIMATOR_BUFFER_SIZE * sizeof(float));
}

void stepStabilizer_estimation_run(tofMeasurement_t *tofData, float acc_z)
{
  // make sure we are initialized
  if ( acc_z == 0.f) return;

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
    sse->step_height_estimation += dt * (sse->vel_z_estimated - sse->dhdt); // s * m/s = m

    // apply limits
    if ( sse->step_height_estimation >  STEP_LIMIT) sse->step_height_estimation =  STEP_LIMIT;
    if ( sse->step_height_estimation < -STEP_LIMIT) sse->step_height_estimation = -STEP_LIMIT;
  }

  // store the last "raw" tof data
  sse->prev_tof = *tofData;

  // modify the TOF data
  tofData->distance += sse->step_height_estimation;

  // apply limits to the tof data
  if ( tofData->distance < 0) tofData->distance = 0;
}

// start inclusive, end exclusive
float sumbuffer(float* array, uint32_t start_index, uint32_t end_index, uint32_t buffer_length)
{
  float sum = 0;
  uint32_t i;
  if ( end_index <= start_index) 
  {
    for(i = start_index; i < buffer_length; i++) sum += array[i];
    for(i = 0; i < end_index; i++) sum += array[i];
  }
  else
  {
    for(i = start_index; i < end_index; i++) sum += array[i];
  }
  return sum;
}

LOG_GROUP_START(stepstabilizer)
LOG_ADD(LOG_FLOAT, TOFslope, &slopes[SLOPE_TOF])
LOG_ADD(LOG_FLOAT, ACCZslope, &slopes[SLOPE_ACC_Z])
LOG_GROUP_STOP(stepstabilizer)

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

PARAM_GROUP_START(stepstabilizer)
PARAM_ADD(PARAM_UINT8, type, &step_detection_approach)
PARAM_ADD(PARAM_UINT8, reset, &step_detection_reset)
PARAM_ADD(PARAM_UINT8, print_data, &step_detection_print_data)
PARAM_GROUP_STOP(stepstabilizer)

PARAM_GROUP_START(ssep)
PARAM_ADD(PARAM_UINT32, max_step_duration, &(stepStabilizer_estimation_parameters.max_step_duration))
PARAM_ADD(PARAM_FLOAT, step_dv_hist_low, &(stepStabilizer_estimation_parameters.step_dv_hist_low))
PARAM_ADD(PARAM_FLOAT, step_dv_hist_high, &(stepStabilizer_estimation_parameters.step_dv_hist_high))
PARAM_ADD(PARAM_UINT32, after_step_cooldown , &(stepStabilizer_estimation_parameters.after_step_cooldown))
PARAM_GROUP_STOP(ssep)
