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

/* ---------- PRIVATE VARIABLES --------- */

typedef enum {
  SLOPE_TOF = 0,
  SLOPE_ACC_Z = 1,
} slope_t;


// TOF Data buffer for linear regression (edge detection)
buffered_linear_regression_t tof_buffer;
uint32_t tof_time_buffer[NUM_TOF_EDGE_DETECT];
uint32_t tof_data_buffer[NUM_TOF_EDGE_DETECT];

// accelerometer Data buffer for linear regression (edge detection)
buffered_linear_regression_float_t acc_buffer;
float acc_time_buffer[NUM_ACC_EDGE_DETECT];
float acc_data_buffer[NUM_ACC_EDGE_DETECT];

// Measurements of TOF from laser sensor
static xQueueHandle tofUnfilteredDataQueue;
STATIC_MEM_QUEUE_ALLOC(tofUnfilteredDataQueue, 1, sizeof(tofMeasurement_t));

// slopes from linear regression
float slopes[2];

/* ------------- FUNCTIONS -------------- */

void appMain()
{
  DEBUG_PRINT("Starting Application\n");

  // initialize the buffers
  buffered_linear_regression_init( &tof_buffer, tof_time_buffer, tof_data_buffer, NUM_TOF_EDGE_DETECT);
  buffered_linear_regression_init_float( &acc_buffer, acc_time_buffer, acc_data_buffer, NUM_ACC_EDGE_DETECT);

  // initialize the queue
  tofUnfilteredDataQueue = STATIC_MEM_QUEUE_CREATE(tofUnfilteredDataQueue);
  xQueueReset(tofUnfilteredDataQueue);

  logVarId_t idTOF = logGetVarId("range", "zrange");
  logVarId_t idAcc_z = logGetVarId("acc", "z");

  tofMeasurement_t tofData;

  while(1)
  {
    // wait for a new TOF measurement
    if( xQueueReceive(tofUnfilteredDataQueue, &tofData, M2T(100)) == pdTRUE )
    {
      // collect data
      uint32_t new_time = T2M(xTaskGetTickCount());
      uint32_t tof_new_data = logGetUint(idTOF);
      float acc_new_data = logGetFloat(idAcc_z);

      // DEBUG_PRINT("New data: TOF=%lu, ACC=%f\n", tof_new_data, (double) acc_new_data);

      buffered_linear_regression_add_new_data( &tof_buffer, new_time, tof_new_data);
      buffered_linear_regression_add_new_float_data( &acc_buffer, new_time, acc_new_data);
      buffered_linear_regression_result_t tof_reg_res = buffered_linear_regression_calculate_fit(&tof_buffer);
      buffered_linear_regression_result_t acc_reg_res = buffered_linear_regression_calculate_float_fit(&acc_buffer);

      // prevent unused warnings
      slopes[SLOPE_TOF] = tof_reg_res.a;
      slopes[SLOPE_ACC_Z] = acc_reg_res.a;

      // do the magic with the step detection and estimation
      // TODO

      // enqueue another height estimation for the controller
      // Note: Even though it is called TOF data, the value actually encodes the estimated down range
      // of the drone relative to the liftoff point and not the current distance the drone has to the floor
      estimatorEnqueueTOF(&tofData);
      
      // DEBUG_PRINT("TOF slope: %f, ACC slope: %f\n", (double) slopes[SLOPE_TOF], (double) slopes[SLOPE_ACC_Z]);
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

LOG_GROUP_START(stepstabilizer)
LOG_ADD(LOG_FLOAT, TOFslope, &slopes[SLOPE_TOF])
LOG_ADD(LOG_FLOAT, ACCZslope, &slopes[SLOPE_ACC_Z])
LOG_GROUP_STOP(stepstabilizer)