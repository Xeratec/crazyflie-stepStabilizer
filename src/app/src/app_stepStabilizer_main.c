/*
 * app_stepStabilizer_main.c
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
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "app.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"

// Crazyflie Firmware
#include "estimator.h"
#include "log.h"
#include "param.h"

// User files
#include "app_stepStabilizer_main.h"
#include "app_stepStabilizer_filter.h"
#include "app_stepStabilizer_ml.h"

/* --------------- MACROS --------------- */
#define FREQUENCY_ML                   25
#define FREQUENCY_ESTIMATION           0

/* ---------- PRIVATE VARIABLES --------- */

// Measurements of TOF from laser sensor
static xQueueHandle tofUnfilteredDataQueue;
STATIC_MEM_QUEUE_ALLOC(tofUnfilteredDataQueue, 1, sizeof(tofMeasurement_t));

stepStabilizerConfig_t stepStabilizer_config =
{
  .type = SSALGORITHM_NONE,
  .reset = 0,
  .test = 0,
  .print_data = 0,
  .tof_stdDev_multiplier = 1,
};

float step_height_estimation = 0;

TickType_t last_estimation_inference;

/* ------------- FUNCTIONS -------------- */

void appMain()
{
  DEBUG_PRINT("Starting Application\n");
  DEBUG_PRINT("Free heap: %d bytes\n", xPortGetFreeHeapSize());

  // Initialize the queue
  tofUnfilteredDataQueue = STATIC_MEM_QUEUE_CREATE(tofUnfilteredDataQueue);
  xQueueReset(tofUnfilteredDataQueue);

  //logVarId_t idTOF = logGetVarId("range", "zrange");
  logVarId_t idAcc_z = logGetVarId("acc", "z");
  logVarId_t idPosCtrl_z = logGetVarId("posCtl", "targetZ");

  tofMeasurement_t tofData;

  DEBUG_PRINT("Init Filter Algorithm\n");
  stepStabilizer_estimation_init();

  DEBUG_PRINT("Init Neural Network Algorithm\n");
  stepStabilizer_machine_learning_init();

  last_estimation_inference = xTaskGetTickCount();

  while(1)
  {
     // check for reset
    if ( stepStabilizer_config.reset ) {
      DEBUG_PRINT("Reset Application State\n");
      stepStabilizer_estimation_reset();
      stepStabilizer_machine_learning_reset();
      stepStabilizer_config.reset = 0;
    }

    if (stepStabilizer_config.test ) {
      DEBUG_PRINT("Test Application\n");
      // Test TFMicro
      stepStabilizer_estimation_test();
      stepStabilizer_machine_learning_test();
      stepStabilizer_config.test = 0;
    }

    // Wait for a new TOF measurement
    if( xQueueReceive(tofUnfilteredDataQueue, &tofData, M2T(5)) == pdTRUE )
    {
      if ( 
        (stepStabilizer_config.type == SSALGORITHM_ESTIMATION) && 
        (xTaskGetTickCount() - last_estimation_inference > FREQUENCY_ESTIMATION) 
      ) { 
        step_height_estimation = stepStabilizer_estimation_run(&tofData, logGetFloat(idAcc_z));
      } else if ( 
        (stepStabilizer_config.type == SSALGORITHM_MACHINE_LEARNING) && 
        (xTaskGetTickCount() - last_estimation_inference > FREQUENCY_ML) 
      ) {
        last_estimation_inference = xTaskGetTickCount();
        step_height_estimation = stepStabilizer_machine_learning_run(&tofData, logGetFloat(idAcc_z), logGetFloat(idPosCtrl_z));
      } else {
        step_height_estimation = 0;
      }
   
      // Modify the tof std deviation if the measurement is not to be trusted (set by param)
      tofData.stdDev *= stepStabilizer_config.tof_stdDev_multiplier;

      // Modify the TOF data
      tofData.distance += step_height_estimation;

      // apply limits to the tof data
      if ( tofData.distance < 0) tofData.distance = 0;

      // enqueue another height estimation for the controller
      // Note: Even though it is called TOF data, the value actually encodes the estimated down range
      // of the drone relative to the liftoff point and not the current distance the drone has to the floor
      estimatorEnqueueTOF(&tofData);
    }   
  }
}

void stepStabilizer_enqueueTOF(tofMeasurement_t *tofData)
{
  xQueueOverwrite( tofUnfilteredDataQueue, tofData );
}

stepStabilizerConfig_t* stepStabilizer_getConfig()
{
  return &stepStabilizer_config;
}

PARAM_GROUP_START(stepstabilizer)
PARAM_ADD(PARAM_UINT8, type, &stepStabilizer_config.type)
PARAM_ADD(PARAM_UINT8, reset, &stepStabilizer_config.reset)
PARAM_ADD(PARAM_UINT8, test, &stepStabilizer_config.test)
PARAM_ADD(PARAM_UINT8, print_data, &stepStabilizer_config.print_data)
PARAM_ADD(PARAM_UINT32, stdDevMult, &stepStabilizer_config.tof_stdDev_multiplier)
PARAM_GROUP_STOP(stepstabilizer)