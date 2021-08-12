/**
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

  while(1)
  {
     // check for reset
    if ( stepStabilizer_config.reset )
    {
      DEBUG_PRINT("Reset Application State\n");
      // stepStabilizer_estimation_init();
      stepStabilizer_config.reset = 0;
    }

    if (stepStabilizer_config.test ) {
      DEBUG_PRINT("Test Application\n");
      // Test TFMicro
      stepStabilizer_machine_learning_test();
      stepStabilizer_config.test = 0;
    }
    // wait for a new TOF measurement
    if( xQueueReceive(tofUnfilteredDataQueue, &tofData, M2T(100)) == pdTRUE )
    {
      // collect data
      float acc_new_data = logGetFloat(idAcc_z);
      float pos_new_data = logGetFloat(idPosCtrl_z);

      // do the magic with the step detection and estimation
      switch(stepStabilizer_config.type)
      {
        case SSALGORITHM_NONE:
          break;
        case SSALGORITHM_MACHINE_LEARNING:
          stepStabilizer_machine_learning_run(&tofData, acc_new_data, pos_new_data);
          break;
        case SSALGORITHM_ESTIMATION:
          stepStabilizer_estimation_run(&tofData, acc_new_data);
          break;
      }

      // modify the tof std deviation if the measurement is not to be trusted (set by param)
      tofData.stdDev *= stepStabilizer_config.tof_stdDev_multiplier;

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