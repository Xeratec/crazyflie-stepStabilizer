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

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"

// User files
#include "app_stepStabilizer_ml.h"
#include "machinelearning.h"

/* --------------- MACROS --------------- */

/* ---------- PRIVATE VARIABLES --------- */

const CTfLiteModel* tfl_model;
CTfLiteInterpreter* tfl_interpreter;

/* ------------- FUNCTIONS -------------- */

void stepStabilizer_machine_learning_init()
{
  
  tfl_model = CTfLiteModel_create(TFMICRO_MODEL);
  tfl_interpreter = CTFLiteInterpreter_create(tfl_model);
}

void stepStabilizer_machine_learning_reset() {
   return;
}

void stepStabilizer_machine_learning_test() {
    machine_learning_test();
}

void stepStabilizer_machine_learning_run(tofMeasurement_t *tofData, float acc_z, float posCtrl_z)
{
  // Run empty inference
  uint32_t inference_cycles = CTfLiteInterpreter_run(tfl_interpreter, NULL, 0, NULL, 0);
  if ( stepStabilizer_getConfig()->print_data ) {
    double inference_time = inference_cycles / (1.0f * configCPU_CLOCK_HZ) * 1000.0f;
    DEBUG_PRINT("%.3f ms (%lu CPU cycles).\r\n", inference_time, inference_cycles);
  }
}

