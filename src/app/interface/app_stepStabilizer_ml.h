#ifndef __STEP_STABILIZER_APP_ML_H__
#define __STEP_STABILIZER_APP_ML_H__

#include "app_stepStabilizer_main.h"

void stepStabilizer_machine_learning_init(void);
void stepStabilizer_machine_learning_reset(void);
void stepStabilizer_machine_learning_test(void);
void stepStabilizer_machine_learning_run(tofMeasurement_t *tofData, float acc_z, float posCtrl_z);

#endif //__STEP_STABILIZER_APP_ML_H__