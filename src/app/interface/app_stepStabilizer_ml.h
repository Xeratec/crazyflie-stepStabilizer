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