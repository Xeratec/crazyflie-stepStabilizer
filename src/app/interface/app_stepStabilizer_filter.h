#ifndef __STEP_STABILIZER_APP_FILTER_H__
#define __STEP_STABILIZER_APP_FILTER_H__

#include "app_stepStabilizer_main.h"


typedef struct stepStabilizer_estimation_s {
    // values calculated in the latest iteration (only used for log, not state-relevant)
    bool step_detected;
    float dhdt;
    float acc_z_dt;
    float vel_reference;
    float vel_integrated;
    float vel_z_estimated;
    float vel_difference;
    
    // step duration and cooldown
    uint32_t after_step_cooldown;
    uint32_t step_duration;
    
    // TOF measurement and current height
    tofMeasurement_t prev_tof;
    float step_height_estimation;

    // other values buffer
    float* v_tof_buffer;
    float* acc_z_dt_buffer;
    uint32_t buffer_idx;
    uint32_t last_valid_v_tof_index;
} stepStabilizer_estimation_t;

typedef struct stepStabilizer_estimation_paramters_s {
    uint32_t max_step_duration;
    float step_dv_hist_low;
    float step_dv_hist_high;
    uint32_t after_step_cooldown;
} stepStabilizer_estimation_parameters_t;

void stepStabilizer_estimation_init(void);
void stepStabilizer_estimation_reset(void);
void stepStabilizer_estimation_test(void);
float stepStabilizer_estimation_run(tofMeasurement_t *tofData, float acc_z);

#endif //__STEP_STABILIZER_APP_FILTER_H__