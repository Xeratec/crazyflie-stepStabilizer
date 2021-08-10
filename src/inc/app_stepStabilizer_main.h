
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"

#include "stabilizer_types.h"

typedef enum stepStabilizerAlgorithm_e {
    SSALGORITHM_NONE= 0,
    SSALGORITHM_ESTIMATION = 1,
    SSALGORITHM_MACHINE_LEARNING = 2,
} stepStabilizerAlgorithm_t;

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

void stepStabilizerEnqueueTOF(tofMeasurement_t *tofData);

void stepStabilizer_estimation_init(void);
void stepStabilizer_machine_learning_init(void);

void stepStabilizer_estimation_run(tofMeasurement_t *tofData, float acc_z);
void stepStabilizer_machine_learning_run(tofMeasurement_t *tofData, float acc_z, float posCtrl_z);

float sumbuffer(float* array, uint32_t start_index, uint32_t end_index, uint32_t buffer_length);