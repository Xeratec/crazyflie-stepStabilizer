#ifndef __STEP_STABILIZER_APP_MAIN_H__
#define __STEP_STABILIZER_APP_MAIN_H__

// C Libraries
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

// Crazyflie Firmware
#include "stabilizer_types.h"
#include "debug.h"

// User files
#include "utils.h"

#define STEP_LIMIT                     1.5f  // maximum (cumulative) step height in meters

typedef enum stepStabilizerAlgorithm_e {
    SSALGORITHM_NONE= 0,
    SSALGORITHM_ESTIMATION = 1,
    SSALGORITHM_MACHINE_LEARNING = 2,
} stepStabilizerAlgorithm_t;

typedef struct stepStabilizerConfig_s {
    uint8_t type;
    uint8_t reset;
    uint8_t test;
    uint8_t print_data;
    uint32_t tof_stdDev_multiplier;
} stepStabilizerConfig_t;

void stepStabilizer_enqueueTOF(tofMeasurement_t *tofData);
stepStabilizerConfig_t* stepStabilizer_getConfig(void);

#endif //__STEP_STABILIZER_APP_MAIN_H__