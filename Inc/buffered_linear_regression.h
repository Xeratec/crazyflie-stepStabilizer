/**
 *      Compute a linear regression over the latest N datapoints
 */

#include <stdint.h>

typedef struct {
    uint32_t buffer_size;

    uint32_t* buffer_x;
    uint32_t* buffer_y;
    uint32_t buffer_idx;

    uint32_t sum_x;
    uint32_t sum_y;
    uint32_t sum_xy;
    uint32_t sum_x2;
} buffered_linear_regression_t;

// linear regression of type y = a * x + b
typedef struct {
    float a;
    float b;
} buffered_linear_regression_result_t;
/**
 *  Initialize a buffered_linear_regression with the given buffers and buffer length
 */
void buffered_linear_regression_init(buffered_linear_regression_t* hBuffer, 
                                     uint32_t* buffer_x, 
                                     uint32_t* buffer_y, 
                                     uint32_t length);

/**
 *  Insert new data into the linear regression buffer
 */
void buffered_linear_regression_add_new_data(buffered_linear_regression_t* hBuffer, uint32_t x, uint32_t y);

/**
 *  Compute the result of a linear regression fit
 */
buffered_linear_regression_result_t buffered_linear_regression_calculate_fit(buffered_linear_regression_t* hBuffer);