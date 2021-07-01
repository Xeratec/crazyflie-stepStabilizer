
#include <string.h>
#include "buffered_linear_regression.h"

void buffered_linear_regression_init(buffered_linear_regression_t* hBuffer, 
                                     uint32_t* buffer_x, 
                                     uint32_t* buffer_y, 
                                     uint32_t length)
{
    hBuffer->buffer_size = length;
    hBuffer->buffer_idx = 0;
    hBuffer->buffer_x = buffer_x;
    hBuffer->buffer_y = buffer_y;

    hBuffer->sum_x = 0;
    hBuffer->sum_y = 0;
    hBuffer->sum_x2 = 0;
    hBuffer->sum_xy = 0;

    memset(buffer_x, 0, length * sizeof(uint32_t));
    memset(buffer_y, 0, length * sizeof(uint32_t));
}

void buffered_linear_regression_add_new_data(buffered_linear_regression_t* hBuffer, uint32_t x, uint32_t y)
{
    // extract old data
    uint32_t oldest_x = hBuffer->buffer_x[hBuffer->buffer_idx];
    uint32_t oldest_y = hBuffer->buffer_y[hBuffer->buffer_idx];

    // insert new data into the buffer
    hBuffer->buffer_x[hBuffer->buffer_idx] = x;
    hBuffer->buffer_y[hBuffer->buffer_idx] = y;

    // progess buffer the index
    hBuffer->buffer_idx++;
    if( hBuffer->buffer_idx >= hBuffer->buffer_size) hBuffer->buffer_idx = 0;

    // update the sums
    hBuffer->sum_x = hBuffer->sum_x - oldest_x + x;
    hBuffer->sum_y = hBuffer->sum_y - oldest_y + y;
    hBuffer->sum_xy = hBuffer->sum_xy - oldest_x * oldest_y + x * y;
    hBuffer->sum_x2 = hBuffer->sum_x2 - oldest_x * oldest_x + x * x;
}

buffered_linear_regression_result_t buffered_linear_regression_calculate_fit(buffered_linear_regression_t* hBuffer)
{
    buffered_linear_regression_result_t res;

    // calculate the common denominator
    float denom = (hBuffer->buffer_size * hBuffer->sum_x2 - hBuffer->sum_x * hBuffer->sum_x);

    // calculate the linear regression fit ( y = a * x + b )
    res.a = (-1.f * hBuffer->sum_x  * hBuffer->sum_y + hBuffer->buffer_size * hBuffer->sum_xy) / denom;
    res.b = ( 1.f * hBuffer->sum_x2 * hBuffer->sum_y - hBuffer->sum_x       * hBuffer->sum_xy) / denom;

    return res;
}