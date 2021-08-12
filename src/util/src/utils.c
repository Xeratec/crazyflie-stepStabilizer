
#include "utils.h"

// start inclusive, end exclusive
float sumbuffer(float* array, uint32_t start_index, uint32_t end_index, uint32_t buffer_length)
{
  float sum = 0;
  uint32_t i;
  if ( end_index <= start_index) 
  {
    for(i = start_index; i < buffer_length; i++) sum += array[i];
    for(i = 0; i < end_index; i++) sum += array[i];
  }
  else
  {
    for(i = start_index; i < end_index; i++) sum += array[i];
  }
  return sum;
}