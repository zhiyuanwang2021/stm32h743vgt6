#include "filter.h"

/**
 * @brief  Initialize sliding window average filter
 * @param  f: filter instance pointer
 */
void Filter_Init(SlidingAvgFilter *f)
{
    memset(f->buffer, 0, sizeof(f->buffer));
    f->index = 0;
    f->sum = 0.0f;
    f->is_full = 0;
}

/**
 * @brief  Update filter with new sample value (sliding window average)
 * @param  f: filter instance pointer
 * @param  input: new raw sample
 * @retval filtered output value (window average)
 */
float Filter_Update(SlidingAvgFilter *f, float input)
{
    /* 1. Subtract the oldest data from sum */
    f->sum -= f->buffer[f->index];

    /* 2. Write new data and add to sum */
    f->buffer[f->index] = input;
    f->sum += input;

    /* 3. Update index (ring buffer) */
    f->index++;
    if (f->index >= FILTER_SIZE) {
        f->index = 0;
        f->is_full = 1;
    }

    /* 4. Calculate and return average */
    if (!f->is_full && f->index > 0) {
        return f->sum / f->index;
    }
    return f->sum / FILTER_SIZE;
}
