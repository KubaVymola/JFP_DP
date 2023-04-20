//==============================================================================
// low_pass_filter.cpp
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#include "low_pass_filter.h"

#include <math.h>
#include "defines.h"

void low_pass_filter_init(low_pass_filter_t &low_pass_filter, float cutoff_freq) {
    low_pass_filter.cutoff_freq = cutoff_freq;
    low_pass_filter.output = -INFINITY;
}

float low_pass_filter_update(low_pass_filter_t &low_pass_filter, float input, float delta_t_s) {
    if (low_pass_filter.output == -INFINITY) {
        low_pass_filter.output = input;
    }

    float tau = 1.0f / (2.0f * M_PI * low_pass_filter.cutoff_freq);
    float alpha = delta_t_s / (tau + delta_t_s);

    low_pass_filter.output = alpha * input
                            + (1.0f - alpha) * low_pass_filter.output;

    return low_pass_filter.output;
}
