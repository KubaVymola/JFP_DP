#include "complementary_filter.h"

float complementary_filter_update(float alpha, float low_pass_input, float high_pass_input) {
    return alpha * low_pass_input
         + (1.0f - alpha) * high_pass_input;
}

