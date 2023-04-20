//==============================================================================
// complementary_filter.cpp
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#include "complementary_filter.h"

float complementary_filter_update(float alpha, float low_pass_input, float high_pass_input) {
    return alpha * low_pass_input
         + (1.0f - alpha) * high_pass_input;
}

