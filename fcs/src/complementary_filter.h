//==============================================================================
// complementary_filter.h
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef COMPLEMENTARYFILTER_H
#define COMPLEMENTARYFILTER_H

float complementary_filter_update(float alpha, float low_pass_input, float high_pass_input);

#endif
