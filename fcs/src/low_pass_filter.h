//==============================================================================
// low_pass_filter.h
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

struct low_pass_filter_t {
  float cutoff_freq;
  float output;
};

void low_pass_filter_init(low_pass_filter_t &low_pass_filter, float cutoff_freq);
float low_pass_filter_update(low_pass_filter_t &low_pass_filter, float input, float delta_t_s);

#endif