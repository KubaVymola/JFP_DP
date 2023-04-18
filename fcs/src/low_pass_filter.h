#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

struct low_pass_filter_t {
  float cutoff_freq;
  float output;
};

void low_pass_filter_init(low_pass_filter_t &low_pass_filter, float cutoff_freq);
float low_pass_filter_update(low_pass_filter_t &low_pass_filter, float input, float delta_t_s);

#endif