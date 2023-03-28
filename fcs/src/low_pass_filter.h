#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

struct low_pass_filter_t {
  float cutoff_freq;
  float output;
};

void low_pass_filter_init(low_pass_filter_t &low_pass_filter, float cutoff_freq);
float low_pass_filter_update(low_pass_filter_t &low_pass_filter, float input, float delta_t_s);

    
    
// float low_pass_filter_update(float alpha, float prev_estimate, float prev_measurement, float current_measurement, float deltaT) {
//     return (1.0f - alpha) * prev_estimate + alpha * (prev_measurement + current_measurement) / 2;
// }

#endif