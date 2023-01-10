#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

float lowpass_filter_update(float alpha, float prev_estimate, float prev_measurement, float current_measurement, float deltaT) {
    return (1.0f - alpha) * prev_estimate + alpha * (prev_measurement + current_measurement) / 2;
}

#endif