#include "pid.h"

#include <stdio.h>
#include <algorithm>


void pid_reset(struct pid_state_t &pid) {
    pid.differentiator = 0;
    pid.integrator = 0;
    pid.prev_error = 0;
    pid.prev_measurement = 0;
}

void pid_integrator_disable(struct pid_state_t& pid) {
    pid.integrator_enable = 0;
}

void pid_integrator_enable(struct pid_state_t& pid) {
    pid.integrator_enable = 1;
}

void pid_init(struct pid_state_t& pid, float k_p, float k_i, float k_d, float tau, float lim_min, float lim_max) {
    pid.k_p = k_p;
    pid.k_i = k_i;
    pid.k_d = k_d;

    pid.tau = tau;
    pid.lim_min = lim_min;
    pid.lim_max = lim_max;

    pid.integrator_enable = 1;

    pid_reset(pid);
}

float pid_update(struct pid_state_t &pid, float set_point, float measurement, float delta_t) {
    float error = set_point - measurement;

    pid.proportional = pid.k_p * error;

    float limMaxInt = std::max(0.0f, pid.lim_max - pid.proportional);
    float limMinInt = std::min(0.0f, pid.lim_min + pid.proportional);
    
    if (pid.integrator_enable) {
        pid.integrator = pid.integrator + 0.5 * pid.k_i * delta_t * (error + pid.prev_error);
        pid.integrator = std::min(limMaxInt, std::max(limMinInt, pid.integrator));

    } else {
        pid.integrator = 0;
    }

    pid.differentiator = (2.0 * -pid.k_d * (measurement - pid.prev_measurement) + (2.0 *  pid.tau - delta_t) * pid.differentiator)
                       / (2.0 *  pid.tau + delta_t);


    pid.output = pid.proportional + pid.integrator + pid.differentiator;
    pid.output = std::max(pid.lim_min, std::min(pid.lim_max, pid.output));

    pid.prev_error = error;
    pid.prev_measurement = measurement;

    return pid.output;
}
