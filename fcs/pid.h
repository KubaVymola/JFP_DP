#ifndef PID_H
#define PID_H

#include <stdio.h>
#include <algorithm>

struct pid_state_t {
    double k_p;
    double k_i;
    double k_d;

    double tau;

    double lim_max;
    double lim_min;

    double proportional;
    double differentiator;
    double integrator;
    double output;
    
    double prev_error;
    double prev_measurement;
};

void pid_reset(struct pid_state_t &pid) {
    pid.differentiator = 0;
    pid.integrator = 0;
    pid.prev_error = 0;
    pid.prev_measurement = 0;
}

void pid_init(struct pid_state_t &pid, double k_p, double k_i, double k_d, double tau, double lim_min, double lim_max) {
    pid.k_p = k_p;
    pid.k_i = k_i;
    pid.k_d = k_d;

    pid.tau = tau;
    pid.lim_min = lim_min;
    pid.lim_max = lim_max;

    pid_reset(pid);
}

double pid_update(struct pid_state_t &pid, double set_point, double measurement, double delta_t) {
    double error = set_point - measurement;

    // printf("meas  %f\n", measurement);
    // printf("sp    %f\n", set_point);
    // printf("error %f\n", error);

    pid.proportional = pid.k_p * error;

    double limMaxInt = std::max(0.0, pid.lim_max - pid.proportional);
    double limMinInt = std::min(0.0, pid.lim_min - pid.proportional);
    
    pid.integrator = pid.integrator + 0.5 * pid.k_i * delta_t * (error + pid.prev_error);
    pid.integrator = std::min(limMaxInt, std::max(limMinInt, pid.integrator));

    pid.differentiator = (2.0 * -pid.k_d * (measurement - pid.prev_measurement) 
                       + (2.0 *  pid.tau - delta_t) * pid.differentiator)
                       / (2.0 *  pid.tau + delta_t);

    pid.output = pid.proportional + pid.integrator + pid.differentiator;
    pid.output = std::max(pid.lim_min, std::min(pid.lim_max, pid.output));

    pid.prev_error = error;
    pid.prev_measurement = measurement;

    // printf("pid_p %f\n", pid.proportional);
    // printf("pid_i %f\n", pid.integrator);
    // printf("pid_d %f\n", pid.differentiator);
    // printf("pid_o %f\n", pid.output);

    return pid.output;
}

#endif
