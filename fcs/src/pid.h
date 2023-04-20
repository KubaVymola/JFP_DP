//==============================================================================
// pid.h
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef PID_H
#define PID_H

#include <stdio.h>
#include <algorithm>

struct pid_state_t {
    float k_p;
    float k_i;
    float k_d;

    float tau;

    float lim_max;
    float lim_min;

    float proportional;
    float differentiator;
    float integrator;
    float output;
    
    float prev_error;
    float prev_measurement;

    uint8_t integrator_enable;
};

/**
 * Initialization of the pid controller, tau is a time constant for the differentiator
*/
void pid_init(struct pid_state_t& pid, float k_p, float k_i, float k_d, float tau, float lim_min, float lim_max);

/**
 * Resets the pid differentiator, integrator, prev_error and prev_measurement
*/
void pid_reset(struct pid_state_t &pid);
void pid_integrator_disable(struct pid_state_t& pid);
void pid_integrator_enable(struct pid_state_t& pid);

float pid_update(struct pid_state_t &pid, float set_point, float measurement, float delta_t);

#endif
