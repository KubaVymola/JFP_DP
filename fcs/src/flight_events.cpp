//==============================================================================
// flight_events.cpp
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#include "flight_events.h"

#include "string.h"
#include "defines.h"
#include "global_variables.h"
#include "utils.h"
#include "pid.h"
#include "utils.h"

/**
 * Needs maximum sensitivity. Cannot miss take-off detection but can incorrectly detect take-off
 * @return Returns true only once per take-off
 */
bool detect_takeoff() {
    if (is_airborne
    || vector_norm(lin_acc_x_g, lin_acc_y_g, lin_acc_z_g) < TAKEOFF_LIN_ACC_THRESHLD
    || (vertical_mode != VERTICAL_MODE_SP && ctrl_channels_norm[THROTTLE_CHANNEL] < TAKEOFF_THRUST_POS_THRESHLD)) {
        takeoff_event_start_s = EVENT_TIME_INVALID;
        return false;
    }

    /**
     * All checks OK
     */

    if (takeoff_event_start_s == EVENT_TIME_INVALID) {
        takeoff_event_start_s = time_s;
        return false;
    }

    if (time_s - takeoff_event_start_s < TAKEOFF_EVENT_DURATION) {
        return false;
    }

    send_jpacket_info(0x03, "Take-off detected", 64);

    takeoff_event_start_s = EVENT_TIME_INVALID;
    return true;
}

/**
 * Needs maximum specificity. Cannot detect landing whilst in the air, but can miss actual landing
 * @return Returns true only once per landing
 */
bool detect_landing() {
    if (!is_airborne
    || vector_norm(lin_acc_x_g, lin_acc_y_g, lin_acc_z_g) > LANDING_LIN_ACC_THRESHLD
    || roll_rate_dps > LANDING_RATES_THRESHLD
    || pitch_rate_dps > LANDING_RATES_THRESHLD
    || (vertical_mode != VERTICAL_MODE_SP && ctrl_channels_norm[THROTTLE_CHANNEL] > LANDING_THRUST_POS_THRESHLD)) {
        landing_event_start_s = EVENT_TIME_INVALID;
        return false;
    }

    /**
     * All checks OK
     */

    if (landing_event_start_s == EVENT_TIME_INVALID) {
        landing_event_start_s = time_s;
        return false;
    }

    if (time_s - landing_event_start_s < LANDING_EVENT_DURATION) {
        return false;
    }

    send_jpacket_info(0x03, "Landing detected", 64);

    landing_event_start_s = EVENT_TIME_INVALID;
    return true;
}


/**
 * Cannot false-arm, for safety
*/
bool detect_arm() {
    if (!is_armed && AUTOARM_L) return true;

    if (is_armed) return false;
    if (ctrl_channels_norm[THROTTLE_CHANNEL] > IDLE_THRUST_POS_THRESHLD) return false;
    if (is_airborne) return false;
    if (arm_channel_norm_prev > -0.5f) return false;
    if (ctrl_channels_norm[ARM_CHANNEL] < 0.5f) return false;

    send_jpacket_info(0x03, "Arm detected!", 64);

    return true;
}

/**
 * Has to be very easy to disarm, only ARM_CHANNEL value needed to disarm
*/
bool detect_disarm() {
    if (AUTOARM_L) return false;
    if (!is_armed) return false;
    if (force_arm) return false;
    if (ctrl_channels_norm[ARM_CHANNEL] > 0.5f) return false;

    send_jpacket_info(0x03, "Disarm detected", 64);

    return true;
}

void disable_pid_integrators() {
    pid_integrator_disable(alt_sp_pid);
    pid_integrator_disable(alt_rate_pid);
    pid_integrator_disable(yaw_sp_pid);
    pid_integrator_disable(yaw_rate_pid);
    pid_integrator_disable(x_body_pid);
    pid_integrator_disable(y_body_pid);
    pid_integrator_disable(roll_pid);
    pid_integrator_disable(pitch_pid);
    pid_integrator_disable(roll_rate_pid);
    pid_integrator_disable(pitch_rate_pid);
}

void enable_pid_integrators() {
    pid_integrator_enable(alt_sp_pid);
    pid_integrator_enable(alt_rate_pid);
    pid_integrator_enable(yaw_sp_pid);
    pid_integrator_enable(yaw_rate_pid);
    pid_integrator_enable(x_body_pid);
    pid_integrator_enable(y_body_pid);
    pid_integrator_enable(roll_pid);
    pid_integrator_enable(pitch_pid);
    pid_integrator_enable(roll_rate_pid);
    pid_integrator_enable(pitch_rate_pid);
}

