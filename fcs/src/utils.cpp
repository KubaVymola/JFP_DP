//==============================================================================
// utils.cpp
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#include "utils.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "defines.h"
#include "j_packets.h"
#include "j_packet_send_callback.h"
#include "MadgwickAHRS.h"

const float get_x_body(const float yaw_rad, const float x_world, const float y_world) {
    return x_world * cos(-yaw_rad) + y_world * sin(-yaw_rad);
}

const float get_y_body(const float yaw_rad, const float x_world, const float y_world) {
    return x_world * -sin(-yaw_rad) + y_world * cos(-yaw_rad);
}

const float vector_norm(float x, float y, float z) {
    return sqrtf(x*x + y*y + z*z);
}

void send_jpacket_info(uint8_t channel, const char *text, int max_msg_len, ...) {
    va_list argp;

    char buf[max_msg_len];

    va_start(argp, max_msg_len);
    vsnprintf(buf, sizeof(buf), text, argp);
    va_end(argp);

    j_packet_send(channel, buf, strlen(buf), 1, J_PACKET_SIZE, j_packet_send_callback);
}

void MagdwickGetOuput(float *qw, float *qx, float *qy, float *qz, float *yaw, float *pitch, float *roll) {
    *qw = q0;
    *qx = q1;
    *qy = q2;
    *qz = q3;

    *yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * RAD_TO_DEG;
    *pitch = asinf(-2.0f * (q1 * q3 - q0 * q2)) * RAD_TO_DEG;
    *roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * RAD_TO_DEG;
}
