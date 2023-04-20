//==============================================================================
// utils.h
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <stdint.h>

/**
 * Transform x-coordinate from world into body coordinates. Used for autonomous navigation when the
 * FCS knows its position and true north heading
*/
const float get_x_body(const float yaw_rad, const float x_world, const float y_world);

/**
 * Transform y-coordinate from world into body coordinates. Used for autonomous navigation when the
 * FCS knows its position and true north heading
*/
const float get_y_body(const float yaw_rad, const float x_world, const float y_world);

/**
 * @return Get the legth of a three-dimensional vector
*/
const float vector_norm(float x, float y, float z);

/**
 * Performs a 'printf' through j-packets to the fdm. Is used to send command and debug output
*/
void send_jpacket_info(uint8_t channel, const char *text, int max_msg_len, ...);

void MagdwickGetOuput(float *qw, float *qx, float *qy, float *qz, float *yaw, float *pitch, float *roll);

#endif