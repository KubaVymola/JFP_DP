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

#endif