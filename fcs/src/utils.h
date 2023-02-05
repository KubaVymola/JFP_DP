#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <stdint.h>

const float get_x_body(const float yaw_rad, const float x_world, const float y_world);
const float get_y_body(const float yaw_rad, const float x_world, const float y_world);
const float vector_norm(float x, float y, float z);

void send_jpacket_info(uint8_t channel, const char *text, int max_msg_len, ...);

#endif