#ifndef UTILS_H
#define UTILS_H

#include <math.h>

int round_down_to_multiple(int number, int multiple);
const float get_x_body(const float yaw_rad, const float x_world, const float y_world);
const float get_y_body(const float yaw_rad, const float x_world, const float y_world);
const float vector_norm(float x, float y, float z);

#endif