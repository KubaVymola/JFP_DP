#include "utils.h"

int round_down_to_multiple(int number, int multiple) {
    return number - number % multiple;    
}

const float get_x_body(const float yaw_rad, const float x_world, const float y_world) {
    return x_world * cos(-yaw_rad) + y_world * sin(-yaw_rad);
}

const float get_y_body(const float yaw_rad, const float x_world, const float y_world) {
    return x_world * -sin(-yaw_rad) + y_world * cos(-yaw_rad);
}

const float vector_norm(float x, float y, float z) {
    return sqrtf(x*x + y*y + z*z);
}
