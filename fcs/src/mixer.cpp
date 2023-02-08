#include "mixer.h"

#include <math.h>
#include <algorithm>
#include "defines.h"

/**
 * @param engine_id Engine id from 0 to 3
 */
const float engine_mixer(const int engine_id,
                         const float throttle_cmd,
                         const float yaw_cmd,
                         const float pitch_cmd,
                         const float roll_cmd) {
    float min_output = throttle_cmd - abs(yaw_cmd) - abs(pitch_cmd) - abs(roll_cmd);
    float max_output = throttle_cmd + abs(yaw_cmd) + abs(pitch_cmd) + abs(roll_cmd);

    // TODO test this
    float thrust_offset = 0.0f;
    if (min_output < IDLE_ARM_THRUST) thrust_offset = IDLE_ARM_THRUST - min_output;
    if (max_output > 1.0f)            thrust_offset = 1.0f - max_output;

    if (engine_id == 0) {
        return throttle_cmd - yaw_cmd - pitch_cmd - roll_cmd + thrust_offset;
    }

    if (engine_id == 1) {
        return throttle_cmd + yaw_cmd + pitch_cmd - roll_cmd + thrust_offset;
    }

    if (engine_id == 2) {
        return throttle_cmd + yaw_cmd - pitch_cmd + roll_cmd + thrust_offset;
    }

    if (engine_id == 3) {
        return throttle_cmd - yaw_cmd + pitch_cmd + roll_cmd + thrust_offset;
    }

    return 0;
}

const float mixer_to_cmd(const float mixer_output) {
    return std::max(IDLE_ARM_THRUST, std::min(1.0f, mixer_output));
}