#include "mixer.h"

#include <math.h>
#include <algorithm>
#include "defines.h"
#include "utils.h"

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

    const float idle_2 = IDLE_ARM_THRUST * IDLE_ARM_THRUST;

    float thrust_offset = 0.0f;
    if (min_output < idle_2) thrust_offset = idle_2 - min_output;
    if (max_output > 1.0f)   thrust_offset = 1.0f - max_output;

    // send_jpacket_info(0x00, "idl2 %.4f, th %.4f, yw %.4f, rl %.4f, pt %.4f, mx %.4f, mn %.4f, off %.4f\n", 128, idle_2, throttle_cmd, yaw_cmd,roll_cmd, pitch_cmd, max_output, min_output, thrust_offset);

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
    // return std::max(IDLE_ARM_THRUST, std::min(1.0f, mixer_output));

    return sqrtf(std::max(IDLE_ARM_THRUST * IDLE_ARM_THRUST, std::min(1.0f, mixer_output)));
}