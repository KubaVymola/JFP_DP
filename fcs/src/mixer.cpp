//==============================================================================
// mixer.cpp
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#include "mixer.h"

#include <math.h>
#include <algorithm>
#include "defines.h"
#include "utils.h"

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

    if (engine_id == 1) {
        return throttle_cmd - yaw_cmd - pitch_cmd - roll_cmd + thrust_offset;
    }

    if (engine_id == 2) {
        return throttle_cmd + yaw_cmd + pitch_cmd - roll_cmd + thrust_offset;
    }

    if (engine_id == 3) {
        return throttle_cmd + yaw_cmd - pitch_cmd + roll_cmd + thrust_offset;
    }

    if (engine_id == 4) {
        return throttle_cmd - yaw_cmd + pitch_cmd + roll_cmd + thrust_offset;
    }

    return 0;
}

const float mixer_to_cmd(const float mixer_output) {
    return sqrtf(std::max(IDLE_ARM_THRUST * IDLE_ARM_THRUST, std::min(1.0f, mixer_output)));
}