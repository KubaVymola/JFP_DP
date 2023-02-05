#ifndef MIXER_H
#define MIXER_H

const float engine_mixer(const int engine_id,
                         const float throttle_cmd,
                         const float yaw_cmd,
                         const float pitch_cmd,
                         const float roll_cmd);
const float mixer_to_cmd(const float mixer_output);

#endif
