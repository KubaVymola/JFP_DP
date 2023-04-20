//==============================================================================
// miser.h
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef MIXER_H
#define MIXER_H

/**
 * Function to transform the control commands into motor commands. Performs clipping for thrust
 * higher than maximum and lower than minimum
 * 
 * @param engine_id Engine id from 1 to 4 to math the FC motor numbering
 */
const float engine_mixer(const int engine_id,
                         const float throttle_cmd,
                         const float yaw_cmd,
                         const float pitch_cmd,
                         const float roll_cmd);

/**
 * Clips the motor command to interval (IDLE_ARM_THRUST; 1.0)
*/
const float mixer_to_cmd(const float mixer_output);

#endif
