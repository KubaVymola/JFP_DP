#include "sitl_tunning.h"
#include "global_variables.h"

#ifdef SITL

extern "C" void init_override(std::map<std::string, float> &config) {
    
    /**
     * alt_sp_pid_*
     * alt_rate_pid_*
     * yaw_sp_pid_*
     * yaw_rate_pid_*
     * pos_pid_*
     * angle_pid_*
     * angle_rate_pid_*
     */

    /**
     * 00
     */
    if (config.count("angle_sp_tune")) {
        current_tunning = TUNE_ANGLE_SP;

        vertical_mode = VERTICAL_MODE_DIRECT;
        heading_mode = HEADING_MODE_NONE;
        lateral_mode = LATERAL_MODE_ANGLE;
    }

    /**
     * 01
     */
    if (config.count("pos_sp_tune")) {
        current_tunning = TUNE_POS_SP;
        vertical_mode = VERTICAL_MODE_DIRECT;
        heading_mode = HEADING_MODE_NONE;
        lateral_mode = LATERAL_MODE_FIXED_POS;
    }

    /**
     * 02
     */
    if (config.count("alt_sp_tune")) {
        current_tunning = TUNE_ALT_SP;

        vertical_mode = VERTICAL_MODE_SP;
        heading_mode = HEADING_MODE_NONE;
        lateral_mode = LATERAL_MODE_FIXED_POS;
    }

    /**
     * 03
     */
    if (config.count("yaw_sp_tune")) {
        current_tunning = TUNE_YAW_SP;

        vertical_mode = VERTICAL_MODE_SP;
        heading_mode = HEADING_MODE_SP;
        lateral_mode = LATERAL_MODE_FIXED_POS;
    }

    /**
     * 04
     */
    if (config.count("yaw_rate_tune")) {
        current_tunning = TUNE_YAW_RATE;

        vertical_mode = VERTICAL_MODE_SP;
        heading_mode = HEADING_MODE_RATE;
        lateral_mode = LATERAL_MODE_FIXED_POS;
    }

    /**
     * 05
     */
    if (config.count("alt_rate_tune")) {
        current_tunning = TUNE_ALT_RATE;

        vertical_mode = VERTICAL_MODE_RATE;
        heading_mode = HEADING_MODE_SP;
        lateral_mode = LATERAL_MODE_FIXED_POS;
    }


    if (config.count("alt_sp_pid_p") > 0) alt_sp_pid.k_p = config["alt_sp_pid_p"];
    if (config.count("alt_sp_pid_i") > 0) alt_sp_pid.k_i = config["alt_sp_pid_i"];
    if (config.count("alt_sp_pid_d") > 0) alt_sp_pid.k_d = config["alt_sp_pid_d"];

    if (config.count("alt_rate_pid_p") > 0) alt_rate_pid.k_p = config["alt_rate_pid_p"];
    if (config.count("alt_rate_pid_i") > 0) alt_rate_pid.k_i = config["alt_rate_pid_i"];
    if (config.count("alt_rate_pid_d") > 0) alt_rate_pid.k_d = config["alt_rate_pid_d"];

    if (config.count("yaw_sp_pid_p") > 0) yaw_sp_pid.k_p = config["yaw_sp_pid_p"];
    if (config.count("yaw_sp_pid_i") > 0) yaw_sp_pid.k_i = config["yaw_sp_pid_i"];
    if (config.count("yaw_sp_pid_d") > 0) yaw_sp_pid.k_d = config["yaw_sp_pid_d"];

    if (config.count("yaw_rate_pid_p") > 0) yaw_rate_pid.k_p = config["yaw_rate_pid_p"];
    if (config.count("yaw_rate_pid_i") > 0) yaw_rate_pid.k_i = config["yaw_rate_pid_i"];
    if (config.count("yaw_rate_pid_d") > 0) yaw_rate_pid.k_d = config["yaw_rate_pid_d"];

    if (config.count("pos_pid_p") > 0) x_body_pid.k_p = config["pos_pid_p"];
    if (config.count("pos_pid_p") > 0) y_body_pid.k_p = config["pos_pid_p"];
    if (config.count("pos_pid_i") > 0) x_body_pid.k_i = config["pos_pid_i"];
    if (config.count("pos_pid_i") > 0) y_body_pid.k_i = config["pos_pid_i"];
    if (config.count("pos_pid_d") > 0) x_body_pid.k_d = config["pos_pid_d"];
    if (config.count("pos_pid_d") > 0) y_body_pid.k_d = config["pos_pid_d"];

    if (config.count("angle_pid_p") > 0) roll_pid.k_p = config["angle_pid_p"];
    if (config.count("angle_pid_p") > 0) pitch_pid.k_p = config["angle_pid_p"];
    if (config.count("angle_pid_i") > 0) roll_pid.k_i = config["angle_pid_i"];
    if (config.count("angle_pid_i") > 0) pitch_pid.k_i = config["angle_pid_i"];
    if (config.count("angle_pid_d") > 0) roll_pid.k_d = config["angle_pid_d"];
    if (config.count("angle_pid_d") > 0) pitch_pid.k_d = config["angle_pid_d"];

    if (config.count("angle_rate_pid_p") > 0) roll_rate_pid.k_p = config["angle_rate_pid_p"];
    if (config.count("angle_rate_pid_p") > 0) pitch_rate_pid.k_p = config["angle_rate_pid_p"];
    if (config.count("angle_rate_pid_i") > 0) roll_rate_pid.k_i = config["angle_rate_pid_i"];
    if (config.count("angle_rate_pid_i") > 0) pitch_rate_pid.k_i = config["angle_rate_pid_i"];
    if (config.count("angle_rate_pid_d") > 0) roll_rate_pid.k_d = config["angle_rate_pid_d"];
    if (config.count("angle_rate_pid_d") > 0) pitch_rate_pid.k_d = config["angle_rate_pid_d"];
}

void tunning_config() {
    if (current_tunning < 0) return;

    throttle_channel = -1.0f;

    if (current_tunning == TUNE_ANGLE_SP) {
        if (time_s >  5.0f) throttle_channel =  0.1f;
        if (time_s > 15.0f) pitch_channel    = -1.0f;
    }

    if (current_tunning == TUNE_POS_SP) {
        if (time_s >  5.0f) throttle_channel = 0.55f;
        if (time_s > 15.0f) x_world_sp_m = 0.00005f * DEG_TO_GEO_M;
    }

    if (current_tunning == TUNE_ALT_SP) {
        if (time_s >  5.0f) alt_sp = 5.0f;
    }

    if (current_tunning == TUNE_YAW_SP) {
        if (time_s >  5.0f) alt_sp = 5.0f;
        if (time_s > 15.0f) yaw_sp_deg = 25.0f;
    }

    if (current_tunning == TUNE_YAW_RATE) {
        if (time_s >  5.0f) alt_sp = 5.0f;
        if (time_s > 15.0f) yaw_channel = 1.0f;
    }

    if (current_tunning == TUNE_ALT_RATE) {
        if (time_s >  5.0f) throttle_channel = 1.0f;
        if (time_s > 20.0f) throttle_channel = 0.7f; /* Ajdusted for deadband to result in 5 m/s rate */
    }
}

#endif // STIL