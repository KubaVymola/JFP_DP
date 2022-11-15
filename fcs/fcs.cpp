#include <stdio.h>
#include <unistd.h>
#include <mutex>
#include <algorithm>
#include <math.h>

#include "nlohmann/json.hpp"

#include "pid.h"

#define DEG_TO_GEO_M        6378000 * M_PI / 180;

using json = nlohmann::json;

json local_sim_data;

pid_state_t alt_pid;
pid_state_t yaw_pid;
pid_state_t x_body_pid;
pid_state_t y_body_pid;
pid_state_t roll_pid;
pid_state_t pitch_pid;

extern "C" void init(void);
extern "C" void data_to_fcs(json *sim_data, std::mutex& sim_data_lock);
extern "C" void data_from_fcs(json *sim_data, std::mutex& sim_data_lock);
extern "C" void loop(void);
const double engine_mixer(const int engine_id,
                    const double throttle_cmd,
                    const double yaw_cmd,
                    const double pitch_cmd,
                    const double roll_cmd);
const double mixer_to_cmd(const double mixer_output);
const double get_x_body(const double yaw_rad, const double x_world, const double y_world);
const double get_y_body(const double yaw_rad, const double x_world, const double y_world);

extern "C" void init(void) {
    printf("Hello from FCS\n");

    pid_init(alt_pid,    0.10,  0.05,  0.08,     0.15, -0.5, 0.5);
    pid_init(yaw_pid,    0.022, 0.088, 0.001325, 0.15, -0.2, 0.2);
    pid_init(x_body_pid, 3,     0,     6,    0.15, -5,   5);  // Output is used as roll  setpoint
    pid_init(y_body_pid, 3,     0,     6,    0.15, -5,   5);  // Output is used as pitch setpoint
    pid_init(roll_pid,   0.001, 0,     0.0005,   0.15, -0.1, 0.1);
    pid_init(pitch_pid,  0.001, 0,     0.0005,   0.15, -0.1, 0.1);

    sleep(3);
}

extern "C" void data_to_fcs(json *sim_data, std::mutex& sim_data_lock) {
    {
        std::lock_guard<std::mutex> guard(sim_data_lock);
        local_sim_data.merge_patch(*sim_data);
    }
}

extern "C" void data_from_fcs(json *sim_data, std::mutex& sim_data_lock) {
    {
        std::lock_guard<std::mutex> guard(sim_data_lock);
        sim_data->merge_patch(local_sim_data);
    }
    
}

extern "C" void loop(void) {
    const double deltaT = 1.0 / 60;
    const double time = local_sim_data.value<double>("time_sec", 0);
    
    /**
     * ==== Altitude ====
     */
    const double alt_sp = 1.0;
    const double alt_measure = local_sim_data.value<double>("altitude_m", alt_sp);

    /**
     * ==== Yaw ====
     */
    double yaw_sp = 0;
    if (alt_measure > 0.5) yaw_sp = 60;
    double yaw_measure = local_sim_data.value<double>("attitude/heading-true-deg", yaw_sp);
    if (yaw_sp - yaw_measure >  180) yaw_measure += 360;
    if (yaw_sp - yaw_measure < -180) yaw_measure -= 360;
    
    /**
     * Position: world_x -> east -> longitude
     *           world_y -> north -> latitude
    */
    const double yaw_rad = local_sim_data.value<double>("attitude/heading-true-rad", 0);

    const double x_world_measure_m = local_sim_data.value<double>("longitude_deg", 0) * DEG_TO_GEO_M;
    const double y_world_measure_m = local_sim_data.value<double>("latitude_deg", 0) * DEG_TO_GEO_M;
    double x_world_sp_m = 0;
    double y_world_sp_m = 0;

    if (time > 7) {
        x_world_sp_m = 1;
        y_world_sp_m = 2.5;
    }

    const double x_body_measure_m = get_x_body(-yaw_rad, x_world_measure_m, y_world_measure_m);
    const double y_body_measure_m = get_y_body(-yaw_rad, x_world_measure_m, y_world_measure_m);
    const double x_body_sp_m = get_x_body(-yaw_rad, x_world_sp_m, y_world_sp_m);
    const double y_body_sp_m = get_y_body(-yaw_rad, x_world_sp_m, y_world_sp_m);

    const double roll_measure = local_sim_data.value<double>("euler_roll", 0);
    const double pitch_measure = local_sim_data.value<double>("euler_pitch", 0);

    printf("pos measure: %f %f\n", x_body_measure_m, y_body_measure_m);
    
    /**
     * ==== PIDs ====
     */
    const double alt_pid_out = 0.255 + pid_update(alt_pid, alt_sp, alt_measure, deltaT);
    const double yaw_pid_out = pid_update(yaw_pid, yaw_sp, yaw_measure, deltaT);

    const double x_body_pid_out = pid_update(x_body_pid, x_body_sp_m, x_body_measure_m, deltaT);
    const double y_body_pid_out = pid_update(y_body_pid, y_body_sp_m, y_body_measure_m, deltaT);

    const double roll_pid_out  = pid_update(roll_pid,   x_body_pid_out, roll_measure,  deltaT);
    const double pitch_pid_out = pid_update(pitch_pid, -y_body_pid_out, pitch_measure, deltaT);

    /**
     * ==== Output ====
     */
    local_sim_data["fcs/throttle-cmd-norm[0]"]
        = mixer_to_cmd(engine_mixer(0, alt_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));;
    local_sim_data["fcs/throttle-cmd-norm[1]"]
        = mixer_to_cmd(engine_mixer(1, alt_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));;
    local_sim_data["fcs/throttle-cmd-norm[2]"]
        = mixer_to_cmd(engine_mixer(2, alt_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));;
    local_sim_data["fcs/throttle-cmd-norm[3]"]
        = mixer_to_cmd(engine_mixer(3, alt_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));;
}

/**
 * @param engine_id Engine id from 1 to 4
*/
const double engine_mixer(const int engine_id,
                    const double throttle_cmd,
                    const double yaw_cmd,
                    const double pitch_cmd,
                    const double roll_cmd) {
    if (engine_id == 0) {
        return throttle_cmd + yaw_cmd + pitch_cmd - roll_cmd;
    }

    if (engine_id == 1) {
        return throttle_cmd + yaw_cmd - pitch_cmd + roll_cmd;
    }

    if (engine_id == 2) {
        return throttle_cmd - yaw_cmd + pitch_cmd + roll_cmd;
    }

    if (engine_id == 3) {
        return throttle_cmd - yaw_cmd - pitch_cmd - roll_cmd;
    }
    
    return 0;
}

const double mixer_to_cmd(const double mixer_output) {
    return std::max(0.0, std::min(1.0, mixer_output));
}

const double get_x_body(const double yaw_rad, const double x_world, const double y_world) {
    return x_world * cos(yaw_rad) + y_world * sin(yaw_rad);
}

const double get_y_body(const double yaw_rad, const double x_world, const double y_world) {
    return x_world * -sin(yaw_rad) + y_world * cos(yaw_rad);
}