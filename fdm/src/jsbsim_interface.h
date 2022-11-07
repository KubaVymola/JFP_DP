#ifndef JSBSIM_INTERFACE_H
#define JSBSIM_INTERFACE_H

#include <string>
#include <condition_variable>
#include <mutex>

#include "nlohmann/json.hpp"

using json = nlohmann::json;

// struct sim_data_t {
//     double altitude_m;
//     double latitude_deg;
//     double latitude_rad;
//     double longitude_deg;
//     double longitude_rad;

//     double euler_yaw;
//     double euler_pitch;
//     double euler_roll;
    
//     double local_q_1;
//     double local_q_2;
//     double local_q_3;
//     double local_q_4;

//     double ecef_q_1;
//     double ecef_q_2;
//     double ecef_q_3;
//     double ecef_q_4;

//     double cg_x_m;
//     double cg_y_m;
//     double cg_z_m;

//     double m0_rpm;
//     double m1_rpm;
//     double m2_rpm;
//     double m3_rpm;
// };

void jsbsim_init(const std::string& root_dir,
                 const std::string& script_path,
                 bool *continue_running,
                 json *sim_data,
                 std::mutex& sim_data_lock);
void jsbsim_run();

#endif
