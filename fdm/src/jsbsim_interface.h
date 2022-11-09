#ifndef JSBSIM_INTERFACE_H
#define JSBSIM_INTERFACE_H

#include <string>
#include <condition_variable>
#include <mutex>

#include "nlohmann/json.hpp"

using json = nlohmann::json;

void jsbsim_init(const std::string& root_dir,
                 const std::string& script_path,
                 bool *continue_running,
                 json *sim_data,
                 std::mutex& sim_data_lock);
                 
void update_data(json *sim_data,
                 std::mutex& sim_data_lock);

#endif
