#ifndef FCSINTERFACE_H
#define FCSINTERFACE_H

#include <string>
#include <mutex>

#include "nlohmann/json.hpp"

using json = nlohmann::json;

void fcs_init(const std::string& fcs_path,
              bool *continue_running,
              json *sim_data,
              std::mutex& sim_data_lock);

#endif
