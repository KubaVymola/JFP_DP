#include <stdio.h>
#include <unistd.h>
#include <mutex>

#include "nlohmann/json.hpp"

#include "pid.h"

using json = nlohmann::json;

json local_sim_data;
pid_state_t alt_pid;

extern "C" void init(void) {
    printf("Hello from FCS\n");

    pid_init(alt_pid, 0.15, 0.3, 0.1, 0.15, 0, 0.5);

    sleep(2);

    printf("Wake up\n");
}

extern "C" void data_to_fcs(json *sim_data, std::mutex& sim_data_lock) {
    {
        std::lock_guard<std::mutex> guard(sim_data_lock);
        local_sim_data.merge_patch(*sim_data);
    }

    // printf("received data\n");
}

extern "C" void data_from_fcs(json *sim_data, std::mutex& sim_data_lock) {
    {
        std::lock_guard<std::mutex> guard(sim_data_lock);
        sim_data->merge_patch(local_sim_data);
        // printf("sent  data, cmd[0] %f\n", (double)sim_data->value<double>("fcs/throttle-cmd-norm[0]", 0));
    }
    
}

extern "C" void loop(void) {
    // printf("loopin'\n");

    const double throttle_output = pid_update(alt_pid, 1.0, local_sim_data.value<double>("altitude_m", 1.0), 1.0 / 60);

    local_sim_data["fcs/throttle-cmd-norm[0]"] = throttle_output;
    local_sim_data["fcs/throttle-cmd-norm[1]"] = throttle_output;
    local_sim_data["fcs/throttle-cmd-norm[2]"] = throttle_output;
    local_sim_data["fcs/throttle-cmd-norm[3]"] = throttle_output;

    // std::cout << local_sim_data.dump(4) << std::endl;
    // printf("local data, cmd[0] %f\n", local_sim_data.value<double>("fcs/throttle-cmd-norm[0]", 0));
}

