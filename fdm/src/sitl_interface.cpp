#include "sitl_interface.h"

#include <unistd.h>
#include <dlfcn.h>
#include <mutex>

#include "nlohmann/json.hpp"

using json = nlohmann::json;

void SITLInterface::sitl_init(sim_config_t& sim_config,
                              json *sim_data) {
    
    iter_num = 0;
    sitl_div = (sim_config.sitl_div > 0) ? sim_config.sitl_div : 1;
    
    void *lib = dlopen(sim_config.sitl_path.c_str(), RTLD_NOW);

    if (lib == nullptr) {
        printf("lib not found\n");
        return;
    }

    init_fcs =      (init_func)dlsym(lib, "init");
    data_to_fcs =   (data_to_fcs_func)dlsym(lib, "data_to_fcs");
    loop_fcs =      (loop_func)dlsym(lib, "loop");
    data_from_fcs = (data_from_fcs_func)dlsym(lib, "data_from_fcs");

    /**
     * Init FCS
     */
    init_fcs(sim_data);
}

void SITLInterface::handle_event(const std::string& event_name, json *sim_data) {
    if (event_name == "sim:before_iter") {
        
        if (iter_num % sitl_div == 0) {
            data_from_fcs(sim_data);
        }
    }

    if (event_name == "sim:after_iter") {
        if (iter_num % sitl_div == 0) {
            data_to_fcs(sim_data);
            loop_fcs();
        }

        iter_num++;
    }
}
