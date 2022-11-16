#include "fcs_interface.h"

#include <unistd.h>
#include <dlfcn.h>
#include <mutex>

#include "nlohmann/json.hpp"

using json = nlohmann::json;

void fcs_init(sim_config_t& sim_config,
              bool *continue_running,
              json *sim_data,
              std::mutex& sim_data_lock) {
    
    void *lib = dlopen(sim_config.fcs_path.c_str(), RTLD_NOW);

    if (lib == nullptr) {
        printf("lib not found\n");
        return;
    }

    init_func init =                   (init_func)dlsym(lib, "init");
    data_to_fcs_func data_to_fcs =     (data_to_fcs_func)dlsym(lib, "data_to_fcs");
    loop_func loop =                   (loop_func)dlsym(lib, "loop");
    data_from_fcs_func data_from_fcs = (data_from_fcs_func)dlsym(lib, "data_from_fcs");

    init();
    
    while (*continue_running) {
        data_to_fcs(sim_data, std::ref(sim_data_lock));
        loop();
        data_from_fcs(sim_data, std::ref(sim_data_lock));

        usleep(1'000'000.0 / 60);
    }
}
