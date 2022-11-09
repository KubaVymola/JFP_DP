#include "fcs_interface.h"

#include <unistd.h>
#include <dlfcn.h>
#include <mutex>

#include "nlohmann/json.hpp"

using json = nlohmann::json;

// typedef void (*setup)(void);

void fcs_init(const std::string& fcs_path,
              bool *continue_running,
              json *sim_data,
              std::mutex& sim_data_lock) {
    
    printf("starting to open\n");
    
    void *lib = dlopen(fcs_path.c_str(), RTLD_NOW);

    if (lib == nullptr) {
        printf("lib not found\n");
        return;
    }

    auto init = (void (*)(void))dlsym(lib, "init");
    auto data_to_fcs = (void (*)(json*, std::mutex&))dlsym(lib, "data_to_fcs");
    auto loop = (void (*)(void))dlsym(lib, "loop");
    auto data_from_fcs = (void (*)(json*, std::mutex&))dlsym(lib, "data_from_fcs");

    init();
    
    while (*continue_running) {
        data_to_fcs(sim_data, std::ref(sim_data_lock));
        loop();
        data_from_fcs(sim_data, std::ref(sim_data_lock));

        usleep(1'000'000.0 / 60);
    }
}
