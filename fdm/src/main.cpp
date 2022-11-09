#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <math.h>
#include <mutex>
#include <condition_variable>
#include <sstream>

#include "nlohmann/json.hpp"

#include "ws_server.h"
#include "jsbsim_interface.h"
#include "fcs_interface.h"

using json = nlohmann::json;

bool continue_running = true;

std::mutex sim_data_lock;
json sim_data;

void sighandler(int signum) {
    if (signum == SIGINT) {
        continue_running = false;
    }
}

void viz_thread(websocket_server* server_instance) {
    while (continue_running) {
        json sim_data_cpy;
        
        {
            std::lock_guard<std::mutex> guard(sim_data_lock);
            sim_data_cpy.merge_patch(sim_data);
        }
        
        /**
         * Sending message outside of a sim_data_lock to avoid deadlock
         * TODO find better solution
         */
        server_instance->send_message(sim_data_cpy.dump());
        
        usleep(1'000'000.0 / 60);
    };

    printf("Push thread stopped\n");
}

void user_input_cb(std::string message) {
    // printf("Callback with data %s\n", message.c_str());

    {
        std::lock_guard<std::mutex> guard(sim_data_lock);
        auto tmp_data = json::parse(message);

        sim_data.merge_patch(tmp_data);
    }
}

void print_help() {
    printf("Usage: fdm <root_dir> <script_path> <fcs_path>\n");
    printf("<root_dir>: relative path to the directory that containes "
           "aircraft, engine, scripts, etc. directories\n");
    printf("<script_path>: path to the intended script, relative to the <root_dir>\n");
    printf("<fcs_path>: relative path from the current directory to the dynamic library\n");
}

int main(int argc, char **argv) {
    if (argc < 4) {
        print_help();
        exit(1);
    }

    signal(SIGINT, sighandler);
    
    std::string root_dir(argv[1]);
    std::string script_path(argv[2]);
    std::string fcs_path(argv[3]);

    // printf("fcs path %s\n", fcs_path.c_str());
    // fcs_init(fcs_path, &continue_running, &sim_data, std::ref(sim_data_lock));
    // exit(0);

    websocket_server server_instance;
    server_instance.register_on_message_cb(user_input_cb);

    std::thread server_thread(std::bind(&websocket_server::run, &server_instance, 9002));

    std::thread push_thread(viz_thread,
                            &server_instance);
                            
    std::thread sim_thread(jsbsim_init,
                           root_dir,
                           script_path,
                           &continue_running,
                           &sim_data,
                           std::ref(sim_data_lock));
                           
    std::thread fcs_thread(fcs_init,
                           fcs_path,
                           &continue_running,
                           &sim_data,
                           std::ref(sim_data_lock));

    /**
     * TODO have app exit in Ctrl-C signal
     */
    while (continue_running) {
        sleep(3);
    }

    server_instance.stop();

    sim_thread.join();
    server_thread.join();
    push_thread.join();

    return 0;
}
