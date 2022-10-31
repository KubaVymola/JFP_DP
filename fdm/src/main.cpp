#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <math.h>
#include <mutex>
#include <condition_variable>
#include <sstream>

#include "ws_server.h"
#include "jsbsim_interface.h"
#include "nlohmann/json.hpp"

bool continue_running = true;

using json = nlohmann::json;

void sighandler(int signum) {
    if (signum == SIGINT) {
        continue_running = false;
    }
}

void message_thread(websocket_server* server_instance,
                    sim_data_t* sim_data,
                    std::mutex& sim_data_lock) {
    while (continue_running) {
        {
            std::lock_guard<std::mutex> guard(sim_data_lock);

            json j;
            j["altitude_m"] = sim_data->altitude_m;
            j["latitude_deg"] = sim_data->latitude_deg;
            j["latitude_rad"] = sim_data->latitude_rad;
            j["longitude_deg"] = sim_data->longitude_deg;
            j["longitude_rad"] = sim_data->longitude_rad;

            j["euler_yaw"] = sim_data->euler_yaw;
            j["euler_pitch"] = sim_data->euler_pitch;
            j["euler_roll"] = sim_data->euler_roll;
            
            j["local_q_1"] = sim_data->local_q_1;
            j["local_q_2"] = sim_data->local_q_2;
            j["local_q_3"] = sim_data->local_q_3;
            j["local_q_4"] = sim_data->local_q_4;

            j["ecef_q_1"] = sim_data->ecef_q_1;
            j["ecef_q_2"] = sim_data->ecef_q_2;
            j["ecef_q_3"] = sim_data->ecef_q_3;
            j["ecef_q_4"] = sim_data->ecef_q_4;

            j["cg_x_m"] = sim_data->cg_x_m;
            j["cg_y_m"] = sim_data->cg_y_m;
            j["cg_z_m"] = sim_data->cg_z_m;
            
            server_instance->send_message(j.dump());
        }
        
        usleep(1'000'000.0 / 60);
        // sleep(1);
    };

    printf("Push thread stopped\n");
}

void print_help() {
    printf("Usage: fdm <root_dir> <script_path>\n");
    printf("<root_dir>: relative path to the directory that containes "
           "aircraft, engine, scripts, etc. directories\n");
    printf("<script_path>: path to the intended script, relative to the <root_dir>\n");
}

int main(int argc, char **argv) {
    if (argc < 3) {
        print_help();
        exit(1);
    }

    signal(SIGINT, sighandler);
    
    std::string root_dir(argv[1]);
    std::string script_path(argv[2]);

    websocket_server server_instance;

    std::mutex sim_data_lock;
    sim_data_t sim_data;

    std::thread server_thread(std::bind(&websocket_server::run, &server_instance, 9002));
    std::thread push_thread(message_thread,
                            &server_instance,
                            &sim_data,
                            std::ref(sim_data_lock));
    std::thread sim_thread(jsbsim_init,
                           root_dir,
                           script_path,
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
