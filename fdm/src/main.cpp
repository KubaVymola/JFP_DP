#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <math.h>
#include <mutex>
#include <condition_variable>
#include <sstream>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

#include "ws_server.h"
#include "jsbsim_interface.h"
#include "fcs_interface.h"
#include "sim_config.h"

using json = nlohmann::json;


/**
 * ==== FORWARD DECLARATION ====
 */

void parse_cli_options(sim_config_t& sim_config, int argc, char **argv);
void print_help();

/**
 * ==== END FORWARD DECLARATION ====
 */



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

int main(int argc, char **argv) {
    sim_config_t sim_config;
    sim_config.realtime = true;
    sim_config.end_time = 60;
    sim_config.override_sim_rate = false;

    sim_config.simulation_rate = 1./120.;
    sim_config.print_all_properties = false;
    sim_config.root_dir = ".";
    sim_config.ws_port = 0;

    // sim_config.frame_duration;
    
    parse_cli_options(sim_config, argc, argv);
    
    signal(SIGINT, sighandler);

    websocket_server server_instance;
    server_instance.register_on_message_cb(user_input_cb);

    std::thread push_thread;
    std::thread server_thread;
    std::thread fcs_thread;
    std::thread sim_thread;

    if (sim_config.ws_port != 0) {
        printf("Starting websocket server\n");
        
        server_thread = std::thread(std::bind(&websocket_server::run, &server_instance, sim_config.ws_port));

        push_thread   = std::thread(viz_thread,
                                    &server_instance);
    }

    sim_thread = std::thread(jsbsim_init,
                             std::ref(sim_config),
                             &continue_running,
                             &sim_data,
                             std::ref(sim_data_lock));

    if (!sim_config.fcs_path.empty()) {
        printf("Starting FCS thread\n");
        
        fcs_thread = std::thread(fcs_init,
                                std::ref(sim_config),
                                &continue_running,
                                &sim_data,
                                std::ref(sim_data_lock));
    }

    /**
     * TODO have app exit in Ctrl-C signal
     */
    while (continue_running) {
        sleep(3);
    }

    if (sim_config.ws_port != 0) {
        server_instance.stop();

        server_thread.join();
        push_thread.join();
    }

    sim_thread.join();

    if (!sim_config.fcs_path.empty()) {
        fcs_thread.join();
    }

    return 0;
}

void parse_cli_options(sim_config_t& sim_config, int argc, char **argv) {
    int position_arg = 0;
    
    for (int i = 1; i < argc; ++i) {
        std::string argument = std::string(argv[i]);
        std::string keyword(argument);
        std::string value("");
        std::string::size_type n = argument.find("=");

        if (n != std::string::npos && n > 0) {
            keyword = argument.substr(0, n);
            value = argument.substr(n + 1);
        }

        if (keyword == "--help") {
            print_help();
            exit(0);

        } else if (keyword == "--version") {
            printf("JFP version: 0.0.1\n");
            exit(0);
            
        } else if (keyword == "--print_props") {
            sim_config.print_all_properties = true;
            
        } else if (keyword == "--batch") {
            sim_config.realtime = false;
            
        } else if (keyword == "--realtime") {
            sim_config.realtime = true;
            
        } else if (keyword == "--sim_end") {
            sim_config.end_time = atof(value.c_str());
            
        } else if (keyword == "--sim_rate") {
            sim_config.simulation_rate = atof(value.c_str());
            sim_config.override_sim_rate = true;
            
        } else if (keyword == "--root_dir") {
            sim_config.root_dir = value;

        } else if (keyword == "--ws") {
            sim_config.ws_port = atoi(value.c_str());

        } else if (keyword == "--fcs") {
            sim_config.fcs_path = value;

        } else if (keyword == "--set") {
            std::string prop_name = value.substr(0, value.find("="));
            std::string propValueString = value.substr(value.find("=") + 1);
            double prop_value = atof(propValueString.c_str());
            
            sim_config.command_line_properties.push_back(prop_name);
            sim_config.command_line_property_values.push_back(prop_value);
            
        } else if (keyword.substr(0, 2) != "--" && value.empty()) {
            if (position_arg == 0) {
                sim_config.script_path = keyword;
            
            } else {
                sim_config.log_outputs.push_back(keyword);
            }

            ++position_arg;

        } else {
            printf("Unknown command\n");
            print_help();
            exit(1);
        }
    }

    if (position_arg < 1) {
        printf("Not enough positional arguments\n");
        print_help();
        exit(1);
    }
}

void print_help() {
    printf("\n");
    printf("Usage: fdm <script_file> <fcs_file> [<output_file> [<output_file>...]] <options>\n");
    printf("\n");
    printf("options:\n");
    printf("  <script_file>  relative path to the main script\n");
    printf("  <output_file>  relative path to a file the specifies the output format\n");
    printf("  --fcs=<path>  relative path to the dynamic library with FCS\n");
    printf("  --ws=<port>  start a websocket server on a given port\n");
    printf("  --root_dir=<dir>  root path for JSBSim assets (aircraft, script, engine; default '.')\n");
    printf("  --sim_rate=<hertz>  how many iterations will the simulation do in a second\n");
    printf("  --sim_end=<seconds>  how long the simulation will run (0 for endless, default is 60)\n");
    printf("  --set=<property=value>  set property to given value\n");
    printf("  --realtime  the simulation will run in real time (default)\n");
    printf("  --batch  the simulation will run as fast as possible\n");
    printf("  --print_props  print all properties before running\n");
    printf("  --version  print version and exit\n");
    printf("  --help  print help and exit\n");
    printf("\n");
    printf("notes:\n");
    printf("  script, aircraft and engine paths are relative to the root directory (CWD by default)\n");
    printf("  output definitions and fcs paths are always relative to CWD");
    printf("\n");
}

