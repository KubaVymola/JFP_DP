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
#include "tinyxml2.h"

#include "jsbsim_interface.h"
#include "realtime_loop.h"
#include "state_logger.h"
#include "sitl_interface.h"
#include "serial_interface.h"
#include "sim_config.h"
#include "sim_events.h"
#include "command_interface.h"

using json = nlohmann::json;

/**
 * ==== FORWARD DECLARATION ====
 */

void parse_cli_options(sim_config_t& sim_config, int argc, char **argv);
void check_valid_options(sim_config_t& sim_config);
void get_craft_config_path(sim_config_t& sim_config);
void print_help();

/**
 * ==== END FORWARD DECLARATION ====
 */

bool continue_running = true;

std::mutex sim_data_lock;
json sim_data = {};

void sighandler(int signum) {
    if (signum == SIGINT) {
        continue_running = false;
    }
}

void push_thread_func(websocket_server* server_instance) {
    while (continue_running) {
        json sim_data_cpy;
        
        {
            std::lock_guard<std::mutex> guard(sim_data_lock);
            sim_data_cpy.merge_patch(sim_data);
        }
        
        /**
         * Sending message outside of a sim_data_lock to avoid deadlock
         */

        // std::cout << sim_data_cpy.dump(4);

        server_instance->send_message(sim_data_cpy.dump());
        
        usleep(1'000'000.0 / 60);
    };

    printf("Push thread stopped\n");
}

void user_input_cb(std::string message) {
    {
        std::lock_guard<std::mutex> guard(sim_data_lock);
        auto tmp_data = json::parse(message);

        sim_data.merge_patch(tmp_data);
    }
}

int main(int argc, char **argv) {
    /**
     * Config
     */
    sim_config_t sim_config;
    sim_config.realtime = true;
    sim_config.end_time = 0.0;
    sim_config.override_sim_rate = false;

    sim_config.simulation_rate = 1./120.;
    sim_config.print_all_properties = false;
    sim_config.root_dir = ".";
    sim_config.ws_port = 0;
    sim_config.cmd_port = 0;

    sim_config.sitl_path = "";
    sim_config.sitl_div = 1;
    sim_config.serial_path = "";
    sim_config.use_hitl = false;

    sim_config.log_output_def = "";
    sim_config.save_telemetry_path = "";
    sim_config.rt_telem = false;

    parse_cli_options(sim_config, argc, argv);
    check_valid_options(sim_config);

    signal(SIGINT, sighandler);

    websocket_server server_instance;
    SimEvents sim_events;
    JSBSimInterface jsbsim_interface;
    RealtimeLoop realtime_loop;
    StateLogger state_logger;
    SITLInterface sitl_interface;
    SerialInterface serial_interface;
    CommandInterface command_interface;
    
    std::thread push_thread;
    std::thread server_thread;
    std::thread sim_thread;

    /**
     * Init
     */
    get_craft_config_path(sim_config);
    jsbsim_interface.init(sim_config, &sim_data);

    printf("Init done\n");

    if (!sim_config.log_output_def.empty()) {
        state_logger.init(sim_config);
        sim_events.register_client(EVENT_SIM_AFTER_ITER, &state_logger);
    }

    if (sim_config.cmd_port != 0) {
        command_interface.init(sim_config.cmd_port);
    }

    if (!sim_config.sitl_path.empty()) {
        sitl_interface.sitl_init(sim_config, &sim_data);
        sim_events.register_client(EVENT_SIM_BEFORE_ITER, &sitl_interface);
        sim_events.register_client(EVENT_SIM_AFTER_ITER,  &sitl_interface);
    }

    if (!sim_config.serial_path.empty()) {
        serial_interface.serial_init(sim_config, &sim_data, &command_interface);
        sim_events.register_client(EVENT_SIM_BEFORE_ITER, &serial_interface);
        sim_events.register_client(EVENT_SIM_AFTER_ITER,  &serial_interface);
    }

    printf("initialized\n");

    /**
     * Run
     */
    if (!sim_config.rt_telem) {
        sim_thread = std::thread(&JSBSimInterface::jsbsim_iter,
                                 &jsbsim_interface,
                                 std::ref(sim_config),
                                 &continue_running,
                                 &sim_data,
                                 std::ref(sim_data_lock),
                                 std::ref(sim_events));
    } else {
        sim_thread = std::thread(&RealtimeLoop::realtime_iter,
                                 &realtime_loop,
                                 &continue_running,
                                 &sim_data,
                                 std::ref(sim_data_lock),
                                 std::ref(sim_events));
    }

    printf("sim running\n");

    if (sim_config.ws_port != 0) {
        printf("Starting websocket server\n");
        
        server_instance.register_on_message_cb(user_input_cb);

        server_thread = std::thread(std::bind(&websocket_server::run, &server_instance, sim_config.ws_port));
        push_thread   = std::thread(push_thread_func, &server_instance);

        printf("ws server running\n");
    }

    while (continue_running) {
        usleep(250000);
    }

    /**
     * Stop
     */
    if (sim_config.ws_port != 0) {
        server_instance.stop();

        server_thread.join();
        push_thread.join();
    }

    sim_thread.join();

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

        } else if (keyword == "--cmd") {
            sim_config.cmd_port = atoi(value.c_str());

        } else if (keyword == "--sitl") {
            sim_config.sitl_path = value;

        } else if (keyword == "--sitl_div") {
            sim_config.sitl_div = atoi(value.c_str());
            
        } else if (keyword == "--serial") {
            sim_config.serial_path = value;
        
        } else if (keyword == "--hitl") {
            sim_config.use_hitl = true;
            
        } else if (keyword == "--save_telem") {
            sim_config.save_telemetry_path = value;
            
        } else if (keyword == "--rt_telem") {
            sim_config.rt_telem = true;
            
        } else if (keyword == "--jsbsim_output") {
            sim_config.jsbsim_outputs.push_back(value);
            
        } else if (keyword == "--set") {
            std::string prop_name = value.substr(0, value.find("="));
            std::string propValueString = value.substr(value.find("=") + 1);
            double prop_value = atof(propValueString.c_str());
            
            sim_config.command_line_properties.push_back(prop_name);
            sim_config.command_line_property_values.push_back(prop_value);
            
        } else if (keyword == "--sitl_config") {
            std::string prop_name = value.substr(0, value.find("="));
            std::string propValueString = value.substr(value.find("=") + 1);

            if (propValueString.empty()) {
                sim_config.sitl_config_props[prop_name] = 1.0f;

            } else {
                double prop_value = atof(propValueString.c_str());
                sim_config.sitl_config_props[prop_name] = prop_value;
            }

        } else if (keyword == "--output_path_override") {
            sim_config.log_output_override = value;
            
        } else if (keyword.substr(0, 2) != "--" && value.empty()) {
            if (position_arg == 0) {
                sim_config.script_path = keyword;
            
            } else if (position_arg == 1) {
                sim_config.log_output_def = keyword;
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

void check_valid_options(sim_config_t& sim_config) {
    if (!sim_config.serial_path.empty() && !sim_config.sitl_path.empty()) {
        printf("\nERR: Cannot use serial device when running as SITL\n");
        print_help();
        exit(1);
    }

    if (sim_config.serial_path.empty() && sim_config.use_hitl) {
        printf("\nERR: Cannot run as HITL without specifying the serial device\n");
        print_help();
        exit(1);
    }
}

void get_craft_config_path(sim_config_t& sim_config) {
    using namespace tinyxml2;

    SGPath script_path;
    SGPath aircraft_path;

    /**
     * Load main script to get aircraft name
     */
    script_path = SGPath(sim_config.root_dir)/sim_config.script_path;
    
    XMLDocument script_doc;
    script_doc.LoadFile(script_path.c_str());

    XMLElement *runscript_elem = script_doc.FirstChildElement("runscript");
    XMLElement *use_elem = runscript_elem->FirstChildElement("use");

    /**
     * Load aircraft to get properites 
     */
    std::string aircraft_name(use_elem->Attribute("aircraft"));
    
    aircraft_path = SGPath(sim_config.root_dir)/"aircraft"/aircraft_name/(aircraft_name + ".xml");

    sim_config.craft_config_path = aircraft_path.str();
}

void print_help() {
    printf(
        "\n"
        "Usage: fdm <script_file> [<output_file>] <options>\n"
        "\n"
        "options:\n"
        "  <script_file>            relative path to the main script\n"
        "  <output_file>            relative path to a file the specifies the output format\n"
        "  --sitl=<path>            relative path to the dynamic library with SITL FCS\n"
        "  --sitl_div=<value>       divide the simulation loop rate to get SITL FCS rate (integer only, default 1)\n"
        "  --serial=<path>          path to the serial port that sends channel packets\n"
        "  --hitl                   allow the serial device to run as HITL\n"
        "  --save_telem=<path>      specify path where the telemetry from the serial device will get saved\n"
        "  --rt_telem               do not run JSBSim, only log telemetry in real time\n"
        "  --ws=<port>              start a websocket server on a given port\n"
        "  --cmd=<port>             open port for command input and output\n"
        "  --root_dir=<dir>         root path for JSBSim assets (aircraft, script, engine; default '.')\n"
        "  --sim_rate=<hertz>       how many iterations will the simulation do in a second\n"
        "  --sim_end=<seconds>      how long the simulation will run (0 for endless, default is 60)\n"
        "  --jsbsim_output=<path>   path to JSBSim output definition (legacy)\n"
        "  --set=<property=value>   set property to given value\n"
        "  --sitl_config=<property[=value]>  set property for SITL tunning\n"
        "  --output_path_override=<path>     new path for output CSV (relative to root)\n"
        "  --realtime               the simulation will run in real time (default)\n"
        "  --batch                  the simulation will run as fast as possible\n"
        "  --print_props            print all properties before running\n"
        "  --version                print version and exit\n"
        "  --help                   print help and exit\n"
        "\n"
        "notes:\n"
        "  script, aircraft and engine paths are relative to the root directory (CWD by default)\n"
        "  output definitions and FCS paths are always relative to CWD"
        "\n"
    );
}

