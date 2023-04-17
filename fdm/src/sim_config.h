#ifndef SIMCONFIG_H
#define SIMCONFIG_H

#include <vector>
#include <string>
#include <map>

/**
 * Struct that holds CLI configuration from the user
*/
struct sim_config_t {
    std::string root_dir;
    std::string script_path;
    std::string sitl_path;
    std::string serial_path;
    std::string craft_config_path;

    std::string log_output_def;
    std::string log_output_override;
    std::string save_telemetry_path;

    std::vector<std::string> jsbsim_outputs;
    std::vector<std::string> command_line_properties;
    std::vector<double> command_line_property_values;

    std::map<std::string, float> sitl_config_props;

    bool realtime;   // realtime == !batch
    bool print_all_properties;
    bool override_sim_rate;
    bool use_hitl;
    bool rt_telem;

    double simulation_rate;
    double end_time;

    int sitl_div;

    uint16_t ws_port;
    uint16_t cmd_port;
};

#endif
