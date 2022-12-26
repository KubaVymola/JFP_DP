#ifndef SIMCONFIG_H
#define SIMCONFIG_H

struct sim_config_t {
    std::string root_dir;
    std::string script_path;
    std::string sitl_path;
    std::string serial_path;
    std::string craft_config_path;

    std::string log_output_def;
    std::string save_telemetry_path;

    std::vector<std::string> jsbsim_outputs;
    std::vector<std::string> command_line_properties;
    std::vector<double> command_line_property_values;

    bool realtime;   // realtime == !batch
    bool print_all_properties;
    bool override_sim_rate;
    bool use_hitl;
    bool rt_telem;

    double simulation_rate;
    double end_time;

    int sitl_div;

    uint16_t ws_port;
};

#endif
