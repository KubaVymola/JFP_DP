//==============================================================================
// sitl_interface.h
//==============================================================================
//
// Source code of the fdm program (JSBSim wrapper) developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef SITLINTERFACE_H
#define SITLINTERFACE_H

#include <string>
#include <vector>
#include <map>
#include <fstream>

#include "nlohmann/json.hpp"
#include "tinyxml2.h"

#include "sim_config.h"
#include "i_sim_client.h"
#include "command_interface.h"

using json = nlohmann::json;


typedef void (*init_func)(json *);
typedef void (*init_override_func)(std::map<std::string, float>&);
typedef uint8_t (*data_from_jsbsim_func)(uint8_t *data, int len);
typedef void (*from_jsbsim_to_glob_state_func)(void);
typedef void (*control_loop_func)(void);
typedef void (*after_loop_func)(void);
typedef uint8_t (*data_to_jsbsim_func)(uint8_t *);

/**
 * SITL interface encapsulates loading and interfacing with the FCS compiled into a library file.
 * 
 * The library must implement the following list of functions: init, init_override,
 * data_from_jsbsim, from_jsbsim_to_glob_state, control_loop, after_loop, data_to_jsbsim
 * 
 * The functionality is very similar to the SerialInterface, which allows the SITLInterface to
 * perform advanced tasks, such as using the CommandInterface or saving telemetry.
 * 
 * The communication with the FCS library is achieved by data_from_jsbsim and data_to_jsbsim
 * functions. These functions relay buffers with the same packets as the SerialInterface's
 * USB communication.
 * 
 * These packets contain simulated data and commands in SITL simulation.
 * 
 * There is a telemetry stream present in the data that gets saved to a file.
 * 
 * There's also a command input/output stream that gets relayed between a socket server and the FCS.
 * 
 * There can be a debug stream that gets printed to a terminal with printf
*/
class SITLInterface : public ISimClient {
public:
    void sitl_init(sim_config_t& sim_config,
                  json *sim_data,
                  CommandInterface *command_interface);

    void parse_xml_config(sim_config_t& sim_config);

    virtual void handle_event(const std::string& event_name, json *sim_data);
private:
    void j_packet_recv_callback(json *sim_data, uint8_t channel_number, uint8_t *current_data, uint16_t data_size, uint16_t data_offset);

    unsigned int iter_num;
    unsigned int sitl_div;

    CommandInterface *command_interface;

    std::ofstream save_telemetry_file;

    std::vector<std::string> from_jsbsim_properties;
    std::vector<std::string> to_jsbsim_properties;
    std::vector<std::string> telemetry_properties;

    init_func init_fcs;
    init_override_func init_override;
    data_from_jsbsim_func data_from_jsbsim;
    from_jsbsim_to_glob_state_func from_jsbsim_to_glob_state;
    control_loop_func control_loop;
    after_loop_func after_loop;
    data_to_jsbsim_func data_to_jsbsim;
};

#endif
