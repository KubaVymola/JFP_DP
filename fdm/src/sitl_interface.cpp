#include "sitl_interface.h"

#include <unistd.h>
#include <dlfcn.h>
#include <functional>

#include "sim_events.h"
#include "j_packets.h"

using json = nlohmann::json;
using namespace std::placeholders;

void SITLInterface::sitl_init(sim_config_t& sim_config,
                              json *sim_data,
                              CommandInterface *command_interface) {
    
    iter_num = 0;
    sitl_div = (sim_config.sitl_div > 0) ? sim_config.sitl_div : 1;

    this->command_interface = command_interface;

    void *lib = dlopen(sim_config.sitl_path.c_str(), RTLD_NOW);

    if (lib == nullptr) {
        printf("SITL lib file not found\n");
        exit(1);
    }

    init_fcs            = (init_func)dlsym(lib, "init");
    init_override       = (init_override_func)dlsym(lib, "init_override");
    data_from_jsbsim    = (data_from_jsbsim_func)dlsym(lib, "data_from_jsbsim");
    from_jsbsim_to_glob_state = (from_jsbsim_to_glob_state_func)dlsym(lib, "from_jsbsim_to_glob_state");
    control_loop        = (control_loop_func)dlsym(lib, "control_loop");
    after_loop          = (control_loop_func)dlsym(lib, "after_loop");
    data_to_jsbsim      = (data_to_jsbsim_func)dlsym(lib, "data_to_jsbsim");

    parse_xml_config(sim_config);


    if (!sim_config.save_telemetry_path.empty()) {
        save_telemetry_file.open(sim_config.save_telemetry_path, std::ios::out);

        for (int i = 0; i < telemetry_properties.size(); ++i) {
            if (i > 0) save_telemetry_file << ",";
            save_telemetry_file << telemetry_properties[i];
        }
        save_telemetry_file << std::endl;
    }

    /**
     * Init FCS
     */
    init_fcs(sim_data);

    init_override(sim_config.sitl_config_props);
}

void SITLInterface::parse_xml_config(sim_config_t& sim_config) {
    using namespace tinyxml2;

    XMLDocument craft_doc(true, COLLAPSE_WHITESPACE);
    craft_doc.LoadFile(sim_config.craft_config_path.c_str());

    XMLElement *root_elem = craft_doc.FirstChildElement("fdm_config");
    XMLElement *fcs_elem = root_elem->FirstChildElement("fcs_interface");

    /**
     * Get all properties that go from FCS to sim_data
     */
    XMLElement *to_jsb_elem = fcs_elem->FirstChildElement("to_jsbsim");
    XMLElement *to_jsb_prop_elem = to_jsb_elem->FirstChildElement("property");
    for (; to_jsb_prop_elem != nullptr; to_jsb_prop_elem = to_jsb_prop_elem->NextSiblingElement("property")) {
        to_jsbsim_properties.push_back(to_jsb_prop_elem->GetText());
    }
    
    /**
     * Get all properties that go from sim_data to FCS
     */
    XMLElement *from_jsb_elem = fcs_elem->FirstChildElement("from_jsbsim");

    XMLElement *from_jsb_prop_elem = from_jsb_elem->FirstChildElement("property");
    for (; from_jsb_prop_elem != nullptr; from_jsb_prop_elem = from_jsb_prop_elem->NextSiblingElement("property")) {
        from_jsbsim_properties.push_back(from_jsb_prop_elem->GetText());
    }


    /**
     * Get all telemetry properties
     * This is used to print the telemetry header and potentially propagate the telemetry to
     * sim_data (for visualization when using rt_telem or replay_telem)
     */
    XMLElement *telemetry_elem = fcs_elem->FirstChildElement("telemetry");
    XMLElement *telemetry_prop_elem = telemetry_elem->FirstChildElement("property");
    int i = 0;
    for (; telemetry_prop_elem != nullptr; telemetry_prop_elem = telemetry_prop_elem->NextSiblingElement("property")) {
        telemetry_properties.push_back(telemetry_prop_elem->GetText());
        i++;
    }
}

void SITLInterface::handle_event(const std::string& event_name, json *sim_data) {
    if (event_name == EVENT_SIM_BEFORE_ITER) {
        if (iter_num % sitl_div == 0) {
            while (1) {
                uint8_t buf[J_PACKET_SIZE];
                uint8_t len = data_to_jsbsim(buf);

                if (len == 0) break;

                auto j_packet_recv_cb_wrapper = std::bind(&SITLInterface::j_packet_recv_callback, this, sim_data, _1, _2, _3, _4);
                j_packet_recv(buf, len, j_packet_recv_cb_wrapper);
            }
        }
    }

    if (event_name == EVENT_SIM_AFTER_ITER) {
        if (iter_num % sitl_div == 0) {
            float data[from_jsbsim_properties.size()];

            int i = 0;
            for (const std::string& to_fcs_property : from_jsbsim_properties) {
                data[i++] = (float)sim_data->value<double>(to_fcs_property, 0.0);
            }
            
            j_packet_send(0x02, data, sizeof(data), sizeof(float), J_PACKET_SIZE, data_from_jsbsim);

            std::string command = command_interface->update_socket_and_read_commands();
            if (!command.empty()) {
                j_packet_send(0x00, (void *)command.c_str(), command.length(), 1, J_PACKET_SIZE, data_from_jsbsim);

                printf("CMD: Sending command %s\n", command.c_str());
            }

            from_jsbsim_to_glob_state();
            control_loop();
            after_loop();
        }

        iter_num++;
    }
}


void SITLInterface::j_packet_recv_callback(json *sim_data, uint8_t channel_number, uint8_t *current_data, uint16_t data_size, uint16_t data_offset) {
    /**
     * Receive and print command output
     */
    if (channel_number == 0x00) {
        printf("CMD: Received cmd output\n");
        command_interface->send_command_output((char *)(current_data + J_PACKET_HEADER_SIZE), data_size);
    }
    
    /**
     * Receive and log telemetry
     */
    if (channel_number == 0x01) {
        if (save_telemetry_file.is_open()) {
            save_telemetry_file.write((char *)(current_data + J_PACKET_HEADER_SIZE), data_size);
            save_telemetry_file.flush();
        }
    }

    /**
     * Receive HITL data
     */
    if (channel_number == 0x02) {
        float *new_data = (float *)(current_data + J_PACKET_HEADER_SIZE);
        int data_items_offset = data_offset / sizeof(float);
        
        for (int i = 0; i < data_size / sizeof(float); ++i) {
            (*sim_data)[to_jsbsim_properties[i + data_items_offset]] = new_data[i];
        }
    }

    /**
     * Receive and print debug output
     */
    if (channel_number == 0x03) {
        char debug_buf[data_size + 1];
        memcpy(debug_buf, current_data + J_PACKET_HEADER_SIZE, data_size);
        debug_buf[data_size] = '\0';
        printf("FCS: %s\n", debug_buf);
        
        // write(STDOUT_FILENO, current_data + J_PACKET_HEADER_SIZE, data_size);
    }
}
