#include "sitl_interface.h"

#include <unistd.h>
#include <dlfcn.h>

using json = nlohmann::json;

void SITLInterface::sitl_init(sim_config_t& sim_config,
                              json *sim_data) {
    
    iter_num = 0;
    sitl_div = (sim_config.sitl_div > 0) ? sim_config.sitl_div : 1;

    void *lib = dlopen(sim_config.sitl_path.c_str(), RTLD_NOW);

    if (lib == nullptr) {
        printf("SITL lib file not found\n");
        exit(1);
    }

    init_fcs =      (init_func)dlsym(lib, "init");
    data_from_jsbsim =   (data_to_fcs_func)dlsym(lib, "data_from_jsbsim");
    loop_fcs =      (loop_func)dlsym(lib, "loop");
    data_to_jsbsim = (data_from_fcs_func)dlsym(lib, "data_to_jsbsim");

    parse_xml_config(sim_config);

    /**
     * Init FCS
     */
    init_fcs(sim_data);
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
}

void SITLInterface::handle_event(const std::string& event_name, json *sim_data) {
    if (event_name == "sim:before_iter") {
        
        if (iter_num % sitl_div == 0) {
            float data[to_jsbsim_properties.size()];
            
            data_to_jsbsim(data);

            int i = 0;
            for (const std::string& from_fcs_property : to_jsbsim_properties) {
                (*sim_data)[from_fcs_property] = (double)data[i++];
            }
        }
    }

    if (event_name == "sim:after_iter") {
        if (iter_num % sitl_div == 0) {
            float data[from_jsbsim_properties.size()];

            int i = 0;
            for (const std::string& to_fcs_property : from_jsbsim_properties) {
                data[i++] = (float)sim_data->value<double>(to_fcs_property, 0.0);
            }
            
            data_from_jsbsim(data);
            loop_fcs();
        }

        iter_num++;
    }
}
