#include "sitl_interface.h"

#include <unistd.h>
#include <dlfcn.h>
#include <mutex>

#include "nlohmann/json.hpp"
#include "tinyxml2.h"

using json = nlohmann::json;

void SITLInterface::sitl_init(sim_config_t& sim_config,
                              json *sim_data) {
    
    iter_num = 0;
    sitl_div = (sim_config.sitl_div > 0) ? sim_config.sitl_div : 1;
    
    void *lib = dlopen(sim_config.sitl_path.c_str(), RTLD_NOW);

    if (lib == nullptr) {
        printf("lib not found\n");
        return;
    }

    init_fcs =      (init_func)dlsym(lib, "init");
    data_to_fcs =   (data_to_fcs_func)dlsym(lib, "data_to_fcs");
    loop_fcs =      (loop_func)dlsym(lib, "loop");
    data_from_fcs = (data_from_fcs_func)dlsym(lib, "data_from_fcs");

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
    XMLElement *to_fcs_elem = fcs_elem->FirstChildElement("to_fcs");
    
    XMLElement *to_fcs_impl_prop_elem = to_fcs_elem->FirstChildElement("impl_property");
    for (; to_fcs_impl_prop_elem != nullptr; to_fcs_impl_prop_elem = to_fcs_impl_prop_elem->NextSiblingElement("impl_property")) {
        to_fcs_properties.push_back(to_fcs_impl_prop_elem->GetText());
    }

    XMLElement *to_fcs_prop_elem = to_fcs_elem->FirstChildElement("property");
    for (; to_fcs_prop_elem != nullptr; to_fcs_prop_elem = to_fcs_prop_elem->NextSiblingElement("property")) {
        to_fcs_properties.push_back(to_fcs_prop_elem->GetText());
    }
    
    /**
     * Get all properties that go from sim_data to FCS
     */
    XMLElement *from_fcs_elem = fcs_elem->FirstChildElement("from_fcs");
    XMLElement *from_fcs_prop_elem = from_fcs_elem->FirstChildElement("property");
    for (; from_fcs_prop_elem != nullptr; from_fcs_prop_elem = from_fcs_prop_elem->NextSiblingElement("property")) {
        from_fcs_properties.push_back(from_fcs_prop_elem->GetText());
    }
}

void SITLInterface::handle_event(const std::string& event_name, json *sim_data) {
    if (event_name == "sim:before_iter") {
        
        if (iter_num % sitl_div == 0) {
            double data[from_fcs_properties.size()];
            
            data_from_fcs(data);

            int i = 0;
            for (const std::string& from_fcs_property : from_fcs_properties) {
                (*sim_data)[from_fcs_property] = data[i++];
            }
        }
    }

    if (event_name == "sim:after_iter") {
        if (iter_num % sitl_div == 0) {
            double data[to_fcs_properties.size()];

            int i = 0;
            for (const std::string& to_fcs_property : to_fcs_properties) {
                data[i++] = sim_data->value<double>(to_fcs_property, 0.0);
            }
            
            data_to_fcs(data);
            loop_fcs();
        }

        iter_num++;
    }
}
