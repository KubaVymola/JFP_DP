#include "state_logger.h"

#include "FGFDMExec.h"
#include "tinyxml2.h"

void StateLogger::init(sim_config_t& sim_config) {
    /**
     * Parse log output def
     */
    using namespace tinyxml2;

    SGPath log_output_def_path = SGPath(sim_config.root_dir)/sim_config.log_output_def;
    SGPath log_output_out_path;

    
    XMLDocument log_def_doc(true, COLLAPSE_WHITESPACE);
    log_def_doc.LoadFile(log_output_def_path.c_str());

    XMLElement *root_elem = log_def_doc.FirstChildElement("output");
    log_output_out_path = SGPath(sim_config.root_dir)/std::string(root_elem->Attribute("path"));
    XMLElement *property_elem = root_elem->FirstChildElement("property");

    for (; property_elem != nullptr; property_elem = property_elem->NextSiblingElement("property")) {
        logged_properties.push_back(property_elem->GetText());
    }

    if (!sim_config.log_output_override.empty()) {
        log_output_out_path = SGPath(sim_config.root_dir)/sim_config.log_output_override;
    }

    printf("Data output: %s\n", log_output_def_path.c_str());
    
    outstream.open(log_output_out_path.str(), std::ios::out);
    
    /**
     * Print header
     */
    for (int i = 0; i < logged_properties.size(); ++i) {
        if (i > 0) outstream << ",";
        outstream << logged_properties[i];
    }

    outstream << std::endl;
}

void StateLogger::handle_event(const std::string& event_name, json* sim_data) {
    outstream.precision(18);
    
    if (event_name == EVENT_SIM_AFTER_ITER) {
        for (int i = 0; i < logged_properties.size(); ++i) {
            if (i > 0) outstream << ",";
            outstream << sim_data->value<double>(logged_properties[i], 0.0);
        }
        
        outstream << std::endl;
    }
}
