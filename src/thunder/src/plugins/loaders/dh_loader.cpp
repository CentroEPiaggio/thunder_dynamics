#include "plugins/loaders/dh_loader.h"


namespace thunder_ns {

	std::shared_ptr<Robot> DHLoader::load(std::shared_ptr<Robot> robot_ptr) {
        debug_log("Loading started", VERB_INFO);
        auto robot = robot_ptr;
        robot_name = config_["robot_name"].as<std::string>();
        robot->robotName = robot_name;
        // --- load parameters --- //
        // robot->load_config(config_);
        debug_log("Configuration Loaded", VERB_DEBUG);
        // - symbolic selectivity - //
        // robot->init_symb_parameters();
        debug_log("Kinematic loading finished", VERB_INFO);
        return robot;
    }

} // namespace thunder_ns