#ifndef DH_LOADER_H
#define DH_LOADER_H

#include <yaml-cpp/yaml.h>

#include "../../plugin_interfaces.h"
#include "../../robot.h"

namespace thunder_ns {

	class DHLoader : public BaseLoader {
		private:
		std::string robot_name;

		public:
		DHLoader() : BaseLoader("DH Loader", "Creates a robot structure using DH parameters (modified convention).") {}
		
		std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot_ptr) override{
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

	};

} // namespace thunder_ns

#endif // DH_LOADER_H
