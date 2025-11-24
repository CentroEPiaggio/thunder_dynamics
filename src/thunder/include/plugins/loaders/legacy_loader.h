#ifndef LEGACY_LOADER_H
#define LEGACY_LOADER_H

#include <yaml-cpp/yaml.h>

#include "../../plugin_interfaces.h"
#include "../../robot.h"

namespace thunder_ns {

	class LegacyLoader : public BaseLoader {
		private:
		YAML::Node config_;
		std::string robot_name;

		public:
		LegacyLoader() : BaseLoader("Legacy Loader", "Creates a robot with the old yaml structure.") {}

		int configure(const YAML::Node& config) override{
			config_ = config;
			if (config_["robot_name"]) robot_name = config_["robot_name"].as<std::string>();
			// default robot_name?
			debug_log("Configured", VERB_INFO);
			return 0;
		}
		
		
		std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot_ptr) override{
			debug_log("Loading started", VERB_INFO);
			auto robot = std::make_shared<Robot>(Robot(config_));
			robot->robotName = robot_name;
			// --- load parameters --- //
			// robot->load_config(config_);
			debug_log("Configuration Loaded", VERB_DEBUG);
			// - symbolic selectivity - //
			// robot->init_symb_parameters();
			debug_log("Loading finished", VERB_INFO);
			return robot;
		}

	};

} // namespace thunder_ns

#endif // LEGACY_LOADER_H
