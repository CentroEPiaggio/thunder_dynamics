#ifndef DH_LOADER_H
#define DH_LOADER_H

#include "../plugin_interfaces.h"
#include "../robot.h"
#include <yaml-cpp/yaml.h>

namespace thunder_ns {


    class DHLoader : public BaseLoader {
        private:
        YAML::Node config_;
        std::string robot_name;

        public:
        DHLoader() : BaseLoader("DH Loader", "Loads a robot using modified DH parameters.") {}

        int configure(const YAML::Node& config) override{
            config_ = config;
            robot_name = config_["robot_name"].as<std::string>();
            debug_log("Configured", VERB_INFO);
            return 0;
        }
        
        
        std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot_ptr) override{
            debug_log("Loading started", VERB_INFO);
            auto robot = std::make_shared<Robot>(Robot(config_));
            robot->robotName = robot_name;
            // --- load parameters --- //
            robot->load_conf_par_yml(config_);
            debug_log("Configuration Loaded", VERB_DEBUG);
            // - symbolic selectivity - //
            robot->init_symb_parameters();
            debug_log("Symbolic paramters ok, loading finished", VERB_INFO);
            return robot;
        }

    };

} // namespace thunder_ns

#endif // DH_LOADER_H
