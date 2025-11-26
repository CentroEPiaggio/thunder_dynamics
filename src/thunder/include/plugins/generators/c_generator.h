#ifndef C_GEN_H
#define C_GEN_H

#include <yaml-cpp/yaml.h>
#include <filesystem>

#include "../../plugin_interfaces.h"
#include "../../robot.h"

using std::cout;
using std::endl;


namespace thunder_ns {

	class CGenerator : public BaseGenerator {
		private:
			YAML::Node config_;

		public:
		
			CGenerator() : BaseGenerator("C Generator", "Generates a plain C library for Robot") {}
		
		
		int configure(const YAML::Node& config) override{
			config_ = config;
			debug_log("Configured", VERB_INFO);
			return 0;
		}
		
		
		void generate(const std::shared_ptr<Robot> robot) override{
			int nj = robot->get<int>("numJoints");
			// --- Generate merge code --- //

			std::string robot_name = robot->robotName;
			std::string robot_name_gen = robot_name + "_gen";
			std::string relativePath = robot_name + "_generatedFiles/";

			std::filesystem::path currentPath = std::filesystem::current_path();
			std::string absolutePath = currentPath / relativePath;

			// Create directory
			try {
				std::filesystem::create_directory(absolutePath);
			} catch(std::exception & e){
				std::cerr<<"Problem creating directory generatedFiles/"<<std::endl;
				return;
			}

			// Generate library
			robot->generate_library(absolutePath, robot_name_gen, false);

			debug_log("C library generated", VERB_INFO);

		}

	};

} // namespace thunder_ns

#endif // C_GEN_H
