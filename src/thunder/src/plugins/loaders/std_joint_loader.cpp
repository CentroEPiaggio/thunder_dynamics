#include "plugins/loaders/dh_loader.h"
#include "utils.h"


namespace thunder_ns {

	// --- Load function --- //
	std::shared_ptr<Robot> DHLoader::load(std::shared_ptr<Robot> robot){
		debug_log("Loading started", VERB_INFO);

		// ----- Parsing YAML File ----- //
		try {
			// Local properties for parsing
			int numJoints = 0;
			vector<string> jointsType;


			// --- Basic Robot properties --- //

			// - numJoints - //
			if (config_["num_joints"]) {
				numJoints = config_["num_joints"].as<int>();
				robot->add_property<int>("numJoints", numJoints, "int", "Number of joints", true);
			} else {
				throw std::runtime_error("No num_joints in yaml file.");
			}
			// - jointsType - //
			if (config_["type_joints"]) {
				jointsType = config_["type_joints"].as<vector<string>>();
				if (jointsType.size() != numJoints) {
					throw std::runtime_error("Mismatch between 'num_joints' and the size of 'type_joints' vector.");
				} else {
					robot->add_property<vector<string>>("jointsType", jointsType, "vector<string>", "Number of joints", true);
				}
			} else {
				throw std::runtime_error("No joints type specified in yaml file.");
			}
			

			// --- Variables --- //
			// - Normal joints - //
			robot->add_variable("q", SX::sym("q",numJoints,1), vector<double>(numJoints,0), {1}, "Configuration", true);
			robot->add_variable("dq", SX::sym("dq",numJoints,1), vector<double>(numJoints,0), {1}, "Velocity", true);

		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
			return robot; // Indicate failure
		}

		debug_log("Loading finished", VERB_INFO);
		return robot;
	}

} // namespace thunder_ns