#ifndef KIN_BUILDER_H
#define KIN_BUILDER_H

#include <yaml-cpp/yaml.h>

#include "../../plugin_interfaces.h"
#include "../../robot.h"
#include "../../kinematics.h"
#include "../../dynamics.h"
#include "../../regressors.h"
#include "../../utils.h"
#include "../../userDefined.h"

namespace thunder_ns {

	class DHKinBuilder : public BaseBuilder {

	private:
		// Check if the function f_name has to be ignored
		bool should_ignore(std::string f_name) {
			if (config_["functions"] && config_["functions"][f_name]) {
				if (config_["functions"][f_name]["ignore"].as<bool>(false)) {
					debug_log("Skipping function '" + f_name + "' (ignored in config)", VERB_INFO);
					return false;
				}
			}
		}

	public:
		DHKinBuilder() : BaseBuilder("DH Kinematic Builder", "Build kinematics and differential kinematics expressions, like Jacobians and Transform matrixes T, using DH.") {}

		void build(std::shared_ptr<Robot> robot) override {
			debug_log("Starting kinematic computations", VERB_INFO);
			compute_kinematics(*robot, 1);
			debug_log("Kinematics computed", VERB_INFO);
		}
	};

} // namespace thunder_ns

#endif // KIN_BUILDER_H