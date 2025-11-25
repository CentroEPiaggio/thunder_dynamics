#ifndef LEGACY_BUILDER_H
#define LEGACY_BUILDER_H

#include <yaml-cpp/yaml.h>

#include "../../plugin_interfaces.h"
#include "../../robot.h"
#include "../../kinematics.h"
#include "../../dynamics.h"
#include "../../regressors.h"
#include "../../utils.h"
#include "../../userDefined.h"

namespace thunder_ns {

	class LegacyBuilder : public BaseBuilder {
		public:
		LegacyBuilder() : BaseBuilder("Legacy Builder", "Build everythink like the old times.") {}
		
		int configure(const YAML::Node& config) override{
			debug_log("Configured", VERB_INFO);
			return 0;
		}

		void execute(std::shared_ptr<Robot> robot) override{
			debug_log("Starting kinematic computations", VERB_INFO);
			compute_kinematics(*robot, 1);
			debug_log("Kinematics computed", VERB_INFO);
			compute_dynamics(*robot, 1);
			debug_log("Dynamics computed", VERB_INFO);
			compute_regressors(*robot);
			debug_log("Regressors computed", VERB_INFO);
			compute_userDefined(*robot);
			debug_log("User functions computed", VERB_INFO);
		}

	};

} // namespace thunder_ns

#endif // LEGACY_BUILDER_H