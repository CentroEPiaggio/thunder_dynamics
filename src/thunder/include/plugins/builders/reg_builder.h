#ifndef REG_BUILDER_H
#define REG_BUILDER_H

#include "plugin_interfaces.h"
#include "plugins/builders/common/regressors.h"


namespace thunder_ns {

	class RegBuilder : public BaseBuilder {

		public:
			RegBuilder() : BaseBuilder("Regressors Builder", "Build the kinematic and dynamic regressors.") {}

			void init(std::shared_ptr<Robot> robot){
				// --- Basic Robot properties --- //
				int numJoints = robot->get<int>("numJoints");

				// - Parameters per link
				int STD_PAR_LINK = 10;
				if (robot->properties.count("STD_PAR_LINK")){
					STD_PAR_LINK = robot->get<int>("STD_PAR_LINK");
				} else {
					robot->add_property<int>("STD_PAR_LINK", STD_PAR_LINK, "int", "Standard number of dynamic parameters per link", true);
				}

				// --- Variables --- //
				// - Slotine regressor - //
				robot->add_variable("dqr", SX::sym("dqr",numJoints,1), vector<double>(numJoints,0), {1}, "Velocity reference", true);
				robot->add_variable("ddqr", SX::sym("ddqr",numJoints,1), vector<double>(numJoints,0), {1}, "Acceleration reference", true);
				// - Kinematic regressor - //
				robot->add_variable("w", SX::sym("w",6,1), vector<double>(6,0), {1}, "Wrench", true);
			}
		
			void build(std::shared_ptr<Robot> robot) override {
				init(robot);
				debug_log("Starting regressor computations", VERB_INFO);
				compute_regressors(*robot);
				debug_log("Regressors computed", VERB_INFO);
			}

	};


} // namespace thunder_ns

#endif // REG_BUILDER_H