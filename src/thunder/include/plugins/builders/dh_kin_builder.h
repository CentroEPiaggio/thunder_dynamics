#ifndef KIN_BUILDER_H
#define KIN_BUILDER_H

#include "plugin_interfaces.h"
#include "plugins/builders/common/kinematics.h"


namespace thunder_ns {

	class DHKinBuilder : public BaseBuilder {

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