#ifndef KIN_BUILDER_H
#define KIN_BUILDER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class KinBuilder : public BaseBuilder {

		public:
			KinBuilder() : BaseBuilder("Kinematic Builder", "Build kinematics and differential kinematics expressions, like Jacobians and Transform matrixes T.") {}

			
			SX apply_joint(std::shared_ptr<Robot> robot, const SX& frame, string joint_type, const SX& qi, const SX& axis);
			int compute_chain(std::shared_ptr<Robot> robot);
			int compute_jacobians(std::shared_ptr<Robot> robot);
			int compute_kin_adv(std::shared_ptr<Robot> robot);

			int create_std_joints(std::shared_ptr<Robot> robot);

			void build(std::shared_ptr<Robot> robot) override;
			
	};

	
} // namespace thunder_ns

#endif // KIN_BUILDER_H