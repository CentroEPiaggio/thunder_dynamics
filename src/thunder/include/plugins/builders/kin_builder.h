#ifndef KIN_BUILDER_H
#define KIN_BUILDER_H

#include "plugin_interfaces.h"
#include "plugins/builders/common/kinematics.h"


namespace thunder_ns {

	class KinBuilder : public BaseBuilder {

		public:
			KinBuilder() : BaseBuilder("Kinematic Builder", "Build kinematics and differential kinematics expressions, like Jacobians and Transform matrixes T.") {}

			void create_std_joints(std::shared_ptr<Robot> robot){
				// --- Standard joint functions --- //
				SX q_joint;
				casadi::Slice rot(0, 3);      // [0,1,2] indexes
				SX Ti = SX::eye(4);

				// Prismatic classical joint
				Ti = SX::eye(4);
				q_joint = SX::sym("q_joint");
				Ti(2,3) = q_joint;
				if (!robot->add_function("T_JOINT_P", Ti, {}, "Template transformation of joint P", {q_joint})) {
					std::cerr << "Error adding joint function: T_JOINT_P" << std::endl;
				}

				// Rotoidal classical joint
				Ti = SX::eye(4);
				q_joint = SX::sym("q_joint");
				Ti(rot,rot) = R_z(q_joint);
				if (!robot->add_function("T_JOINT_R", Ti, {}, "Template transformation of joint R", {q_joint})) {
					std::cerr << "Error adding joint function: T_JOINT_R" << std::endl;
				}
			}

			void build(std::shared_ptr<Robot> robot) override {
				create_std_joints(robot);
				
				debug_log("Starting kinematic computations", VERB_INFO);
				compute_kinematics(*robot, 1);
				debug_log("Kinematics computed", VERB_INFO);
			}
			
	};

	
} // namespace thunder_ns

#endif // KIN_BUILDER_H