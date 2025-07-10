#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "robot.h"

#include <urdf/model.h>
#include <urdf/link.h>    

namespace thunder_ns{

	casadi::SX R_x(const casadi::SX& angle);
	casadi::SX R_y(const casadi::SX& angle);
	casadi::SX R_z(const casadi::SX& angle);
	casadi::SX get_transform(casadi::SX frame);
	int compute_chain(Robot& robot);
	int compute_jacobians(Robot& robot);
	int compute_kin_adv(Robot& robot);
	int compute_kinematics(Robot& robot, bool advanced=1);
	

	void accumulateChainTransforms(std::shared_ptr<urdf::Link> link,
								const std::string& base,
								std::vector<std::shared_ptr<urdf::Link>>& chain);
}


#endif