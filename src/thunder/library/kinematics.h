#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "robot.h"

namespace thunder_ns{

	int compute_chain(Robot& robot);
	int compute_jacobians(Robot& robot);
	int compute_kin_adv(Robot& robot);
	int compute_kinematics(Robot& robot, bool advanced=1);
	
}


#endif