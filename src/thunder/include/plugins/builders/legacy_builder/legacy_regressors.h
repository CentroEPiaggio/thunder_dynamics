#ifndef LEGACY_REGRESSORS_H
#define LEGACY_REGRESSORS_H

#include "robot.h"

namespace thunder_ns{
namespace legacy {

	casadi::SXVector createQ();
	casadi::SXVector createE();
	int compute_Yr(Robot& robot);
	int compute_reg_Dl(Robot& robot);
	int compute_reg_elastic(Robot& robot);
	int compute_reg_J(Robot& robot);

	int compute_regressors(Robot& robot, bool advanced=1);
	
}
}


#endif