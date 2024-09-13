#ifndef REGRESSORS_H
#define REGRESSORS_H

#include "robot.h"

namespace thunder_ns{

	casadi::SXVector createQ();
	casadi::SXVector createE();
	int compute_Yr(Robot& robot);
	int compute_reg_Dl(Robot& robot);

	int compute_regressors(Robot& robot, bool advanced=1);
	
}


#endif