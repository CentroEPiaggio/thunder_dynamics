#ifndef REGRESSORS_H
#define REGRESSORS_H

#include "Robot.h"

namespace thunder_ns{

	casadi::SXVector createQ();
	casadi::SXVector createE();

	int compute_regressors(Robot& robot, bool advanced=1);
	
}


#endif