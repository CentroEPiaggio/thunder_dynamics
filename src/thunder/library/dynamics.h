#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "robot.h"

namespace thunder_ns{

    std::tuple<casadi::SXVector,casadi::SXVector, casadi::SXVector> createInertialParameters(casadi::SX);
    casadi::SX dq_select(const casadi::SX& dq_);
    casadi::SX stdCmatrix(const casadi::SX& B, const casadi::SX& q_, const casadi::SX& dq_, const casadi::SX& dq_sel_);
    std::tuple<casadi::SXVector,casadi::SXVector> DHJacCM(Robot& robot);
	
	// int compute_Mass(Robot& robot);
	// int compute_Coriolis(Robot& robot);
    // int compute_Gravity(Robot& robot);
	// int compute_dynamics(Robot& robot, bool advanced = false);
    casadi::SXVector compute_dynamics(Robot& robot);
	
}


#endif