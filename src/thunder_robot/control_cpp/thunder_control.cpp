#include "thunder_control.h"

#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>

template<class Robot> thunder_control<Robot>::thunder_control(Robot* robot){
    this->robot = robot;
}

template<class Robot> int thunder_control<Robot>::init(){
    // initialize control
    return 1;
}

template<class Robot> int thunder_control<Robot>::start(){
    // start control (first cycle)
    return 1;
}

template<class Robot> int thunder_control<Robot>::update(){
    // control law
    return 1;
}

template<class Robot> VectorXd thunder_control<Robot>::get_command(){
    // control law
    VectorXd tau = VectorXd::Zero(robot->get_numJoints());
}