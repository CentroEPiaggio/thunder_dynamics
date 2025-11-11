#ifndef THUNDERCONTROL_H
#define THUNDERCONTROL_H

#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>

using Eigen::VectorXd;

// namespace thunder_ns{

template<class Robot> class thunder_control{
    private:
        Robot* robot;

    public:
        thunder_control(Robot* robot);
        // ~thunder_control();

        int init();
        int start();
        int update();

        VectorXd get_action();
};

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

template<class Robot> VectorXd thunder_control<Robot>::get_action(){
    // control law
    VectorXd tau = VectorXd::Zero(robot->get_numJoints());
    return tau;
}
	
// }


#endif