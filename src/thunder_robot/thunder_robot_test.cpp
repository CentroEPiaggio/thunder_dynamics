/* Test of classes and check validation of regressor compare standard equation of dynamic */

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>
#include <chrono>
// #include <yaml-cpp/yaml.h>

#include "thunder_robot.h"

// #define NJ 2
// #define N_PAR 10
constexpr std::string inertial_file = "../robots/robot/inertial_REG.yaml";

using namespace thunder_ns;
using std::cout;
using std::endl;

// bool use_gripper = false;

// Eigen::Matrix3d hat(const Eigen::Vector3d v);

int main(){

	thunder_robot robot;
	// robot.init(7);

	robot.loadInertialParam(inertial_file);
	int NJ = robot.get_numJoints();
	int N_PAR = robot.get_numParams();

	// Eigen::VectorXd param_REG(N_PAR*NJ);
	// Eigen::VectorXd param_DYN(N_PAR*NJ);

	/* Matrix */
	
	Eigen::Matrix<double, NJ, N_PAR> Yr;
	Eigen::Matrix<double, NJ, NJ> myM;
	Eigen::Matrix<double, NJ, NJ> myC;
	Eigen::Matrix<double, NJ, 1> myG;
	Eigen::Matrix<double, 6, NJ> myJac;
	Eigen::Matrix<double, 6, NJ> myJacCM;

	Eigen::Matrix<double, NJ, 1> tau_cmd_dyn;
	Eigen::Matrix<double, NJ, 1> tau_cmd_reg;

	Eigen::VectorXd q(NJ), dq(NJ), dqr(NJ), ddqr(NJ);

	/* Test */
	q = q.setOnes()*0.0;
	dq = dq.setOnes()*0.0;
	dqr = dqr.setOnes()*0.0;
	ddqr = ddqr.setOnes()*0.0;

	// kinrobot.setArguments(q,dq);
    // regrobot.setArguments(q,dq,dqr,ddqr);
    // dynrobot.setArguments(q,dq,param_DYN);

	robot.setArguments(q, dq, dqr, ddqr);

	Yr = regrobot.getRegressor();
	cout<<"\nYr\n"<<Yr;
	myM = dynrobot.getMass();
	cout<<"\nM\n"<<myM;
	myC = dynrobot.getCoriolis();
	cout<<"\nC\n"<<myC;
	myG = dynrobot.getGravity();
	cout<<"\nG\n"<<myG;
	myJac = dynrobot.getJacobian();
	cout<<"\nJac\n"<<myJac;

	tau_cmd_dyn = myM*ddqr + myC*dqr + myG;
	tau_cmd_reg = Yr*param_REG;

	cout<<"\ntau_cmd_dyn:\n"<<tau_cmd_dyn<<endl;
	cout<<"\ntau_cmd_reg:\n"<<tau_cmd_reg<<endl;
	cout<<"\nfunziona diff tau_cmd:\n"<<tau_cmd_dyn-tau_cmd_reg<<endl;

	return 0;
}

Eigen::Matrix3d hat(const Eigen::Vector3d v){
	Eigen::Matrix3d vhat;
			
	// chech
	if(v.size() != 3 ){
		std::cout<<"in function hat of class FrameOffset invalid dimension of input"<<std::endl;
	}
	
	vhat(0,0) = 0;
	vhat(0,1) = -v[2];
	vhat(0,2) = v[1];
	vhat(1,0) = v[2];
	vhat(1,1) = 0;
	vhat(1,2) = -v[0];
	vhat(2,0) = -v[1];
	vhat(2,1) = v[0];
	vhat(2,2) = 0;

	return vhat;
}
