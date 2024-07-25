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

// #define NJ 3
// #define N_PAR 30
const std::string inertial_file = "../robots/robot/robot_inertial_REG.yaml";
const std::string saved_inertial_file = "../robots/robot/saved_robot_inertial_REG.yaml";

using namespace thunder_ns;
using std::cout;
using std::endl;

// bool use_gripper = false;

// Eigen::Matrix3d hat(const Eigen::Vector3d v);

int main(){

	thunder_robot robot;
	// robot.init(7);

	robot.load_inertial_REG(inertial_file);
	const int NJ = robot.get_numJoints();
	const int N_PAR = robot.get_numParams();

	// Eigen::VectorXd param_REG(N_PAR*NJ);
	// Eigen::VectorXd param_DYN(N_PAR*NJ);

	/* Matrix */
	
	// Eigen::MatrixXd<double, NJ, N_PAR> Yr;
	// Eigen::MatrixXd<double, NJ, NJ> myM;
	// Eigen::MatrixXd<double, NJ, NJ> myC;
	// Eigen::MatrixXd<double, NJ, 1> myG;
	// Eigen::MatrixXd<double, 6, NJ> myJac;
	// Eigen::MatrixXd<double, 6, NJ> myJacCM;
	// Eigen::MatrixXd<double, NJ, 1> tau_cmd_dyn;
	// Eigen::MatrixXd<double, NJ, 1> tau_cmd_reg;

	Eigen::MatrixXd Yr(NJ, N_PAR);
	Eigen::MatrixXd myM(NJ, NJ);
	Eigen::MatrixXd myC(NJ, NJ);
	Eigen::VectorXd myG(NJ);
	Eigen::MatrixXd myJac(6,NJ);
	Eigen::MatrixXd myJacCM(6,NJ);
	Eigen::VectorXd tau_cmd_dyn(NJ);
	Eigen::VectorXd tau_cmd_reg(NJ);

	Eigen::VectorXd param_REG(N_PAR);

	Eigen::VectorXd q(NJ), dq(NJ), dqr(NJ), ddqr(NJ);

	/* Test */
	q = q.setOnes();
	dq = dq.setOnes();
	dqr = dqr.setOnes();
	ddqr = ddqr.setOnes();

	// kinrobot.setArguments(q,dq);
	// regrobot.setArguments(q,dq,dqr,ddqr);
	// dynrobot.setArguments(q,dq,param_DYN);

	robot.setArguments(q, dq, dqr, ddqr);

	param_REG = robot.get_inertial_REG();
	cout<<"\nPar_REG\n"<<param_REG;

	Yr = robot.getReg();
	cout<<"\nYr\n"<<Yr;
	myM = robot.getMass();
	cout<<"\nM\n"<<myM;
	myC = robot.getCoriolis();
	cout<<"\nC\n"<<myC;
	myG = robot.getGravity();
	cout<<"\nG\n"<<myG;
	myJac = robot.getJac();
	cout<<"\nJac\n"<<myJac;

	tau_cmd_dyn = myM*ddqr + myC*dqr + myG;
	
	tau_cmd_reg = Yr*param_REG;

	cout<<"\ntau_cmd_dyn:\n"<<tau_cmd_dyn<<endl;
	cout<<"\ntau_cmd_reg:\n"<<tau_cmd_reg<<endl;
	cout<<"\ndiff tau_cmd:\n"<<tau_cmd_dyn-tau_cmd_reg<<endl;

	robot.save_inertial_REG(saved_inertial_file);

	// Eigen::VectorXd param_DYN(N_PAR);
	// Eigen::VectorXd param_DYN_afterLoad(N_PAR);
	// param_DYN = robot.get_inertial_DYN();
	// cout<<"\nparam_DYN:\n"<<param_DYN<<endl;
	// robot.save_inertial_DYN("file_inertial_dyn");
	// robot.load_inertial_DYN("file_inertial_dyn");
	// param_DYN_afterLoad = robot.get_inertial_DYN();
	// cout<<"\nparam_DYN error:\n"<<param_DYN-param_DYN_afterLoad<<endl;

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
