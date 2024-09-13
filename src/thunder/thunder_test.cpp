/* Test of classes and check validation of regressor compare standard equation of dynamic */

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>
#include <chrono>
#include <yaml-cpp/yaml.h>

#include "library/robot.h"
#include "library/kinematics.h"
#include "library/dynamics.h"
#include "library/regressors.h"

using namespace thunder_ns;
using std::cout;
using std::endl;

Eigen::Matrix3d hat(const Eigen::Vector3d v);
// extern int compute_kinematics(Robot robot);

int main(){

	std::string config_file = "../robots/RRR/RRR.yaml";

	//-------------------Obtain param_REG for regressor------------------------//
	// Eigen::Matrix3d I0,IG;
	// Eigen::Vector3d dOG;
	// double m;
	
	// for(int i=0;i<NJ;i++){

	// 	m = param_DYN[i*PARAM];
	// 	dOG << param_DYN[i*PARAM+1], param_DYN[i*PARAM+2],param_DYN[i*PARAM+3];
		
	// 	IG(0, 0) = param_DYN[i*PARAM+4];
	// 	IG(0, 1) = param_DYN[i*PARAM+5];
	// 	IG(0, 2) = param_DYN[i*PARAM+6];
	// 	IG(1, 0) = IG(0, 1);
	// 	IG(1, 1) = param_DYN[i*PARAM+7];
	// 	IG(1, 2) = param_DYN[i*PARAM+8];
	// 	IG(2, 0) = IG(0, 2);
	// 	IG(2, 1) = IG(1, 2);
	// 	IG(2, 2) = param_DYN[i*PARAM+9];

	// 	I0 = IG + m * hat(dOG).transpose() * hat(dOG);

	// 	param_REG[i*PARAM] = m;
	// 	param_REG[i*PARAM+1] = m*dOG[0];
	// 	param_REG[i*PARAM+2] = m*dOG[1];
	// 	param_REG[i*PARAM+3] = m*dOG[2];
	// 	param_REG[i*PARAM+4] = I0(0,0);
	// 	param_REG[i*PARAM+5] = I0(0,1);
	// 	param_REG[i*PARAM+6] = I0(0,2);
	// 	param_REG[i*PARAM+7] = I0(1,1);
	// 	param_REG[i*PARAM+8] = I0(1,2);
	// 	param_REG[i*PARAM+9] = I0(2,2);
	// }
	// std::cout<<"\nparam REG \n"<<param_REG<<std::endl;


	// ---------------------------------------------------------------------------------//
	// ------------------------------TEST CLASSES---------------------------------------//
	// ---------------------------------------------------------------------------------//

	Robot robot = robot_from_file(config_file, 1);
	// computation are inside "robot_from_file()"
	// compute_kinematics(robot);
	// compute_dynamics(robot);
	// compute_regressors(robot);
	// cout<<"robot created"<<endl;

	int NJ = robot.get_numJoints();
	int N_PARAM_DYN = robot.get_numParDYN();
	int N_PARAM_REG = robot.get_numParREG();
	// int N_PARAM_ELA = robot.get_numParELA();

	/* Matrices declaration*/
	Eigen::VectorXd param_DYN(N_PARAM_DYN);
	Eigen::VectorXd param_REG(N_PARAM_REG);
	// Eigen::VectorXd param_ELA(N_PARAM_ELA);
	Eigen::MatrixXd Yr(NJ, N_PARAM_DYN);
	Eigen::MatrixXd reg_M(NJ, N_PARAM_DYN);
	Eigen::MatrixXd reg_C(NJ, N_PARAM_DYN);
	Eigen::MatrixXd reg_G(NJ, N_PARAM_DYN);
	Eigen::MatrixXd M(NJ, NJ);
	Eigen::MatrixXd C(NJ, NJ);
	Eigen::MatrixXd C_std(NJ, NJ);
	Eigen::MatrixXd G(NJ, 1);
	Eigen::MatrixXd D(NJ, 1);
	Eigen::MatrixXd Kin(4, 4);
	Eigen::MatrixXd Jac(6, NJ);

	Eigen::MatrixXd tau_cmd_dyn(NJ, 1);
	Eigen::MatrixXd tau_cmd_reg(NJ, 1);
	Eigen::MatrixXd tau_cmd_regMat(NJ, 1);

	// arguments
	Eigen::VectorXd q(NJ), dq(NJ), dqr(NJ), ddqr(NJ);

	// get quantities
	param_REG = robot.get_par_REG();
	param_DYN = robot.get_par_DYN();
	cout<<"par_DYN:"<<endl<<param_DYN<<endl<<endl;
	cout<<"par_REG:"<<endl<<param_REG<<endl<<endl;

	/* Test */
	q.setOnes();// = Eigen::Vector<double,NJ>::Random();
	dq.setOnes();// = Eigen::Vector<double,NJ>::Random();
	dqr.setOnes();// = Eigen::Vector<double,NJ>::Random();
	ddqr.setOnes();// = Eigen::Vector<double,NJ>::Random();

	robot.set_q(q);
	// cout<<"q set"<<endl;
	robot.set_dq(dq);
	// cout<<"dq set"<<endl;
	robot.set_dqr(dqr);
	// cout<<"dqr set"<<endl;
	robot.set_ddqr(ddqr);
	// cout<<"ddqr set"<<endl;
	robot.set_par_DYN(param_DYN);
	cout<<"par_DYN set"<<endl<<robot.get_par_DYN()<<endl<<endl;
	cout<<"par_REG set"<<endl<<robot.get_par_REG()<<endl<<endl;
	robot.set_par_REG(param_REG);
	cout<<"par_DYN set"<<endl<<robot.get_par_DYN()<<endl<<endl;
	cout<<"par_REG set"<<endl<<robot.get_par_REG()<<endl<<endl;

	Kin = robot.get("T_0_ee");
	cout<<endl<<"Kin_ee\n"<<Kin<<endl;
	Kin = robot.get("T_0_0");
	cout<<endl<<"Kin0\n"<<Kin<<endl;
	Kin = robot.get("T_0_1");
	cout<<endl<<"Kin1\n"<<Kin<<endl;
	Kin = robot.get("T_0_2");
	cout<<endl<<"Kin2\n"<<Kin<<endl;
	Kin = robot.get("T_0_3");
	cout<<endl<<"Kin3\n"<<Kin<<endl;

	Jac = robot.get("J_ee");
	cout<<endl<<"Jac\n"<<Jac<<endl;
	Jac = robot.get("J_0");
	cout<<endl<<"Jac0\n"<<Jac<<endl;
	Jac = robot.get("J_1");
	cout<<endl<<"Jac1\n"<<Jac<<endl;
	Jac = robot.get("J_2");
	cout<<endl<<"Jac2\n"<<Jac<<endl;
	Jac = robot.get("J_3");
	cout<<endl<<"Jac3\n"<<Jac<<endl;

	M = robot.get("M");
	cout<<endl<<"M\n"<<M<<endl;
	C = robot.get("C");
	cout<<endl<<"C\n"<<C<<endl;
	C_std = robot.get("C_std");
	cout<<endl<<"C_std\n"<<C_std<<endl;
	G = robot.get("G");
	cout<<endl<<"G\n"<<G<<endl;
	D = robot.get("D");
	cout<<endl<<"D\n"<<D<<endl;

	Yr = robot.get("Yr");
	reg_M = robot.get("reg_M");
	reg_C = robot.get("reg_C");
	reg_G = robot.get("reg_G");
	// cout<<endl<<"Yr\n"<<Yr<<endl;

	tau_cmd_dyn = M*ddqr + C*dqr + G;
	tau_cmd_reg = Yr*param_REG;
	tau_cmd_regMat = (reg_M + reg_C + reg_G)*param_REG;

	cout<<endl<<"tau_cmd_dyn:\n"<<tau_cmd_dyn<<endl;
	cout<<endl<<"tau_cmd_reg:\n"<<tau_cmd_reg<<endl;
	cout<<endl<<"tau_cmd_regMat:\n"<<tau_cmd_regMat<<endl;

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
