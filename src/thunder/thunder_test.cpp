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

// Eigen::Matrix3d hat(const Eigen::Vector3d v);
// extern int compute_kinematics(Robot robot);

int main(){

	// std::string config_file = "../robots/RRR/RRR.yaml";
	// std::string config_file = "../robots/franka/franka.yaml";
	// std::string config_file = "../robots/RRR_sea/seaRRR.yaml";
	std::string config_file = "../robots/ego/ego_arm.yaml";
	Robot robot = robot_from_file("testRobot", config_file, 1); 	// create robot and compute quantities

	// ---------------------------------------------------------------------------------//
	// ------------------------------TEST CLASSES---------------------------------------//
	// ---------------------------------------------------------------------------------//

	int NJ = robot.get_numJoints();
	int NEJ = robot.get_numElasticJoints();
	int N_PARAM_DYN = robot.get_numParDYN();
	int N_PARAM_REG = robot.get_numParREG();
	int N_PARAM_DL = NJ*robot.get_Dl_order();
	int N_PARAM_K = NEJ*robot.get_K_order();
	int N_PARAM_D = NEJ*robot.get_D_order();
	int N_PARAM_DM = NEJ*robot.get_Dm_order();
	// int N_PARAM_ELA = robot.get_numParELA();

	/* Matrices declaration*/
	Eigen::VectorXd par_DYN(N_PARAM_DYN);
	Eigen::VectorXd par_REG(N_PARAM_REG);
	Eigen::VectorXd par_Dl(N_PARAM_DL);
	Eigen::VectorXd par_K(N_PARAM_K);
	Eigen::VectorXd par_D(N_PARAM_D);
	Eigen::VectorXd par_Dm(N_PARAM_DM);
	// Eigen::VectorXd par_ELA(N_PARAM_ELA);
	Eigen::MatrixXd Yr(NJ, N_PARAM_DYN);
	Eigen::MatrixXd reg_M(NJ, N_PARAM_DYN);
	Eigen::MatrixXd reg_C(NJ, N_PARAM_DYN);
	Eigen::MatrixXd reg_G(NJ, N_PARAM_DYN);
	Eigen::MatrixXd reg_Dl(NJ, N_PARAM_DL);
	Eigen::MatrixXd reg_K(NEJ, N_PARAM_K);
	Eigen::MatrixXd reg_D(NEJ, N_PARAM_D);
	Eigen::MatrixXd reg_Dm(NEJ, N_PARAM_DM);
	Eigen::MatrixXd M(NJ, NJ);
	Eigen::MatrixXd C(NJ, NJ);
	Eigen::MatrixXd C_std(NJ, NJ);
	Eigen::MatrixXd G(NJ, 1);
	Eigen::MatrixXd Dl(NJ, 1);
	Eigen::MatrixXd K(NEJ, 1);
	Eigen::MatrixXd D(NEJ, 1);
	Eigen::MatrixXd Dm(NEJ, 1);
	Eigen::MatrixXd Kin(4, 4);
	Eigen::MatrixXd Jac(6, NJ);

	Eigen::MatrixXd tau_cmd_dyn(NJ, 1);
	Eigen::MatrixXd tau_cmd_reg(NJ, 1);
	Eigen::MatrixXd tau_cmd_regMat(NJ, 1);

	// arguments
	Eigen::VectorXd q(NJ), dq(NJ), dqr(NJ), ddqr(NJ);
	Eigen::VectorXd x(NEJ), dx(NEJ), ddxr(NEJ);

	// get quantities
	par_REG = robot.get_par_REG();
	par_DYN = robot.get_par_DYN();
	par_Dl = robot.get_arg("par_Dl");
	par_K = robot.get_arg("par_K");
	par_D = robot.get_arg("par_D");
	par_Dm = robot.get_arg("par_Dm");
	Eigen::MatrixXd DHtable = robot.get("DHtable");
	cout<<"DHtable:"<<endl<<DHtable.transpose()<<endl<<endl;
	cout<<"par_DYN:"<<endl<<par_DYN.transpose()<<endl<<endl;
	cout<<"par_REG:"<<endl<<par_REG.transpose()<<endl<<endl;
	cout<<"par_Dl:"<<endl<<par_Dl.transpose()<<endl<<endl;
	cout<<"par_K:"<<endl<<par_K.transpose()<<endl<<endl;
	cout<<"par_D:"<<endl<<par_D.transpose()<<endl<<endl;
	cout<<"par_Dm:"<<endl<<par_Dm.transpose()<<endl<<endl;
	// // test change par
	// robot.set_par_REG(par_REG);
	// par_DYN = robot.get_par_DYN();
	// cout<<"par_diff:"<<endl<<(par_REG-robot.get_par_REG()).transpose()<<endl<<endl;

	/* Test */
	q.setRandom();
	dq.setRandom();
	dqr.setRandom();
	ddqr.setRandom();
	x = 2*x.setZero();// = Eigen::Vector<double,NJ>::Random();
	dx = 2*dx.setZero();// = Eigen::Vector<double,NJ>::Random();
	ddxr = 2*ddxr.setZero();// = Eigen::Vector<double,NJ>::Random();

	robot.set_q(q);
	robot.set_dq(dq);
	robot.set_dqr(dqr);
	robot.set_ddqr(ddqr);
	robot.set_x(x);
	robot.set_dx(dx);
	robot.set_ddxr(ddxr);
	// cout<<"ddqr set"<<endl;
	// robot.set_par_DYN(par_DYN);
	// cout<<"par_DYN set"<<endl<<robot.get_par_DYN()<<endl<<endl;
	// cout<<"par_REG set"<<endl<<robot.get_par_REG()<<endl<<endl;
	// robot.set_par_REG(par_REG);
	// cout<<"par_DYN set"<<endl<<robot.get_par_DYN()<<endl<<endl;
	// cout<<"par_REG set"<<endl<<robot.get_par_REG()<<endl<<endl;

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
	if (robot.get_Dl_order()){
		Dl = robot.get("Dl");
		cout<<endl<<"D_link\n"<<Dl<<endl;
		reg_Dl = robot.get("reg_Dl");
		cout<<endl<<"reg_Dl\n"<<reg_Dl<<endl;
	} else {
		Dl.setZero();
	}
	if (robot.get_ELASTIC()){
		K = robot.get("K");
		cout<<endl<<"K\n"<<K<<endl;
		D = robot.get("D");
		cout<<endl<<"D_coupling\n"<<D<<endl;
		Dm = robot.get("Dm");
		cout<<endl<<"D_motor\n"<<Dm<<endl;
		reg_K = robot.get("reg_K");
		reg_D = robot.get("reg_D");
		reg_Dm = robot.get("reg_Dm");
		cout<<endl<<"reg_K\n"<<reg_K<<endl;
		cout<<endl<<"reg_Dm\n"<<reg_Dm<<endl;
		cout<<endl<<"reg_D\n"<<reg_D<<endl;
	}

	Yr = robot.get("Yr");
	reg_M = robot.get("reg_M");
	reg_C = robot.get("reg_C");
	reg_G = robot.get("reg_G");

	// cout<<endl<<"Yr\n"<<Yr<<endl;

	tau_cmd_dyn = M*ddqr + C*dqr + G;
	tau_cmd_reg = Yr*par_REG;
	// tau_cmd_regMat = (reg_M + reg_C + reg_G)*par_REG + reg_Dl*par_Dl;

	cout << endl << "err_dyn_reg:\n" << tau_cmd_dyn - tau_cmd_reg << endl<<endl;

	// cout << "q0_dist: " << robot.get("q0_dist") << endl<<endl;

	// auto par_error = robot.model["G"] - mtimes(robot.model["reg_G"], robot.model["par_REG"]);
	// cout<<"par_error: \n" << par_error << endl<<endl;
	// cout<<endl<<"tau_cmd_regMat:\n"<<tau_cmd_regMat<<endl<<endl;

	// - symbolic quantities - //
	// cout << "par_DYN: " << robot.model["par_DYN"] << endl;
	// cout << "M_symb: " << robot.model["M"] << endl;
	// cout << "world2L0: " << robot.model["world2L0"] << endl<<endl;
	// cout << "Ln2EE: " << robot.model["Ln2EE"] << endl<<endl;

	// - kinematic regressors - //
	// // Eigen::VectorXd wrench(6);
	// // wrench << 1, 1, 1, 1, 1, 1;
	// // robot.set_arg("w", wrench);
	// auto reg_omega = robot.get("reg_Jdq");
	// // auto reg_tau = robot.get("reg_JTw");
	// // // auto reg_omega = robot.model["reg_Jdq"];
	// // // auto reg_tau = robot.model["reg_JTw"];
	// // cout << "reg_omega: " << endl << reg_omega << endl<<endl;
	// auto par_dh = robot.get_arg("DHtable");
	// auto par_base = robot.get_arg("world2L0");
	// auto par_ee = robot.get_arg("Ln2EE");
	// Eigen::VectorXd par(20,1);
	// par << par_dh, par_base, par_ee;
	// Eigen::VectorXd omega_reg = reg_omega * par;
	// Eigen::VectorXd omega_kin = robot.get("J_ee")*dq;
	// cout << "omega_reg: " << omega_reg.transpose() << endl;
	// cout << "omega_kin: " << omega_kin.transpose() << endl;
	// cout << "diff: " << omega_reg - omega_kin << endl;
	// // cout << "reg_tau: " << endl << reg_tau << endl<<endl;

	// // - Dynamic derivatives - //
	// auto M_dot = robot.get("M_dot");
	// auto M_ddot = robot.get("M_ddot");
	// cout << "M_dot: " << endl << M_dot << endl<<endl;
	// cout << "M_ddot: " << endl << M_ddot << endl<<endl;

	// - save parameters - //
	// robot.save_par("../robots/RRR/RRR_generatedFiles/saved_par.yaml", {"world2L0", "Ln2EE"});
	// robot.load_par("../robots/RRR/RRR_generatedFiles/saved_par.yaml", {});
	// cout << "world2L0: " << robot.get_arg("world2L0") << endl<<endl;
	// cout << "Ln2EE: " << robot.get_arg("Ln2EE") << endl<<endl;

	return 0;
}
