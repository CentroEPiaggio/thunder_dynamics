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

	std::string config_file = "../robots/RRR/RRR.yaml";
	// std::string config_file = "../robots/franka/franka.yaml";
	// std::string config_file = "../robots/RRR_sea/seaRRR.yaml";
	// std::string config_file = "../robots/ego/egoRightArm.yaml";
	// std::string config_file = "../robots/frankaWrist/frankaWrist.yaml";
	// std::string config_file = "../robots/testRobots/R9_noDynSymb.yaml";
	Robot robot = robot_from_file("testRobot", config_file, 1); 	// create robot and compute quantities

	// ---------------------------------------------------------------------------------//
	// ------------------------------TEST CLASSES---------------------------------------//
	// ---------------------------------------------------------------------------------//

	int NJ = robot.get_numJoints();
	int NEJ = robot.get_numElasticJoints();
	int N_PARAM_DL = NJ*robot.get_Dl_order();
	int N_PARAM_K = NEJ*robot.get_K_order();
	int N_PARAM_D = NEJ*robot.get_D_order();
	int N_PARAM_DM = NEJ*robot.get_Dm_order();
	// int N_PARAM_ELA = robot.get_numParELA();

	// // arguments
	// casadi::DM q(NJ), dq(NJ), dqr(NJ), ddqr(NJ);
	// casadi::DM x(NEJ), dx(NEJ), ddx(NEJ);

	/* Test */
	robot.set_par("q", std::vector<double>(NJ,0));
	robot.set_par("dq", std::vector<double>(NJ,0));
	robot.set_par("dqr", std::vector<double>(NJ,0));
	robot.set_par("ddqr", std::vector<double>(NJ,0));
	robot.set_par("x", std::vector<double>(NEJ,0));
	robot.set_par("dx", std::vector<double>(NEJ,0));
	robot.set_par("ddxr", std::vector<double>(NEJ,0));

	cout << "q: " << robot.get_value("q") << endl;

	/* Matrices declaration*/
	casadi::DM par_DYN(robot.get_par("par_DYN"));
	casadi::DM par_REG(robot.get_par("par_REG"));
	casadi::DM par_Dl(robot.get_par("par_Dl"));
	casadi::DM par_K(robot.get_par("par_K"));
	casadi::DM par_D(robot.get_par("par_D"));
	casadi::DM par_Dm(robot.get_par("par_Dm"));
	casadi::DM par_Mm(robot.get_par("par_Mm"));
	casadi::DM par_DHtable(robot.get_par("par_DHtable"));
	casadi::DM Yr(robot.get_value("Yr"));
	casadi::DM reg_M(robot.get_value("reg_M"));
	casadi::DM reg_C(robot.get_value("reg_C"));
	casadi::DM reg_G(robot.get_value("reg_G"));
	casadi::DM reg_Dl(robot.get_value("reg_Dl"));
	casadi::DM reg_K(robot.get_value("reg_K"));
	casadi::DM reg_D(robot.get_value("reg_D"));
	casadi::DM reg_Dm(robot.get_value("reg_Dm"));
	casadi::DM M(robot.get_value("M"));
	casadi::DM C(robot.get_value("C"));
	casadi::DM C_std(robot.get_value("C_std"));
	casadi::DM G(robot.get_value("G"));
	casadi::DM Dl(robot.get_value("Dl"));
	casadi::DM K(robot.get_value("k"));
	casadi::DM D(robot.get_value("d"));
	casadi::DM Dm(robot.get_value("dm"));
	casadi::DM Kin(robot.get_value("T_0_ee"));
	casadi::DM Jac(robot.get_value("J_ee"));

	casadi::DM tau_cmd_dyn(NJ, 1);
	casadi::DM tau_cmd_reg(NJ, 1);
	casadi::DM tau_cmd_regMat(NJ, 1);

	cout<<"par_DHtable:"<<endl<<par_DHtable<<endl<<endl;
	cout<<"par_DYN:"<<endl<<par_DYN<<endl<<endl;
	cout<<"par_REG:"<<endl<<par_REG<<endl<<endl;
	cout<<"par_Dl:"<<endl<<par_Dl<<endl<<endl;
	cout<<"par_K:"<<endl<<par_K<<endl<<endl;
	cout<<"par_D:"<<endl<<par_D<<endl<<endl;
	cout<<"par_Dm:"<<endl<<par_Dm<<endl<<endl;
	cout<<"par_Mm:"<<endl<<par_Mm<<endl<<endl;
	// // test change par
	// robot.set_par_REG(par_REG);
	// par_DYN = robot.get_par_DYN();
	// cout<<"par_diff:"<<endl<<(par_REG-robot.get_par_REG()).transpose()<<endl<<endl;

	// cout<<"ddqr set"<<endl;
	// robot.set_par_DYN(par_DYN);
	// cout<<"par_DYN set"<<endl<<robot.get_par_DYN()<<endl<<endl;
	// cout<<"par_REG set"<<endl<<robot.get_par_REG()<<endl<<endl;
	// robot.set_par_REG(par_REG);
	// cout<<"par_DYN set"<<endl<<robot.get_par_DYN()<<endl<<endl;
	// cout<<"par_REG set"<<endl<<robot.get_par_REG()<<endl<<endl;

	Kin = robot.get_value("T_0_ee");
	cout<<endl<<"Kin_ee\n"<<Kin<<endl;
	Kin = robot.get_value("T_0_0");
	cout<<endl<<"Kin0\n"<<Kin<<endl;
	Kin = robot.get_value("T_0_1");
	cout<<endl<<"Kin1\n"<<Kin<<endl;
	Kin = robot.get_value("T_0_2");
	cout<<endl<<"Kin2\n"<<Kin<<endl;
	Kin = robot.get_value("T_0_3");
	cout<<endl<<"Kin3\n"<<Kin<<endl;

	Jac = robot.get_value("J_ee");
	cout<<endl<<"Jac\n"<<Jac<<endl;
	Jac = robot.get_value("J_1");
	cout<<endl<<"Jac1\n"<<Jac<<endl;
	Jac = robot.get_value("J_2");
	cout<<endl<<"Jac2\n"<<Jac<<endl;
	Jac = robot.get_value("J_3");
	cout<<endl<<"Jac3\n"<<Jac<<endl;

	M = robot.get_value("M");
	cout<<endl<<"M\n"<<M<<endl;
	C = robot.get_value("C");
	cout<<endl<<"C\n"<<C<<endl;
	C_std = robot.get_value("C_std");
	cout<<endl<<"C_std\n"<<C_std<<endl;
	G = robot.get_value("G");
	cout<<endl<<"G\n"<<G<<endl;
	if (robot.get_Dl_order()){
		Dl = robot.get_value("dl");
		cout<<endl<<"D_link\n"<<Dl<<endl;
		reg_Dl = robot.get_value("reg_dl");
		cout<<endl<<"reg_Dl\n"<<reg_Dl<<endl;
	} else {
		cout<<endl<<"Robot have no Dl_order"<<endl;
	}
	if (robot.get_ELASTIC()){
		K = robot.get_value("k");
		cout<<endl<<"K\n"<<K<<endl;
		D = robot.get_value("d");
		cout<<endl<<"D_coupling\n"<<D<<endl;
		Dm = robot.get_value("dm");
		cout<<endl<<"D_motor\n"<<Dm<<endl;
		casadi::DM Mm = robot.get_value("Mm");
		cout<<endl<<"M_motor\n"<<Mm<<endl;
		reg_K = robot.get_value("reg_k");
		reg_D = robot.get_value("reg_d");
		reg_Dm = robot.get_value("reg_dm");
		casadi::DM reg_Mm = robot.get_value("reg_Mm");
		cout<<endl<<"reg_K\n"<<reg_K<<endl;
		cout<<endl<<"reg_Dm\n"<<reg_Dm<<endl;
		cout<<endl<<"reg_D\n"<<reg_D<<endl;
		cout<<endl<<"reg_Mm\n"<<reg_Mm<<endl;
	}

	Yr = robot.get_value("Yr");
	reg_M = robot.get_value("reg_M");
	reg_C = robot.get_value("reg_C");
	reg_G = robot.get_value("reg_G");

	// cout<<endl<<"Yr\n"<<Yr<<endl;

	tau_cmd_dyn = mtimes(M,robot.get_value("ddqr")) + mtimes(C,robot.get_value("dqr")) + G;
	tau_cmd_reg = mtimes(Yr, par_REG);
	// tau_cmd_regMat = (reg_M + reg_C + reg_G)*par_REG + reg_Dl*par_Dl;

	cout << endl << "err_dyn_reg:\n" << tau_cmd_dyn - tau_cmd_reg << endl<<endl;

	// cout << "q0_dist: " << robot.get_value("q0_dist") << endl<<endl;

	// auto par_error = robot.model["G"] - mtimes(robot.model["reg_G"], robot.model["par_REG"]);
	// cout<<"par_error: \n" << par_error << endl<<endl;
	// cout<<endl<<"tau_cmd_regMat:\n"<<tau_cmd_regMat<<endl<<endl;

	// - symbolic quantities - //
	// cout << "par_DYN: " << robot.model["par_DYN"] << endl;
	// cout << "M_symb: " << robot.model["M"] << endl;
	// cout << "par_world2L0: " << robot.model["par_world2L0"] << endl<<endl;
	// cout << "par_Ln2EE: " << robot.model["par_Ln2EE"] << endl<<endl;

	// - kinematic regressors - //
	// // casadi::DM wrench(6);
	// // wrench << 1, 1, 1, 1, 1, 1;
	// // robot.set_arg("w", wrench);
	// auto reg_omega = robot.get_value("reg_Jdq");
	// // auto reg_tau = robot.get_value("reg_JTw");
	// // // auto reg_omega = robot.model["reg_Jdq"];
	// // // auto reg_tau = robot.model["reg_JTw"];
	// // cout << "reg_omega: " << endl << reg_omega << endl<<endl;
	// auto par_dh = robot.get_arg("par_DHtable");
	// auto par_base = robot.get_arg("par_world2L0");
	// auto par_ee = robot.get_arg("par_Ln2EE");
	// casadi::DM par(20,1);
	// par << par_dh, par_base, par_ee;
	// casadi::DM omega_reg = reg_omega * par;
	// casadi::DM omega_kin = robot.get_value("J_ee")*dq;
	// cout << "omega_reg: " << omega_reg.transpose() << endl;
	// cout << "omega_kin: " << omega_kin.transpose() << endl;
	// cout << "diff: " << omega_reg - omega_kin << endl;
	// // cout << "reg_tau: " << endl << reg_tau << endl<<endl;

	// // - Dynamic derivatives - //
	// auto M_dot = robot.get_value("M_dot");
	// auto M_ddot = robot.get_value("M_ddot");
	// cout << "M_dot: " << endl << M_dot << endl<<endl;
	// cout << "M_ddot: " << endl << M_ddot << endl<<endl;

	// - save parameters - //
	// robot.save_par("../robots/RRR/RRR_generatedFiles/saved_par.yaml", {"par_world2L0", "par_Ln2EE"});
	// robot.load_par("../robots/RRR/RRR_generatedFiles/saved_par.yaml", {});
	// cout << "par_world2L0: " << robot.get_arg("par_world2L0") << endl<<endl;
	// cout << "par_Ln2EE: " << robot.get_arg("par_Ln2EE") << endl<<endl;

	return 0;
}
