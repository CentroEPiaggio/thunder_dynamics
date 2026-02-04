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

#include "robot.h"
#include "plugin_manager.h"

using std::cout;
using std::endl;
using std::string;
using std::vector;
using namespace thunder_ns;



std::shared_ptr<Robot> legacy_robot_from_file(string robot_name, string file){

	std::shared_ptr<Robot> robot;

	try {
		// Load YAML
		YAML::Node config_node = YAML::LoadFile(file);

		// Configure Manager
		PluginManager manager;
		manager.set_verbose(1);
		manager.configure_pipeline(config_node, 1);

		// Run Pipeline
		robot = manager.execute(robot_name);

	} catch (const std::exception& e) {
		std::cerr << "[ERROR] " << e.what() << std::endl;
	}

	return robot;
}

int main(){

	// std::string legacy_robot_conf = "../robots/debug/legacy_RRR.yaml";
	// std::string new_robot_conf = "../robots/debug/RRR.yaml";
	std::string new_robot_conf = "../robots/debug/treeRRR.yaml";
	// std::string legacy_robot_conf = "../robots/debug/legacy_seaRRR.yaml";
	// std::string new_robot_conf = "../robots/debug/seaRRR.yaml";
	// std::string config_file = "../robots/franka/franka.yaml";
	// std::string config_file = "../robots/RRR_sea/seaRRR.yaml";
	// std::string config_file = "../robots/ego/egoRightArm.yaml";
	// std::string config_file = "../robots/frankaWrist/frankaWrist.yaml";
	// std::string config_file = "../robots/testRobots/R9_noDynSymb.yaml";
	auto robot = legacy_robot_from_file("robot", new_robot_conf);


	int NJ = robot->get<int>("numJoints");

	robot->set("q", std::vector<double>(NJ,0));
	robot->set("dq", std::vector<double>(NJ,1));
	robot->set("dqr", std::vector<double>(NJ,0));
	robot->set("ddqr", std::vector<double>(NJ,0));

	for (int i=0; i<=NJ; i++){
		auto new_fun = robot->get("T_w_"+std::to_string(i));
		cout << "new robot T_w_"+std::to_string(i)+": " << new_fun << endl << endl;
	}

	cout << "new robot T_w_ee: " << robot->get("T_w_ee") << endl << endl;

	// for (int i=0; i<=NJ; i++){
	// 	auto legacy_fun = legacyRobot->get("T_w_"+std::to_string(i));
	// 	auto new_fun = robot->get("T_w_"+std::to_string(i));
	// 	cout << "legacy robot T_w_"+std::to_string(i)+": " << legacy_fun << endl << endl;
	// 	cout << "new robot T_w_"+std::to_string(i)+": " << new_fun << endl << endl;
	// }

	// for (int i=0; i<=NJ; i++){
	// 	auto legacy_fun = legacyRobot->get_model("T_w_"+std::to_string(i));
	// 	auto new_fun = robot->get_model("T_w_"+std::to_string(i));
	// 	cout << "legacy robot T_w_"+std::to_string(i)+": " << legacy_fun << endl << endl;
	// 	cout << "new robot T_w_"+std::to_string(i)+": " << new_fun << endl << endl;
	// }

	// cout << "new robot par_DHtable: " << robot->get_model("par_DHtable") << endl << endl;
	// cout << "new robot par_KIN: " << robot->get_model("par_KIN") << endl << endl;

	// // ---------------------------------------------------------------------------------//
	// // ------------------------------TEST CLASSES---------------------------------------//
	// // ---------------------------------------------------------------------------------//

	// int NEJ = 0;
	// int N_PARAM_K = 0;
	// int N_PARAM_D = 0;
	// int N_PARAM_DM = 0;

	// int NJ = robot->get<int>("numJoints");
	// int N_PARAM_DL = NJ*robot->get<int>("Dl_order");

	// bool ELASTIC = robot->get<bool>("ELASTIC");
	// if (ELASTIC){
	// 	NEJ = robot->get<int>("numSoftJoints");
	// 	N_PARAM_K = NEJ*robot->get<int>("K_order");
	// 	N_PARAM_D = NEJ*robot->get<int>("D_order");
	// 	N_PARAM_DM = NEJ*robot->get<int>("Dm_order");
	// }
	
	// // int N_PARAM_ELA = robot->get_numParELA();

	// // // arguments
	// // casadi::DM q(NJ), dq(NJ), dqr(NJ), ddqr(NJ);
	// // casadi::DM x(NEJ), dx(NEJ), ddx(NEJ);

	// /* Test */
	// robot->set("q", std::vector<double>(NJ,0));
	// robot->set("dq", std::vector<double>(NJ,0));
	// robot->set("dqr", std::vector<double>(NJ,0));
	// robot->set("ddqr", std::vector<double>(NJ,0));
	// robot->set("x", std::vector<double>(NEJ,0));
	// robot->set("dx", std::vector<double>(NEJ,0));
	// robot->set("ddxr", std::vector<double>(NEJ,0));

	// cout << "q: " << robot->get("q") << endl;

	// /* Matrices declaration*/
	casadi::DM par_DYN(robot->get("par_DYN"));
	casadi::DM par_REG(robot->get("par_REG"));
	casadi::DM dyn2reg(robot->get("dyn2reg"));
	casadi::DM reg2dyn(robot->get("reg2dyn"));
	casadi::DM par_Dl(robot->get("par_Dl"));
	casadi::SX par_Dl_symb(robot->get_model("par_Dl"));
	// casadi::DM par_K(robot->get("par_K"));
	// casadi::DM par_D(robot->get("par_D"));
	// casadi::DM par_Dm(robot->get("par_Dm"));
	// casadi::DM par_Mm(robot->get("par_Mm"));
	// casadi::DM par_DHtable(robot->get("par_DHtable"));
	// casadi::DM Yr(robot->get("Yr"));
	// casadi::DM reg_M(robot->get("reg_M"));
	// casadi::DM reg_C(robot->get("reg_C"));
	// casadi::DM reg_G(robot->get("reg_G"));
	// casadi::DM reg_Dl(robot->get("reg_Dl"));
	// casadi::DM reg_K(robot->get("reg_K"));
	// casadi::DM reg_D(robot->get("reg_D"));
	// casadi::DM reg_Dm(robot->get("reg_Dm"));
	// casadi::DM M(robot->get("M"));
	// casadi::DM C(robot->get("C"));
	// casadi::DM C_std(robot->get("C_std"));
	// casadi::DM G(robot->get("G"));
	// casadi::DM Dl(robot->get("Dl"));
	// casadi::DM K(robot->get("k"));
	// casadi::DM D(robot->get("d"));
	// casadi::DM Dm(robot->get("dm"));
	// casadi::DM Kin(robot->get("T_w_ee"));
	// casadi::DM Jac(robot->get("J_ee"));

	// casadi::DM tau_cmd_dyn(NJ, 1);
	// casadi::DM tau_cmd_reg(NJ, 1);
	// casadi::DM tau_cmd_regMat(NJ, 1);

	// cout<<"par_DHtable:"<<endl<<par_DHtable<<endl<<endl;
	cout<<"par_DYN:"<<endl<<par_DYN<<endl<<endl;
	cout<<"par_REG:"<<endl<<par_REG<<endl<<endl;
	cout<<"dyn2reg:"<<endl<<dyn2reg<<endl<<endl;
	cout<<"reg2dyn:"<<endl<<reg2dyn<<endl<<endl;
	cout<<"par_Dl:"<<endl<<par_Dl<<endl<<endl;
	cout<<"par_Dl symb:"<<endl<<par_Dl_symb<<endl<<endl;
	// cout<<"par_K:"<<endl<<par_K<<endl<<endl;
	// cout<<"par_D:"<<endl<<par_D<<endl<<endl;
	// cout<<"par_Dm:"<<endl<<par_Dm<<endl<<endl;
	// cout<<"par_Mm:"<<endl<<par_Mm<<endl<<endl;
	// // // test change par
	// // robot->set("par_REG", par_REG);
	// // par_DYN = robot->get("par_DYN");
	// // cout<<"par_diff:"<<endl<<(par_REG-robot->get("par_REG"))<<endl<<endl;

	// // cout<<"ddqr set"<<endl;
	// // robot->set("par_DYN", par_DYN);
	// // cout<<"par_DYN set"<<endl<<robot->get("par_DYN")<<endl<<endl;
	// // cout<<"par_REG set"<<endl<<robot->get("par_REG")<<endl<<endl;
	// // robot->set("par_REG", par_REG);
	// // cout<<"par_DYN set"<<endl<<robot->get("par_DYN")<<endl<<endl;
	// // cout<<"par_REG set"<<endl<<robot->get("par_REG")<<endl<<endl;

	// Kin = robot->get("T_w_ee");
	// cout<<endl<<"Kin_ee\n"<<Kin<<endl;
	// Kin = robot->get("T_w_0");
	// cout<<endl<<"Kin0\n"<<Kin<<endl;
	// Kin = robot->get("T_w_1");
	// cout<<endl<<"Kin1\n"<<Kin<<endl;
	// Kin = robot->get("T_w_2");
	// cout<<endl<<"Kin2\n"<<Kin<<endl;
	// Kin = robot->get("T_w_3");
	// cout<<endl<<"Kin3\n"<<Kin<<endl;

	// Jac = robot->get("J_ee");
	// cout<<endl<<"Jac\n"<<Jac<<endl;
	// Jac = robot->get("J_1");
	// cout<<endl<<"Jac1\n"<<Jac<<endl;
	// Jac = robot->get("J_2");
	// cout<<endl<<"Jac2\n"<<Jac<<endl;
	// Jac = robot->get("J_3");
	// cout<<endl<<"Jac3\n"<<Jac<<endl;

	// M = robot->get("M");
	// cout<<endl<<"M\n"<<M<<endl;
	// C = robot->get("C");
	// cout<<endl<<"C\n"<<C<<endl;
	// C_std = robot->get("C_std");
	// cout<<endl<<"C_std\n"<<C_std<<endl;
	// G = robot->get("G");
	// cout<<endl<<"G\n"<<G<<endl;
	// if (robot->get<int>("Dl_order")){
	// 	Dl = robot->get("dl");
	// 	cout<<endl<<"D_link\n"<<Dl<<endl;
	// 	reg_Dl = robot->get("reg_dl");
	// 	cout<<endl<<"reg_Dl\n"<<reg_Dl<<endl;
	// } else {
	// 	cout<<endl<<"Robot have no Dl_order"<<endl;
	// }
	// if (robot->get<bool>("ELASTIC")){
	// 	K = robot->get("k");
	// 	cout<<endl<<"K\n"<<K<<endl;
	// 	D = robot->get("d");
	// 	cout<<endl<<"D_coupling\n"<<D<<endl;
	// 	Dm = robot->get("dm");
	// 	cout<<endl<<"D_motor\n"<<Dm<<endl;
	// 	casadi::DM Mm = robot->get("Mm");
	// 	cout<<endl<<"M_motor\n"<<Mm<<endl;
	// 	reg_K = robot->get("reg_k");
	// 	reg_D = robot->get("reg_d");
	// 	reg_Dm = robot->get("reg_dm");
	// 	casadi::DM reg_Mm = robot->get("reg_Mm");
	// 	cout<<endl<<"reg_K\n"<<reg_K<<endl;
	// 	cout<<endl<<"reg_Dm\n"<<reg_Dm<<endl;
	// 	cout<<endl<<"reg_D\n"<<reg_D<<endl;
	// 	cout<<endl<<"reg_Mm\n"<<reg_Mm<<endl;
	// }

	// Yr = robot->get("Yr");
	// reg_M = robot->get("reg_M");
	// reg_C = robot->get("reg_C");
	// reg_G = robot->get("reg_G");

	// // cout<<endl<<"Yr\n"<<Yr<<endl;

	// tau_cmd_dyn = mtimes(M,robot->get("ddqr")) + mtimes(C,robot->get("dqr")) + G;
	// tau_cmd_reg = mtimes(Yr, par_REG);
	// // tau_cmd_regMat = (reg_M + reg_C + reg_G)*par_REG + reg_Dl*par_Dl;

	// cout << endl << "err_dyn_reg:\n" << tau_cmd_dyn - tau_cmd_reg << endl<<endl;

	// // cout << "q0_dist: " << robot->get("q0_dist") << endl<<endl;

	// // auto par_error = robot->model["G"] - mtimes(robot->model["reg_G"], robot->model["par_REG"]);
	// // cout<<"par_error: \n" << par_error << endl<<endl;
	// // cout<<endl<<"tau_cmd_regMat:\n"<<tau_cmd_regMat<<endl<<endl;

	// // - symbolic quantities - //
	// // cout << "par_DYN: " << robot->model["par_DYN"] << endl;
	// // cout << "M_symb: " << robot->model["M"] << endl;
	// // cout << "par_world2L0: " << robot->model["par_world2L0"] << endl<<endl;
	// // cout << "par_Ln2EE: " << robot->model["par_Ln2EE"] << endl<<endl;

	// // - kinematic regressors - //
	// // // casadi::DM wrench(6);
	// // // wrench << 1, 1, 1, 1, 1, 1;
	// // // robot->set_arg("w", wrench);
	// // auto reg_omega = robot->get("reg_Jdq");
	// // // auto reg_tau = robot->get("reg_JTw");
	// // // // auto reg_omega = robot->model["reg_Jdq"];
	// // // // auto reg_tau = robot->model["reg_JTw"];
	// // // cout << "reg_omega: " << endl << reg_omega << endl<<endl;
	// // auto par_dh = robot->get_arg("par_DHtable");
	// // auto par_base = robot->get_arg("par_world2L0");
	// // auto par_ee = robot->get_arg("par_Ln2EE");
	// // casadi::DM par(20,1);
	// // par << par_dh, par_base, par_ee;
	// // casadi::DM omega_reg = reg_omega * par;
	// // casadi::DM omega_kin = robot->get("J_ee")*dq;
	// // cout << "omega_reg: " << omega_reg.T() << endl;
	// // cout << "omega_kin: " << omega_kin.T() << endl;
	// // cout << "diff: " << omega_reg - omega_kin << endl;
	// // // cout << "reg_tau: " << endl << reg_tau << endl<<endl;

	// // // - Dynamic derivatives - //
	// // auto M_dot = robot->get("M_dot");
	// // auto M_ddot = robot->get("M_ddot");
	// // cout << "M_dot: " << endl << M_dot << endl<<endl;
	// // cout << "M_ddot: " << endl << M_ddot << endl<<endl;

	// // - save parameters - //
	// // robot->save_par("../robots/RRR/RRR_generatedFiles/saved_par.yaml", {"par_world2L0", "par_Ln2EE"});
	// // robot->load_par("../robots/RRR/RRR_generatedFiles/saved_par.yaml", {});
	// // cout << "par_world2L0: " << robot->get_arg("par_world2L0") << endl<<endl;
	// // cout << "par_Ln2EE: " << robot->get_arg("par_Ln2EE") << endl<<endl;

	return 0;
}
