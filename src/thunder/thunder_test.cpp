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
	// std::string robot_conf = "../robots/debug/RRR.yaml";
	std::string robot_conf = "../robots/debug/treeSerialRRR.yaml";
	// std::string legacy_robot_conf = "../robots/debug/legacy_seaRRR.yaml";
	// std::string config_file = "../robots/franka/franka.yaml";
	// std::string config_file = "../robots/RRR_sea/seaRRR.yaml";
	// std::string config_file = "../robots/ego/egoRightArm.yaml";
	// std::string config_file = "../robots/frankaWrist/frankaWrist.yaml";
	// std::string config_file = "../robots/testRobots/R9_noDynSymb.yaml";
	auto robot = legacy_robot_from_file("robot", robot_conf);

	// - Properties - //
	int NJ = robot->get<int>("numJoints");
	int ndof = robot->get<int>("ndof");
	// NEJ = robot->get<int>("numSoftJoints");
	// N_PARAM_K = NEJ*robot->get<int>("K_order");
	// N_PARAM_D = NEJ*robot->get<int>("D_order");
	// N_PARAM_DM = NEJ*robot->get<int>("Dm_order");

	// - set variables - //
	robot->set("q", std::vector<double>(ndof,1.0));
	robot->set("dq", std::vector<double>(ndof,1.0));
	robot->set("dqr", std::vector<double>(ndof,1.0));
	robot->set("ddqr", std::vector<double>(ndof,1.0));
	// robot->set("x", std::vector<double>(NEJ,0));
	// robot->set("dx", std::vector<double>(NEJ,0));
	// robot->set("ddxr", std::vector<double>(NEJ,0));

	// - parameters - //
	// cout << "new robot par_DHtable: " << robot->get_model("par_DHtable") << endl << endl;
	cout << "par_KIN: " << robot->get("par_KIN") << endl << endl;
	cout << "par_DYN:" << endl << robot->get("par_DYN") << endl << endl;
	cout << "par_REG:" << endl << robot->get("par_REG") << endl << endl;
	cout << "dyn2reg:" << endl << robot->get("dyn2reg") << endl << endl;
	cout << "reg2dyn:" << endl << robot->get("reg2dyn") << endl << endl;
	cout << "par_Dl:" << endl << robot->get("par_Dl") << endl << endl;
	cout << "par_Dl symb:" << endl << robot->get_model("par_Dl") << endl << endl;

	// - Transforms - //
	for (int i=0; i<=NJ; i++){
		auto new_fun = robot->get("T_w_"+std::to_string(i));
		cout << endl << "new robot T_w_"+std::to_string(i)+": " << new_fun << endl << endl;
	}
	// - Jacobians - //

	// - dynamic matrices - //
	cout << "M: " << robot->get("M") << endl << endl;
	cout << "C: " << robot->get("C") << endl << endl;
	cout << "C_std: " << robot->get("C_std") << endl << endl;
	cout << "G: " << robot->get("G") << endl << endl;
	cout << "reg_M: " << robot->get("reg_M") << endl << endl;
	cout << "reg_C: " << robot->get("reg_C") << endl << endl;
	cout << "reg_G: " << robot->get("reg_G") << endl << endl;
	cout << "Yr: " << robot->get("Yr") << endl << endl;
	if (robot->get<int>("Dl_order")){
		cout << "dl: " << robot->get("dl") << endl << endl;
		cout << "reg_dl: " << robot->get("reg_dl") << endl << endl;
	} else {
		cout<<endl<<"Robot have no Dl_order"<<endl;
	}

	// if (robot->get<bool>("ELASTIC")){
	// 	cout << "k: " << robot->get("k") << endl << endl;
	// 	cout << "d: " << robot->get("d") << endl << endl;
	// 	cout << "dm: " << robot->get("dm") << endl << endl;
	// 	cout << "Mm: " << robot->get("Mm") << endl << endl;
	// 	cout << "reg_k: " << robot->get("reg_k") << endl << endl;
	// 	cout << "reg_D: " << robot->get("reg_D") << endl << endl;
	// 	cout << "reg_dm: " << robot->get("reg_dm") << endl << endl;
	// 	cout << "reg_Mm: " << robot->get("reg_Mm") << endl << endl;
	// }

	auto M = robot->get("M");
	auto C = robot->get("C");
	auto G = robot->get("G");
	auto Yr = robot->get("Yr");
	auto reg_M = robot->get("reg_M");
	auto reg_C = robot->get("reg_C");
	auto reg_G = robot->get("reg_G");

	// // cout<<endl<<"Yr\n"<<Yr<<endl;

	auto tau_cmd_dyn = mtimes(M,robot->get("ddqr")) + mtimes(C,robot->get("dqr")) + G;
	auto tau_cmd_reg = mtimes(Yr, robot->get("par_REG"));
	auto tau_cmd_regMat = mtimes(reg_M + reg_C + reg_G, robot->get("par_REG")); // + mtimes(reg_Dl, par_Dl);
	// tau_cmd_regMat = mtimes(reg_C, robot->get("par_REG")); // + mtimes(reg_Dl, par_Dl);

	cout << endl << "tau_cmd_dyn:\n" << tau_cmd_dyn << endl;
	cout << endl << "tau_cmd_reg:\n" << tau_cmd_reg << endl;
	cout << endl << "tau_cmd_regMat:\n" << tau_cmd_regMat << endl;
	cout << endl << "err_dyn_reg:\n" << tau_cmd_dyn - tau_cmd_reg << endl;

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
