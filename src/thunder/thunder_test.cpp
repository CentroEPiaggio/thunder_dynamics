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

	std::string robot_conf = "../robots/debug/franka.yaml";

	// std::string robot2_conf = "../robots/debug/franka_dh.yaml";
	std::string robot2_conf = "../robots/debug/franka_urdf.yaml";

	// std::string robot_conf = "../robots/debug/RRR_dh.yaml";
	// std::string robot_conf = "../robots/debug/dynaarm.yaml";
	// std::string robot_conf = "../robots/debug/serialRRR.yaml";
	// std::string robot_conf = "../robots/debug/treeRRR.yaml";
	// std::string robot_conf = "../robots/RRR_sea/seaRRR.yaml";
	// std::string robot_conf = "../robots/ego/egoRightArm.yaml";
	// std::string robot_conf = "../robots/frankaWrist/frankaWrist.yaml";
	// std::string robot_conf = "../robots/testRobots/R9_noDynSymb.yaml";

	auto robot = legacy_robot_from_file("robot", robot_conf);
	auto robot2 = legacy_robot_from_file("robot", robot2_conf);


	// - Properties - //
	int NJ = robot->get<int>("numJoints");
	int ndof = robot->get<int>("ndof");
	int NJ_2 = robot2->get<int>("numJoints");
	int ndof_2 = robot2->get<int>("ndof");
	
	string joint_ids;
	cout << "### --- Robot 1 --- ###" << endl;
	joint_ids = "{0";
	for (int i=1; i<NJ; i++) joint_ids += ", " + std::to_string(i);
	joint_ids += "}";
	cout << "numJoints: " << NJ << endl;
	cout << "ndof: " << ndof << endl;
	cout << "jointsName: " << robot->properties["jointsName"].get_value_str() << endl;
	cout << "joint_ids: " << joint_ids << endl;
	cout << "jointsParent: " << robot->properties["jointsParent"].get_value_str() << endl;
	cout << "jointsType: " << robot->properties["jointsType"].get_value_str() << endl;
	cout << "jointsAvailable: " << robot->properties["jointsAvailable"].get_value_str() << endl;
	cout << "jointsDerivatives: " << robot->properties["jointsDerivatives"].get_value_str() << endl;
	cout << "jointsAxis: " << robot->properties["jointsAxis"].get_value_str() << endl;
	cout << "jointsDimension: " << robot->properties["jointsDimension"].get_value_str() << endl;
	cout << "### --- Robot 2 --- ###" << endl;
	joint_ids = "{0";
	for (int i=1; i<NJ_2; i++) joint_ids += ", " + std::to_string(i);
	joint_ids += "}";
	cout << "numJoints: " << NJ_2 << endl;
	cout << "ndof: " << ndof_2 << endl;
	cout << "jointsName: " << robot2->properties["jointsName"].get_value_str() << endl;
	cout << "joint_ids: " << joint_ids << endl;
	cout << "jointsParent: " << robot2->properties["jointsParent"].get_value_str() << endl;
	cout << "jointsType: " << robot2->properties["jointsType"].get_value_str() << endl;
	cout << "jointsAvailable: " << robot2->properties["jointsAvailable"].get_value_str() << endl;
	cout << "jointsDerivatives: " << robot2->properties["jointsDerivatives"].get_value_str() << endl;
	cout << "jointsAxis: " << robot2->properties["jointsAxis"].get_value_str() << endl;
	cout << "jointsDimension: " << robot2->properties["jointsDimension"].get_value_str() << endl;

	
	// NEJ = robot->get<int>("numSoftJoints");
	// N_PARAM_K = NEJ*robot->get<int>("K_order");
	// N_PARAM_D = NEJ*robot->get<int>("D_order");
	// N_PARAM_DM = NEJ*robot->get<int>("Dm_order");

	// - set variables - //
	robot->set("q", std::vector<double>(ndof, 1.0));
	robot->set("dq", std::vector<double>(ndof, 1.0));
	robot->set("dqr", std::vector<double>(ndof, 1.0));
	robot->set("ddqr", std::vector<double>(ndof, 1.0));
	// robot->set("x", std::vector<double>(NEJ,0));
	// robot->set("dx", std::vector<double>(NEJ,0));
	// robot->set("ddxr", std::vector<double>(NEJ,0));
	robot2->set("q", std::vector<double>(ndof, 1.0));
	robot2->set("dq", std::vector<double>(ndof, 1.0));
	robot2->set("dqr", std::vector<double>(ndof, 1.0));
	robot2->set("ddqr", std::vector<double>(ndof, 1.0));

	// - parameters - //
	cout << "par_KIN:" << endl << robot->get("par_KIN") << endl << endl;
	cout << "par_DYN:" << endl << robot->get("par_DYN") << endl << endl;
	cout << "par_REG:" << endl << robot->get("par_REG") << endl << endl;
	cout << "par_KIN_2:" << endl << robot2->get("par_KIN") << endl << endl;
	cout << "par_DYN_2:" << endl << robot2->get("par_DYN") << endl << endl;
	cout << "par_REG_2:" << endl << robot2->get("par_REG") << endl << endl;
	// cout << "par_DYN_diff:" << endl << robot->get("par_DYN") - robot2->get("par_DYN") << endl << endl;
	// cout << "dyn2reg:" << endl << robot->get("dyn2reg") << endl << endl;
	// cout << "reg2dyn:" << endl << robot->get("reg2dyn") << endl << endl;
	if (robot->get<int>("Dl_order")){
		cout << "par_Dl:" << endl << robot->get("par_Dl") << endl << endl;
		cout << "par_Dl symb:" << endl << robot->get_model("par_Dl") << endl << endl;
	} else {
		cout<<endl<<"Robot have no Dl_order"<<endl;
	}
	// cout << "diff par_KIN:" << endl << robot->get("par_KIN") - robot2->get("par_KIN") << endl << endl;
	// cout << "diff par_DYN:" << endl << robot->get("par_DYN") - robot2->get("par_DYN") << endl << endl;
	// cout << "diff par_REG:" << endl << robot->get("par_REG") - robot2->get("par_REG") << endl << endl;
	
	cout << "par_gravity: " << endl << robot->get_model("par_gravity") << endl << endl;
	cout << "par_gravity_2: " << endl << robot2->get_model("par_gravity") << endl << endl;

	// - Transforms - //
	for (int i=0; i<NJ-1; i++){
		auto fun = robot->get("T_w_"+std::to_string(i));
		cout << endl << "T_w_"+std::to_string(i)+": " << fun << endl << endl;
		fun = robot2->get("T_w_"+std::to_string(i));
		cout << endl << "T_w_2_"+std::to_string(i)+": " << fun << endl << endl;
	}
	// - Jacobians - //
	for (int i=0; i<NJ-1; i++){
		auto fun = robot->get("J_"+std::to_string(i));
		cout << endl << "J_"+std::to_string(i)+": " << fun << endl << endl;
		fun = robot2->get("J_"+std::to_string(i));
		cout << endl << "J_2_"+std::to_string(i)+": " << fun << endl << endl;
	}

	// - dynamic matrices - //
	cout << "M: " << robot->get("M") << endl << endl;
	cout << "M_2: " << robot2->get("M") << endl << endl;
	cout << "C: " << robot->get("C") << endl << endl;
	cout << "C_std: " << robot->get("C_std") << endl << endl;
	cout << "G: " << robot->get("G") << endl << endl;
	cout << "G_2: " << robot2->get("G") << endl << endl;
	if (robot->get<int>("Dl_order")){
		cout << "dl: " << robot->get("dl") << endl << endl;
		cout << "reg_dl: " << robot->get("reg_dl") << endl << endl;
	} else {
		cout<<endl<<"Robot have no Dl_order"<<endl;
	}
	if (robot->functions.count("Yr")) {
		cout << "reg_M: " << robot->get("reg_M") << endl << endl;
		cout << "reg_C: " << robot->get("reg_C") << endl << endl;
		cout << "reg_G: " << robot->get("reg_G") << endl << endl;
		cout << "Yr: " << robot->get("Yr") << endl << endl;
	} else {
		cout<<endl<<"Robot have no regressors"<<endl;
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
	auto tau_cmd_dyn = mtimes(M,robot->get("ddqr")) + mtimes(C,robot->get("dqr")) + G;
	cout << endl << "tau_cmd_dyn:\n" << tau_cmd_dyn << endl;
	// auto M_2 = robot2->get("M");
	// auto C_2 = robot2->get("C");
	// auto G_2 = robot2->get("G");
	// auto tau_cmd_dyn_2 = mtimes(M_2,robot2->get("ddqr")) + mtimes(C_2,robot2->get("dqr")) + G_2;
	// cout << endl << "tau_2_diff:\n" << tau_cmd_dyn - tau_cmd_dyn_2 << endl;

	if (robot->functions.count("Yr")) {
		auto reg_M = robot->get("reg_M");
		auto reg_C = robot->get("reg_C");
		auto reg_G = robot->get("reg_G");
		auto Yr = robot->get("Yr");
		auto tau_cmd_reg = mtimes(Yr, robot->get("par_REG"));
		auto tau_cmd_regMat = mtimes(reg_M + reg_C + reg_G, robot->get("par_REG")); // + mtimes(reg_Dl, par_Dl);
		cout << endl << "tau_cmd_reg:\n" << tau_cmd_reg << endl;
		cout << endl << "tau_cmd_regMat:\n" << tau_cmd_regMat << endl;
		cout << endl << "err_dyn_reg:\n" << tau_cmd_dyn - tau_cmd_reg << endl;
	}
	
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
	// // casadi::DM wrench(6);
	// // wrench << 1, 1, 1, 1, 1, 1;
	// // robot->set_arg("w", wrench);
	// auto reg_omega = robot->get("reg_Jdq");
	// // auto reg_tau = robot->get("reg_JTw");
	// // // auto reg_omega = robot->model["reg_Jdq"];
	// // // auto reg_tau = robot->model["reg_JTw"];
	// // cout << "reg_omega: " << endl << reg_omega << endl<<endl;
	// casadi::DM omega_reg = reg_omega * par;
	// casadi::DM omega_kin = robot->get("J_ee")*dq;
	// cout << "omega_reg: " << omega_reg.T() << endl;
	// cout << "omega_kin: " << omega_kin.T() << endl;
	// cout << "diff: " << omega_reg - omega_kin << endl;
	// // cout << "reg_tau: " << endl << reg_tau << endl<<endl;

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
