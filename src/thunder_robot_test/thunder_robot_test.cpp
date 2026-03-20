/* Test of classes and check validation of regressor compare standard equation of dynamic */

#include <iostream>
#include <string>
#include <sstream>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>
#include <chrono>
#include <concepts>
// #include <yaml-cpp/yaml.h>

#include "thunder_RRR.h"
#include "thunder_treeRRR.h"
#include "thunder_franka.h"
#include "thunder_franka_urdf.h"
#include "thunder_dynaarm.h"
#include "thunder_seaRRR.h"
// #include "thunder_egoArm.h"
// #include "thunder_frankaWrist.h"

// #define thunder_robot thunder_dynaarm
#define thunder_robot thunder_franka

// const std::string par_file = "../robots/RRR_par.yaml";
// const std::string par_file = "../robots/treeRRR_par.yaml";
// const std::string par_file = "../robots/seaRRR_conf.yaml";
// const std::string par_file = "../robots/franka_conf.yaml";
// const std::string par_file = "../robots/franka_urdf_conf.yaml";
// const std::string par_file = "../robots/dynaarm_conf.yaml";
// const std::string par_file = "../robots/egoArm_conf.yaml";
// const std::string par_file = "../robots/frankaWrist_conf.yaml";
const std::string saved_inertial_file = "../robots/saved_par_tmp.yaml";

using namespace std::chrono;
using std::cout;
using std::endl;
using std::string;
using Eigen::VectorXd;

template <typename T>
struct Tester {
	T& robot;
	Tester() : robot(*(new T())) {}

	void set_q(const Eigen::VectorXd& value) {
		if constexpr (requires (T& x) { x.set_q(value); }) robot.set_q(value);
	}
	void set_dq(const Eigen::VectorXd& value) {
		if constexpr (requires (T& x) { x.set_dq(value); }) robot.set_dq(value);
	}
	void set_ddq(const Eigen::VectorXd& value) {
		if constexpr (requires (T& x) { x.set_ddq(value); }) robot.set_ddq(value);
	}
	void set_d3q(const Eigen::VectorXd& value) {
		if constexpr (requires (T& x) { x.set_d3q(value); }) robot.set_d3q(value);
	}
	void set_d4q(const Eigen::VectorXd& value) {
		if constexpr (requires (T& x) { x.set_d4q(value); }) robot.set_d4q(value);
	}
	void set_dqr(const Eigen::VectorXd& value) {
		if constexpr (requires (T& x) { x.set_dqr(value); }) robot.set_dqr(value);
	}
	void set_ddqr(const Eigen::VectorXd& value) {
		if constexpr (requires (T& x) { x.set_ddqr(value); }) robot.set_ddqr(value);
	}
	void set_w(const Eigen::VectorXd& value) {
		if constexpr (requires (T& x) { x.set_w(value); }) robot.set_w(value);
	}

	string get_name() {
		if constexpr (requires (T& x) { x.name; }) {
			return robot.name;
		} else {
			return "not defined!";
		}
	}

	int get_numJoints() {
		if constexpr (requires (T& x) { x.numJoints; }) {
			return robot.numJoints;
		} else {
			return 0;
		}
	}

	int get_ndof() {
		if constexpr (requires (T& x) { x.ndof; }) {
			return robot.ndof;
		} else {
			return 0;
		}
	}

	string get_T_w_i() {
		std::stringstream ss;
		if constexpr (requires (T& x) { x.get_T_w_0(); }) {
			ss << "T_w_0: " << endl << robot.get_T_w_0() << endl << endl;
		} else {
			ss << "T_w_0 not defined!" << endl << endl;
		}
		if constexpr (requires (T& x) { x.get_T_w_1(); }) {
			ss << "T_w_1: " << endl << robot.get_T_w_1() << endl << endl;
		} else {
			ss << "T_w_1 not defined!" << endl << endl;
		}
		if constexpr (requires (T& x) { x.get_T_w_2(); }) {
			ss << "T_w_2: " << endl << robot.get_T_w_2() << endl << endl;
		} else {
			ss << "T_w_2 not defined!" << endl << endl;
		}
		if constexpr (requires (T& x) { x.get_T_w_3(); }) {
			ss << "T_w_3: " << endl << robot.get_T_w_3() << endl << endl;
		} else {
			ss << "T_w_3 not defined!" << endl << endl;
		}
		if constexpr (requires (T& x) { x.get_T_w_4(); }) {
			ss << "T_w_4: " << endl << robot.get_T_w_4() << endl << endl;
		} else {
			ss << "T_w_4 not defined!" << endl << endl;
		}
		if constexpr (requires (T& x) { x.get_T_w_5(); }) {
			ss << "T_w_5: " << endl << robot.get_T_w_5() << endl << endl;
		} else {
			ss << "T_w_5 not defined!" << endl << endl;
		}
		return ss.str();
	}

	string get_J_ee() {
		if constexpr (requires (T& x) { x.get_J_ee(); }) {
			std::stringstream ss;
			ss << robot.get_J_ee();
			return ss.str();
		} else {
			return "not defined!";
		}
	}

	string get_MCGY() {
		std::stringstream ss;
		VectorXd tau_diff;
		if constexpr (requires (T& x) { x.get_M(); }) {
			ss << "M: " << endl << robot.get_M() << endl << endl;
			tau_diff = robot.get_M() * robot.get_ddqr();
			ss << "tau_M difference: " << endl << robot.get_M()*robot.get_ddqr() - robot.get_reg_M()*robot.get_par_REG() << endl << endl;
		} else {
			ss << "M: not defined!" << endl << endl;
		}
		if constexpr (requires (T& x) { x.get_C(); }) {
			ss << "C: " << endl << robot.get_C() << endl << endl;
			tau_diff += robot.get_C() * robot.get_dqr();
			ss << "tau_C difference: " << endl << robot.get_C()*robot.get_dqr() - robot.get_reg_C()*robot.get_par_REG() << endl << endl;
		} else {
			ss << "C: not defined!" << endl << endl;
		}
		if constexpr (requires (T& x) { x.get_G(); }) {
			ss << "G: " << endl << robot.get_G().transpose() << endl << endl;
			tau_diff += robot.get_G();
			ss << "tau_G difference: " << endl << robot.get_G() - robot.get_reg_G()*robot.get_par_REG() << endl << endl;
		} else {
			ss << "G: not defined!" << endl << endl;
		}
		if constexpr (requires (T& x) { x.get_Yr(); }) {
			ss << "Yr: " << endl << robot.get_Yr() << endl << endl;
			tau_diff -= robot.get_Yr() * robot.get_par_REG();
		} else {
			ss << "Yr: not defined!" << endl << endl;
		}
		ss << "tau difference: " << endl << tau_diff.transpose() << endl << endl;
		return ss.str();
	}

	string get_parameters() {
		std::stringstream ss;
		if constexpr (requires (T& x) { x.get_par_KIN(); }) {
			ss << "par_KIN: " << endl << robot.get_par_KIN().transpose() << endl << endl;
		} else {
			ss << "par_KIN: not defined!" << endl << endl;
		}
		if constexpr (requires (T& x) { x.get_par_DYN(); }) {
			ss << "par_DYN: " << endl << robot.get_par_DYN().transpose() << endl << endl;
		} else {
			ss << "par_DYN: not defined!" << endl << endl;
		}
		if constexpr (requires (T& x) { x.get_par_REG(); }) {
			ss << "par_REG: " << endl << robot.get_par_REG().transpose() << endl << endl;
		} else {
			ss << "par_REG: not defined!" << endl << endl;
		}
		if constexpr (requires (T& x) { x.get_par_Dl(); }) {
			ss << "par_Dl: " << endl << robot.get_par_Dl().transpose() << endl << endl;
		} else {
			ss << "par_Dl: not defined!" << endl << endl;
		}
		if constexpr (requires (T& x) { x.get_par_K(); }) {
			ss << "par_K: " << endl << robot.get_par_K().transpose() << endl << endl;
		} else {
			ss << "par_K: not defined!" << endl << endl;
		}
		if constexpr (requires (T& x) { x.get_par_D(); }) {
			ss << "par_D: " << endl << robot.get_par_D().transpose() << endl << endl;
		} else {
			ss << "par_D: not defined!" << endl << endl;
		}
		if constexpr (requires (T& x) { x.get_par_Dm(); }) {
			ss << "par_Dm: " << endl << robot.get_par_Dm().transpose() << endl << endl;
		} else {
			ss << "par_Dm: not defined!" << endl << endl;
		}
		return ss.str();
	}

	// bar returns bool: true if call was forwarded, false otherwise
	bool bar(int v) {
		if constexpr (requires (T& x) { x.bar(v); }) {
			robot.bar(v);
			return true;
		} else {
			return false;
		}
	}
};

int main(){

	Tester<thunder_robot> robot;
	cout << "Robot: " << robot.get_name() << endl;

	// std::vector<std::string> robots = {"R3", "R5", "R7", "R9", "R15", "R30"};

	// std::string config_file = "../robots/R3_conf.yaml";
	// thunder_RRR robot;
	// cout<<"Robot: R9"<<endl;

	// thunder_RRR robot;

	// robot.load_par(par_file);
	const int NJ = robot.get_numJoints();
	const int NDOF = robot.get_ndof();

	/* Test */
	Eigen::VectorXd q = (Eigen::VectorXd(NDOF) << 0, 0, 1, 0, 0, 0, 0).finished();
	Eigen::VectorXd dq = (Eigen::VectorXd(NDOF) << 0.1, -0.2, 0.3, -0.1, 0.05, 0.02, 0.1).finished();
	Eigen::VectorXd dqr = (Eigen::VectorXd(NDOF) << 0.1, -0.2, 0.3, -0.1, 0.05, 0.02, 0.1).finished();
	Eigen::VectorXd ddqr = (Eigen::VectorXd(NDOF) << 0.5, -0.3, 0.2, 0.1, -0.05, 0.01, 0.2).finished();
	// Vector<double,NDOF> q({0, 0, 1, 0, 0, 0});
	// Vector<double,NDOF> dq({0.1, -0.2, 0.3, -0.1, 0.05, 0.02});
	// Vector<double,NDOF> dqr({0.1, -0.2, 0.3, -0.1, 0.05, 0.02});
	// Vector<double,NDOF> ddqr({0.5, -0.3, 0.2, 0.1, -0.05, 0.01});

	q.setRandom();
	dq.setRandom();
	robot.set_q(q);
	robot.set_dq(dq);
	robot.set_dqr(dqr);
	robot.set_ddqr(ddqr);

	cout << "##################" << endl <<
			"### Parameters ###" << endl <<
			"##################" << endl << endl << robot.get_parameters() << endl;

	cout << "#################" << endl <<
			"### Functions ###" << endl <<
			"#################" << endl << endl;
	
	cout << robot.get_T_w_i() << endl << endl;

	cout << robot.get_MCGY() << endl << endl; 

	// Vector<double,1> q_joint;
	// q_joint << 1.5;
	// auto T_joint_R = robot.get_T_JOINT_R(q_joint);
	// cout << "T_joint_R: " << T_joint_R << endl;

	// // - save par test - //
	// robot.save_par(saved_inertial_file);
	// robot.load_par(saved_inertial_file);
	// robot.save_par(saved_inertial_file, {"par_DYN"});
	// robot.load_par(saved_inertial_file, {"par_DYN"});
	// robot.load_par(saved_inertial_file, {"q"});

	// // - conf loading test - //
	// cout << "world2L0: " << robot.get_par_world2L0() << endl;
	// cout << "par_Ln2EE: " << robot.get_par_Ln2EE() << endl;

	// // - set par test - //
	// Eigen::Vector3d par_ee({3, 3, 3});
	// robot.set_par_Ln2EE(par_ee);
	// cout << "world2L0: \n" << robot.get_world2L0() << endl<<endl;
	// cout << "par_Ln2EE: \n" << robot.get_Ln2EE() << endl<<endl;

	// // - kinematic regressors - //
	// Eigen::Vector<double,6> wrench({1, 1, 1, 1, 1, 1});
	// robot.set_w(wrench);
	// cout << "reg_Jdq: \n" << robot.get_reg_Jdq() << endl << "size: " << robot.get_reg_Jdq().size() << endl;
	// cout << "reg_JTw: \n" << robot.get_reg_JTw() << endl<<endl;
	// Eigen::VectorXd dhtable = robot.get_par_DHtable();
	// cout << "par_kin: " << dhtable.transpose() << endl<<endl;
	// robot.set_par_DHtable(dhtable);
	// cout << "size_dh: " << dhtable.size() << endl;


	// --- Should be commented if ELASTIC = 0, Uncomment for elastic behavior --- //
	// if (robot.ELASTIC){
	// 	int NEJ = robot.numSoftJoints;
	// 	cout<<endl<<"num elastic joints: "<< NEJ<<endl;
	// 	// robot.load_par_(elastic_file);

	// 	Eigen::VectorXd x(NEJ), dx(NEJ), ddxr(NEJ);
	// 	x = 1.57*x.setOnes();
	// 	dx.setZero();
	// 	ddxr.setZero();
	// 	robot.set_x(x);
	// 	robot.set_dx(dx);
	// 	robot.set_ddxr(ddxr);

	// 	int N_PARAM_K = NEJ*robot.K_order;
	// 	int N_PARAM_D = NEJ*robot.D_order;
	// 	int N_PARAM_DM = NEJ*robot.Dm_order;
	// 	Eigen::VectorXd par_K(N_PARAM_K);
	// 	Eigen::VectorXd par_D(N_PARAM_D);
	// 	Eigen::VectorXd par_Dm(N_PARAM_DM);
	// 	Eigen::MatrixXd K(NEJ, 1);
	// 	Eigen::MatrixXd D(NEJ, 1);
	// 	Eigen::MatrixXd Dm(NEJ, 1);
	// 	Eigen::MatrixXd reg_K(NDOF, N_PARAM_K);
	// 	Eigen::MatrixXd reg_D(NDOF, N_PARAM_D);
	// 	Eigen::MatrixXd reg_Dm(NDOF, N_PARAM_DM);

	// 	par_K = robot.get_par_K();
	// 	par_D = robot.get_par_D();
	// 	par_Dm = robot.get_par_Dm();
	// 	cout<<endl<<"par_K:"<<endl<<par_K.transpose()<<endl;
	// 	cout<<endl<<"par_D:"<<endl<<par_D.transpose()<<endl;
	// 	cout<<endl<<"par_Dm:"<<endl<<par_Dm.transpose()<<endl;

	// 	K = robot.get_K1();
	// 	cout<<endl<<"K\n"<<K<<endl;
	// 	// D = robot.get_D();
	// 	// cout<<endl<<"D_coupling\n"<<D<<endl;
	// 	Dm = robot.get_Dm1();
	// 	cout<<endl<<"D_motor\n"<<Dm<<endl;
	// }
	// --- end --- //

	return 0;
}
