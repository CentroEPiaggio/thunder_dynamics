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

// #include "thunder_RRR.h"
// #include "include/thunder_R3.h"
// #include "include/thunder_R5.h"
// #include "include/thunder_R7.h"
// #include "include/thunder_R9.h"
// #include "include/thunder_R15.h"
// #include "include/thunder_R30.h"
// #include "thunder_robot.h"
#include "thunder_RRR.h"
// #include "thunder_treeRRR.h"
// #include "thunder_franka.h"
// #include "thunder_franka_urdf.h"
// #include "thunder_seaRRR.h"
// #include "thunder_egoArm.h"
// #include "thunder_frankaWrist.h"

#define thunder_robot thunder_RRR

// const std::string par_file = "../robots/RRR_par.yaml";
// const std::string par_file = "../robots/treeRRR_par.yaml";
// const std::string par_file = "../robots/seaRRR_conf.yaml";
// const std::string par_file = "../robots/franka_conf.yaml";
// const std::string par_file = "../robots/franka_urdf_conf.yaml";
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
	int ndof;
	Eigen::VectorXd q, dq, dqr, ddqr;
	int n_rep = 10000;
	int min_dur = 999999999;
	std::chrono::time_point<std::chrono::high_resolution_clock> time_start = high_resolution_clock::now();
	std::chrono::time_point<std::chrono::high_resolution_clock> time_stop = high_resolution_clock::now();
	int64_t duration = duration_cast<nanoseconds>(time_stop - time_start).count();

	string get_name() {
		if constexpr (requires (T& x) { x.name; }) {
			return robot.name;
		} else {
			return "not defined!";
		}
	}

	string test_MCGY() {
		std::stringstream ss;
		Eigen::MatrixXd myM, myC, myC_std, myG, myYr;
		// - get properties - //
		if constexpr (requires (T& x) { x.ndof; }) {
			ndof = robot.ndof;
			q = Eigen::VectorXd::Zero(ndof);
			dq = Eigen::VectorXd::Zero(ndof);
			dqr = Eigen::VectorXd::Zero(ndof);
			ddqr = Eigen::VectorXd::Zero(ndof);
		} else {
			throw("No ndof in robot");
		}
		// - set state - //
		robot.set_q(q.setOnes());
		robot.set_dq(dq.setOnes());
		robot.set_dqr(dqr.setOnes());
		robot.set_ddqr(ddqr.setOnes());

		// - tests - //
		if constexpr (requires (T& x) { x.get_M(); }) {
			min_dur = 999999999;
			for (int i=0; i<n_rep; i++){
				time_start = high_resolution_clock::now();
				myM = robot.get_M();
				time_stop = high_resolution_clock::now();
				duration = duration_cast<nanoseconds>(time_stop - time_start).count();
				min_dur = ((duration<min_dur)&&(duration>0)) ? duration : min_dur;
			}
			ss << "time M: "<<(double)min_dur/1000<<" us"<<endl;
		} else {
			ss << "M: not defined!" << endl << endl;
		}

		if constexpr (requires (T& x) { x.get_C(); }) {
			min_dur = 999999999;
			for (int i=0; i<n_rep; i++){
				time_start = high_resolution_clock::now();
				myC = robot.get_C();
				time_stop = high_resolution_clock::now();
				duration = duration_cast<nanoseconds>(time_stop - time_start).count();
				min_dur = ((duration<min_dur)&&(duration>0)) ? duration : min_dur;
			}
			ss << "time C: "<<(double)min_dur/1000<<" us"<<endl;
		} else {
			ss << "C: not defined!" << endl << endl;
		}

		if constexpr (requires (T& x) { x.get_G(); }) {
			min_dur = 999999999;
			for (int i=0; i<n_rep; i++){
				time_start = high_resolution_clock::now();
				myG = robot.get_G();
				time_stop = high_resolution_clock::now();
				duration = duration_cast<nanoseconds>(time_stop - time_start).count();
				min_dur = ((duration<min_dur)&&(duration>0)) ? duration : min_dur;
			}
			ss << "time G: "<<(double)min_dur/1000<<" us"<<endl;
		} else {
			ss << "G: not defined!" << endl << endl;
		}

		if constexpr (requires (T& x) { x.get_Yr(); }) {
			min_dur = 999999999;
			for (int i=0; i<n_rep; i++){
				time_start = high_resolution_clock::now();
				myYr = robot.get_Yr();
				time_stop = high_resolution_clock::now();
				duration = duration_cast<nanoseconds>(time_stop - time_start).count();
				min_dur = ((duration<min_dur)&&(duration>0)) ? duration : min_dur;
			}
			ss << "time Yr: "<<(double)min_dur/1000<<" us"<<endl<<endl;
		} else {
			ss << "Yr: not defined!" << endl << endl;
		}

		return ss.str();
	}
};

using namespace std::chrono;
using std::cout;
using std::endl;

int main(){

	// std::vector<std::string> robots = {"R3", "R5", "R7", "R9", "R15", "R30"};

	Tester<thunder_robot> robot;
	cout << "Robot: " << robot.get_name() << endl;

	cout << robot.test_MCGY() << endl;

	// std::string config_file = "../robots/R3_conf.yaml";
	// thunder_RRR robot;
	// cout<<"Robot: R9"<<endl;

	// int n_rep = 10000;
	// int min_dur = 999999999;
	// auto time_start = high_resolution_clock::now();
	// auto time_stop = high_resolution_clock::now();
	// auto duration = duration_cast<nanoseconds>(time_stop - time_start).count();

	// // robot.load_conf(config_file);
	// const int NJ = robot.numJoints;
	// const int N_PAR = robot.get_par_DYN().size();

	// Eigen::MatrixXd myKin(4, 4);
	// Eigen::MatrixXd myJac(6,NJ);
	// Eigen::MatrixXd myM(NJ, NJ);
	// Eigen::MatrixXd myC(NJ, NJ);
	// Eigen::MatrixXd myC_std(NJ, NJ);
	// Eigen::VectorXd myG(NJ);
	// Eigen::MatrixXd myYr(NJ, N_PAR);

	// Eigen::VectorXd q(NJ), dq(NJ), dqr(NJ), ddqr(NJ);

	// /* Test */
	// q = q.setOnes();
	// dq = dq.setOnes();
	// dqr = dqr.setOnes();
	// ddqr = ddqr.setOnes();

	// robot.set_q(q);
	// robot.set_dq(dq);
	// robot.set_dqr(dqr);
	// robot.set_ddqr(ddqr);

	// min_dur = 999999999;
	// for (int i=0; i<n_rep; i++){
	// 	time_start = high_resolution_clock::now();
	// 	myKin = robot.get_T_w_ee();
	// 	time_stop = high_resolution_clock::now();
	// 	duration = duration_cast<nanoseconds>(time_stop - time_start).count();
	// 	min_dur = ((duration<min_dur)&&(duration>0)) ? duration : min_dur;
	// }
	// cout<<"time Kin: "<<(double)min_dur/1000<<" us"<<endl;

	// min_dur = 999999999;
	// for (int i=0; i<n_rep; i++){
	// 	time_start = high_resolution_clock::now();
	// 	myJac = robot.get_J_ee();
	// 	time_stop = high_resolution_clock::now();
	// 	duration = duration_cast<nanoseconds>(time_stop - time_start).count();
	// 	min_dur = ((duration<min_dur)&&(duration>0)) ? duration : min_dur;
	// }
	// cout<<"time Jac: "<<(double)min_dur/1000<<" us"<<endl;

	return 0;
}
