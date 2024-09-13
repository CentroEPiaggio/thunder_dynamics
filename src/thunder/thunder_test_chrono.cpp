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

// #define nj 3
#define N_PAR_LINK 10

using namespace thunder_ns;
using namespace std::chrono;
using std::cout;
using std::endl;

bool use_gripper = false;

Eigen::Matrix3d hat(const Eigen::Vector3d v);
// extern int compute_kinematics(Robot robot);

int main(){
	// std::string jType;
	// Eigen::MatrixXd DH_table;
	// FrameOffset Base_to_L0;
	// FrameOffset Ln_to_EE;
	std::vector<std::string> robots = {"R3", "R5", "R7", "R9"};//, "R15"};

	// ----------------------------------------------------------------------------//
	// ------------------------------ TESTS ---------------------------------------//
	// ----------------------------------------------------------------------------//

	for (std::string& r : robots){
		auto time_start_rob = high_resolution_clock::now();
		std::cout<<"Robot: "<< r <<std::endl;
		Robot robot = robot_from_file("../robots/testRobots/" + r + ".yaml");
		auto time_stop_rob = high_resolution_clock::now();
		auto duration_rob = duration_cast<microseconds>(time_stop_rob - time_start_rob).count();
		cout<<"robot created in "<<duration_rob<<" us"<<endl;

		int nj = robot.get_numJoints();
		Eigen::VectorXd param_DYN = robot.get_par_DYN();

		int n_rep = 100;
		int min_dur = 999999999;
		auto time_start = high_resolution_clock::now();
		auto time_stop = high_resolution_clock::now();
		auto duration = duration_cast<nanoseconds>(time_stop - time_start).count();

		/* Matrices */
		Eigen::MatrixXd myYr(nj, nj*N_PAR_LINK);
		Eigen::MatrixXd myM(nj, nj);
		Eigen::MatrixXd myC(nj, nj);
		Eigen::MatrixXd myC_std(nj, nj);
		Eigen::MatrixXd myG(nj, 1);
		Eigen::MatrixXd myKin(4, 4);
		Eigen::MatrixXd myJac(6, nj);

		Eigen::VectorXd q(nj), dq(nj), dqr(nj), ddqr(nj);

		/* Test */
		q.setOnes();// = Eigen::Vector<double,nj>::Random();//setOnes();
		dq.setOnes();// = Eigen::Vector<double,nj>::Random();//setOnes();
		dqr.setOnes();// = Eigen::Vector<double,nj>::Random();//setOnes();
		ddqr.setOnes();// = Eigen::Vector<double,nj>::Random();//setOnes();

		robot.set_q(q);
		// cout<<"q set"<<endl;
		robot.set_dq(dq);
		// cout<<"dq set"<<endl;
		robot.set_dqr(dqr);
		// cout<<"dqr set"<<endl;
		robot.set_ddqr(ddqr);
		// cout<<"ddqr set"<<endl;

		min_dur = 999999999;
		for (int i=0; i<n_rep; i++){
			time_start = high_resolution_clock::now();
			myKin = robot.get("T_0_ee");
			time_stop = high_resolution_clock::now();
			duration = duration_cast<nanoseconds>(time_stop - time_start).count();
			min_dur = (duration<min_dur) ? duration : min_dur;
		}
		cout<<"time Kin: "<<(float)min_dur/1000<<" us"<<endl;

		min_dur = 999999999;
		for (int i=0; i<n_rep; i++){
			time_start = high_resolution_clock::now();
			myJac = robot.get("J_ee");
			time_stop = high_resolution_clock::now();
			duration = duration_cast<nanoseconds>(time_stop - time_start).count();
			min_dur = (duration<min_dur) ? duration : min_dur;
		}
		cout<<"time Jac: "<<(float)min_dur/1000<<" us"<<endl;

		min_dur = 999999999;
		for (int i=0; i<n_rep; i++){
			time_start = high_resolution_clock::now();
			myM = robot.get("M");
			time_stop = high_resolution_clock::now();
			duration = duration_cast<nanoseconds>(time_stop - time_start).count();
			min_dur = (duration<min_dur) ? duration : min_dur;
		}
		cout<<"time M: "<<(float)min_dur/1000<<" us"<<endl;

		min_dur = 999999999;
		for (int i=0; i<n_rep; i++){
			time_start = high_resolution_clock::now();
			myC = robot.get("C");
			time_stop = high_resolution_clock::now();
			duration = duration_cast<nanoseconds>(time_stop - time_start).count();
			min_dur = (duration<min_dur) ? duration : min_dur;
		}
		cout<<"time C: "<<(float)min_dur/1000<<" us"<<endl;

		min_dur = 999999999;
		for (int i=0; i<n_rep; i++){
			time_start = high_resolution_clock::now();
			myC_std = robot.get("C_std");
			time_stop = high_resolution_clock::now();
			duration = duration_cast<nanoseconds>(time_stop - time_start).count();
			min_dur = (duration<min_dur) ? duration : min_dur;
		}
		cout<<"time C_std: "<<(float)min_dur/1000<<" us"<<endl;

		min_dur = 999999999;
		for (int i=0; i<n_rep; i++){
			time_start = high_resolution_clock::now();
			myG = robot.get("G");
			time_stop = high_resolution_clock::now();
			duration = duration_cast<nanoseconds>(time_stop - time_start).count();
			min_dur = (duration<min_dur) ? duration : min_dur;
		}
		cout<<"time G: "<<(float)min_dur/1000<<" us"<<endl;

		min_dur = 999999999;
		for (int i=0; i<n_rep; i++){
			time_start = high_resolution_clock::now();
			myYr = robot.get("Yr");
			time_stop = high_resolution_clock::now();
			duration = duration_cast<nanoseconds>(time_stop - time_start).count();
			min_dur = (duration<min_dur) ? duration : min_dur;
		}
		cout<<"time Yr: "<<(float)min_dur/1000<<" us"<<endl<<endl;
	}

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
