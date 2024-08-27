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

// #include "library/thunder_R3.h"
// #include "library/thunder_R5.h"
// #include "library/thunder_R7.h"
#include "library/thunder_R9.h"
// #include "library/thunder_R15.h"
// #include "library/thunder_R30.h"

// #define NJ 3
// #define N_PAR 30
// const std::string inertial_file = "../robots/robot/robot_inertial_REG.yaml";

using namespace thunder_ns;
using namespace std::chrono;
using std::cout;
using std::endl;

int main(){

	// std::vector<std::string> robots = {"R3", "R5", "R7", "R9", "R15", "R30"};

	std::string inertial_file = "../robots/R9_inertial_DYN.yaml";
	thunder_R9 robot;
	cout<<"Robot: R9"<<endl;

	int n_rep = 10000;
	int min_dur = 999999999;
	auto time_start = high_resolution_clock::now();
	auto time_stop = high_resolution_clock::now();
	auto duration = duration_cast<nanoseconds>(time_stop - time_start).count();

	robot.load_inertial_DYN(inertial_file);
	const int NJ = robot.get_numJoints();
	const int N_PAR = robot.get_numParams();

	Eigen::MatrixXd myKin(4, 4);
	Eigen::MatrixXd myJac(6,NJ);
	Eigen::MatrixXd myM(NJ, NJ);
	Eigen::MatrixXd myC(NJ, NJ);
	Eigen::MatrixXd myC_std(NJ, NJ);
	Eigen::VectorXd myG(NJ);
	Eigen::MatrixXd myYr(NJ, N_PAR);

	Eigen::VectorXd q(NJ), dq(NJ), dqr(NJ), ddqr(NJ);

	/* Test */
	q = q.setOnes();
	dq = dq.setOnes();
	dqr = dqr.setOnes();
	ddqr = ddqr.setOnes();

	robot.setArguments(q, dq, dqr, ddqr);

	// time_start = high_resolution_clock::now();
	// for(int i=0; i<n_rep; i++){myKin = robot.getKin();};
	// time_stop = high_resolution_clock::now();
	// duration = duration_cast<nanoseconds>(time_stop - time_start).count();
	// cout<<"time kin: "<<(float)duration/1000<<" us"<<endl;
	min_dur = 999999999;
	for (int i=0; i<n_rep; i++){
		time_start = high_resolution_clock::now();
		myKin = robot.get_T_0_ee();
		time_stop = high_resolution_clock::now();
		duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		min_dur = (duration<min_dur) ? duration : min_dur;
	}
	cout<<"time Kin: "<<(float)min_dur/1000<<" us"<<endl;

	min_dur = 999999999;
	for (int i=0; i<n_rep; i++){
		time_start = high_resolution_clock::now();
		myJac = robot.get_J_ee();
		time_stop = high_resolution_clock::now();
		duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		min_dur = (duration<min_dur) ? duration : min_dur;
	}
	cout<<"time Jac: "<<(float)min_dur/1000<<" us"<<endl;

	min_dur = 999999999;
	for (int i=0; i<n_rep; i++){
		time_start = high_resolution_clock::now();
		myM = robot.get_M();
		time_stop = high_resolution_clock::now();
		duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		min_dur = (duration<min_dur) ? duration : min_dur;
	}
	cout<<"time M: "<<(float)min_dur/1000<<" us"<<endl;

	min_dur = 999999999;
	for (int i=0; i<n_rep; i++){
		time_start = high_resolution_clock::now();
		myC = robot.get_C();
		time_stop = high_resolution_clock::now();
		duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		min_dur = (duration<min_dur) ? duration : min_dur;
	}
	cout<<"time C: "<<(float)min_dur/1000<<" us"<<endl;

	min_dur = 999999999;
	for (int i=0; i<n_rep; i++){
		time_start = high_resolution_clock::now();
		myC_std = robot.get_C_std();
		time_stop = high_resolution_clock::now();
		duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		min_dur = (duration<min_dur) ? duration : min_dur;
	}
	cout<<"time C_std: "<<(float)min_dur/1000<<" us"<<endl;

	min_dur = 999999999;
	for (int i=0; i<n_rep; i++){
		time_start = high_resolution_clock::now();
		myG = robot.get_G();
		time_stop = high_resolution_clock::now();
		duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		min_dur = (duration<min_dur) ? duration : min_dur;
	}
	cout<<"time G: "<<(float)min_dur/1000<<" us"<<endl;

	min_dur = 999999999;
	for (int i=0; i<n_rep; i++){
		time_start = high_resolution_clock::now();
		myYr = robot.get_Yr();
		time_stop = high_resolution_clock::now();
		duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		min_dur = (duration<min_dur) ? duration : min_dur;
	}
	cout<<"time Yr: "<<(float)min_dur/1000<<" us"<<endl<<endl;

	// cout<<endl<<"\nKin\n"<<myKin;
	// cout<<"\nJac\n"<<myJac;
	// cout<<"\nM\n"<<myM;
	// cout<<"\nC\n"<<myC;
	// cout<<"\nG\n"<<myG;
	// cout<<"\nYr\n"<<myYr;

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
