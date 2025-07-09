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

#include "library/thunder_R3.h"
// #include "library/thunder_R5.h"
// #include "library/thunder_R7.h"
// #include "library/thunder_R9.h"
// #include "library/thunder_R15.h"
// #include "library/thunder_R30.h"

// #define NJ 3
// #define N_PAR 30
// const std::string config_file = "../robots/robot/robot_inertial_REG.yaml";

using namespace std::chrono;
using std::cout;
using std::endl;

int main(){

	// std::vector<std::string> robots = {"R3", "R5", "R7", "R9", "R15", "R30"};

	std::string config_file = "../robots/R3_conf.yaml";
	thunder_R3 robot;
	cout<<"Robot: R3"<<endl;

	int n_rep = 10000;
	int min_dur = 999999999;
	auto time_start = high_resolution_clock::now();
	auto time_stop = high_resolution_clock::now();
	auto duration = duration_cast<nanoseconds>(time_stop - time_start).count();

	robot.load_conf(config_file);
	const int NJ = robot.get_numJoints();
	const int N_PAR = robot.get_numParDYN();

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
	// cout<<"time kin: "<<(double)duration/1000<<" us"<<endl;
	min_dur = 999999999;
	for (int i=0; i<n_rep; i++){
		time_start = high_resolution_clock::now();
		myKin = robot.get_T_0_ee();
		time_stop = high_resolution_clock::now();
		duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		min_dur = ((duration<min_dur)&&(duration>0)) ? duration : min_dur;
	}
	cout<<"time Kin: "<<(double)min_dur/1000<<" us"<<endl;

	min_dur = 999999999;
	for (int i=0; i<n_rep; i++){
		time_start = high_resolution_clock::now();
		myJac = robot.get_J_ee();
		time_stop = high_resolution_clock::now();
		duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		min_dur = ((duration<min_dur)&&(duration>0)) ? duration : min_dur;
	}
	cout<<"time Jac: "<<(double)min_dur/1000<<" us"<<endl;

	min_dur = 999999999;
	for (int i=0; i<n_rep; i++){
		time_start = high_resolution_clock::now();
		myM = robot.get_M();
		time_stop = high_resolution_clock::now();
		duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		min_dur = ((duration<min_dur)&&(duration>0)) ? duration : min_dur;
	}
	cout<<"time M: "<<(double)min_dur/1000<<" us"<<endl;

	min_dur = 999999999;
	for (int i=0; i<n_rep; i++){
		time_start = high_resolution_clock::now();
		myC = robot.get_C();
		time_stop = high_resolution_clock::now();
		duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		min_dur = ((duration<min_dur)&&(duration>0)) ? duration : min_dur;
	}
	cout<<"time C: "<<(double)min_dur/1000<<" us"<<endl;

	min_dur = 999999999;
	for (int i=0; i<n_rep; i++){
		time_start = high_resolution_clock::now();
		myC_std = robot.get_C_std();
		time_stop = high_resolution_clock::now();
		duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		min_dur = ((duration<min_dur)&&(duration>0)) ? duration : min_dur;
	}
	cout<<"time C_std: "<<(double)min_dur/1000<<" us"<<endl;

	min_dur = 999999999;
	for (int i=0; i<n_rep; i++){
		time_start = high_resolution_clock::now();
		myG = robot.get_G();
		time_stop = high_resolution_clock::now();
		duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		min_dur = ((duration<min_dur)&&(duration>0)) ? duration : min_dur;
	}
	cout<<"time G: "<<(double)min_dur/1000<<" us"<<endl;

	min_dur = 999999999;
	for (int i=0; i<n_rep; i++){
		time_start = high_resolution_clock::now();
		myYr = robot.get_Yr();
		time_stop = high_resolution_clock::now();
		duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		min_dur = ((duration<min_dur)&&(duration>0)) ? duration : min_dur;
	}
	cout<<"time Yr: "<<(double)min_dur/1000<<" us"<<endl<<endl;

	// cout<<endl<<"\nKin\n"<<myKin;
	// cout<<"\nJac\n"<<myJac;
	// cout<<"\nM\n"<<myM;
	// cout<<"\nC\n"<<myC;
	// cout<<"\nG\n"<<myG;
	// cout<<"\nYr\n"<<myYr;

	return 0;
}
