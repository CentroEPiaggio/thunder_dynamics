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

// #include "library/RobKinAdv.h"
// #include "library/RobReg.h"
// #include "library/RobDyn.h"
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

Robot robot_from_file(std::string file){
	int nj;
	std::string jType;
	Eigen::MatrixXd DH_table;
	FrameOffset Base_to_L0;
	FrameOffset Ln_to_EE;
	Eigen::VectorXd param_REG;
	Eigen::VectorXd param_DYN;
	// ----- parsing yaml file ----- //
	try {
		// load yaml
		YAML::Node config = YAML::LoadFile(file);

		// Number of joints
		YAML::Node num_joints = config["num_joints"];
		nj = num_joints.as<double>();

		// joints_type
		YAML::Node type_joints = config["type_joints"];
		jType = type_joints.as<std::string>();

		// Denavit-Hartenberg
		std::vector<double> dh_vect = config["DH"].as<std::vector<double>>();
		DH_table = Eigen::Map<Eigen::VectorXd>(&dh_vect[0], nj*4).reshaped<Eigen::RowMajor>(nj, 4);

		// gravity
		std::vector<double> gravity = config["gravity"].as<std::vector<double>>();

		// frames offsets
		YAML::Node frame_base = config["Base_to_L0"];
		YAML::Node frame_ee = config["Ln_to_EE"];

		std::vector<double> tr = frame_base["tr"].as<std::vector<double>>();
		std::vector<double> ypr = frame_base["ypr"].as<std::vector<double>>();
		Base_to_L0.set_translation(tr);
		Base_to_L0.set_ypr(ypr);
		Base_to_L0.set_gravity(gravity);

		tr = frame_ee["tr"].as<std::vector<double>>();
		ypr = frame_ee["ypr"].as<std::vector<double>>();
		Ln_to_EE.set_translation(tr);
		Ln_to_EE.set_ypr(ypr);

		YAML::Node inertial = config["inertial"];
		param_REG.resize(N_PAR_LINK*nj);
		param_DYN.resize(N_PAR_LINK*nj);
		int i = 0;
		for (const auto& node : inertial) {
			
			if (i==nj) break;

			std::string linkName = node.first.as<std::string>();
			
			param_DYN[10*i] = node.second["mass"].as<double>();
			param_DYN[10*i+1] = node.second["CoM_x"].as<double>();
			param_DYN[10*i+2] = node.second["CoM_y"].as<double>();
			param_DYN[10*i+3] = node.second["CoM_z"].as<double>();
			param_DYN[10*i+4] = node.second["Ixx"].as<double>();
			param_DYN[10*i+5] = node.second["Ixy"].as<double>();
			param_DYN[10*i+6] = node.second["Ixz"].as<double>();
			param_DYN[10*i+7] = node.second["Iyy"].as<double>();
			param_DYN[10*i+8] = node.second["Iyz"].as<double>();
			param_DYN[10*i+9] = node.second["Izz"].as<double>();
			i++;
		}
		// std::cout<<"YAML_DH letto"<<std::endl;
		// std::cout<<"\nparam DYN \n"<<param_DYN<<std::endl;

	} catch (const YAML::Exception& e) {
		std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
	}

	auto time_start = high_resolution_clock::now();
	Robot robot(nj, jType, DH_table, Base_to_L0, Ln_to_EE);
	compute_kinematics(robot);
	compute_dynamics(robot);
	compute_regressors(robot);
	robot.set_par_DYN(param_DYN);

	auto time_stop = high_resolution_clock::now();
	auto duration = duration_cast<microseconds>(time_stop - time_start).count();
	cout<<"robot created in "<<duration<<" us"<<endl;
	return robot;
}

int main(){
	// std::string jType;
	// Eigen::MatrixXd DH_table;
	// FrameOffset Base_to_L0;
	// FrameOffset Ln_to_EE;
	std::vector<std::string> robots = {"R3", "R5", "R7", "R9", "R15"};

	// ----------------------------------------------------------------------------//
	// ------------------------------ TESTS ---------------------------------------//
	// ----------------------------------------------------------------------------//

	for (std::string& r : robots){
		std::cout<<"Robot: "<< r <<std::endl;
		Robot robot = robot_from_file("../robots/testRobots/" + r + ".yaml");

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

		// time_start = high_resolution_clock::now();
		// for(int i=0; i<n_rep; i++){myKin = robot.get("T_0_ee");};
		// time_stop = high_resolution_clock::now();
		// duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		// cout<<"time kin: "<<duration/n_rep<<" us"<<endl;
		min_dur = 999999999;
		for (int i=0; i<n_rep; i++){
			time_start = high_resolution_clock::now();
			myKin = robot.get("T_0_ee");
			time_stop = high_resolution_clock::now();
			duration = duration_cast<nanoseconds>(time_stop - time_start).count();
			min_dur = (duration<min_dur) ? duration : min_dur;
		}
		cout<<"time Kin: "<<(float)min_dur/1000<<" us"<<endl;

		// time_start = high_resolution_clock::now();
		// for(int i=0; i<n_rep; i++){myJac = robot.get("J_ee");};
		// time_stop = high_resolution_clock::now();
		// duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		// cout<<"time jac: "<<duration/n_rep<<" us"<<endl;
		min_dur = 999999999;
		for (int i=0; i<n_rep; i++){
			time_start = high_resolution_clock::now();
			myJac = robot.get("J_ee");
			time_stop = high_resolution_clock::now();
			duration = duration_cast<nanoseconds>(time_stop - time_start).count();
			min_dur = (duration<min_dur) ? duration : min_dur;
		}
		cout<<"time Jac: "<<(float)min_dur/1000<<" us"<<endl;

		// time_start = high_resolution_clock::now();
		// for(int i=0; i<n_rep; i++){myM = robot.get("M");};
		// time_stop = high_resolution_clock::now();
		// duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		// cout<<"time M: "<<duration/n_rep<<" us"<<endl;
		min_dur = 999999999;
		for (int i=0; i<n_rep; i++){
			time_start = high_resolution_clock::now();
			myM = robot.get("M");
			time_stop = high_resolution_clock::now();
			duration = duration_cast<nanoseconds>(time_stop - time_start).count();
			min_dur = (duration<min_dur) ? duration : min_dur;
		}
		cout<<"time M: "<<(float)min_dur/1000<<" us"<<endl;

		// time_start = high_resolution_clock::now();
		// for(int i=0; i<n_rep; i++){myC = robot.get("C");};
		// time_stop = high_resolution_clock::now();
		// duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		// cout<<"time C: "<<duration/n_rep<<" us"<<endl;
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

		// time_start = high_resolution_clock::now();
		// for(int i=0; i<n_rep; i++){myG = robot.get("G");};
		// time_stop = high_resolution_clock::now();
		// duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		// cout<<"time G: "<<duration/n_rep<<" us"<<endl;
		min_dur = 999999999;
		for (int i=0; i<n_rep; i++){
			time_start = high_resolution_clock::now();
			myG = robot.get("G");
			time_stop = high_resolution_clock::now();
			duration = duration_cast<nanoseconds>(time_stop - time_start).count();
			min_dur = (duration<min_dur) ? duration : min_dur;
		}
		cout<<"time G: "<<(float)min_dur/1000<<" us"<<endl;

		// time_start = high_resolution_clock::now();
		// for(int i=0; i<n_rep; i++){myYr = robot.get("Yr");};
		// time_stop = high_resolution_clock::now();
		// duration = duration_cast<nanoseconds>(time_stop - time_start).count();
		// cout<<"time Yr: "<<duration/n_rep<<" us"<<endl<<endl;
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
