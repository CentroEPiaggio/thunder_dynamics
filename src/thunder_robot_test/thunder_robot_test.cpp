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

// #include "thunder_robot.h"
#include "thunder_RRR.h"
// #include "thunder_franka.h"
// #include "thunder_seaRRR.h"
// #include "thunder_egoArm.h"
// #include "thunder_frankaWrist.h"

const std::string par_file = "../robots/RRR_par.yaml";
// const std::string par_file = "../robots/seaRRR_conf.yaml";
// const std::string par_file = "../robots/franka_conf.yaml";
// const std::string par_file = "../robots/egoArm_conf.yaml";
// const std::string par_file = "../robots/frankaWrist_conf.yaml";
const std::string saved_inertial_file = "../robots/saved_par_tmp.yaml";

using namespace std::chrono;
using std::cout;
using std::endl;

int main(){

	int n_rep = 10;
	auto time_start = high_resolution_clock::now();
	auto time_stop = high_resolution_clock::now();
	auto duration = duration_cast<nanoseconds>(time_stop - time_start).count();

	thunder_RRR robot;

	// robot.load_par(par_file);
	const int NJ = robot.numJoints;

	/* Test */
	VectorXd q = robot.get_q();
	VectorXd dq = robot.get_dq();
	VectorXd dqr = robot.get_dqr();
	VectorXd ddqr = robot.get_ddqr();

	q.setRandom();
	robot.set_q(q);

	// get parameters
	auto par_REG = robot.get_par_REG();
	auto par_DYN = robot.get_par_DYN();
	// auto par_Dl = robot.get_par_Dl();
	cout<<"par_DYN:"<<endl<<par_DYN.transpose()<<endl<<endl;
	cout<<"par_REG:"<<endl<<par_REG.transpose()<<endl<<endl;
	// cout<<"par_Dl:"<<endl<<par_Dl.transpose()<<endl<<endl;

	// kinematics and dynamics
	auto myKin = robot.get_T_0_ee();
	cout<<"\n\nKin\n"<<myKin;
	auto myJac = robot.get_J_ee();
	cout<<"\n\nJac\n"<<myJac;
	auto myM = robot.get_M();
	cout<<"\n\nM\n"<<myM;
	auto myC = robot.get_C();
	cout<<"\n\nC\n"<<myC;
	auto myG = robot.get_G();
	cout<<"\n\nG\n"<<myG;
	// --- Should be commented if Dl does not exists, Uncomment for link friction --- //
	// if (robot.Dl_order){
	// 	Dl = robot.get_Dl();
	// 	cout<<"\n\nD_link\n"<<Dl;
	// }
	// --- end --- //
	auto Yr = robot.get_Yr();
	cout<<"\n\nYr\n"<<Yr;

	auto tau_cmd_dyn = myM*ddqr + myC*dqr + myG;
	
	auto tau_cmd_reg = Yr*par_REG;

	cout<<"\ntau_cmd_dyn:\n"<<tau_cmd_dyn<<endl;
	cout<<"\ntau_cmd_reg:\n"<<tau_cmd_reg<<endl;
	cout<<"\ndiff tau_cmd:\n"<<tau_cmd_dyn-tau_cmd_reg<<endl<<endl;

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
	// 	int NEJ = robot.numElasticJoints;
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
	// 	Eigen::MatrixXd reg_K(NJ, N_PARAM_K);
	// 	Eigen::MatrixXd reg_D(NJ, N_PARAM_D);
	// 	Eigen::MatrixXd reg_Dm(NJ, N_PARAM_DM);

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
