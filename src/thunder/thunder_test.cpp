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

#include "library/RobKinAdv.h"
#include "library/RobReg.h"
#include "library/RobReg_Classic.h"
#include "library/RobDyn.h"

#define NJ 7
#define PARAM 10

using namespace thunder_ns;
using std::cout;
using std::endl;

bool use_gripper = false;

Eigen::Matrix3d hat(const Eigen::Vector3d v);

int main(){

	Eigen::VectorXd param_REG(PARAM*NJ);
	Eigen::VectorXd param_DYN(PARAM*NJ);
	int nj;
	std::string jType;
	Eigen::MatrixXd DH_table;
	FrameOffset Base_to_L0;
	FrameOffset Ln_to_EE;
	std::string config_file = "../robots/robot/robot.yaml";

	//-------------------------------Parsing yaml-----------------------------------//
	try {
		// --- load yaml --- //
		YAML::Node config = YAML::LoadFile(config_file);

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
		int i = 0;
		for (const auto& node : inertial) {
			
			if (i==NJ) break;

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
		std::cout<<"YAML_DH letto"<<std::endl;
		std::cout<<"\nparam DYN \n"<<param_DYN<<std::endl;

	} catch (const YAML::Exception& e) {
		std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
		return 0;
	}

	//-------------------Obtain param_REG for regressor (no via YAML)------------------------//
	
	Eigen::Matrix3d I0,IG;
	Eigen::Vector3d dOG;
	double m;
	
	for(int i=0;i<NJ;i++){

		m = param_DYN[i*PARAM];
		dOG << param_DYN[i*PARAM+1], param_DYN[i*PARAM+2],param_DYN[i*PARAM+3];
		
		IG(0, 0) = param_DYN[i*PARAM+4];
		IG(0, 1) = param_DYN[i*PARAM+5];
		IG(0, 2) = param_DYN[i*PARAM+6];
		IG(1, 0) = IG(0, 1);
		IG(1, 1) = param_DYN[i*PARAM+7];
		IG(1, 2) = param_DYN[i*PARAM+8];
		IG(2, 0) = IG(0, 2);
		IG(2, 1) = IG(1, 2);
		IG(2, 2) = param_DYN[i*PARAM+9];

		I0 = IG + m * hat(dOG).transpose() * hat(dOG);

		param_REG[i*PARAM] = m;
		param_REG[i*PARAM+1] = m*dOG[0];
		param_REG[i*PARAM+2] = m*dOG[1];
		param_REG[i*PARAM+3] = m*dOG[2];
		param_REG[i*PARAM+4] = I0(0,0);
		param_REG[i*PARAM+5] = I0(0,1);
		param_REG[i*PARAM+6] = I0(0,2);
		param_REG[i*PARAM+7] = I0(1,1);
		param_REG[i*PARAM+8] = I0(1,2);
		param_REG[i*PARAM+9] = I0(2,2);
	}
	std::cout<<"\nparam REG \n"<<param_REG<<std::endl;


	// ---------------------------------------------------------------------------------//
	// ------------------------------TEST CLASSES---------------------------------------//
	// ---------------------------------------------------------------------------------//

	/* RobKinAdv, RobReg, RobDyn object */

	RobKinAdv kinrobot;
	RobReg regrobot;
	RobReg_Classic regrobot_classic;
	RobDyn dynrobot;
	
	kinrobot.init(NJ, jType, DH_table, Base_to_L0, Ln_to_EE, 0.001);
	regrobot.init(NJ, jType, DH_table, Base_to_L0, Ln_to_EE);
	regrobot_classic.init(NJ, jType, DH_table, Base_to_L0, Ln_to_EE);
	dynrobot.init(NJ, jType, DH_table, Base_to_L0, Ln_to_EE);

	/* Matrix */
	
	Eigen::Matrix<double, NJ, NJ*PARAM> Yr;
	Eigen::Matrix<double, NJ, NJ*PARAM> Yr_classic;
	Eigen::Matrix<double, NJ, NJ*PARAM> Yr_dyn;
	Eigen::Matrix<double, NJ, NJ> myM;
	Eigen::Matrix<double, NJ, NJ> myC;
	Eigen::Matrix<double, NJ, 1> myG;
	Eigen::Matrix<double, 4, 4> myKin;
	Eigen::Matrix<double, 6, NJ> myJac;
	Eigen::Matrix<double, 6, NJ> myJacCM;

	Eigen::Matrix<double, NJ, 1> tau_cmd_dyn;
	Eigen::Matrix<double, NJ, 1> tau_cmd_reg;
	Eigen::Matrix<double, NJ, 1> tau_cmd_reg_classic;

	Eigen::VectorXd q(NJ), dq(NJ), dqr(NJ), ddqr(NJ);

	/* Test */
	q = q.setOnes();
	dq = dq.setOnes();
	dqr = dqr.setOnes();
	ddqr = ddqr.setOnes();

	kinrobot.setArguments(q,dq);
	regrobot.setArguments(q,dq,dqr,ddqr);
	regrobot_classic.setArguments(q,dq,dqr,ddqr);
	dynrobot.setArguments(q,dq,param_DYN);

	myKin = kinrobot.getKinematic();
	cout<<endl<<"Kin_ee\n"<<myKin<<endl;
	myKin = kinrobot.getT0i(0);
	cout<<endl<<"Kin0\n"<<myKin<<endl;
	myKin = kinrobot.getT0i(1);
	cout<<endl<<"Kin1\n"<<myKin<<endl;
	myKin = kinrobot.getT0i(2);
	cout<<endl<<"Kin2\n"<<myKin<<endl;
	myKin = kinrobot.getT0i(3);
	cout<<endl<<"Kin3\n"<<myKin<<endl;

	myJac = kinrobot.getJacobian();
	cout<<endl<<"Jac\n"<<myJac<<endl;
	myJac = kinrobot.getJi(0);
	cout<<endl<<"Jac0\n"<<myJac<<endl;
	myJac = kinrobot.getJi(1);
	cout<<endl<<"Jac1\n"<<myJac<<endl;
	myJac = kinrobot.getJi(2);
	cout<<endl<<"Jac2\n"<<myJac<<endl;
	myJac = kinrobot.getJi(3);
	cout<<endl<<"Jac3\n"<<myJac<<endl;

	myM = dynrobot.getMass();
	cout<<endl<<"M\n"<<myM<<endl;
	myC = dynrobot.getCoriolis();
	cout<<endl<<"C\n"<<myC<<endl;
	myG = dynrobot.getGravity();
	cout<<endl<<"G\n"<<myG<<endl;
	Yr = regrobot.getRegressor();
	cout<<endl<<"Yr\n"<<Yr<<endl;
	Yr_classic = regrobot_classic.getRegressor();
	cout<<endl<<"Yr_classic\n"<<Yr_classic<<endl;
	// Yr_dyn = dynrobot.getDynReg(q,dq,dqr,ddqr);
	// cout<<endl<<"Yr_dyn\n"<<Yr_dyn<<endl;

	tau_cmd_dyn = myM*ddqr + myC*dqr + myG;
	tau_cmd_reg = Yr*param_REG;
	tau_cmd_reg_classic = Yr_classic*param_REG;

	cout<<endl<<"tau_cmd_dyn:\n"<<tau_cmd_dyn<<endl;
	cout<<endl<<"tau_cmd_reg:\n"<<tau_cmd_reg<<endl;
	cout<<endl<<"tau_cmd_reg_classic:\n"<<tau_cmd_reg_classic<<endl;

	// cout << "test: " << Yr_classic(1,20) << endl;s

	// /* Get Casadi Functions */
	// std::vector<casadi::Function> kin_vec, reg_vec, dyn_vec, all_vec;
	// kin_vec = kinrobot.getCasadiFunctions();
	// reg_vec = regrobot.getCasadiFunctions();
	// dyn_vec = dynrobot.getCasadiFunctions();

	// /* Merge casadi function */
	// int dim1, dim2,dim3;
	// dim1 = kin_vec.size();
	// dim2 = reg_vec.size();
	// dim3 = dyn_vec.size();

	// for (int i=2; i<dim1; i++){     // exclude kinematic and jacobian
	// all_vec.push_back(kin_vec[i]);
	// }
	// for (int i=0; i<dim2; i++){
	// all_vec.push_back(reg_vec[i]);
	// }
	// for (int i=2; i<dim3; i++){     // exclude kinematic and jacobian
	// all_vec.push_back(dyn_vec[i]);
	// }
	// if(all_vec.size()!=dim1+dim2+dim3-4) cout<<"Merge Error"<<endl;

	// /* Generate merge code */
	// std::string relativePath = "";

	// std::filesystem::path currentPath = std::filesystem::current_path();
	// std::string absolutePath = currentPath / relativePath;
	// std::cout << "Absolute path: " << absolutePath << std::endl;

	// regrobot.generate_mergeCode(all_vec, absolutePath, "regr_fun_3R_classic");


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
