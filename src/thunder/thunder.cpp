/* Generate C++ code from classes that use casadi library.
In particular generate code to compute for franka emika panda robot:
	- regressor
	- jacobian
	- dot jacobian
	- pseudo-inverse jacobian
	- dot pseudo-inverse jacobian
	- forward kinematic
	- matrix mass
	- matrix coriolis
	- matrix gravity
*/

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>
#include <chrono>

#include "library/RobKinAdv.h"
#include "library/RobReg.h"
#include "library/RobDyn.h"

#include <yaml-cpp/yaml.h>
#include "library/urdf2dh_inertial.h"

using namespace thunder_ns;

using std::cout;
using std::endl;

bool use_gripper = false;
bool COPY_FLAG = true;
#define MU_JACOB 0.0

// std::string gen_files = "gen_regr_fun";
std::string gen_files = "robot_gen";
std::string path_gen = "../robots/robot/generatedFiles/";
std::string config_file = "../robots/robot/robot.yaml";
std::string path_copy_H = "../../thunder_robot/library/robot_gen.h";
std::string path_copy_CPP = "../../thunder_robot/src/robot_gen.cpp";

int main(){
	// --- Variables --- //
	int nj;
	std::string jType;
	Eigen::MatrixXd DH_table;
	FrameOffset Base_to_L0;
	FrameOffset Ln_to_EE;

	// ----------------------------- //
	// ---------- CONSOLE ---------- //
	// ----------------------------- //
	// ----- todo!!!

	// ---------------------------------- //
	// ---------- YAML PARSING ---------- //
	// ---------------------------------- //
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

	} catch (const YAML::Exception& e) {
		std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
	}
	// ---------- end parsing ---------- //

	/* RobKinAdv and RobReg object */;
	RobKinAdv kinrobot;
	RobReg regrobot;
	RobDyn dynrobot;

	kinrobot.init(nj,jType,DH_table,Base_to_L0,Ln_to_EE, MU_JACOB);
	regrobot.init(nj,jType,DH_table,Base_to_L0,Ln_to_EE);
	dynrobot.init(nj,jType,DH_table,Base_to_L0,Ln_to_EE);

	/* Get Casadi Functions */
	std::vector<casadi::Function> kin_vec, reg_vec, dyn_vec, all_vec;
	kin_vec = kinrobot.getCasadiFunctions();
	reg_vec = regrobot.getCasadiFunctions();
	dyn_vec = dynrobot.getCasadiFunctions();

	/* Merge casadi function */
	int dim1, dim2,dim3;
	dim1 = kin_vec.size();
	dim2 = reg_vec.size();
	dim3 = dyn_vec.size();

	for (int i=2; i<dim1; i++){     // exclude kinematic and jacobian
		all_vec.push_back(kin_vec[i]);
	}
	for (int i=0; i<dim2; i++){
		all_vec.push_back(reg_vec[i]);
	}
	for (int i=2; i<dim3; i++){     // exclude kinematic and jacobian
		all_vec.push_back(dyn_vec[i]);
	}
	if(all_vec.size()!=dim1+dim2+dim3-4) cout<<"Merge Error"<<endl;

	// --- Generate merge code --- //
	std::string relativePath = path_gen;

	std::filesystem::path currentPath = std::filesystem::current_path();
	std::string absolutePath = currentPath / relativePath;

	// Create directory
	try {
		std::filesystem::create_directory(absolutePath);
	} catch(std::exception & e){
		// creation failed
	}

	// Generate library
	regrobot.generate_mergeCode(all_vec, absolutePath, gen_files);

	if(COPY_FLAG){
	    /* Copy files in particular path */
	    std::filesystem::path sourcePath;
	    std::filesystem::path sourceDestPath;

	    sourcePath = absolutePath + gen_files + ".h";
	    sourceDestPath = path_copy_H;
	    std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

	    sourcePath = absolutePath + gen_files + ".cpp";
	    sourceDestPath = path_copy_CPP;
	    std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);
	}

	return 0;
}