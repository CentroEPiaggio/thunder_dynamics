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
bool COPY_GEN_FLAG = false;
#define MU_JACOB 0.0

// --- paths and files --- //
std::string robot_name = "robot";
std::string path_robot = "../robots/";
std::string config_file = path_robot + robot_name + "/robot.yaml";
std::string robot_name_gen = robot_name + "_gen";
std::string path_gen = path_robot + robot_name + "/generatedFiles/";
const std::string PATH_THUNDER_ROBOT = "../../thunder_robot/";
const std::string PATH_COPY_H = PATH_THUNDER_ROBOT + "library/robot_gen.h";
const std::string PATH_COPY_CPP = PATH_THUNDER_ROBOT + "src/robot_gen.cpp";


int main(int argc, char* argv[]){
	// --- Variables --- //
	int nj;
	std::string jType;
	Eigen::MatrixXd DH_table;
	FrameOffset Base_to_L0;
	FrameOffset Ln_to_EE;

	// ----------------------------- //
	// ---------- CONSOLE ---------- //
	// ----------------------------- //
	// Arguments
	if (argc > 1) {
		// The first argument (argv[0]) is the program name (thunder)
		std::string arg1 = argv[1];
		if (arg1 == "gen"){
			if (argc > 2){ // take argument robot.yaml
				config_file = argv[2];
				int index_yaml = config_file.find_last_of(".yaml");
				if (index_yaml > 0){
					int index_path = config_file.find_last_of("/");
					if (index_path == -1){
						path_robot = "./";
						robot_name = config_file.substr(0, index_yaml);
					}else{
						path_robot = config_file.substr(0, index_path+1);
						robot_name = config_file.substr(index_path+1, index_yaml-index_path-5); // 5 are for ".yaml"
					}
				}else{
					std::cout << "Invalid config file." << std::endl;
					return 0;
				}
				if (argc > 3){ // take the robot name
					robot_name = argv[3];
				}
			}
		}
	} else {
		std::cout << "Nessun argomento passato oltre al nome del programma." << std::endl;
	}
	// Set names
	cout<<"Robot name: "<<robot_name<<endl;
	robot_name_gen = robot_name + "_gen";
	path_gen = path_robot + "/generatedFiles/";

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
	regrobot.generate_mergeCode(all_vec, absolutePath, robot_name_gen);

	if(COPY_GEN_FLAG){
		/* Copy files in particular path */
		std::filesystem::path sourcePath;
		std::filesystem::path sourceDestPath;

		sourcePath = absolutePath + robot_name_gen + ".h";
		sourceDestPath = PATH_COPY_H;
		std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

		sourcePath = absolutePath + robot_name_gen + ".cpp";
		sourceDestPath = PATH_COPY_CPP;
		std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);
	}

	// --- Write thunder_robot into generatedFiles --- //
	std::filesystem::path sourcePath;
	std::filesystem::path sourceDestPath;
	std::string thunder_robot_cpp_path = PATH_THUNDER_ROBOT + "src/thunder_robot.cpp";
	std::string thunder_robot_h_path = PATH_THUNDER_ROBOT + "library/thunder_robot.h";

	sourceDestPath = absolutePath + "thunder_" + robot_name + ".h";
	sourcePath = thunder_robot_h_path;
	std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

	sourceDestPath = absolutePath + "thunder_" + robot_name + ".h";
	sourcePath = thunder_robot_cpp_path;
	std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

	// --- todo! --- change the necessary into thunder_robot --- //
	// std::ifstream file("input.txt"); // Apriamo il file in modalità lettura

	// if (!file.is_open()) {
	// 	std::cerr << "Errore nell'apertura del file." << std::endl;
	// 	return 1;
	// }

	// std::stringstream buffer;
	// buffer << file.rdbuf(); // Leggiamo il contenuto del file nel buffer stringstream
	// std::string file_content = buffer.str(); // Otteniamo il contenuto del file come stringa

	// std::cout << "Contenuto del file:\n" << file_content << std::endl;

	// file.close(); // Chiudiamo il file

	return 0;
}