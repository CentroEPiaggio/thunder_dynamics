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

#include "utils.h"
#include "robot.h"
#include "kinematics.h"
#include "dynamics.h"
#include "regressors.h"

#include <yaml-cpp/yaml.h>
// #include "urdf2dh_inertial.h"
#include "genYaml.h"

using namespace thunder_ns;
using namespace std::chrono;

using std::cout;
using std::endl;

bool COPY_GEN_FLAG = true; // used to copy generated files into thunder_robot project
bool COPY_GEN_CHRONO_FLAG = true; // used to copy generated files into thunder_robot_chrono project
#define MU_JACOB 0.0

// --- paths and files (default) --- //
std::string robot_name = "robot";
std::string path_robot = "../robots/";
std::string config_file = path_robot + robot_name + "/robot.yaml";
std::string robot_name_gen = robot_name + "_gen";
std::string path_gen = path_robot + robot_name + "/generatedFiles/";
const std::string PATH_THUNDER_ROBOT = "../thunder_robot_template/";
const std::string PATH_COPY_H = "../../thunder_robot/library/";
const std::string PATH_COPY_CPP = "../../thunder_robot/src/";
const std::string PATH_COPY_YAML = "../../thunder_robot/robots/";
const std::string PATH_COPY_CHRONO_H = "../../thunder_robot_chrono/library/";
const std::string PATH_COPY_CHRONO_CPP = "../../thunder_robot_chrono/src/";
const std::string PATH_COPY_CHRONO_YAML = "../../thunder_robot_chrono/robots/";

int main(int argc, char* argv[]){
	// --- Variables --- //
	int nj;

	// ----------------------------- //
	// ---------- CONSOLE ---------- //
	// ----------------------------- //
	// Arguments
	if (argc > 1) {
		// The first argument (argv[0]) is the program name (thunder)
		std::string arg1 = argv[1];
		if (arg1 == "gen"){
			if (argc > 2){ // take argument <robot>.yaml
				config_file = argv[2];
				int index_yaml = config_file.find_last_of(".yaml");
				if (index_yaml > 0){
					int index_path = config_file.find_last_of("/");
					if (index_path == std::string::npos){ // no occurrence
						path_robot = "./";
						robot_name = config_file.substr(0, index_yaml);
					}else{
						path_robot = config_file.substr(0, index_path+1);
						robot_name = config_file.substr(index_path+1, index_yaml-index_path-5); // 5 stands for ".yaml"
					}
				}else{
					std::cout << "Invalid config file." << std::endl;
					return 0;
				}
				if (argc > 3){ // take the robot name
					robot_name = argv[3];
					// robot_name is a valid name?
				}
			}
		} else {
			std::cout << arg1 << "not recognised as command." << std::endl;
			return 0;
		}
	} else {
		std::cout << "No arguments to process." << std::endl;
		return 0;
	}
	// Set name and paths
	cout<<"Robot name: "<<robot_name<<endl;
	auto time_start = high_resolution_clock::now();
	robot_name_gen = robot_name + "_gen";
	path_gen = path_robot + robot_name + "_generatedFiles/";

	// --- Robot creation --- //
	Robot robot = robot_from_file(config_file, 0);
	compute_kinematics(robot);
	compute_dynamics(robot);
	compute_regressors(robot);
	nj = robot.get_numJoints();

	// --- Generate merge code --- //
	std::string relativePath = path_gen;

	std::filesystem::path currentPath = std::filesystem::current_path();
	std::string absolutePath = currentPath / relativePath;
	// std::string absolutePath = path_gen; // not absolute but relative to thunder

	// Create directory
	try {
		std::filesystem::create_directory(absolutePath);
	} catch(std::exception & e){
		std::cout<<"Problem creating directory generatedFiles/"<<std::endl;
		return 0;
	}

	// Generate library
	// regrobot.generate_mergeCode(all_vec, absolutePath, robot_name_gen);
	robot.generate_library(absolutePath, robot_name_gen);

	// --- Write thunder_robot into generatedFiles --- //
	std::filesystem::path sourcePath;
	std::filesystem::path sourceDestPath;
	std::string thunder_robot_cpp_path;
	std::string thunder_robot_h_path;
	if (std::filesystem::is_directory(currentPath/"neededFiles")){
		thunder_robot_cpp_path = "neededFiles/thunder_robot_template.cpp";
		thunder_robot_h_path = "neededFiles/thunder_robot_template.h";
		COPY_GEN_FLAG = false;	// not copy into thunder_robot if thunder is used from bin
		COPY_GEN_CHRONO_FLAG = false;
	}else{
		std::cout<<"No neededFiles found, ok if you are using thunder from build!"<<std::endl;
		// thunder_robot_cpp_path = PATH_THUNDER_ROBOT + "src/thunder_robot.cpp";
		// thunder_robot_h_path = PATH_THUNDER_ROBOT + "library/thunder_robot.h";
		thunder_robot_cpp_path = PATH_THUNDER_ROBOT + "thunder_robot.cpp";
		thunder_robot_h_path = PATH_THUNDER_ROBOT + "thunder_robot.h";
	}

	sourceDestPath = absolutePath + "thunder_" + robot_name + ".h";
	sourcePath = thunder_robot_h_path;
	std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

	sourceDestPath = absolutePath + "thunder_" + robot_name + ".cpp";
	sourcePath = thunder_robot_cpp_path;
	std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

	// --- change the necessary into thunder_robot --- //
	int changed = change_to_robot("robot", robot_name, robot, path_gen+"thunder_"+robot_name+".h", path_gen+"thunder_"+robot_name+".cpp");
	if (!changed) {
		cout<<"problem on changing robot name:"<<endl;
		return 0;
	}

	// --- generate inertial_REG --- //
	std::string inertial_DYN_file = path_gen + robot_name + "_inertial_DYN.yaml";
	std::string inertial_REG_file = path_gen + robot_name + "_inertial_REG.yaml";
	if (!genInertial_files(robot_name, nj, config_file, inertial_DYN_file, inertial_REG_file)){
		return 0;
	}

	std::cout<<"Library generated!"<<std::endl;

	// --- copy generated files in thunder_robot project --- //
	if(COPY_GEN_FLAG){
		// Problem here, on inertial_reg for sure!
		std::filesystem::path sourcePath;
		std::filesystem::path sourceDestPath;

		// copy .h generated files
		sourcePath = absolutePath + robot_name_gen + ".h";
		sourceDestPath = PATH_COPY_H + robot_name_gen + ".h";
		std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

		// copy .cpp generated files
		sourcePath = absolutePath + robot_name_gen + ".cpp";
		sourceDestPath = PATH_COPY_CPP + robot_name_gen + ".cpp";
		std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);
		
		// copy inertial files
		sourcePath = absolutePath + robot_name + "_inertial_REG.yaml";
		sourceDestPath = PATH_COPY_YAML + robot_name + "_inertial_REG.yaml";
		std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);
		sourcePath = absolutePath + robot_name + "_inertial_DYN.yaml";
		sourceDestPath = PATH_COPY_YAML + robot_name + "_inertial_DYN.yaml";
		std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

		// copy thunder_robot
		sourcePath = path_gen + "thunder_" + robot_name + ".h";
		sourceDestPath = PATH_COPY_H + "thunder_" + robot_name + ".h";
		std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);
		sourcePath = path_gen + "thunder_" + robot_name + ".cpp";
		sourceDestPath = PATH_COPY_CPP + "thunder_" + robot_name + ".cpp";
		std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

		std::cout<<"Copied to thunder_robot!"<<std::endl;
	}

	// --- copy generated files in thunder_robot project --- //
	if(COPY_GEN_CHRONO_FLAG){
		// Problem here, on inertial_reg for sure!
		std::filesystem::path sourcePath;
		std::filesystem::path sourceDestPath;

		// copy .h generated files
		sourcePath = absolutePath + robot_name_gen + ".h";
		sourceDestPath = PATH_COPY_CHRONO_H + robot_name_gen + ".h";
		std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

		// copy .cpp generated files
		sourcePath = absolutePath + robot_name_gen + ".cpp";
		sourceDestPath = PATH_COPY_CHRONO_CPP + robot_name_gen + ".cpp";
		std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);
		
		// copy inertial files
		sourcePath = absolutePath + robot_name + "_inertial_REG.yaml";
		sourceDestPath = PATH_COPY_CHRONO_YAML + robot_name + "_inertial_REG.yaml";
		std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);
		sourcePath = absolutePath + robot_name + "_inertial_DYN.yaml";
		sourceDestPath = PATH_COPY_CHRONO_YAML + robot_name + "_inertial_DYN.yaml";
		std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

		// copy thunder_robot
		sourcePath = path_gen + "thunder_" + robot_name + ".h";
		sourceDestPath = PATH_COPY_CHRONO_H + "thunder_" + robot_name + ".h";
		std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);
		sourcePath = path_gen + "thunder_" + robot_name + ".cpp";
		sourceDestPath = PATH_COPY_CHRONO_CPP + "thunder_" + robot_name + ".cpp";
		std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

		std::cout<<"Copied to thunder_robot_chrono!"<<std::endl;
	}

	// --- elapsed time --- //
	auto time_stop = high_resolution_clock::now();
	auto duration = duration_cast<microseconds>(time_stop - time_start);
	std::cout<<"done in "<<((double)duration.count())/1000<<" ms!"<<endl; 
	return 1;
}