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
#include "userDefined.h"

#include <yaml-cpp/yaml.h>
// #include "urdf2dh_inertial.h"
#include "genYaml.h"

#include <argparse/argparse.hpp>

using namespace thunder_ns;
using namespace std::chrono;

using std::cout;
using std::endl;

bool COPY_GEN_FLAG = true; // used to copy generated files into thunder_robot project
bool COPY_GEN_CHRONO_FLAG = true; // used to copy generated files into thunder_robot_chrono project
bool GEN_PYTHON_FLAG = false; // used to generate python binding
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


	argparse::ArgumentParser thunder_cli("thunder");
	argparse::ArgumentParser gen_command("gen");
	gen_command.add_description("Generate library from yaml configuration file");
	gen_command.add_argument("robot")
		.help("Robot configuration file");
	gen_command.add_argument("robot_name")
		.help("Robot name");
	gen_command.add_argument("-p", "--python")
		.default_value(false)
		.implicit_value(true)
		.help("Generate python binding");
	gen_command.add_argument("-c", "--casadifunc")
		.default_value(false)
		.implicit_value(true)
		.help("Save casadi functions");

	thunder_cli.add_subparser(gen_command);

	try {
		thunder_cli.parse_args(argc, argv);
	} catch (const std::runtime_error &err) {
		std::cout << err.what() << std::endl;
		std::cout << thunder_cli;
		return 0;
	}

	
	// Get arguments
	config_file = gen_command.get<std::string>("robot");
	robot_name = gen_command.get<std::string>("robot_name");
	GEN_PYTHON_FLAG = gen_command.get<bool>("python");
	

	// Set name and paths
	cout<<"Robot name: "<<robot_name<<endl;
	auto time_start = high_resolution_clock::now();
	robot_name_gen = robot_name + "_gen";
	path_gen = path_robot + robot_name + "_generatedFiles/";

	// --- Robot creation --- //
	Robot robot = robot_from_file(robot_name, config_file, 1);
	// compute_kinematics(robot);
	// compute_dynamics(robot);
	// compute_regressors(robot);
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
	std::string python_cmake_file;

	if (std::filesystem::is_directory(currentPath/"neededFiles")){
		thunder_robot_cpp_path = "neededFiles/thunder_robot_template.cpp";
		thunder_robot_h_path = "neededFiles/thunder_robot_template.h";
		python_cmake_file = "neededFiles/CMakeLists.txt";
		COPY_GEN_FLAG = false;	// not copy into thunder_robot if thunder is used from bin
		COPY_GEN_CHRONO_FLAG = false;
	}else{
		std::cout<<"No neededFiles found, ok if you are using thunder from build!"<<std::endl;
		// thunder_robot_cpp_path = PATH_THUNDER_ROBOT + "src/thunder_robot.cpp";
		// thunder_robot_h_path = PATH_THUNDER_ROBOT + "library/thunder_robot.h";
		thunder_robot_cpp_path = PATH_THUNDER_ROBOT + "thunder_robot.cpp";
		thunder_robot_h_path = PATH_THUNDER_ROBOT + "thunder_robot.h";
		python_cmake_file = PATH_THUNDER_ROBOT + "CMakeLists.txt";
	}

	sourceDestPath = absolutePath + "thunder_" + robot_name + ".h";
	sourcePath = thunder_robot_h_path;
	std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

	sourceDestPath = absolutePath + "thunder_" + robot_name + ".cpp";
	sourcePath = thunder_robot_cpp_path;
	std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

	if (GEN_PYTHON_FLAG){
		// --- Generate python binding --- //
		std::filesystem::copy_file(python_cmake_file, absolutePath +  "CMakeLists.txt", std::filesystem::copy_options::overwrite_existing);
		int changed = update_cmake("robot", robot_name, absolutePath +  "CMakeLists.txt");
		if (!changed) {
			cout<<"problem on changing robot name in the CMakeLists.txt:"<<endl;
			return 0;
		}
		cout<<"Python binding generated!"<<endl;
	}

	// --- change the necessary into thunder_robot --- //
	int changed = change_to_robot("robot", robot_name, robot, path_gen+"thunder_"+robot_name+".h", path_gen+"thunder_"+robot_name+".cpp", GEN_PYTHON_FLAG);
	if (!changed) {
		cout<<"problem on changing robot name:"<<endl;
		return 0;
	}

	// --- generate parameters files --- //
	std::string par_file = path_gen + robot_name + "_conf.yaml";
	std::string par_REG_file = path_gen + robot_name + "_par_REG.yaml";
	robot.save_conf(par_file);
	robot.save_par_REG(par_REG_file);
	// if (!genInertial_files(robot_name, nj, config_file, par_file, par_REG_file)){
	// 	return 0;
	// }

	std::cout<<"Library generated!"<<std::endl;

	
	// --- copy generated files in thunder_robot project --- //
	if(COPY_GEN_FLAG){
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
		sourcePath = absolutePath + robot_name + "_par_REG.yaml";
		sourceDestPath = PATH_COPY_YAML + robot_name + "_par_REG.yaml";
		std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);
		sourcePath = absolutePath + robot_name + "_conf.yaml";
		sourceDestPath = PATH_COPY_YAML + robot_name + "_conf.yaml";
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
		sourcePath = absolutePath + robot_name + "_par_REG.yaml";
		sourceDestPath = PATH_COPY_CHRONO_YAML + robot_name + "_par_REG.yaml";
		std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);
		sourcePath = absolutePath + robot_name + "_conf.yaml";
		sourceDestPath = PATH_COPY_CHRONO_YAML + robot_name + "_conf.yaml";
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