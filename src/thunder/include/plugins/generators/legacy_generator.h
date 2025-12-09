#ifndef LEGACY_GEN_H
#define LEGACY_GEN_H

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

#include "plugin_interfaces.h"
#include "robot.h"
#include "plugins/generators/common/generator_utils.h"

using std::cout;
using std::endl;
using std::string;
using std::to_string;


namespace thunder_ns {

	class LegacyGenerator : public BaseGenerator {
		private:
			bool GEN_CASADI = false;		// generate casadi functions
			bool GEN_PYTHON = false;		// generate python bindings
			bool COPY_GEN = false;			// used to copy generated files into thunder_robot project

			int copy_to(string robot_name, string path_from, string path_conf, string path_par, string path_h, string path_cpp);
			int update_cmake(const string from_robot, const string to_robot, const string file_path);

		public:
		
			LegacyGenerator() : BaseGenerator("Legacy Generator", "Generates an Eigen C++ library for Robot, with python bindings and casadi optional") {}
			void generate(const std::shared_ptr<Robot> robot) override;

	};


	// ----- GENERATE ----- //
	void LegacyGenerator::generate(const std::shared_ptr<Robot> robot){
		int nj = robot->get<int>("numJoints");
		// --- Generate merge code --- //

		if (config_["gen_casadi"]) GEN_CASADI = config_["gen_casadi"].as<bool>();
		if (config_["gen_python"]) GEN_PYTHON = config_["gen_python"].as<bool>();
		if (config_["copy_gen"]) COPY_GEN = config_["copy_gen"].as<bool>();

		string robot_name = robot->robotName;
		string robot_name_gen = robot_name + "_gen";
		string relativePath = robot_name + "_generatedFiles/";

		std::filesystem::path currentPath = std::filesystem::current_path();
		string absolutePath = currentPath / relativePath;

		// Create directory
		try {
			std::filesystem::create_directory(absolutePath);
		} catch(std::exception & e){
			std::cout<<"Problem creating directory generatedFiles/"<<std::endl;
			return;
		}

		// --- Generate C library --- //
		// Options for c-code auto generation
		casadi::Dict opts = casadi::Dict();
		opts["cpp"] = true;
		opts["with_header"] = true;
		
		// generate functions in c code
		casadi::CodeGenerator myCodeGen = casadi::CodeGenerator(robot_name_gen, opts);
		// cout<<"casadi_fun: "<<casadi_fun<<endl;

		for (const auto& f : robot->functions) {
			myCodeGen.add(f.second.fun);
			// cout<<"f_name: "<<f.first<<endl;
			// cout<<"fun: "<<f.second<<endl<<endl;
		}
		myCodeGen.generate(absolutePath);

		// --- Casadi functions generation --- //
		if (GEN_CASADI){
			// Create directory
			try {
				std::filesystem::create_directory(absolutePath + "/casadi_functions");
			} catch(std::exception & e){
				std::cout<<"Problem creating directory casadi_functions/"<<std::endl;
			}
			// Save CasADi functions
			for (const auto& f : robot->functions) {
				std::string function_file = absolutePath + "/casadi_functions/" + f.first + ".casadi";
				f.second.fun.save(function_file);
			}
			debug_log("Casadi functions generated", VERB_INFO);
		}

		// --- Write thunder_robot into generatedFiles --- //
		std::filesystem::path sourcePath;
		std::filesystem::path destPath;
		// string thunder_robot_cpp_path;
		// string thunder_robot_h_path;
		string python_cmake_file;

		// Get home/.local/share directory
		string home = std::getenv("HOME");
		string template_path = "/usr/local/share/thunder_dynamics/templates/";

		if (std::filesystem::is_directory(template_path)){
			python_cmake_file = template_path + "CMakeLists.txt";
		}else{
			std::cerr<<"Template path not found: "<<template_path<<std::endl;
		}

		if (GEN_PYTHON){
			// --- Generate python binding --- //
			std::filesystem::copy_file(python_cmake_file, absolutePath +  "CMakeLists.txt", std::filesystem::copy_options::overwrite_existing);
			int changed = update_cmake("robot", robot_name, absolutePath +  "CMakeLists.txt");
			if (!changed) {
				cout<<"problem on changing robot name in the CMakeLists.txt:"<<endl;
				return;
			}
			debug_log("Python bindings generated", VERB_INFO);
		}

		// --- change the necessary into thunder_robot --- //
		int robot_generated = create_thunder_robot(robot_name, *robot, absolutePath+"thunder_"+robot_name+".h", absolutePath+"thunder_"+robot_name+".cpp", GEN_PYTHON);
		if (!robot_generated) {
			debug_log("Problem on creating thunder_robot", VERB_INFO);
			return;
		}

		// --- generate parameters files --- //
		string par_file = absolutePath + robot_name + "_par.yaml";
		robot->save_par(par_file);

		debug_log("Library generated", VERB_INFO);

		// thunder_robot path
		string COPY_PREFIX;
		if (currentPath.filename() == "build") { // last directory name
			COPY_PREFIX = currentPath/"../../../";
		} else {
			COPY_PREFIX = "/home/thunder_dev/thunder_dynamics/";
		}
		string PATH_COPY_H = COPY_PREFIX + "src/thunder_robot_test/include/";
		string PATH_COPY_CPP = COPY_PREFIX + "src/thunder_robot_test/src/";
		string PATH_COPY_YAML = COPY_PREFIX + "src/thunder_robot_test/robots/";
		
		// --- copy generated files in thunder_robot project --- //
		if(COPY_GEN){
			copy_to(robot_name, absolutePath, PATH_COPY_YAML, PATH_COPY_YAML, PATH_COPY_H, PATH_COPY_CPP);
			debug_log("Copied to thunder_robot_test", VERB_INFO);
		}

	}

	// ------------------------- //
	// ----- OTHER METHODS ----- //
	// ------------------------- //

	// --- COPY_TO --- //
	int LegacyGenerator::copy_to(string robot_name, string path_from, string path_conf, string path_par, string path_h, string path_cpp){
		// --- copy generated files --- //
		try{
			std::filesystem::path sourcePath;
			std::filesystem::path destPath;

			// copy .h generated files
			sourcePath = path_from + robot_name + "_gen" + ".h";
			destPath = path_h + robot_name + "_gen" + ".h";
			std::filesystem::copy_file(sourcePath, destPath, std::filesystem::copy_options::overwrite_existing);

			// copy .cpp generated files
			sourcePath = path_from + robot_name + "_gen" + ".cpp";
			destPath = path_cpp + robot_name + "_gen" + ".cpp";
			std::filesystem::copy_file(sourcePath, destPath, std::filesystem::copy_options::overwrite_existing);
			
			// copy conf and parameters files
			sourcePath = path_from + robot_name + "_par.yaml";
			destPath = path_par + robot_name + "_par.yaml";
			std::filesystem::copy_file(sourcePath, destPath, std::filesystem::copy_options::overwrite_existing);
			sourcePath = path_from + "../" + robot_name + ".yaml";
			destPath = path_conf + robot_name + "_conf.yaml";
			std::filesystem::copy_file(sourcePath, destPath, std::filesystem::copy_options::overwrite_existing);

			// copy thunder_robot
			sourcePath = path_from + "thunder_" + robot_name + ".h";
			destPath = path_h + "thunder_" + robot_name + ".h";
			std::filesystem::copy_file(sourcePath, destPath, std::filesystem::copy_options::overwrite_existing);
			sourcePath = path_from + "thunder_" + robot_name + ".cpp";
			destPath = path_cpp + "thunder_" + robot_name + ".cpp";
			std::filesystem::copy_file(sourcePath, destPath, std::filesystem::copy_options::overwrite_existing);
		} catch (const std::runtime_error &err) {
			std::cout << err.what() << std::endl;
			return 0;
		}

		return 1;
	}

	// --- UPDATE_CMAKE --- //
	int LegacyGenerator::update_cmake(const string from_robot, const string to_robot, const string file_path){
		std::ifstream file_cmake(file_path); // open in reading mode
		if (!file_cmake.is_open()) {
			std::cerr << "error in CMakeLists.txt template opening:" << file_path << endl;
			return 0;
		} else {
			std::stringstream buffer_cmake;
			buffer_cmake << file_cmake.rdbuf(); // read file_cmake on buffer_cmake
			string file_content_cmake = buffer_cmake.str(); // file_cmake as string

			file_cmake.close(); // close the file_cmake

			// - substitute 'from_robot' wiht 'to_robot' - //
			replace_all(file_content_cmake, from_robot, to_robot);

			// - overwrite file_cmake - //
			std::ofstream out_cmake(file_path);
			out_cmake << file_content_cmake;
			out_cmake.close();
		}
		return 1;
	}

} // namespace thunder_ns

#endif // LEGACY_GEN_H
