#ifndef LEGACY_GEN_H
#define LEGACY_GEN_H

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

#include "../../plugin_interfaces.h"
#include "../../robot.h"

using std::cout;
using std::endl;
using std::string;
using std::to_string;


namespace thunder_ns {

	class LegacyGenerator : public BaseGenerator {
		private:
			YAML::Node config_;
			bool GEN_CASADI = false;		// generate casadi functions
			bool GEN_PYTHON = false;		// generate python bindings
			bool COPY_GEN = false;			// used to copy generated files into thunder_robot project

			int copy_to(string robot_name, string path_from, string path_conf, string path_par, string path_h, string path_cpp);
			int update_cmake(const string from_robot, const string to_robot, const string file_path);
			int change_to_robot(const string from_robot, const string to_robot, Robot& robot, const string file_path_h, const string file_path_cpp, const bool gen_python);
			int add_bindings_template(const string file_path_cpp);
		
		public:
		
			LegacyGenerator() : BaseGenerator("Legacy Generator", "Generates an Eigen C++ library for Robot, with python bindings and casadi optional") {}
			int configure(const YAML::Node& config) override;
			void generate(const std::shared_ptr<Robot> robot) override;

	};

	// ----- CONFIGURE ----- //
	int LegacyGenerator::configure(const YAML::Node& config){			
		config_ = config;

		if (config_["gen_casadi"]) GEN_CASADI = config_["gen_casadi"].as<bool>();
		if (config_["gen_python"]) GEN_PYTHON = config_["gen_python"].as<bool>();
		if (config_["copy_gen"]) COPY_GEN = config_["copy_gen"].as<bool>();

		debug_log("Configured", VERB_INFO);

		return 0;
	}

	// ----- GENERATE ----- //
	void LegacyGenerator::generate(const std::shared_ptr<Robot> robot){
		int nj = robot->get_numJoints();
		// --- Generate merge code --- //

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

		// Generate library
		// if( gen_command.get<bool>("casadi"))
		// 	std::cout<<"Saving Casadi functions!"<<std::endl;
		robot->generate_library(absolutePath, robot_name_gen, GEN_CASADI);

		if (GEN_CASADI){
			debug_log("Casadi functions generated", VERB_INFO);
		}

		// --- Write thunder_robot into generatedFiles --- //
		std::filesystem::path sourcePath;
		std::filesystem::path destPath;
		string thunder_robot_cpp_path;
		string thunder_robot_h_path;
		string python_cmake_file;

		// Get home/.local/share directory
		string home = std::getenv("HOME");
		// string template_path = home + "/.local/share/thunder_dynamics/thunder_robot_template/";
		string template_path = "/usr/local/share/thunder_dynamics/thunder_robot_template/";

		if (std::filesystem::is_directory(template_path)){
			thunder_robot_cpp_path = template_path + "thunder_robot.cpp";
			thunder_robot_h_path = template_path + "thunder_robot.h";
			python_cmake_file = template_path + "CMakeLists.txt";
		}else{
			std::cerr<<"Template path not found: "<<template_path<<std::endl;
		}

		destPath = absolutePath + "thunder_" + robot_name + ".h";
		sourcePath = thunder_robot_h_path;
		std::filesystem::copy_file(sourcePath, destPath, std::filesystem::copy_options::overwrite_existing);

		destPath = absolutePath + "thunder_" + robot_name + ".cpp";
		sourcePath = thunder_robot_cpp_path;
		std::filesystem::copy_file(sourcePath, destPath, std::filesystem::copy_options::overwrite_existing);

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
		int changed = change_to_robot("robot", robot_name, *robot, absolutePath+"thunder_"+robot_name+".h", absolutePath+"thunder_"+robot_name+".cpp", GEN_PYTHON);
		if (!changed) {
			debug_log("Problem on modifying template", VERB_INFO);
			return;
		}

		// --- generate parameters files --- //
		string par_file = absolutePath + robot_name + "_par.yaml";
		// string par_REG_file = absolutePath + robot_name + "_par_REG.yaml";
		robot->save_par(par_file);
		// robot->save_par_REG(par_REG_file);

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

	// --- CHANGE_TO_ROBOT --- //
	int LegacyGenerator::change_to_robot(const string from_robot, const string to_robot, Robot& robot, const string file_path_h, const string file_path_cpp, const bool gen_python){
		
		// - get parameters from robot - //
		string robotName = robot.robotName;
		int n_joints = robot.get_numJoints();
		int numElasticJoints = robot.get_numElasticJoints();
		bool ELASTIC = robot.get_ELASTIC();
		int K_order = robot.get_K_order();
		int D_order = robot.get_D_order();
		int Dl_order = robot.get_Dl_order();
		int Dm_order = robot.get_Dm_order();
		std::vector<int> isElasticJoint = robot.get_isElasticJoint();
		
		// int STD_PAR_LINK = robot.STD_PAR_LINK;
		std::vector<fun_obj> functions = robot.get_functions();

		// --- file .h --- //
		std::ifstream file_h(file_path_h); // open in reading mode
		if (!file_h.is_open()) {
			std::cerr << "error in file_h opening:" << file_path_h << endl;
			return 0;
		} else {
			std::stringstream buffer_h;
			buffer_h << file_h.rdbuf(); // read file_h on buffer_h
			string file_content_h = buffer_h.str(); // file_h as string

			file_h.close(); // close the file_h

			// - change num_joints - //
			string header = "THUNDERROBOT";
			size_t index_header = file_content_h.find(header);
			file_content_h.replace(index_header, header.length(), "THUNDER_" + to_robot);
			index_header = file_content_h.find(header);
			file_content_h.replace(index_header, header.length(), "THUNDER_" + to_robot);

			// - substitute 'from_robot' wiht 'to_robot' - //
			replace_all(file_content_h, from_robot, to_robot);

			// - add variables - //
			replace_all(file_content_h, "/*#-ROBOT_NAME-#*/", robotName);
			replace_all(file_content_h, "/*#-n_joints-#*/", to_string(n_joints));
			replace_all(file_content_h, "/*#-ELASTIC-#*/", to_string(ELASTIC));
			replace_all(file_content_h, "/*#-numElasticJoints-#*/", to_string(numElasticJoints));
			replace_all(file_content_h, "/*#-K_order-#*/", to_string(K_order));
			replace_all(file_content_h, "/*#-D_order-#*/", to_string(D_order));
			replace_all(file_content_h, "/*#-Dl_order-#*/", to_string(Dl_order));
			replace_all(file_content_h, "/*#-Dm_order-#*/", to_string(Dm_order));

			// elastic joints
			string eJ_str = "{" + to_string(isElasticJoint[0]);
			for (int i=1; i<n_joints; i++){
				eJ_str += ", " + to_string(isElasticJoint[i]);
			}
			eJ_str += "}";
			replace_all(file_content_h, "/*#-isElasticJoint-#*/", eJ_str);

			// - insert functions - //
			string functions_string = "\n";
			for (int i=0; i<functions.size(); i++){
				functions_string.append("\t\t// - " + functions[i].description + " - //\n");
				functions_string.append("\t\t"+get_ret_type(functions[i])+" get_" + functions[i].name + "();\n\n");
			}
			replace_all(file_content_h, "/*#-FUNCTIONS_H-#*/", functions_string);

			// - overwrite file_h - //
			std::ofstream out_h(file_path_h);
			out_h << file_content_h;
			out_h.close();
		}

		// --- file .cpp --- //
		if(gen_python){
			// - add bindings template - //
			add_bindings_template(file_path_cpp);
		}

		std::ifstream file_cpp(file_path_cpp); // open in reading mode
		if (!file_cpp.is_open()) {
			std::cerr << "error in file_cpp opening:" << file_path_h << endl;
			return 0;
		} else {
			std::stringstream buffer_cpp;
			buffer_cpp << file_cpp.rdbuf(); // read file_cpp on buffer_cpp
			string file_content_cpp = buffer_cpp.str(); // file_cpp as string

			file_cpp.close(); // close the file_cpp

			// - substitute 'from_robot' wiht 'to_robot' - //
			replace_all(file_content_cpp, from_robot, to_robot);

			// - insert functions - //
			string functions_string = "\n";
			string functions_pybindings = "";

			for (int i=0; i<functions.size(); i++){
				string fun_name = "get_" + functions[i].name;
				string fun_name_gen = robotName + "_" + functions[i].name;
				std::vector<string> fun_args = functions[i].args;
				std::vector<int> out_size = functions[i].out_size;
				// // function arguments
				// string args_string = "(" + fun_args[0];
				// for (int j=1; j<fun_args.size(); j++){
				// 	args_string.append(", " + fun_args[j]);
				// }
				// other parts
				string ret_type = get_ret_type(functions[i]);
				functions_string.append("// - " + functions[i].description + " - //\n");
				functions_string.append(ret_type + " thunder_" + to_robot + "::" + fun_name + "(){\n");
				// functions_string.append("\tEigen::MatrixXd out;\n");
				// functions_string.append("\tout.resize("+to_string(out_size[0])+","+to_string(out_size[1])+");\n");
				string size_str = to_string(out_size[0]*out_size[1]);
				functions_string.append("\tthread_local double buffer["+size_str+"];\n"); // alignas("+size_str+") 
				functions_string.append("\tthread_local long long p3[" + fun_name_gen + "_fun_SZ_IW];\n");
				functions_string.append("\tthread_local double p4[" + fun_name_gen + "_fun_SZ_W];\n");
				// inputs
				if (fun_args.size() == 0){
					functions_string.append("\tconst double** input_ = nullptr;\n");
				} else {
					functions_string.append("\tconst double* input_[] = {" + fun_args[0]+".data()");
					for (int j=1; j<fun_args.size(); j++){
						functions_string.append(", " + fun_args[j]+".data()");
					}
					functions_string.append("};\n");
				}
				// output
				functions_string.append("\tdouble* output_[] = {buffer};\n");
				functions_string.append("\tint check = " + fun_name_gen + "_fun(input_, output_, p3, p4, 0);\n");
				functions_string.append("\treturn Eigen::Map<"+ret_type+">(buffer);\n");
				functions_string.append("}\n\n");

				// pybindings
				if(gen_python)
					functions_pybindings.append("\t\t.def(\"" + fun_name + "\", &thunder_" + to_robot + "::" + fun_name + ", \""+ functions[i].description +"\")\n");
			}
			replace_all(file_content_cpp, "/*#-FUNCTIONS_CPP-#*/", functions_string);

			if(gen_python){
				//replace the last \n with a ;
				functions_pybindings.pop_back();
				functions_pybindings.append(";\n");
				replace_all(file_content_cpp, "/*#-GENERATED_PYTHON_BINDINGS-#*/", functions_pybindings);
			}
			
			// - overwrite file_cpp - //
			std::ofstream out_cpp(file_path_cpp);
			out_cpp << file_content_cpp;
			out_cpp.close();
		}

		return 1;
	}

	// --- ADD_BINDINGS_TEMPLATE --- //
	int LegacyGenerator::add_bindings_template(const string file_path_cpp){
		
		string bindings_template = R"(
// ----- Python bindings ----- //
namespace py = pybind11;

PYBIND11_MODULE(thunder_robot_py, m) {
	py::class_<thunder_robot>(m, "thunder_robot")
		.def(py::init<>())
		.def("resizeVariables", &thunder_robot::resizeVariables)
		.def("setArguments", &thunder_robot::setArguments, "Set q, dq, dqr, ddqr", py::arg("q"), py::arg("dq"), py::arg("dqr"), py::arg("ddqr"))
		.def("set_q", &thunder_robot::set_q, "Set q", py::arg("q"))
		.def("set_dq", &thunder_robot::set_dq, "Set dq", py::arg("dq"))
		.def("set_ddq", &thunder_robot::set_ddq, "Set ddq", py::arg("ddq"))
		.def("set_d3q", &thunder_robot::set_d3q, "Set d3q", py::arg("d3q"))
		.def("set_d4q", &thunder_robot::set_d4q, "Set d4q", py::arg("d4q"))
		.def("set_dqr", &thunder_robot::set_dqr, "Set dqr", py::arg("dqr"))
		.def("set_ddqr", &thunder_robot::set_ddqr, "Set ddqr", py::arg("ddqr"))
		.def("set_x", &thunder_robot::set_x, "Set x", py::arg("x"))
		.def("set_dx", &thunder_robot::set_dx, "Set dx", py::arg("dx"))
		.def("set_ddx", &thunder_robot::set_ddx, "Set ddx", py::arg("ddx"))
		.def("set_ddxr", &thunder_robot::set_ddxr, "Set ddxr", py::arg("ddxr"))
		.def("set_w", &thunder_robot::set_w, "Set w", py::arg("w"))
		.def("set_par_REG", &thunder_robot::set_par_REG, "Set inertial parameters REG", py::arg("par"), py::arg("update_DYN") = true)
		.def("set_par_DYN", &thunder_robot::set_par_DYN, "Set inertial parameters DYN", py::arg("par"), py::arg("update_REG") = true)
		.def("set_par_K", &thunder_robot::set_par_K, "Set inertial parameters K", py::arg("par"))
		.def("set_par_D", &thunder_robot::set_par_D, "Set inertial parameters D", py::arg("par"))
		.def("set_par_Dm", &thunder_robot::set_par_Dm, "Set inertial parameters Dm", py::arg("par"))
		.def("set_par_Mm", &thunder_robot::set_par_Mm, "Set inertial parameters Mm", py::arg("par"))
		.def("set_par_Dl", &thunder_robot::set_par_Dl, "Set inertial parameters Dl", py::arg("par"))
		.def("set_par_DHtable", &thunder_robot::set_par_DHtable, "Set inertial parameters DHtable", py::arg("par"))
		.def("set_par_gravity", &thunder_robot::set_par_gravity, "Set inertial parameters gravity", py::arg("par"))
		.def("set_par_world2L0", &thunder_robot::set_par_world2L0, "Set inertial parameters world2L0", py::arg("par"))
		.def("set_par_Ln2EE", &thunder_robot::set_par_Ln2EE, "Set inertial parameters Ln2EE", py::arg("par"))
		.def("get_par_REG", &thunder_robot::get_par_REG, "Get par parameters REG")
		.def("get_par_DYN", &thunder_robot::get_par_DYN, "Get inertial parameters DYN")
		.def("get_par_K", &thunder_robot::get_par_K, "Get par parameters K")
		.def("get_par_D", &thunder_robot::get_par_D, "Get par parameters D")
		.def("get_par_Dm", &thunder_robot::get_par_Dm, "Get par parameters Dm")
		.def("get_par_Mm", &thunder_robot::get_par_Mm, "Get par parameters Mm")
		.def("get_par_Dl", &thunder_robot::get_par_Dl, "Get par parameters Dl")
		.def("get_par_DHtable", &thunder_robot::get_par_DHtable, "Get par parameters par_DHtable")
		.def("get_par_gravity", &thunder_robot::get_par_gravity, "Get par parameters gravity")
		.def("get_par_world2L0", &thunder_robot::get_par_world2L0, "Get par parameters world2L0")
		.def("get_par_Ln2EE", &thunder_robot::get_par_Ln2EE, "Get par parameters Ln2EE")
		.def("load_par_REG", &thunder_robot::load_par_REG, "Load par parameters REG from YAML file", py::arg("file_path"), py::arg("update_DYN") = true)
		.def("load_conf", &thunder_robot::load_conf, "Load configuration from YAML file", py::arg("file_path"), py::arg("update_REG") = true)
		.def("save_par_REG", &thunder_robot::save_par_REG, "Save par parameters REG to YAML file", py::arg("file_path"))
		.def("save_par_DYN", &thunder_robot::save_par_DYN, "Save inertial parameters DYN to YAML file", py::arg("file_path"))
		.def("save_par", &thunder_robot::save_par, "Save all parameters into file", py::arg("file_path"))
		.def("get_numJoints", &thunder_robot::get_numJoints, "Get number of joints")
		.def("get_numParDYN", &thunder_robot::get_numParDYN, "Get number of parameters per link")
		.def("get_numParREG", &thunder_robot::get_numParREG, "Get number of parameters")
/*#-GENERATED_PYTHON_BINDINGS-#*/
}
)";
		string bindings_import = "#include <pybind11/pybind11.h>\n#include <pybind11/eigen.h>\n";

		std::ifstream file_cpp(file_path_cpp); // open in reading mode
		if (!file_cpp.is_open()) {
			std::cerr << "error in file_cpp opening:" << file_path_cpp << endl;
			return 0;
		} else {
			std::stringstream buffer_cpp;
			buffer_cpp << file_cpp.rdbuf(); // read file_cpp on buffer_cpp
			string file_content_cpp = buffer_cpp.str(); // file_cpp as string

			file_cpp.close(); // close the file_cpp

			// - insert bindings_template - //
			replace_all(file_content_cpp, "/*#-OPTIONAL SPACE FOR PYTHON BINDINGS-#*/", bindings_template);
			replace_all(file_content_cpp, "/*OPTIONAL PYBIND11 INCLUDE POINT*/", bindings_import);

			// - overwrite file_cpp - //
			std::ofstream out_cpp(file_path_cpp);
			out_cpp << file_content_cpp;
			out_cpp.close();
		}
		return 1;
	}


} // namespace thunder_ns

#endif // LEGACY_GEN_H
