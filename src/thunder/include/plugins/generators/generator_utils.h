#ifndef GEN_UTILS_H
#define GEN_UTILS_H

#include <vector>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

#include "../../robot.h"
#include "../../utils.h"

using std::endl;
using std::string;
using std::vector;

namespace thunder_ns {

int create_thunder_robot(const string robot_name, Robot& robot, const string file_h, const string file_cpp, const bool gen_python);



// --- CHANGE_TO_ROBOT --- //
	int create_thunder_robot(const string robot_name, Robot& robot, const string file_h, const string file_cpp, const bool gen_python){
		
		// --- get everything from robot --- //
		string robotName = robot.robotName;
		int n_joints = robot.get<int>("numJoints");
		int Dl_order = robot.get<int>("Dl_order");
		bool ELASTIC = robot.get<bool>("ELASTIC");

		int numElasticJoints = 0;
		std::vector<short> isElasticJoint = {0};
		int K_order = 0;
		int D_order = 0;
		int Dm_order = 0;
		if (ELASTIC) {
			numElasticJoints = robot.get<int>("numElasticJoints");
			isElasticJoint = robot.get<vector<short>>("isElasticJoint");
			K_order = robot.get<int>("K_order");
			D_order = robot.get<int>("D_order");
			Dm_order = robot.get<int>("Dm_order");
		}
		
		// int STD_PAR_LINK = robot.STD_PAR_LINK;
		const std::map<string, Property>& properties = robot.properties;
		const std::map<string, par_obj>& parameters = robot.parameters;
		const std::map<string, fun_obj>& functions = robot.functions;
		// std::vector<fun_obj> functions = robot.get_functions();

		// -------------------------------- //
		// --- Create thunder_<robot>.h --- //
		// -------------------------------- //

		string file_content_h = "";

		// - generate headers - //
		file_content_h.append("#ifndef Thunder_" + robot_name + "_H\n");
		file_content_h.append("#define Thunder_" + robot_name + "_H\n\n");
		file_content_h.append("
			#include <iostream>\n
			#include <iostream>\n
			#include <string>\n
			#include <cmath>\n
			#include <eigen3/Eigen/Dense>\n
			#include <yaml-cpp/yaml.h>\n
			#include <fstream>\n\n");
		
		// - add namespace of common type classes - //
		file_content_h.append("
			using std::string;\n
			using std::vector;\n
			using std::map;\n
			using Eigen::MatrixXd;\n
			using Eigen::VectorXd;\n
			using Eigen::Matrix;\n
			using Eigen::Vector;\n\n");
		
		// - class name - //
		file_content_h.append("
			class thunder_" + robot_name + "{\n
				\tpublic:
				\t\tstring robotName = " + robot_name + ";\n");

		// - add properties - //
		string properties_str = "";
		for (auto& prop : properties){
			properties_str.append("
				\t\t" + prop.second.type_str + " " + prop.second.name + " = " + prop.second.get_value_str() + ";\n");
		}
		file_content_h.append(properties_str + "\n");

		// - add parameters - //
		string parameters_str = "";

		// ---- Continue from here !!!!--------------------------------------------------

		// - insert functions - //
		string functions_string = "\n";
		for (int i=0; i<functions.size(); i++){
			functions_string.append("\t\t// - " + functions[i].description + " - //\n");
			functions_string.append("\t\t"+get_ret_type(functions[i])+" get_" + functions[i].name + "();\n\n");
		}
		replace_all(file_content_h, "/*#-FUNCTIONS_H-#*/", functions_string);

		// - Save thunder_<robot>.h - //
		std::ofstream out_h(file_h);
		out_h << file_content_h;
		out_h.close();





		// --- file .cpp --- //
		if(gen_python){
			// - add bindings template - //
			add_bindings_template(file_cpp);
		}

		std::ifstream file_cpp(file_cpp); // open in reading mode
		if (!file_cpp.is_open()) {
			std::cerr << "error in file_cpp opening:" << file_cpp << endl;
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
			std::ofstream out_cpp(file_cpp);
			out_cpp << file_content_cpp;
			out_cpp.close();
		}

		return 1;
	}

}

#endif