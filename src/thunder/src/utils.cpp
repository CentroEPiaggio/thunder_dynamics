#include "utils.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include "robot.h"

using namespace std;

namespace thunder_ns{

	int change_to_robot(const string from_robot, const string to_robot, Robot& robot, const string file_path_h, const string file_path_cpp){
		
		// - get parameters from robot - //
		int n_joints = robot.get_numJoints();
		int numElasticJoints = robot.get_numElasticJoints();
		bool ELASTIC = robot.get_ELASTIC();
		int K_order = robot.get_K_order();
		int D_order = robot.get_D_order();
		int Dl_order = robot.get_Dl_order();
		int Dm_order = robot.get_Dm_order();
		// int numParDYN = robot.get_numParDYN();
		// int numParREG = robot.get_numParREG();
		// int numParELA = robot.get_numParELA();
		std::vector<int> isElasticJoint = robot.get_isElasticJoint();
		
		// int STD_PAR_LINK = robot.STD_PAR_LINK;
		std::vector<fun_obj> functions = robot.get_functions();

		// --- file .h --- //
		ifstream file_h(file_path_h); // open in reading mode
		if (!file_h.is_open()) {
			cerr << "error in file_h opening:" << file_path_h << endl;
			return 0;
		} else {
			stringstream buffer_h;
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
			replace_all(file_content_h, "/*#-n_joints-#*/", to_string(n_joints));
			replace_all(file_content_h, "/*#-ELASTIC-#*/", to_string(ELASTIC));
			replace_all(file_content_h, "/*#-numElasticJoints-#*/", to_string(numElasticJoints));
			replace_all(file_content_h, "/*#-K_order-#*/", to_string(K_order));
			replace_all(file_content_h, "/*#-D_order-#*/", to_string(D_order));
			replace_all(file_content_h, "/*#-Dl_order-#*/", to_string(Dl_order));
			replace_all(file_content_h, "/*#-Dm_order-#*/", to_string(Dm_order));
			// replace_all(file_content_h, "/*#-numParDYN-#*/", to_string(numParDYN));
			// replace_all(file_content_h, "/*#-numParREG-#*/", to_string(numParREG));
			// replace_all(file_content_h, "/*#-numParELA-#*/", to_string(numParELA));
			// elastic joints
			std::string eJ_str = "{" + to_string(isElasticJoint[0]);
			for (int i=0; i<n_joints; i++){
				eJ_str += ", " + to_string(isElasticJoint[i]);
			}
			eJ_str += "};";
			replace_all(file_content_h, "/*#-isElasticJoint-#*/", eJ_str);

			// - insert functions - //
			string functions_string = "";
			for (int i=0; i<functions.size(); i++){
				functions_string.append("\t\t// - " + functions[i].description + " - //\n");
				functions_string.append("\t\tEigen::MatrixXd get_" + functions[i].name + "();\n\n");
			}
			replace_all(file_content_h, "/*#-FUNCTIONS_H-#*/", functions_string);

			// - overwrite file_h - //
			ofstream out_h(file_path_h);
			out_h << file_content_h;
			out_h.close();
		}

		// --- file .cpp --- //
		ifstream file_cpp(file_path_cpp); // open in reading mode
		if (!file_cpp.is_open()) {
			cerr << "error in file_cpp opening:" << file_path_h << endl;
			return 0;
		} else {
			stringstream buffer_cpp;
			buffer_cpp << file_cpp.rdbuf(); // read file_cpp on buffer_cpp
			string file_content_cpp = buffer_cpp.str(); // file_cpp as string

			file_cpp.close(); // close the file_cpp

			// - change num_joints - //
			// size_t index_joints = file_content_cpp.find("N_JOINTS");
			// size_t index_semicolon = file_content_cpp.find(';', index_joints);
			// file_content_cpp.replace(index_joints, index_semicolon - index_joints, "N_JOINTS = " + to_string(n_joints));
			// replace_all(file_content_cpp, "/*#-N_JOINTS-#*/", to_string(n_joints));
			// replace_all(file_content_cpp, "/*#-N_PAR_LINK-#*/", to_string(STD_PAR_LINK));

			// - substitute 'from_robot' wiht 'to_robot' - //
			replace_all(file_content_cpp, from_robot, to_robot);

			// - insert functions - //
			string functions_string = "";
			for (int i=0; i<functions.size(); i++){
				std::string fun_name = functions[i].name;
				std::vector<std::string> fun_args = functions[i].args;
				std::vector<int> out_size = functions[i].out_size;
				functions_string.append("// - " + functions[i].description + " - //\n");
				functions_string.append("Eigen::MatrixXd thunder_" + to_robot + "::get_" + fun_name + "(){\n");
				functions_string.append("\tEigen::MatrixXd out;\n");
				functions_string.append("\tout.resize("+to_string(out_size[0])+","+to_string(out_size[1])+");\n");
				functions_string.append("\tlong long p3[" + fun_name + "_fun_SZ_IW];\n");
				functions_string.append("\tdouble p4[" + fun_name + "_fun_SZ_W];\n");
				// inputs
				functions_string.append("\tconst double* input_[] = {" + fun_args[0]+".data()");
				for (int j=1; j<fun_args.size(); j++){
					functions_string.append(", " + fun_args[j]+".data()");
				}
				functions_string.append("};\n");
				// output
				functions_string.append("\tdouble* output_[] = {out.data()};\n");
				functions_string.append("\tint check = " + fun_name + "_fun(input_, output_, p3, p4, 0);\n");
				functions_string.append("\treturn out;\n");
				functions_string.append("}\n\n");
			}
			replace_all(file_content_cpp, "/*#-FUNCTIONS_CPP-#*/", functions_string);

			// - overwrite file_cpp - //
			ofstream out_cpp(file_path_cpp);
			out_cpp << file_content_cpp;
			out_cpp.close();
		}

		return 1;
	}

	// string file_content_cpp = (string str, string from_str, string to_str){
	// 	// const int ROBOT_LEN = from_str.length();
	// 	// size_t index = str.find(from_str);
	// 	// cout<<"robot_len: "<<ROBOT_LEN<<endl;
	// 	// cout<<"initial index: "<<index<<endl;
		
	// 	// while ((index+ROBOT_LEN+1 <str.length()) && (index != -1)){
	// 	// 	str.replace(index, ROBOT_LEN, to_str);
	// 	// 	index += ROBOT_LEN;
	// 	// 	index = str.find(from_str, index);
	// 	// 	cout<<"index: "<<index<<endl;
	// 	// }
	// 	size_t start_pos;
	// 	while((start_pos = str.find(from_str, start_pos)) != string::npos) {
	// 		str.replace(start_pos, from_str.length(), to_str);
	// 		start_pos += to_str.length(); // Handles case where 'to_str' is a substring of 'from_str'
	// 	}
	// 	return str;
	// }

	// void file_content_cpp = (string &str, const string& from_str, const string& to_str){
	// 	size_t start_pos = 0;
	// 	start_pos = str.find(from_str, start_pos);
	// 	// cout<<"content_h: "<< str <<endl;
	// 	while((start_pos = str.find(from_str, start_pos)) != -1) {
	// 		cout<<"start_pos: "<<start_pos<<endl;
	// 		cout<<"from_str: "<<from_str<<", length: "<<from_str.length()<<endl;
	// 		cout<<"string: "<<str.substr(start_pos, from_str.length())<<endl;
	// 		str.replace(start_pos, from_str.length(), to_str);

	// 		start_pos += to_str.length();
	// 	}
	// 	// cout<<"content_h: "<< str <<endl;
	// }

	// string file_content_cpp = (string str, const string from_str, const string to_str){
	// 	size_t start_pos = 0;
	// 	while((start_pos = str.find(from_str, start_pos)) != string::npos) {
	// 		str.replace(start_pos, from_str.length(), to_str);
	// 		start_pos += to_str.length();
	// 	}
	// 	return str;
	// }

	// do not call replace often
	void replace_all(std::string& source, const std::string& from_str, const std::string& to_str){
		std::string newString;
		newString.reserve(source.length());  // avoids a few memory allocations

		std::string::size_type lastPos = 0;
		std::string::size_type findPos;

		while(std::string::npos != (findPos = source.find(from_str, lastPos))){
			newString.append(source, lastPos, findPos - lastPos);
			newString.append(to_str);
			lastPos = findPos + from_str.length();
		}

		// Care for the rest after last occurrence
		// newString += source.substr(lastPos);
		newString.append(source.substr(lastPos));

		source.swap(newString);
		// return newString;
	}

	casadi::SX hat(const casadi::SX& v) {
		
		casadi::SX skew = casadi::SX::zeros(3,3);
		
		skew(0, 1) = -v(2);
		skew(0, 2) = v(1);
		skew(1, 0) = v(2);
		skew(1, 2) = -v(0);
		skew(2, 0) = -v(1);
		skew(2, 1) = v(0);
		
		return skew;
	}

	Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
		
		Eigen::Matrix3d skew(3, 3);
		skew.setZero();
		
		skew(0, 1) = -v(2);
		skew(0, 2) = v(1);
		skew(1, 0) = v(2);
		skew(1, 2) = -v(0);
		skew(2, 0) = -v(1);
		skew(2, 1) = v(0);
		
		return skew;
	}

	// static double mapFunction(const casadi::SXElem& elem) {return static_cast<double>(casadi::SXElem(elem));};
}
