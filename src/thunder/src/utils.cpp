#include "utils.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include "robot.h"

using namespace std;

namespace thunder_ns{

	void transformBodyInertial(std::vector<double> d_i, std::vector<double> rpy_i, const LinkProp body_urdf, LinkProp &body){
		Eigen::Vector3d OuGi; 
		Eigen::Vector3d OiGi;
		Eigen::Vector3d dist_i;
		Eigen::Matrix3d Riu = rpyRot(rpy_i);
		Eigen::Matrix3d Rub = rpyRot(body_urdf.rpy);
		Eigen::Matrix3d Rib = Riu*Rub;

		Eigen::Matrix3d IGi_B = createI(body_urdf.parI);
		Eigen::Matrix3d IOi_i;
		Eigen::Matrix3d d_hat;

		OuGi << body_urdf.xyz[0], body_urdf.xyz[1],body_urdf.xyz[2]; 
		dist_i << d_i[0], d_i[1], d_i[2];
		OiGi = dist_i + Riu*OuGi;
		d_hat = hat(OiGi);

		IOi_i = Rib*IGi_B*Rib.transpose();

		body.mass = body_urdf.mass;
		body.xyz = {OiGi(0),OiGi(1),OiGi(2)};
		body.parI = {IOi_i(0,0),IOi_i(0,1),IOi_i(0,2),IOi_i(1,1),IOi_i(1,2),IOi_i(2,2)};
		body.name = body_urdf.name;
	}

	void mergeBodyInertial(const LinkProp body1, const LinkProp body2, LinkProp &newBody){
		Eigen::Vector3d G1Gnew;
		Eigen::Vector3d G2Gnew;
		Eigen::Vector3d O1G1;
		Eigen::Vector3d O2G2;
		Eigen::Vector3d newCoM;        
		Eigen::Matrix3d d_hat1;
		Eigen::Matrix3d d_hat2;
		Eigen::Matrix3d newI;
		Eigen::Matrix3d IG1 = createI(body1.parI);
		Eigen::Matrix3d IG2 = createI(body2.parI);
		
		O1G1 << body1.xyz[0], body1.xyz[1], body1.xyz[2]; 
		O2G2 << body2.xyz[0], body2.xyz[1], body2.xyz[2]; 
		
		newCoM = (body1.mass*O1G1 + body2.mass*O2G2)/(body1.mass + body2.mass);

		G1Gnew = newCoM-O1G1;
		G2Gnew = newCoM-O2G2;
		
		d_hat1 = hat(G1Gnew);
		d_hat2 = hat(G2Gnew);

		newI = IG1 + body1.mass*d_hat1*d_hat1.transpose() + IG2 + body2.mass*d_hat2*d_hat2.transpose();

		newBody.mass = body1.mass + body2.mass;
		newBody.xyz = {newCoM(0),newCoM(1),newCoM(2)};
		newBody.parI = {newI(0,0),newI(0,1),newI(0,2),newI(1,1),newI(1,2),newI(2,2)};
	}

	Eigen::Matrix3d rpyRot(const std::vector<double> rpy){
		Eigen::Matrix3d rotTr;
		
		double cy = cos(rpy[2]);
		double sy = sin(rpy[2]);
		double cp = cos(rpy[1]);
		double sp = sin(rpy[1]);
		double cr = cos(rpy[0]);
		double sr = sin(rpy[0]);

		//template R yaw-pitch-roll
		rotTr(0,0)=cy*cp;
		rotTr(0,1)=cy*sp*sr-sy*cr;
		rotTr(0,2)=cy*sp*cr-sy*sr;
		rotTr(1,0)=sy*cp;
		rotTr(1,1)=sy*sp*sr+cy*cr;
		rotTr(1,2)=sy*sp*cr-cy*sr;
		rotTr(2,0)=-sp;
		rotTr(2,1)=cp*sr;
		rotTr(2,2)=cp*cr;

		return rotTr;
	}
	
	Eigen::Matrix3d createI(const std::vector<double> parI){
		Eigen::Matrix3d I;
		I(0, 0) = parI[0];
		I(0, 1) = parI[1];
		I(0, 2) = parI[2];
		I(1, 0) = parI[1];
		I(1, 1) = parI[3];
		I(1, 2) = parI[4];
		I(2, 0) = parI[2];
		I(2, 1) = parI[4];
		I(2, 2) = parI[5];

		return I;
	}

	int change_to_robot(const string from_robot, const string to_robot, Robot& robot, const string file_path_h, const string file_path_cpp, const bool gen_python){
		
		// - get parameters from robot - //
		std::string robotName = robot.robotName;
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
			replace_all(file_content_h, "/*#-ROBOT_NAME-#*/", robotName);
			replace_all(file_content_h, "/*#-n_joints-#*/", to_string(n_joints));
			replace_all(file_content_h, "/*#-ELASTIC-#*/", to_string(ELASTIC));
			replace_all(file_content_h, "/*#-numElasticJoints-#*/", to_string(numElasticJoints));
			replace_all(file_content_h, "/*#-K_order-#*/", to_string(K_order));
			replace_all(file_content_h, "/*#-D_order-#*/", to_string(D_order));
			replace_all(file_content_h, "/*#-Dl_order-#*/", to_string(Dl_order));
			replace_all(file_content_h, "/*#-Dm_order-#*/", to_string(Dm_order));

			// elastic joints
			std::string eJ_str = "{" + to_string(isElasticJoint[0]);
			for (int i=1; i<n_joints; i++){
				eJ_str += ", " + to_string(isElasticJoint[i]);
			}
			eJ_str += "}";
			replace_all(file_content_h, "/*#-isElasticJoint-#*/", eJ_str);

			// - insert functions - //
			string functions_string = "\n";
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
		if(gen_python){
			// - add bindings template - //
			add_bindings_template(file_path_cpp);
		}

		ifstream file_cpp(file_path_cpp); // open in reading mode
		if (!file_cpp.is_open()) {
			cerr << "error in file_cpp opening:" << file_path_h << endl;
			return 0;
		} else {
			stringstream buffer_cpp;
			buffer_cpp << file_cpp.rdbuf(); // read file_cpp on buffer_cpp
			string file_content_cpp = buffer_cpp.str(); // file_cpp as string

			file_cpp.close(); // close the file_cpp

			// - substitute 'from_robot' wiht 'to_robot' - //
			replace_all(file_content_cpp, from_robot, to_robot);

			// - insert functions - //
			string functions_string = "\n";
			string functions_pybindings = "";

			for (int i=0; i<functions.size(); i++){
				std::string fun_name = "get_" + functions[i].name;
				std::string fun_name_gen = robotName + "_" + functions[i].name;
				std::vector<std::string> fun_args = functions[i].args;
				std::vector<int> out_size = functions[i].out_size;
				// // function arguments
				// std::string args_string = "(" + fun_args[0];
				// for (int j=1; j<fun_args.size(); j++){
				// 	args_string.append(", " + fun_args[j]);
				// }
				// other parts
				functions_string.append("// - " + functions[i].description + " - //\n");
				functions_string.append("Eigen::MatrixXd thunder_" + to_robot + "::" + fun_name + "(){\n");
				functions_string.append("\tEigen::MatrixXd out;\n");
				functions_string.append("\tout.resize("+to_string(out_size[0])+","+to_string(out_size[1])+");\n");
				functions_string.append("\tlong long p3[" + fun_name_gen + "_fun_SZ_IW];\n");
				functions_string.append("\tdouble p4[" + fun_name_gen + "_fun_SZ_W];\n");
				// inputs
				if (fun_args.size() == 0){
					functions_string.append("\tconst double* input_[] = {};\n");
				} else {
					functions_string.append("\tconst double* input_[] = {" + fun_args[0]+".data()");
					for (int j=1; j<fun_args.size(); j++){
						functions_string.append(", " + fun_args[j]+".data()");
					}
					functions_string.append("};\n");
				}
				// output
				functions_string.append("\tdouble* output_[] = {out.data()};\n");
				functions_string.append("\tint check = " + fun_name_gen + "_fun(input_, output_, p3, p4, 0);\n");
				functions_string.append("\treturn out;\n");
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
			ofstream out_cpp(file_path_cpp);
			out_cpp << file_content_cpp;
			out_cpp.close();
		}

		return 1;
	}

	int update_cmake(const string from_robot, const string to_robot, const string file_path){
		ifstream file_cmake(file_path); // open in reading mode
		if (!file_cmake.is_open()) {
			cerr << "error in CMakeLists.txt template opening:" << file_path << endl;
			return 0;
		} else {
			stringstream buffer_cmake;
			buffer_cmake << file_cmake.rdbuf(); // read file_cmake on buffer_cmake
			string file_content_cmake = buffer_cmake.str(); // file_cmake as string

			file_cmake.close(); // close the file_cmake

			// - substitute 'from_robot' wiht 'to_robot' - //
			replace_all(file_content_cmake, from_robot, to_robot);

			// - overwrite file_cmake - //
			ofstream out_cmake(file_path);
			out_cmake << file_content_cmake;
			out_cmake.close();
		}
		return 1;
	}

	int add_bindings_template(const string file_path_cpp){
		
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
		.def("set_DHtable", &thunder_robot::set_DHtable, "Set inertial parameters DHtable", py::arg("par"))
		.def("set_gravity", &thunder_robot::set_gravity, "Set inertial parameters gravity", py::arg("par"))
		.def("set_world2L0", &thunder_robot::set_world2L0, "Set inertial parameters world2L0", py::arg("par"))
		.def("set_Ln2EE", &thunder_robot::set_Ln2EE, "Set inertial parameters Ln2EE", py::arg("par"))
		.def("get_par_REG", &thunder_robot::get_par_REG, "Get par parameters REG")
		.def("get_par_DYN", &thunder_robot::get_par_DYN, "Get inertial parameters DYN")
		.def("get_par_K", &thunder_robot::get_par_K, "Get par parameters K")
		.def("get_par_D", &thunder_robot::get_par_D, "Get par parameters D")
		.def("get_par_Dm", &thunder_robot::get_par_Dm, "Get par parameters Dm")
		.def("get_par_Mm", &thunder_robot::get_par_Mm, "Get par parameters Mm")
		.def("get_par_Dl", &thunder_robot::get_par_Dl, "Get par parameters Dl")
		.def("get_DHtable", &thunder_robot::get_DHtable, "Get par parameters DHtable")
		.def("get_gravity", &thunder_robot::get_gravity, "Get par parameters gravity")
		.def("get_world2L0", &thunder_robot::get_world2L0, "Get par parameters world2L0")
		.def("get_Ln2EE", &thunder_robot::get_Ln2EE, "Get par parameters Ln2EE")
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

		ifstream file_cpp(file_path_cpp); // open in reading mode
		if (!file_cpp.is_open()) {
			cerr << "error in file_cpp opening:" << file_path_cpp << endl;
			return 0;
		} else {
			stringstream buffer_cpp;
			buffer_cpp << file_cpp.rdbuf(); // read file_cpp on buffer_cpp
			string file_content_cpp = buffer_cpp.str(); // file_cpp as string

			file_cpp.close(); // close the file_cpp

			// - insert bindings_template - //
			replace_all(file_content_cpp, "/*#-OPTIONAL SPACE FOR PYTHON BINDINGS-#*/", bindings_template);
			replace_all(file_content_cpp, "/*OPTIONAL PYBIND11 INCLUDE POINT*/", bindings_import);

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
