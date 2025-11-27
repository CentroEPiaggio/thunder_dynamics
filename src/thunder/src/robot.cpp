#include <filesystem>

#include "../include/robot.h"
#include "../include/kinematics.h"
#include "../include/dynamics.h"
#include "../include/regressors.h"
#include "../include/utils.h"
#include "../include/userDefined.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;
using casadi::SX;

namespace thunder_ns{

	DM Robot::get(string name){
		if (parameters.count(name)){					// parameter exists
			return parameters[name].get_value_resized();
		} else if (functions.count(name)){				// function exists
			vector<DM> result;
			// cout<<"name: "<<name<<endl;
			auto f_args = functions[name].args;
			int sz = f_args.size();
			// cout<<"f_args:"<<f_args<<", size: "<<sz<<endl;
			casadi::DMVector inputs(sz);
			// cout << "arg_names: ";
			int i=0;
			for (const auto& arg : f_args) {
				// cout << arg << ", ";
				inputs[i] = parameters[arg].get_value_resized();
				i++;
			}
			// cout<<"args: "<<inputs<<endl;
			casadi::Function fun = functions[name].fun;
			// cout<<"fun: "<<fun<<endl;
			functions[name].fun.call(inputs, result);
			// cout<<"result: "<<result<<endl;
			return DM::vertcat(result);
		} else {
			std::cerr << name + " not recognised" << endl;
			return DM::zeros(1,1);
		}
		
		// return result_num;
	}

	int Robot::set(string name, DM value){
		if (parameters.count(name)){
			if (value.size() == parameters[name].num.size()){			// substitute the entire vector
				parameters[name].num = value;
			} else if (value.size() < parameters[name].num.size()){		// substitute only the symbolic if the size match
				int size_symbolic = parameters[name].symb_size();
				if (value.size1() == size_symbolic){
					DM& num = parameters[name].num;
					int i=0;
					for (short x : parameters[name].is_symbolic) num(i) = (x)?value(i++):num(i);
				} else {
					std::cerr << "Size mismatch when setting parameter: " << name << endl;
					return 0;
				}
			} else {
				std::cerr << "Size mismatch when setting parameter: " << name << endl;
				return 0;
			}
		} else {
			std::cerr << "Parameter not found: " << name << endl;
			return 0;
		}
		return 1;
	}

	const par_obj Robot::get_par(string par){
		return parameters[par];
	}

	vector<fun_obj> Robot::get_functions(bool onlyNames) {
		vector<fun_obj> fun_vect;
		int sz = functions.size();
		fun_vect.resize(sz);
		int i=0;
		for (auto &f : functions){
			string name = f.first;
			fun_vect[i].name = f.first;
			fun_vect[i].description = f.second.description;
			fun_vect[i].args = f.second.args;
			fun_vect[i].out_size.resize(2);
			fun_vect[i].out_size[0] = model[name].size1();
			fun_vect[i].out_size[1] = model[name].size2();
			if (!onlyNames){
				fun_vect[i].expr = model[name];
				fun_vect[i].fun = f.second.fun;
			}
			i++;
		}
		return fun_vect;
	}

	int Robot::load_par(string par_file, vector<string> par_list){
		try {
			// load yaml
			YAML::Node yamlFile = YAML::LoadFile(par_file);
			if (par_list.size() == 0){
				for (const auto& node : yamlFile){
					string key = node.first.as<string>();
					if (parameters.count(key)){
						set(key, node.second.as<vector<double>>());
					}
				}
			} else {
				for (string key : par_list){
					if (parameters.count(key)){
						set(key, yamlFile[key].as<vector<double>>());
					} else {
						std::cerr << "Parameter does not exist: " << key << std::endl;
					}
				}
			}
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while loading parameters: " << e.what() << std::endl;
			return 0;
		}
		return 1;
	}

	int Robot::save_par(string par_file, vector<string> par_list){
		try {
			YAML::Emitter emitter;
			emitter.SetIndent(2);
			emitter.SetSeqFormat(YAML::Flow);

			YAML::Node yamlFile;

			if (par_list.size() == 0){
				for (auto& par : parameters){
					string par_name = par.first;
					vector<double> vect_std = par.second.get_value_resized().get_elements();
					yamlFile[par.first] = vect_std;
				}
			} else {
				for (auto& par : par_list){
					// YAML::Node par_node;
					// par_node[par] = args[par];
					// emitter << par_node << YAML::Newline;
					// yamlFile[par] = args[par];

					// std::cout << par + "_sx: " << args[par] << endl;
					// Eigen::VectorXd vect_eig = get_arg(par);
					// std::cout << par + "_eig: " << vect_eig << endl;
					if (parameters.count(par)){
						vector<double> vect_std = parameters[par].get_value_resized().get_elements();
						yamlFile[par] = vect_std;
					} else {
						std::cerr << "Parameter does not exist: " << par << std::endl;
					}
				}
			}

			emitter << yamlFile << YAML::Newline;

			std::ofstream fout(par_file);
			fout << emitter.c_str();
			fout.close();
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while generating YAML: " << e.what() << std::endl;
			return 0;
		}
		return 1;
	}

	int Robot::update_inertial_DYN(){
		int numJoints = this->get<int>("numJoints");
		const int STD_PAR_LINK = this->get<const int>("STD_PAR_LINK");
		DM& par_REG = parameters["par_REG"].num;
		DM& par_DYN = parameters["par_REG"].num;
		for (int i=0; i<numJoints; i++){
			casadi::Slice p_idx(STD_PAR_LINK*i,STD_PAR_LINK*(i+1));
			DM p_reg(par_REG(p_idx));
			DM mass = p_reg(0);
			DM CoM = p_reg(casadi::Slice(1,4))/mass;
			DM I_tmp = mass * DM::mtimes(hat(CoM).T(), hat(CoM));
			DM I_reg = p_reg(casadi::Slice(4,10));
			DM I_tmp_v = DM::vertcat({I_tmp(0,0), I_tmp(0,1), I_tmp(0,2), I_tmp(1,1), I_tmp(1,2), I_tmp(2,2)});
			DM I = I_reg - I_tmp_v;
			par_DYN(p_idx) = DM::vertcat({mass, CoM, I});
		}
		return 1;
	}

	int Robot::update_inertial_REG(){
		int numJoints = this->get<int>("numJoints");
		const int STD_PAR_LINK = this->get<const int>("STD_PAR_LINK");
		DM& par_DYN = parameters["par_DYN"].num;
		DM& par_REG = parameters["par_REG"].num;
		for (int i=0; i<numJoints; i++){
			casadi::Slice p_idx(STD_PAR_LINK*i,STD_PAR_LINK*(i+1));
			DM p_dyn(par_DYN(p_idx));
			DM mass = p_dyn(0);
			DM mCoM = mass*p_dyn(casadi::Slice(1,4));
			DM I_tmp = DM::mtimes(hat(mCoM).T(), hat(mCoM))/mass;
			DM I_dyn = p_dyn(casadi::Slice(4,10));
			DM I_tmp_v = DM::vertcat({I_tmp(0,0), I_tmp(0,1), I_tmp(0,2), I_tmp(1,1), I_tmp(1,2), I_tmp(2,2)});
			DM I = I_dyn + I_tmp_v;
			par_REG(p_idx) = DM::vertcat({mass, mCoM, I});
		}
		return 1;
	}

	int Robot::add_variable(string v_name, SX symb, vector<double> num, vector<short> is_symbolic, string descr, bool overwrite){
		int ret = add_parameter(v_name, symb, num, is_symbolic, descr, overwrite);
		return ret;
	}

	int Robot::add_parameter(string p_name, SX symb, vector<double> num, vector<short> is_symbolic, string descr, bool overwrite){
		if ((!overwrite) && parameters.count(p_name)){
			// key already exists
			return 0;
		} else {
			par_obj param;
			param.name = p_name;
			param.description = descr;
			param.symb = symb;
			int size = symb.size1()*symb.size2();
			// param.size = size;

			if (num.size()!=size){
				std::cerr << "Error dimension of numeric SX: " << std::endl;
				return 0;
			} else {
				param.num = num;
			}

			if (is_symbolic.size()==size){ 				// normal initialization
				param.is_symbolic = is_symbolic;
			}else if (is_symbolic.size() == 1) { 		// one value initialization
				param.is_symbolic.resize(size);
				for (int i=0; i<size; i++){
					param.is_symbolic[i] = is_symbolic[0];
				}
			} else {							
				std::cerr << "Error dimension of symbolic vector: " << p_name << std::endl;
				return 0;
			}

			// add to parameters map
			parameters[p_name] = param;
			model[p_name] = param.get_model();
		}
		return 1;
	}

	int Robot::add_function(string f_name, casadi::SX expr, vector<string> args_raw, string descr, bool overwrite){
		if ((!overwrite) && functions.count(f_name)){
			std::cerr << "Function already exist! set flag for overwrite " << std::endl;
			return 0;
		} else {
			// - substitute expressions in args_raw with parameters list - //
			vector<string> args = {};
			for (auto& arg_raw : args_raw){
				if (parameters.count(arg_raw)) args.push_back(arg_raw);
				else if (functions.count(arg_raw)){
					for (auto arg : functions[arg_raw].args) args.push_back(arg);
				} else {
					std::cerr << "Arg not present in parameters nor functions: " << arg_raw << std::endl;
					return 0;
				}
			}
			// - delete repetitions - //
			int new_size = 0;
			bool new_arg = false;
			for (auto& arg : args){
				// check if already present
				new_arg = true;
				for (int i=0; i<new_size; i++){
					if (arg == args[i]){
						new_arg = false;
						break;
					}
				}
				// add parameter
				if (new_arg) args[new_size++] = arg;
			}
			args.resize(new_size);
			// - delete non-symbolic arguments - //
			vector<string> arg_list = {};
			for (auto& par : args){
				bool is_symb = false;
				for (short v : parameters[par].is_symbolic){
					if (v) is_symb = true;
				}
				if (is_symb){
					arg_list.push_back(par);
				}
			}

			// - creating fun object
			fun_obj fun_struct;
			model[f_name] = expr;
			fun_struct.args = arg_list;
			fun_struct.description = descr;

			// - only symbolic parameters as arguments - //
			casadi::SXVector inputs(arg_list.size());
			int arg_index=0;
			for (const auto& arg : arg_list) {
				// - resize parameters - //
				// std::cout << "fun: " << f_name << std::endl;
				// std::cout << "arg: " << arg << std::endl;
				vector<short>& symb_flag = parameters[arg].is_symbolic;
				vector<casadi::SX> par_symb;
				casadi::SX& par_model = model[arg];
				// cout << "model[arg]: " << par_model << endl;
				// cout << "symb_flag: " << symb_flag << endl;
				int sz_original = par_model.size().first;
				int sz = 0;
				for (int i=0; i<sz_original; i++){
					if (symb_flag[i]){
						par_symb.push_back(par_model(i));
						sz++;
					}
				}
				casadi::SX par_symb_new = casadi::SX::vertcat(par_symb);
				// cout << "par_symb_new: " << par_symb_new << endl;

				inputs[arg_index] = par_symb_new;
				arg_index++;
			}

			casadi::Function fun(robotName+"_"+f_name+"_fun", inputs, {densify(expr)});
			// cout<<"fun: "<<fun<<endl;
			fun_struct.fun = fun;
			functions[f_name] = fun_struct;
		}

		return 1;
	}

	void Robot::generate_library(const string& savePath, const string& name_file, const bool SAVE_CASADI){
		// Options for c-code auto generation
		casadi::Dict opts = casadi::Dict();
		opts["cpp"] = true;
		opts["with_header"] = true;
		
		// generate functions in c code
		casadi::CodeGenerator myCodeGen = casadi::CodeGenerator(name_file, opts);
		// cout<<"casadi_fun: "<<casadi_fun<<endl;

		for (const auto& f : functions) {
			myCodeGen.add(f.second.fun);
			// cout<<"f_name: "<<f.first<<endl;
			// cout<<"fun: "<<f.second<<endl<<endl;
		}
		myCodeGen.generate(savePath);

		if(SAVE_CASADI){
			// Create directory
			try {
				std::filesystem::create_directory(savePath + "/casadi_functions");
			} catch(std::exception & e){
				std::cout<<"Problem creating directory casadi_functions/"<<std::endl;
			}
			// Save CasADi functions
			for (const auto& f : functions) {
				std::string function_file = savePath + "/casadi_functions/" + f.first + ".casadi";
				f.second.fun.save(function_file);
			}
		}
	}

}