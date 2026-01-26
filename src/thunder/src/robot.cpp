#include <filesystem>

#include "../include/robot.h"
#include "../include/utils.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;
using casadi::SX;

namespace thunder_ns{

	DM Robot::get(string name, vector<DM> explicit_args){
		if (parameters.count(name)){					// parameter exists
			return parameters[name].get_value_resized();
		} else if (functions.count(name)){				// function exists
			vector<DM> result;
			// cout<<"name: "<<name<<endl;
			auto f_args = functions[name].args;
			int sz = f_args.size() + explicit_args.size();
			// cout<<"f_args:"<<f_args<<", size: "<<sz<<endl;
			casadi::DMVector inputs(sz);
			// cout << "arg_names: ";
			int i=0;
			for (const auto& arg : f_args) {
				// cout << arg << ", ";
				inputs[i] = parameters[arg].get_value_resized();
				i++;
			}
			for (const auto& arg : explicit_args) {
				// cout << arg << ", ";
				inputs[i] = explicit_args[i-f_args.size()];
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
	}

	SX Robot::get_model(string name, vector<SX> explicit_args){
		if (parameters.count(name)){					// parameter exists
			return parameters[name].get_model();
		} else if (functions.count(name)){				// function exists
			if (explicit_args.size() == 0) return functions[name].expr;
			else {
				// use the function to compute the expression adding the explicit args
				vector<SX> result;
				// cout<<"name: "<<name<<endl;
				auto f_args = functions[name].args;
				int sz = f_args.size() + explicit_args.size();
				// cout<<"f_args:"<<f_args<<", size: "<<sz<<endl;
				casadi::SXVector inputs(sz);
				// cout << "arg_names: ";
				int i=0;
				for (const auto& arg : f_args) {
					// cout << arg << ", ";
					inputs[i] = parameters[arg].get_symb_resized();
					i++;
				}
				for (const auto& arg : explicit_args) {
					// cout << arg << ", ";
					inputs[i] = explicit_args[i-f_args.size()];
					i++;
				}
				// cout<<"args: "<<inputs<<endl;
				casadi::Function fun = functions[name].fun;
				// cout<<"fun: "<<fun<<endl;
				functions[name].fun.call(inputs, result);
				// cout<<"result: "<<result<<endl;
				return SX::vertcat(result);
			}
		} else {
			std::cerr << name + " not recognised" << endl;
			return SX::zeros(1,1);
		}
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

	const vector<Property> Robot::get_properties(vector<string> prop_list) {
		vector<Property> prop_vect;
		int i=0;
		if (prop_list.size() != 0){
			prop_vect.resize(prop_list.size());
			for (string& p : prop_list){
				prop_vect[i] = properties[p];
				i++;
			}
		} else {
			prop_vect.resize(properties.size());
			for (auto &prop : properties){
				prop_vect[i] = prop.second;
				i++;
			}
		}
		return prop_vect;
	}

	const vector<Parameter> Robot::get_parameters(vector<string> par_list) {
		vector<Parameter> par_vect;
		int i=0;
		if (par_list.size() != 0){
			par_vect.resize(par_list.size());
			for (string& p : par_list){
				par_vect[i] = parameters[p];
				i++;
			}
		} else {
			par_vect.resize(parameters.size());
			for (auto &par : parameters){
				par_vect[i] = par.second;
				i++;
			}
		}
		return par_vect;
	}

	const vector<Function> Robot::get_functions(vector<string> fun_list) {
		vector<Function> fun_vect;
		int i=0;
		if (fun_list.size() != 0){
			fun_vect.resize(fun_list.size());
			for (string& f : fun_list){
				fun_vect[i] = functions[f];
				i++;
			}
		} else {
			fun_vect.resize(functions.size());
			for (auto &fun : functions){
				fun_vect[i] = fun.second;
				i++;
			}
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
					if (par.second.symb_size() != 0){
						string par_name = par.first;
						vector<double> vect_std = par.second.get_value_resized().get_elements();
						yamlFile[par.first] = vect_std;
					}
				}
			} else {
				for (auto& par : par_list){
					if (parameters.count(par)){
						if (parameters[par].symb_size() != 0){
							vector<double> vect_std = parameters[par].get_value_resized().get_elements();
							yamlFile[par] = vect_std;
						}
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

	int Robot::save_conf(string conf_file){
		try {
			YAML::Emitter emitter;
			emitter.SetIndent(2);
			emitter.SetSeqFormat(YAML::Flow);

			emitter << this->config_yaml << YAML::Newline;

			std::ofstream fout(conf_file);
			fout << emitter.c_str();
			fout.close();

		} catch (const YAML::Exception& e) {
			std::cerr << "Error while generating YAML: " << e.what() << std::endl;
			return 0;
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
			Parameter param;
			param.name = p_name;
			param.description = descr;
			param.symb = symb;
			int size = symb.size1()*symb.size2();

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
		}
		return 1;
	}

	int Robot::add_function(string f_name, casadi::SX expr, vector<string> args_raw, string descr, vector<FunArg> explicit_args, bool overwrite){
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
			Function function;
			function.name = f_name;
			function.expr = expr;
			function.args = arg_list;
			function.explicit_args = explicit_args;
			function.description = descr;

			// - only symbolic parameters as arguments - //
			casadi::SXVector inputs(arg_list.size() + explicit_args.size());
			int arg_index=0;
			for (const auto& arg : arg_list) {
				// - resize parameters - //
				// std::cout << "fun: " << f_name << std::endl;
				// std::cout << "arg: " << arg << std::endl;
				vector<short>& symb_flag = parameters[arg].is_symbolic;
				vector<casadi::SX> par_symb;
				casadi::SX par_model = parameters[arg].get_model();
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
			for (const auto& arg: explicit_args){
				inputs[arg_index] = arg.value;
				arg_index++;
			}

			casadi::Function fun(robotName+"_"+f_name+"_fun", inputs, {densify(expr)});
			// cout<<"fun: "<<fun<<endl;
			function.fun = fun;
			functions[f_name] = function;
		}

		return 1;
	}

}