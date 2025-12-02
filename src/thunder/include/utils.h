#ifndef THUNDER_UTILS
#define THUNDER_UTILS

#include <string>
#include <any>
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>

using std::string;

namespace thunder_ns{

	class Robot;

	class Property{
		public:
		std::string name;
		std::string description;
		std::any value;
		std::string type_str;

		template<class T> std::string get_vector_str(const string type, const std::vector<T>& vec){
			string res = "{";
			if (vec.size() > 0){
				std::any val = vec[0];
				res.append("\"" + get_value_str(type, val) + "\"");
				for (size_t i = 1; i<vec.size(); i++){
					val = vec[i];
					res.append(", \"" + get_value_str(type, val) + "\"");
				}
			}
			res.append("}");
			return res;
		}

		std::string get_value_str(){return get_value_str(type_str, value);}

		std::string get_value_str(string type, std::any val){	// would you like to use map of callables? not for now!
			if (type == "short") return std::to_string(std::any_cast<short>(val));
			else if (type == "int") return std::to_string(std::any_cast<int>(val));
			else if (type == "long") return std::to_string(std::any_cast<long>(val));
			else if (type == "float") return std::to_string(std::any_cast<float>(val));
			else if (type == "double") return std::to_string(std::any_cast<double>(val));
			else if (type == "bool") return string((std::any_cast<bool>(val))?"true":"false");
			else if ((type == "string") || (type == "std::string")) return std::any_cast<string>(val);
			else if (type.find("vector<") != std::string::npos) {

				// - identify the vector type - //
				int start_idx = type.find("<") + 1;
				int end_idx = type.find(">");
				std::string elem_type = type.substr(start_idx, end_idx - start_idx);

				// - cast the vector - //
				string vec_str;
				if (elem_type == "short") vec_str = get_vector_str<short>("short",std::any_cast<vector<short>>(val));
				else if (elem_type == "int") vec_str = get_vector_str<int>("int", std::any_cast<vector<int>>(val));
				else if (elem_type == "long") vec_str = get_vector_str<long>("long", std::any_cast<vector<long>>(val));
				else if (elem_type == "float") vec_str = get_vector_str<float>("float", std::any_cast<vector<float>>(val));
				else if (elem_type == "double") vec_str = get_vector_str<double>("double", std::any_cast<vector<double>>(val));
				else if (elem_type == "bool") vec_str = get_vector_str<bool>("bool", std::any_cast<vector<bool>>(val));
				else if ((elem_type == "string") || (elem_type == "std::string")) vec_str = get_vector_str<string>("string", std::any_cast<vector<string>>(val));

				return vec_str;
			} else throw std::invalid_argument("Unsupported type in get_value_str");
		}
	};

	typedef struct par_obj{
		string name;
		string description;
		std::vector<short> is_symbolic;
		casadi::SX symb;
		casadi::DM num;
		int size(){
			return num.size1()*num.size2();
		}
		int symb_size(){
			int sz = 0;
			for (short x : is_symbolic) sz = (x)?sz+1:sz;
			return sz;
		}
		casadi::DM get_value_resized(){	// returns DM containing the current value of symbolic parameters
			int new_size = 0;
			int sz = size();
			casadi::DM ret(sz,1);
			for (int i=0; i<sz; i++){
				if (is_symbolic[i]) ret(new_size++) = num(i);
			}
			ret.resize(new_size,1);
			return ret;
		}
		casadi::SX get_symb_resized(){	// returns SX containing only the simbolic parameters
			int new_size = 0;
			int sz = size();
			casadi::SX ret(sz,1);
			for (int i=0; i<sz; i++){
				if (is_symbolic[i]) ret(new_size++) = symb(i);
			}
			ret.resize(new_size,1);
			return ret;
		}
		casadi::SX get_model(){			// returns SX containing symbols and numbers
			int sz = size();
			casadi::SX ret(sz,1);
			for (int i=0; i<sz; i++){
				ret(i) = (is_symbolic[i]) ? symb(i) : casadi::SX(num(i));
			}
			return ret;
		}
	}par_obj;

	typedef struct fun_obj{
		string name;
		string description;
		std::vector<string> args;
		std::vector<int> out_size;
		casadi::SX expr;
		casadi::Function fun;
	}fun_obj;

	void replace_all(string& str, const string& from_str, const string& to_str);
	string get_ret_type(const fun_obj fun);

	casadi::SX hat(const casadi::SX& v);
	Eigen::Matrix3d hat(const Eigen::Vector3d& v);

}

#endif