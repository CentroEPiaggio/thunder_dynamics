#ifndef THUNDER_UTILS
#define THUNDER_UTILS

#include <string>
#include <any>
#include <limits>
#include <sstream>
#include <iomanip>
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>

using std::string;
using std::vector;

namespace thunder_ns{

	template <typename T>
	std::string to_full_precision_string(T value) {
		std::ostringstream oss;
		oss << std::setprecision(std::numeric_limits<T>::max_digits10) << value;
		return oss.str();
	}

	constexpr double EPSILON = 1e-15; // numerical resolution, below is zero

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
				res.append(get_value_str(type, val));
				for (size_t i = 1; i<vec.size(); i++){
					val = vec[i];
					res.append(", " + get_value_str(type, val));
				}
			}
			res.append("}");
			return res;
		}

		template<class T> std::string get_subVector_str(const string type, const std::vector<vector<T>>& vec){
			string res = "{";
			if (vec.size() > 0){
				auto val = vec[0];
				res.append(get_vector_str<T>(type, val));
				for (size_t i = 1; i<vec.size(); i++){
					val = vec[i];
					res.append(", " + get_vector_str<T>(type, val));
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
			else if (type == "float") return to_full_precision_string<float>(std::any_cast<float>(val));
			else if (type == "double") return to_full_precision_string<double>(std::any_cast<double>(val));
			else if (type == "bool") return string((std::any_cast<bool>(val))?"true":"false");
			else if ((type == "string") || (type == "std::string")) return "\"" + std::any_cast<string>(val) + "\"";
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
				else if (elem_type.find("vector<") != std::string::npos) {
					int idx_i = type.find("vector<vector<")+14;
					int idx_f = type.find(">>");
					string subElem_type = type.substr(idx_i, idx_f-idx_i);
					if (subElem_type == "short") vec_str = get_subVector_str<short>("short",std::any_cast<vector<vector<short>>>(val));
					else if (subElem_type == "int") vec_str = get_subVector_str<int>("int", std::any_cast<vector<vector<int>>>(val));
					else if (subElem_type == "long") vec_str = get_subVector_str<long>("long", std::any_cast<vector<vector<long>>>(val));
					else if (subElem_type == "float") vec_str = get_subVector_str<float>("float", std::any_cast<vector<vector<float>>>(val));
					else if (subElem_type == "double") vec_str = get_subVector_str<double>("double", std::any_cast<vector<vector<double>>>(val));
					else if (subElem_type == "bool") vec_str = get_subVector_str<bool>("bool", std::any_cast<vector<vector<bool>>>(val));
					else if ((subElem_type == "string") || (subElem_type == "std::string")) vec_str = get_subVector_str<string>("string", std::any_cast<vector<vector<string>>>(val));
				}

				return vec_str;
			} else throw std::invalid_argument("Unsupported type in get_value_str");
		}
	};

	class Parameter{
		public:
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
			for (short x : is_symbolic) sz = (x) ? sz+1 : sz;
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
		std::string get_value_str(){
			casadi::DM val = get_value_resized();
			std::string res = "{}";
			if (symb_size() != 0){
				res = "{" + to_full_precision_string<double>(static_cast<double>(val(0)));
				for (int i=1; i<symb_size(); i++){
					res.append(", " + to_full_precision_string<double>(static_cast<double>(val(i))));
				}
				res.append("}");
			}
			
			return res;
		}
	};

	class FunArg{
		public:
		string name;
		casadi::SX value;
		int size() { return value.size1()*value.size2(); }
		FunArg(string name, casadi::SX value) : name(name), value(value) {}
	};

	class Function{
		public:
		string name;
		string description;
		std::vector<string> args;
		std::vector<FunArg> explicit_args;
		casadi::SX expr;
		casadi::Function fun;

		std::vector<long> get_out_size(){
			std::vector<long> out_size({expr.size1(), expr.size2()});
			return out_size;
		}

		std::string get_args_str(){
			if (args.size() != 0){
				std::string res = "{" + args[0];
				for (int i=1; i<args.size(); i++){
					res.append(", " + args[i]);
				}
				res.append("}");
				return res;
			} else return "{}";
		}

		// std::string get_explicit_args_str(){
		// 	if (explicit_args.size() != 0){
		// 		std::string res = "(" + explicit_args[0];
		// 		for (int i=1; i<explicit_args.size(); i++){
		// 			res.append(", " + explicit_args[i]);
		// 		}
		// 		res.append("}");
		// 		return res;
		// 	} else return "()";
		// }

		std::string get_ret_type_str(){
			std::vector<long> out_size = get_out_size();
			string ret_type = "Eigen::Matrix<double,"+std::to_string(out_size[0])+","+std::to_string(out_size[1])+">";
			return ret_type;
		}
		
	};

	void replace_all(string& str, const string& from_str, const string& to_str);

	casadi::SX hat(const casadi::SX& v);
	casadi::SX vect(const casadi::SX& S);
	casadi::SX ZIS(const casadi::SX& x, double tol = EPSILON);
	casadi::SX R_x(const casadi::SX& angle);
	casadi::SX R_y(const casadi::SX& angle);
	casadi::SX R_z(const casadi::SX& angle);
	casadi::SX R_aa(const casadi::SX& axis, const casadi::SX& angle);
	casadi::SX get_transform_rpy(casadi::SX frame_rpy);
	casadi::SX get_transform_ypr(casadi::SX frame_ypr);
	casadi::SX get_euler_rpy(casadi::SX T);
	
} // namespace thunder_ns

#endif