#ifndef THUNDER_UTILS
#define THUNDER_UTILS

#include <string>
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>

namespace thunder_ns{

	class Robot;

	typedef struct par_obj{
		std::string name;
		std::string description;
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
		std::string name;
		std::string description;
		std::vector<std::string> args;
		std::vector<int> out_size;
		casadi::SX expr;
		casadi::Function fun;
	}fun_obj;

	typedef struct urdf2dh_T{
		std::vector<double> xyz;
		std::vector<double> rpy;
	}urdf2dh_T;

	void replace_all(std::string& str, const std::string& from_str, const std::string& to_str);
	std::string get_ret_type(const fun_obj fun);

	casadi::SX hat(const casadi::SX& v);
	Eigen::Matrix3d hat(const Eigen::Vector3d& v);

}

#endif