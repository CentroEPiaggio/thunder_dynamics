#ifndef THUNDER_UTILS
#define THUNDER_UTILS

#include <string>
#include <casadi/casadi.hpp>

namespace thunder_ns{

	class fun_obj{
		std::string name;
		std::string description;
		casadi::SX expr;
		std::vector<casadi::SX> args;
		casadi::Function fun;
	};

	fun_obj create_function(std::string name, std::string description, casadi::SX expr, std::vector<casadi::SX> args);

	int change_to_robot(const std::string from_robot, const std::string to_robot, int n_joints, const std::string file_path_h, const std::string file_path_cpp);

	void replace_all(std::string& str, const std::string& from_str, const std::string& to_str);

	casadi::SX hat(const casadi::SX& v);

	/* Function to transform casadi element to double, defined here*/
	static double mapFunction(const casadi::SXElem& elem) {return static_cast<double>(casadi::SXElem(elem));};

}

#endif