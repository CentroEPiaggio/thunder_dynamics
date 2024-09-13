#ifndef THUNDER_UTILS
#define THUNDER_UTILS

#include <string>
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>
#include "FrameOffset.h"

namespace thunder_ns{

	class Robot;

	struct Config{
		int numJoints;
		std::vector<std::string> jointsType;
		Eigen::MatrixXd DHtable;
		FrameOffset base_frame;
		FrameOffset ee_frame;
		int Dl_order = 0;
		bool ELASTIC = false;
		int K_order = 0;
		int D_order = 0;
		int Dm_order = 0;
	};

	struct fun_obj{
		std::string name;
		std::string description;
		std::vector<std::string> args;
		std::vector<int> out_size;
		casadi::SX expr;
		casadi::Function fun;
	};

	// class fun_obj{
	// 	std::string name;
	// 	std::string description;
	// 	casadi::SX expr;
	// 	std::vector<casadi::SX> args;
	// 	casadi::Function fun;
	// };

	// fun_obj create_function(std::string name, std::string description, casadi::SX expr, std::vector<casadi::SX> args);

	int change_to_robot(const std::string from_robot, const std::string to_robot, Robot& robot, const std::string file_path_h, const std::string file_path_cpp);

	void replace_all(std::string& str, const std::string& from_str, const std::string& to_str);

	casadi::SX hat(const casadi::SX& v);
	Eigen::Matrix3d hat(const Eigen::Vector3d& v);

	/* Function to transform casadi element to double, defined here*/
	static double mapFunction(const casadi::SXElem& elem) {return static_cast<double>(casadi::SXElem(elem));};

}

#endif