#ifndef THUNDER_UTILS
#define THUNDER_UTILS

#include <string>
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>
#include "FrameOffset.h"

namespace thunder_ns{

	class Robot;

	typedef struct Config{
		int numJoints;
		std::vector<std::string> jointsType;
		casadi::SX DHtable;
		FrameOffset base_frame;
		FrameOffset ee_frame;
		int Dl_order = 0;
		bool ELASTIC = false;
		int K_order = 0;
		int D_order = 0;
		int Dm_order = 0;
	}Config;

	typedef struct fun_obj{
		std::string name;
		std::string description;
		std::vector<std::string> args;
		std::vector<int> out_size;
		casadi::SX expr;
		casadi::Function fun;
	}fun_obj;

	typedef struct LinkProp {
		std::string name;
		double mass;
		std::vector<double> parI = std::vector<double>(6); // Inertia in the order xx, xy, xz, yy, yz, zz
		std::vector<double> xyz = std::vector<double>(3); // Origin xyz as std::vector
		std::vector<double> rpy = std::vector<double>(3);
	}LinkProp;


	typedef struct urdf2dh_T{
		std::vector<double> xyz;
		std::vector<double> rpy;
	}urdf2dh_T;

	void transformBodyInertial(std::vector<double> d_i, std::vector<double> rpy_i, const LinkProp body_urdf, LinkProp &body);
	void mergeBodyInertial(const LinkProp body1, const LinkProp body2, LinkProp &newBody);

	Eigen::Matrix3d rpyRot(const std::vector<double> rpy);
	Eigen::Matrix3d createI(const std::vector<double> parI);

	int change_to_robot(const std::string from_robot, const std::string to_robot, Robot& robot, const std::string file_path_h, const std::string file_path_cpp);
	int update_cmake(const std::string from_robot, const std::string to_robot, const std::string file_path);
	void replace_all(std::string& str, const std::string& from_str, const std::string& to_str);

	casadi::SX hat(const casadi::SX& v);
	Eigen::Matrix3d hat(const Eigen::Vector3d& v);

	/* Function to transform casadi element to double, defined here*/
	static double mapFunction(const casadi::SXElem& elem) {return static_cast<double>(casadi::SXElem(elem));};

}

#endif