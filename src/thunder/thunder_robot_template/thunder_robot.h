#ifndef THUNDERROBOT_H
#define THUNDERROBOT_H

#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>


/* Class thunder_robot 
Useful functions for adaptive control with Slotine-Li regressor
Usable only for generated code from casadi library! */
class thunder_robot{

	private:
		/* Number of joints */
		int n_joints;
		int n_par_link;
		/* Joints' variables */
		Eigen::VectorXd q, dq, dqr, ddqr, par_REG, par_DYN;

		void update_inertial_DYN();
		void update_inertial_REG();

		struct LinkProp {
			double mass;
			std::vector<double> parI = std::vector<double>(6); // Inertia in the order xx, xy, xz, yy, yz, zz
			std::vector<double> xyz = std::vector<double>(3); // Origin xyz as std::vector
			std::vector<double> rpy = std::vector<double>(3);
			std::string name;
		};

		struct urdf2dh_T{
			std::vector<double> xyz;
			std::vector<double> rpy;
		};

		void fillInertialYaml(int num_joints, YAML::Emitter &emitter_, std::vector<LinkProp> &links_prop_, std::vector<std::string> keys_);
		// void transformBodyInertial(std::vector<double> d_i, std::vector<double> rpy_i, const LinkProp body_urdf, LinkProp &body);
		// void mergeBodyInertial(const LinkProp body1, const LinkProp body2, LinkProp &newBody);

		Eigen::Matrix3d rpyRot(const std::vector<double> rpy);
		Eigen::Matrix3d createI(const std::vector<double> parI);
		Eigen::Matrix3d hat(const Eigen::Vector3d v);
		
	public:
		/* Empty constructor, initialization inside */
		thunder_robot();
		/* Constructor to init variables*/
		// thunder_robot(const int);
		/* Destructor*/
		~thunder_robot() = default;
		
		/* Resize variables */
		void resizeVariables();
		/* Init variables from number of joints*/
		// void init(const int);
		/* Set q, dq, dqr, ddqr, to compute Regressor and update state*/
		void setArguments(const Eigen::VectorXd& q_, const Eigen::VectorXd& dq_, const Eigen::VectorXd& dqr_, const Eigen::VectorXd& ddqr_);
		void set_q(const Eigen::VectorXd& q_);
		void set_dq(const Eigen::VectorXd& dq_);
		void set_dqr(const Eigen::VectorXd& dqr_);
		void set_ddqr(const Eigen::VectorXd& ddqr_);
		void set_inertial_REG(const Eigen::VectorXd& par_);
		void set_inertial_DYN(const Eigen::VectorXd& par_);
		Eigen::VectorXd get_inertial_REG();
		Eigen::VectorXd get_inertial_DYN();

		void load_inertial_REG(std::string);
		void save_inertial_REG(std::string);
		void load_inertial_DYN(std::string);
		void save_inertial_DYN(std::string);

		int get_numJoints();
		int get_numParLink();
		int get_numParams();

		// ----- generated functions ----- //
		/*#-FUNCTIONS_H-#*/
};

#endif