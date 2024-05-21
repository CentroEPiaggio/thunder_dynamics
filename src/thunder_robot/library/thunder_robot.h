#ifndef THUNDERROBOT_H
#define THUNDERROBOT_H

#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace thunder_ns{

	/* Class thunder_robot 
	Useful functions for adaptive control with Slotine-Li regressor
	Usable only for generated code from casadi library! */
	class thunder_robot{

		private:
			/* Number of joints */
			int num_joints;
			/* Joints' variables */
			Eigen::VectorXd q, dq, dqr, ddqr, param_REG, param_DYN;
			/* Output of generated code */
			Eigen::MatrixXd kin_gen, jac_gen, dotJac_gen, pinvJac_gen, pinvJacPos_gen, dotPinvJac_gen, dotPinvJacPos_gen,
				reg_gen, mass_gen, coriolis_gen, gravity_gen;
			/* Compute generated code */
			void computeKin_gen(), computeJac_gen(), computeDotJac_gen(), computePinvJac_gen(), computeDotPinvJac_gen(), 
				computeReg_gen(), computeMass_gen(), computeCoriolis_gen(), computeGravity_gen(), computePinvJacPos_gen(), computeDotPinvJacPos_gen();

			void update_inertial_DYN();
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
			void set_inertial_REG(const Eigen::VectorXd& param_);
			Eigen::VectorXd get_inertial_REG();
			Eigen::VectorXd get_inertial_DYN();

			void load_inertial_REG(std::string);
			void save_inertial_REG(std::string);

			int get_numJoints();
			int get_numParams();
			
			// /* Set q, dq, param, to compute M C G */
			// void setArguments(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&);
			// /* Set q, dq, to compute pseudo-inverse of jacobian and derivate of pseudo-inverse jacobian */
			// void setArguments(const Eigen::VectorXd&, const Eigen::VectorXd&);
			// /* Set q to compute forward kinematic */
			// void setArguments(const Eigen::VectorXd&);

			/* Get regressor matrix */
			Eigen::MatrixXd getReg();
			/* Get regressor matrix */
			Eigen::MatrixXd getMass();
			/* Get regressor matrix */
			Eigen::MatrixXd getCoriolis();
			/* Get regressor matrix */
			Eigen::MatrixXd getGravity();
			/* Get jacobian matrix */
			Eigen::MatrixXd getJac();
			/* Get derivative of jacobian matrix */
			Eigen::MatrixXd getDotJac();
			/* Get pseudo-inverse jacobian matrix */
			Eigen::MatrixXd getPinvJac();
			/* Get derivative of pseudo-inverse jacobian matrix only position */
			Eigen::MatrixXd getPinvJacPos();
			/* Get derivative of pseudo-inverse jacobian matrix */
			Eigen::MatrixXd getDotPinvJac();
			/* Get derivative of pseudo-inverse jacobian matrix only position */
			Eigen::MatrixXd getDotPinvJacPos();
			/* Get regressor matrix */
			Eigen::MatrixXd getKin();

			// Other functions
		private:
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
	};

}
#endif