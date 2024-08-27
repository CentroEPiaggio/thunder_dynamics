#ifndef THUNDER_R3_H
#define THUNDER_R3_H

#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace thunder_ns{

	/* Class thunder_R3 
	Useful functions for adaptive control with Slotine-Li regressor
	Usable only for generated code from casadi library! */
	class thunder_R3{

		private:
			/* Number of joints */
			int num_joints;
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
			thunder_R3();
			/* Constructor to init variables*/
			// thunder_R3(const int);
			/* Destructor*/
			~thunder_R3() = default;
			
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
			int get_numParams();

			// ----- generated functions ----- //
			// - Manipulator Coriolis matrix - //
			Eigen::MatrixXd get_C();

			// - Classic formulation of the manipulator Coriolis matrix - //
			Eigen::MatrixXd get_C_std();

			// - Manipulator gravity terms - //
			Eigen::MatrixXd get_G();

			// - Jacobian of frame 0 - //
			Eigen::MatrixXd get_J_0();

			// - Jacobian of frame 1 - //
			Eigen::MatrixXd get_J_1();

			// - Jacobian of frame 2 - //
			Eigen::MatrixXd get_J_2();

			// - Jacobian of frame 3 - //
			Eigen::MatrixXd get_J_3();

			// - Jacobian of center of mass of link 0 - //
			Eigen::MatrixXd get_J_cm_0();

			// - Jacobian of center of mass of link 1 - //
			Eigen::MatrixXd get_J_cm_1();

			// - Jacobian of center of mass of link 2 - //
			Eigen::MatrixXd get_J_cm_2();

			// - Jacobian of the end-effector - //
			Eigen::MatrixXd get_J_ee();

			// - Time derivative of jacobian matrix - //
			Eigen::MatrixXd get_J_ee_dot();

			// - Pseudo-Inverse of jacobian matrix - //
			Eigen::MatrixXd get_J_ee_pinv();

			// - Manipulator mass matrix - //
			Eigen::MatrixXd get_M();

			// - relative transformation from frame base to frame 1 - //
			Eigen::MatrixXd get_T_0();

			// - absolute transformation from frame base to frame 1 - //
			Eigen::MatrixXd get_T_0_0();

			// - absolute transformation from frame base to frame 2 - //
			Eigen::MatrixXd get_T_0_1();

			// - absolute transformation from frame base to frame 3 - //
			Eigen::MatrixXd get_T_0_2();

			// - absolute transformation from frame base to end_effector - //
			Eigen::MatrixXd get_T_0_3();

			// - absolute transformation from frame 0 to end_effector - //
			Eigen::MatrixXd get_T_0_ee();

			// - relative transformation from frame1to frame 2 - //
			Eigen::MatrixXd get_T_1();

			// - relative transformation from frame2to frame 3 - //
			Eigen::MatrixXd get_T_2();

			// - Manipulator regressor matrix - //
			Eigen::MatrixXd get_Yr();

			// - Regressor matrix of term C*dqr - //
			Eigen::MatrixXd get_reg_C();

			// - Regressor matrix of term G - //
			Eigen::MatrixXd get_reg_G();

			// - Regressor matrix of term M*ddqr - //
			Eigen::MatrixXd get_reg_M();


	};

}
#endif