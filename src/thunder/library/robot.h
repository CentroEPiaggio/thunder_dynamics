#ifndef ROBOT_H
#define ROBOT_H

#include <string>
// #include <map> // used for model and functions into Robot, like dictionaries
// #include <functional>
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>
#include "FrameOffset.h"
#include "utils.h"
// #include "CasadiObj.h"

namespace thunder_ns{

	// module class is a template for the components of the robot that compute functions
	// class Module{
	// 	std::vector<fun_obj> functions;
	// };
	constexpr int N_PAR_LINK = 10;
	
	// contain everything related to a robot, uses the other classes to obtain functions
	class Robot{
		private:
			// int N_functions;
			// std::vector<fun_obj> functions;
			void initVarsFuns();
			/* Create and initialize casadi function */
			// void init_casadi_functions();
			/* Update result of casadi functions */
			// void compute();

		protected:
			int _numJoints_;
			/* Joints' type string */
			std::string _jointsType_;
			/* Denavit-Hartenberg parameterization table */
			Eigen::MatrixXd _DHtable_;
			/* Frame offset between world-frame and link 0*/
			FrameOffset _world2L0_;
			/* Frame offset between end-effector and last link */
			FrameOffset _Ln2EE_;
			// int N_PAR_LINK;
			// casadi::SX gravity;
			// Input of casadi function //
			std::map<std::string, casadi::SX> args;
			std::map<std::string, casadi::Function> casadi_fun;
			std::map<std::string, std::vector<std::string>> fun_args;
			std::map<std::string, std::string> fun_descr;
			// Variable for joints angle //
			casadi::SX _q_, _dq_, _dqr_, _ddqr_;
			casadi::SX _par_KIN_, _par_DYN_, _par_REG_;
			casadi::SXVector _mass_vec_;
			casadi::SXVector _distCM_;
			casadi::SXVector _J_3x3_;
			/* vector of function objects */

			// std::vector<casadi::Function> casadi_functions;
			// std::vector<std::string> fun_names;
			/* Output of casadi function */
			// std::vector<casadi::SX> T_res, J_res;
			// std::vector<casadi::SX> kinematic_res, jacobian_res;
			// std::vector<casadi::SX> dotJacobian_res, pinvJacobian_res, pinvJacobianPos_res, dotPinvJacobian_res, dotPinvJacobianPos_res;
			// std::vector<casadi::SX> mass_res, coriolis_res, gravity_res;
			/* Casadi function */
			// casadi::Function dotJacobian_fun, pinvJacobian_fun, pinvJacobianPos_fun, dotPinvJacobian_fun, dotPinvJacobianPos_fun;
			// casadi::Function kinematic_fun, jacobian_fun;
			// casadi::Function mass_fun, coriolis_fun, gravity_fun;

			/* Damping coefficient for pseudo-inverse */
			// double _mu_;

		public:
			/* Init variables
			numJoints: number of joints
			DHTable: Denavit-Hartenberg table format [a alpha d theta]
			jtsType: is string of "R" and "P"
			base_frame: is used to set transformation between link0 and world_frame */
			Robot(const int numJoints, const std::string jointsType, const Eigen::MatrixXd& DHtable, 
				FrameOffset& base_frame, FrameOffset& ee_frame);
			/* Destructor */
			// ~Robot(){};

			// // - get functions - //
			// casadi::SX get_q_sym() {return _q;};
			// casadi::SX get_dq_sym() {return _dq;};
			// casadi::SX get_dqr_sym() {return _dqr;};
			// casadi::SX get_ddqr_sym() {return _ddqr;};
			// casadi::SX get_psrKIN_sym() {return _par_KIN;};
			// casadi::SX get_parDYN_sym() {return _par_DYN;};
			// casadi::SX get_parREG_sym() {return _par_REG;};

			/* Variable for joints in set arguments */
			int valid;
			std::map<std::string, casadi::SX> model;
			Eigen::MatrixXd get(std::string name);
			// get functions
			int get_numJoints();
			std::string get_jointsType();
			Eigen::MatrixXd get_DHTable();
			FrameOffset get_world2L0();
			FrameOffset get_Ln2EE();
			// int get_N_PAR_LINK();
			Eigen::VectorXd get_par_DYN();
			Eigen::VectorXd get_par_REG();
			// set functions
			int set_arg(std::string name, Eigen::VectorXd);
			int set_q(Eigen::VectorXd);
			int set_dq(Eigen::VectorXd);
			int set_dqr(Eigen::VectorXd);
			int set_ddqr(Eigen::VectorXd);
			int set_par_DYN(Eigen::VectorXd);
			int set_par_REG(Eigen::VectorXd);
			// std::map<std::string, Eigen::MatrixXd> get;
			// std::map<std::string, bool> valid;
			// Eigen::VectorXd q, dq, dqr, ddqr;
			// Eigen::VectorXd param_KIN, par_DYN, par_REG;
			// void update();
			// void generate_code(std::string& savePath);
			std::vector<std::string> getFunctionNames();
			std::vector<casadi::Function> getCasadiFunctions();
			int add_function(std::string name, casadi::SX expr, std::vector<std::string> f_args, std::string descr = "");
			void generate_library(const std::string& savePath, const std::string& name_file);

	};

}
#endif