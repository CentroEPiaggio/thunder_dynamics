#ifndef ROBOT_H
#define ROBOT_H

#include <string>
// #include <map> // used for model and functions into Robot, like dictionaries
// #include <functional>
#include <yaml-cpp/yaml.h>
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>
#include "FrameOffset.h"
#include "utils.h"

namespace thunder_ns{

	// module class is a template for the components of the robot that compute functions
	// class Module{
	// 	std::vector<fun_obj> functions;
	// };
	// constexpr int N_PAR_LINK = 10;

	// configuration struct
	
	typedef struct Config Config;
	typedef struct fun_obj fun_obj;
	// Config load_config(std::string file);
	Robot robot_from_file(std::string robot_name, std::string file, bool compute = 1);
	
	// contain everything related to a robot, uses the other classes to obtain functions
	class Robot{
		private:
			// int N_functions;
			// std::vector<fun_obj> functions;
			int parse_config();
			void initVarsFuns();
			/* Create and initialize casadi function */
			// void init_casadi_functions();
			/* Update result of casadi functions */
			// void compute();

		protected:
			int numJoints;
			int numElasticJoints = 0;
			// int PAR_DYN_LINK;
			// int PAR_REG_LINK;
			// int PAR_ELA_LINK;
			// int numParDYN;
			// int numParREG;
			// int numParELA;
			bool ELASTIC = false;
			// std::vector<int> numParLink;
			// int N_PAR_LINK;
			/* Joints' type string */
			std::vector<std::string> jointsType;
			std::vector<int> isElasticJoint;
			// casadi::SX elasticSel;
			/* Denavit-Hartenberg parameterization table */
			// casadi::SX _DHtable_;
			/* Frame offset between world-frame and link 0*/
			// FrameOffset _world2L0_;
			/* Frame offset between end-effector and last link */
			// FrameOffset _Ln2EE_;
			// elastic parameters
			int K_order=0, D_order=0, Dl_order=0, Dm_order=0;
			// casadi::SX gravity;
			// symbolic selectivity
			// std::vector<int> DHtable_symb;
			// std::vector<int> par_DYN_symb;
			// std::vector<int> par_Dl_symb;
			// std::vector<int> par_K_symb;
			// std::vector<int> par_D_symb;
			// std::vector<int> par_Dm_symb;
			// std::vector<int> base_frame_symb;
			// std::vector<int> ee_frame_symb;
			// std::vector<int> gravity_symb;
			// Input of casadi function //
			std::map<std::string, casadi::SX> args;
			std::map<std::string, casadi::Function> casadi_fun;
			std::map<std::string, std::vector<std::string>> fun_args;
			std::map<std::string, std::string> fun_descr;
			// Variable for joints angle //
			// casadi::SX q, dq, dqr, ddqr;
			// casadi::SX _par_KIN_, par_DYN, par_REG, par_ELA;
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
			std::string robotName = "robot";
			static const int STD_PAR_LINK = 10;
			/* Init variables
			numJoints: number of joints
			DHtable: Denavit-Hartenberg table format [a alpha d theta]
			jtsType: is string of "R" and "P"
			base_frame: is used to set transformation between link0 and world_frame */
			Robot(const std::string config_file);
			Robot(const YAML::Node yaml);
			// Robot(const Config conf);
			YAML::Node config_yaml;
			YAML::Node load_config(std::string config_file);
			int load_config(YAML::Node yaml);
			int init_symb_parameters();
			int subs_symb_par(std::string par);
			std::vector<std::string> obtain_symb_parameters(std::vector<std::string>, std::vector<std::string>);
			int update_symb_parameters();
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
			std::map<std::string, std::vector<int>> symb;
			Eigen::MatrixXd get(std::string name);
			// casadi::SX get_sx(std::string name);
			// get functions
			int get_numJoints();
			// std::vector<int> get_numParLink();
			// int get_numParLink(int i);
			std::vector<std::string> get_jointsType();
			// casadi::SX get_DHTable();
			// FrameOffset get_world2L0();
			// FrameOffset get_Ln2EE();
			// int get_numParLink(int i);
			bool get_ELASTIC();
			int get_K_order();
			int get_D_order();
			int get_Dl_order();
			int get_Dm_order();
			Eigen::VectorXd get_arg(std::string par);
			Eigen::VectorXd get_par_DYN();
			Eigen::VectorXd get_par_REG();
			// Eigen::VectorXd get_par_K();
			// Eigen::VectorXd get_par_D();
			// Eigen::VectorXd get_par_Dm();
			// Eigen::VectorXd get_par_Dl();
			// Eigen::VectorXd get_par_ELA();
			int get_numParDYN();
			int get_numParREG();
			// int get_numParELA();
			std::vector<int> get_isElasticJoint();
			int get_numElasticJoints();
			// set functions
			int set_arg(std::string name, Eigen::VectorXd);
			int set_q(Eigen::VectorXd);
			int set_dq(Eigen::VectorXd);
			int set_dqr(Eigen::VectorXd);
			int set_ddqr(Eigen::VectorXd);
			int set_x(Eigen::VectorXd);
			int set_dx(Eigen::VectorXd);
			int set_ddxr(Eigen::VectorXd);
			int set_par_DYN(Eigen::VectorXd);
			int set_par_REG(Eigen::VectorXd);
			// int set_par_K(Eigen::VectorXd);
			// int set_par_D(Eigen::VectorXd);
			// int set_par_Dm(Eigen::VectorXd);
			// int set_par_Dl(Eigen::VectorXd);
			// int set_par_ELA(Eigen::VectorXd);
			// load functions
			int load_conf_par(std::string config_file, bool update_REG = 1);
			casadi::SX load_par_REG(std::string config_file, bool update_DYN = 1);
			int load_par(std::string par_file, std::vector<std::string> par_list = {});
			// int load_par_elastic(std::string file);
			void update_conf();
			int save_conf(std::string par_file);
			int save_par_REG(std::string par_file);
			int save_par(std::string par_file, std::vector<std::string> par_list = {});
			int update_inertial_DYN();
			int update_inertial_REG();
			// std::map<std::string, Eigen::MatrixXd> get;
			// std::map<std::string, bool> valid;
			// Eigen::VectorXd q, dq, dqr, ddqr;
			// Eigen::VectorXd param_KIN, par_DYN, par_REG;
			// void update();
			// void generate_code(std::string& savePath);
			std::vector<fun_obj> get_functions(bool onlyNames = 1);
			// std::vector<casadi::Function> getCasadiFunctions();
			int add_function(std::string name, casadi::SX expr, std::vector<std::string> f_args, std::string descr = "");
			void generate_library(const std::string& savePath, const std::string& name_file);

	};

}
#endif