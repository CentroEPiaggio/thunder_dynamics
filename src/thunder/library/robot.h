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
	
	typedef struct par_obj par_obj;
	typedef struct fun_obj fun_obj;

	Robot robot_from_file(std::string robot_name, std::string file, bool compute = 1);
	
	// contain everything related to a robot, uses the other classes to obtain functions
	class Robot{
		private:
			int parse_config();
			void initVarsFuns();

		protected:
			int numJoints;
			int numElasticJoints = 0;
			bool ELASTIC = false;
			std::vector<std::string> jointsType;
			std::vector<int> isElasticJoint;
			// elastic parameters
			int K_order=0, D_order=0, Dl_order=0, Dm_order=0;
			// Input of casadi function //
			std::map<std::string, casadi::SX> args;
			std::map<std::string, casadi::Function> casadi_fun;
			std::map<std::string, std::vector<std::string>> fun_args;
			std::map<std::string, std::string> fun_descr;
			// Dynamic parameters //
			casadi::SXVector _mass_vec_;
			casadi::SXVector _distCM_;
			casadi::SXVector _J_3x3_;

		public:
			Robot(const std::string config_file);
			Robot(const YAML::Node yaml);

			std::string robotName = "robot";
			static const int STD_PAR_LINK = 10;
			YAML::Node config_yaml;
			YAML::Node load_config(std::string config_file);
			int load_config(YAML::Node yaml);
			int init_symb_parameters();
			int subs_symb_par(std::string par);
			std::vector<std::string> obtain_symb_parameters(std::vector<std::string>, std::vector<std::string>);
			int update_symb_parameters();
			/* Destructor */
			// ~Robot(){};

			/* Variable for joints in set arguments */
			int valid;
			std::map<std::string, casadi::SX> model;
			std::map<std::string, std::vector<int>> symb;
			Eigen::MatrixXd get(std::string name);
			// get functions
			int get_numJoints();
			std::vector<std::string> get_jointsType();
			bool get_ELASTIC();
			int get_K_order();
			int get_D_order();
			int get_Dl_order();
			int get_Dm_order();
			Eigen::VectorXd get_arg(std::string par);
			Eigen::VectorXd get_par_DYN();
			Eigen::VectorXd get_par_REG();
			int get_numParDYN();
			int get_numParREG();
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
			int set_ddx(Eigen::VectorXd);
			int set_ddxr(Eigen::VectorXd);
			int set_par_DYN(Eigen::VectorXd);
			int set_par_REG(Eigen::VectorXd);
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
			std::vector<fun_obj> get_functions(bool onlyNames = 1);
			int add_function(std::string name, casadi::SX expr, std::vector<std::string> f_args, std::string descr = "", bool overwrite = true);
			void generate_library(const std::string& savePath, const std::string& name_file, const bool SAVE_CASADI);

	};

}
#endif