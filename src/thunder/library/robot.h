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

using std::string;
using std::vector;

namespace thunder_ns{
	
	typedef struct par_obj par_obj;
	typedef struct fun_obj fun_obj;

	Robot robot_from_file(string robot_name, string file, bool compute = 1);
	
	// contain everything related to a robot, uses the other classes to obtain functions
	class Robot{
		private:
			int parse_config();
			void initVarsFuns();

		protected:
			int numJoints;
			int numElasticJoints = 0;
			bool ELASTIC = false;
			vector<string> jointsType;
			vector<int> isElasticJoint;
			// elastic parameters
			int K_order=0, D_order=0, Dl_order=0, Dm_order=0;
			// Input of casadi function //
			// maybe map<string, fun_obj>functions?
			std::map<string, casadi::SX> args;
			std::map<string, casadi::Function> casadi_fun;
			std::map<string, vector<string>> fun_args;
			std::map<string, string> fun_descr;
			// - Parameters map - //
			std::map<string, par_obj> parameters;
			// Dynamic parameters //
			casadi::SXVector _mass_vec_;
			casadi::SXVector _distCM_;
			casadi::SXVector _J_3x3_;

		public:
			Robot(const string config_file);
			Robot(const YAML::Node yaml);

			string robotName = "robot";
			static const int STD_PAR_LINK = 10;
			YAML::Node config_yaml;
			YAML::Node load_config(string config_file);
			int load_config(YAML::Node yaml);
			int init_symb_parameters();
			int subs_symb_par(string par);
			vector<string> obtain_symb_parameters(vector<string>, vector<string>);
			int update_symb_parameters();
			/* Destructor */
			// ~Robot(){};

			/* Variable for joints in set arguments */
			int valid;
			std::map<string, casadi::SX> model;
			std::map<string, vector<int>> symb;
			Eigen::MatrixXd get(string name);
			// get functions
			int get_numJoints();
			vector<string> get_jointsType();
			bool get_ELASTIC();
			int get_K_order();
			int get_D_order();
			int get_Dl_order();
			int get_Dm_order();
			Eigen::VectorXd get_arg(string par);
			Eigen::VectorXd get_par_DYN();
			Eigen::VectorXd get_par_REG();
			int get_numParDYN();
			int get_numParREG();
			vector<int> get_isElasticJoint();
			int get_numElasticJoints();
			// set functions
			int set_arg(string name, Eigen::VectorXd);
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
			int load_conf_par(string config_file, bool update_REG = 1);
			casadi::SX load_par_REG(string config_file, bool update_DYN = 1);
			int load_par(string par_file, vector<string> par_list = {});
			// int load_par_elastic(string file);
			void update_conf();
			int save_conf(string par_file);
			int save_par_REG(string par_file);
			int save_par(string par_file, vector<string> par_list = {});
			int update_inertial_DYN();
			int update_inertial_REG();
			vector<fun_obj> get_functions(bool onlyNames = 1);
			int add_variable(string name, casadi::SX symb, casadi::SX num, vector<bool> is_symbolic, string descr = "", bool overwrite = true);
			int add_parameter(string name, casadi::SX symb, casadi::SX num, vector<bool> is_symbolic, string descr = "", bool overwrite = true);
			int add_function(string name, casadi::SX expr, vector<string> f_args, string descr = "", bool overwrite = true);
			void generate_library(const string& savePath, const string& name_file, const bool SAVE_CASADI);

	};

}
#endif