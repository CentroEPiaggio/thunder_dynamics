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
using casadi::SX;

namespace thunder_ns{
	
	typedef struct par_obj par_obj;
	typedef struct fun_obj fun_obj;

	Robot robot_from_file(string robot_name, string file, bool compute = 1);
	
	// contain everything related to a robot, uses the other classes to obtain functions
	class Robot{

		protected:
			// --- Properties --- //
			int numJoints;
			int numElasticJoints = 0;
			bool ELASTIC = false;
			vector<string> jointsType;
			vector<int> isElasticJoint;
			// elastic parameters
			int K_order=0, D_order=0, Dl_order=0, Dm_order=0;

			// --- Internal methods --- //
			int parse_config();
			int update_inertial_DYN();
			int update_inertial_REG();

		public:
			// --- Robot constants --- //
			static const int STD_PAR_LINK = 10;

			// --- Constructors --- //
			Robot(const string config_file);
			Robot(const YAML::Node yaml);
			// - destructor - //
			// ~Robot(){};

			// --- Robot configuration --- //
			string robotName = "robot";
			YAML::Node config_yaml;
			YAML::Node load_config(string config_file);
			int load_config(YAML::Node yaml);
			void update_conf();
			int save_conf(string par_file);

			// --- Symbolic selectivity --- //
			int init_symb_parameters();
			int subs_symb_par(string par);
			vector<string> obtain_symb_parameters(vector<string>, vector<string>);
			int update_symb_parameters();

			// --- Robot maps --- //
			// - Parameters map - //
			std::map<string, par_obj> parameters;
			// - Functions map - //
			// maybe map<string, fun_obj>functions?
			std::map<string, casadi::Function> casadi_fun;
			std::map<string, vector<string>> fun_args;
			std::map<string, string> fun_descr;
			// --- Model map --- //
			std::map<string, casadi::SX> model;

			// --- Parameters functions --- //
			vector<double> get_par(string par);
			int set_par(string name, vector<double> value);
			casadi::SX load_par_REG(string config_file, bool update_DYN = 1);
			int save_par_REG(string par_file);
			int load_par(string par_file, vector<string> par_list = {});
			int save_par(string par_file, vector<string> par_list = {});

			// to clean ! --------------------------------------------
			Eigen::MatrixXd get_value(string name);
			// _value functions
			int get_numJoints();
			vector<string> get_jointsType();
			bool get_ELASTIC();
			int get_K_order();
			int get_D_order();
			int get_Dl_order();
			int get_Dm_order();
			vector<int> get_isElasticJoint();
			int get_numElasticJoints();
			// load functions
			vector<fun_obj> get_functions(bool onlyNames = 1);
			// --------------------------------------------------------

			// --- Map populators --- //
			int add_variable(string name, SX symb, vector<double> num, vector<short> is_symbolic = {1}, string descr = "", bool overwrite = true);
			int add_parameter(string name, SX symb, vector<double> num, vector<short> is_symbolic = {0}, string descr = "", bool overwrite = true);
			int add_function(string name, SX expr, vector<string> f_args, string descr = "", bool overwrite = true);
			
			// --- Library generation --- //
			void generate_library(const string& savePath, const string& name_file, const bool SAVE_CASADI);

	};

}
#endif