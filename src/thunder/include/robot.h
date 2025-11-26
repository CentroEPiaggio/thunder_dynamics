#ifndef ROBOT_H
#define ROBOT_H

#include <string>
#include <map>
#include <any>
#include <yaml-cpp/yaml.h>
#include <casadi/casadi.hpp>

#include "utils.h"

using std::string;
using std::vector;
using casadi::SX;
using casadi::DM;

namespace thunder_ns{
	
	class Property;
	typedef struct par_obj par_obj;
	typedef struct fun_obj fun_obj;
	// Config load_config(std::string file);
	Robot robot_from_file(std::string robot_name, std::string file, bool compute = 1);
	Robot robot_from_yaml(std::string robot_name, const YAML::Node& config_node, bool compute = 1);
	
	// contain everything related to a robot, uses the other classes to obtain functions
	class Robot{

		protected:
			// --- Properties --- //
			// int numJoints;
			// int numElasticJoints = 0;
			// bool ELASTIC = false;
			// vector<string> jointsType;
			// vector<short> isElasticJoint;
			// // elastic parameters
			// int K_order=0, D_order=0, Dl_order=0, Dm_order=0;

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
			Robot() = default;
			// - destructor - //
			// ~Robot(){};

			// --- Robot configuration --- //
			string robotName = "robot";
			YAML::Node config_yaml;
			YAML::Node load_config(string config_file);
			int load_config(YAML::Node yaml);
			// void update_conf();
			// int save_conf(string par_file);

			// --- Robot maps --- //
			// - Properties map - //
			std::map<string, Property> properties;
			// - Parameters map - //
			std::map<string, par_obj> parameters;
			// - Functions map - //
			std::map<string, fun_obj> functions;
			// --- Model map --- //
			std::map<string, casadi::SX> model;

			// --- Parameters functions --- //
			const par_obj get_par(string par);
			int set(string name, DM value);
			casadi::SX load_par_REG(string config_file, bool update_DYN = 1);
			int save_par_REG(string par_file);
			int load_par(string par_file, vector<string> par_list = {});
			int save_par(string par_file, vector<string> par_list = {});

			// --- Robot interactions --- //
			template<class T> T get(string key){
				if (!properties.count(key)){
					throw std::runtime_error("Property " + key + " not found in robot " + robotName);
				}
				return std::any_cast<T>(properties.at(key).value);
			}
			DM get(string name);
			vector<fun_obj> get_functions(bool onlyNames = 1);

			// --- Robot populators --- //
			template<class T> int add_property(string name, T value, string type, string descr = "", bool overwrite = true){
				if ((!overwrite) && properties.count(name)){
					// key already exists
					return 0;
				} else {
					Property prop;
					prop.name = name;
					prop.value = value;
					prop.type = type;
					prop.description = descr;
					properties[name] = prop;
				}
				return 1;
			}
			int add_variable(string name, SX symb, vector<double> num, vector<short> is_symbolic = {1}, string descr = "", bool overwrite = true);
			int add_parameter(string name, SX symb, vector<double> num, vector<short> is_symbolic = {0}, string descr = "", bool overwrite = true);
			int add_function(string name, SX expr, vector<string> f_args, string descr = "", bool overwrite = true);
			
			// --- Library generation --- //
			void generate_library(const string& savePath, const string& name_file, const bool SAVE_CASADI);

	};

}

#endif