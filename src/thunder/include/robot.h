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
	
	// contain everything related to a robot, uses the other classes to obtain functions
	class Robot{
		public:
			// -------------------- //
			// --- Constructors --- //
			// -------------------- //

			// Base costructor, set robot name
			Robot(string name) {robotName = name;}

			// Default costructor
			Robot() = default;

			// Destructor
			// ~Robot(){};


			// ----------------------- //
			// --- Robot variables --- //
			// ----------------------- //

			string robotName = "robot";
			YAML::Node config_yaml;


			// ------------------ //
			// --- Robot maps --- //
			// ------------------ //

			// Property map, contains robot structure
			std::map<string, Property> properties;

			// Parameter map, contains robot variables and parameters
			std::map<string, Parameter> parameters;

			// Function map, contains robot expressions and functions
			std::map<string, Function> functions;


			// --------------------------- //
			// --- Interface functions --- //
			// --------------------------- //

			// Return the value of property <key> ot type <T>
			template<class T> T get(string key){
				if (!properties.count(key)){
					throw std::runtime_error("Property " + key + " not found in robot " + robotName);
				}
				return std::any_cast<T>(properties.at(key).value);
			}

			// Return the model (casadi::SX) of <key>
			SX get_model(string key);

			// Return the value (casadi::DM) of <key>
			DM get(string key);

			// Return a vector of Property elements
			const vector<Property> get_properties(vector<string> prop_list = {});

			// Return a vector of Parameter elements
			const vector<Parameter> get_parameters(vector<string> par_list = {});

			// Return a vector of Function elements
			const vector<Function> get_functions(vector<string> fun_list = {});

			// Set the value of parameter <name> to <value>
			int set(string name, DM value);

			// Load parameters from file, {} load all
			int load_par(string par_file, vector<string> par_list = {});

			// Save parameters from file, {} save all
			int save_par(string par_file, vector<string> par_list = {});


			// ------------------------ //
			// --- Robot populators --- //
			// ------------------------ //

			// Add the property <name> of type <T> to the property map
			template<class T> int add_property(string name, T value, string type_str, string descr = "", bool overwrite = true){
				if ((!overwrite) && properties.count(name)){
					// key already exists
					return 0;
				} else {
					Property prop;
					prop.name = name;
					prop.value = value;
					prop.type_str = type_str;
					prop.description = descr;
					properties[name] = prop;
				}
				return 1;
			}

			// Add the variable <name> to the parameters map
			int add_variable(string name, SX symb, vector<double> num, vector<short> is_symbolic = {1}, string descr = "", bool overwrite = true);
			
			// Add the parameter <name> to the parameters map
			int add_parameter(string name, SX symb, vector<double> num, vector<short> is_symbolic = {0}, string descr = "", bool overwrite = true);
			
			// Add the function <name> to the function map
			int add_function(string name, SX expr, vector<string> f_args, string descr = "", bool overwrite = true);
			
	};

}

#endif