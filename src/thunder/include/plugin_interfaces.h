#pragma once

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include "robot.h"

#define VERB_INFO 0
#define VERB_DEBUG 1
#define VERB_ERROR 2


namespace thunder_ns{ 

	/**
	 * @brief The absolute base class for all plugins.
	 */
	class BasePlugin {

	private:
		std::string name_;
		std::string description_;
		int debug_flag_ = 0;

	public:

		YAML::Node config_; 
		
		BasePlugin(std::string name, std::string desc) : name_(name), description_(desc) {}

		~BasePlugin() = default;

		/**
		 * @brief Enables verbose output when set to 1, otherwise prints only essential logs.
		 */
		void set_debug_flag(int flag) {
			debug_flag_ = flag ? 1 : 0;
		}

		/**
		 * @brief Returns the currently selected debug flag value.
		 */
		int get_debug_flag() const {
			return debug_flag_;
		}
		
		/**
		 * @brief Prints name and descripion to cout.
		 */
		void print_info() const{
			std::cout << "Plugin " << name_ << ". Description: " << description_;
		}

		/**
		 * @brief Load configuration from yaml
		 * 
		 * @param config The configuration node
		 */ 
		int configure(const YAML::Node& config) {
			config_ = config;
			return 1;
		}


		/**
		 * @brief Set descripion, name and 
		 */
	protected:
		/**
		 * @brief Helper method to print plugin specific debug information.
		 *
		 * @param message The message to print
		 * @param verbosity Verbosity level (0-basic, 1-detailed)
		 */
		void debug_log(const std::string& message, int verbosity) const {
			if (verbosity == VERB_INFO || debug_flag_ == 1) {
				std::cout << "    [" << name_ << "] [" << (verbosity == VERB_DEBUG ? "DEBUG" : "INFO") << "]: "
						  << message << std::endl;
			}
		}

	};







	/**
	 * @brief Base class for plugins that load initial robot data (e.g., from URDF, DH).
	 * Their 'load' method is responsible for initializing the Robot object.
	 */
	class BaseLoader : public BasePlugin {
		public:

		/**
		 * @brief Create and initialize a Robot object from configuration
		 * 
		 * @return A unique pointer to the newly created Robot object
		 * 
		 * @throws std::runtime_error if loading fails
		 */
		virtual std::shared_ptr<Robot> load(std::shared_ptr<Robot>) = 0;

		BaseLoader(std::string name, std::string desc): BasePlugin(name, desc) {}
	};

	/**
	 * @brief Base class for plugins that populate symbolic functions.
	 * Their 'build' method reads parameters from the Robot object,
	 * performs symbolic calculations (e.g., kinematics, dynamics),
	 * and adds new functions to the Robot using `registrer_function`.
	 */
	class BaseBuilder : public BasePlugin {
		
  		private:
		std::vector<std::string> registered_function_names_;

		public:

		/**
		 * @brief Executes the plugin's logic.
		 *
		 *
		 * @param robot The Robot instance
		 */
		virtual void build(std::shared_ptr<Robot> robot) = 0;

		BaseBuilder(std::string name, std::string desc): BasePlugin(name, desc) {}

		/**
		 * @brief Get list of functions registered by this builder
		 */
		const std::vector<std::string>& get_registered_functions() const {
			return registered_function_names_;
		}

		protected:

		/**
		 * @brief Wrapper to register functions to the robot.
		 * 
		 * Checks YAML config and if functions/<name>/ignore is true, skips registration.
		 * 
		 * @param robot The robot instance
		 * @param f_name Name of the function to add
		 * @param expr The casadi expression
		 * @param args_raw tentative list of arguments
		 * @param descr	description of the functions
		 * @param overwrite if true, overwrite existing functions with the same name.
		 * 
		 * @return true if added, false if ignored
		 * 
		 */
		bool register_function(std::shared_ptr<Robot> robot, string f_name, casadi::SX expr, vector<string> args_raw, string descr, bool overwrite) {
			
			// Check YAML for ignore flag
			if (config_["functions"] && config_["functions"][f_name]) {
				if (config_["functions"][f_name]["ignore"].as<bool>(false)) {
					debug_log("Skipping function '" + f_name + "' (ignored in config)", VERB_INFO);
					return false;
				}
			}

			// Perform actual addition
			robot->add_function(f_name, expr,  args_raw, descr, overwrite);
			
			// Add to local registry
			registered_function_names_.push_back(f_name);
			debug_log("Registered function: " + f_name, VERB_DEBUG);
			return true;
		}

	};

	/**
	 * @brief Base class for plugins that generate code.
	 * Their 'generate' method should treat the Robot object as read-only.
	 * It reads functions and generates output files
	 */
	class BaseGenerator : public BasePlugin {
		public:

		/**
		 * @brief Executes the plugin's logic.
		 *
		 * @param robot The Robot instance
		 */
		virtual void generate(const std::shared_ptr<Robot> robot) = 0;

		BaseGenerator(std::string name, std::string desc): BasePlugin(name, desc) {}


	};


} // namespace thunder
