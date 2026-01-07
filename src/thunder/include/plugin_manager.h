#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H

#include <stdexcept>
#include <string>
#include <vector>

#include "plugin_interfaces.h"
#include "plugin_registry.h"

namespace thunder_ns {

	class PluginManager {

	private:

		// the yaml configuration (needed to copy into robot)
		YAML::Node config_yaml;

		// The active pipeline for the current run
		std::vector<std::shared_ptr<BaseLoader>> active_loaders_;
		std::vector<std::shared_ptr<BaseBuilder>> active_builders_;
		std::vector<std::shared_ptr<BaseGenerator>> active_generators_;

		bool verbose_ = false;

		//f Print a specific group of plugins (Loaders, Builders, etc.)
		template <typename PluginType>
		void print_plugin_group(const std::string &title,
								const std::map<std::string, std::shared_ptr<PluginType>> &plugins,
								bool verbosity) const {
			std::cout << title << ":\n";
			std::cout << "--------------------------------------------------------------------------------\n";
			if (plugins.empty()) {
				std::cout << std::left << std::setw(30) << "(none)" << "\n";
				return;
			}
			for (const auto &pair : plugins) {
				std::cout << std::left << std::setw(30) << pair.first;
				if (verbosity && pair.second) { std::cout << " | " << pair.second->get_description(); }
				std::cout << "\n";
			}
			std::cout << "--------------------------------------------------------------------------------\n";
		}

	public:
	
		PluginManager() = default;

		const std::vector<std::string> default_loaders = {"kin_loader", "dyn_loader", "soft_loader"};
		const std::vector<std::string> default_builders = {"kin_builder", "dyn_builder", "soft_builder", "reg_builder"};
		const std::vector<std::string> default_generators = {"robot_generator"};

		// Sets verbosity for all plugins
		void set_verbose(bool v) { verbose_ = v; }

		// Prints all registered plugins from the global registry.
		void print_available_plugins(bool verbose) const {
			std::cout << "Available Thunder plugins" << std::endl;
			print_plugin_group("Loaders", LOADERS, verbose);
			print_plugin_group("Builders", POPULATORS, verbose);
			print_plugin_group("Generators", GENERATORS, verbose);
		}

		// Clears current pipeline and sets up plugins based on a yaml file. Additionally saves file path in config
		void configure_pipeline(const std::string &config_path, int NO_GENERATION = 0) {

			// make the path absolute wrt current working directory
			std::filesystem::path absolute_path = std::filesystem::absolute(config_path);

			YAML::Node config = YAML::LoadFile(absolute_path.string());
			config["config_path"] = absolute_path.string();
			configure_pipeline(config, NO_GENERATION);
		}

		// Clears current pipeline and sets up plugins based on YAML config
		void configure_pipeline(const YAML::Node &config, int NO_GENERATION = 0) {
			config_yaml = config;

			active_loaders_.clear();
			active_builders_.clear();
			active_generators_.clear();

			std::vector<std::string> loader_names;
			std::vector<std::string> builder_names;
			std::vector<std::string> generator_names;

			if (config["pipeline"]) {
				if (config["pipeline"]["loaders"]) {
					loader_names = config["pipeline"]["loaders"].as<std::vector<std::string>>();
				} else {
					loader_names = default_loaders;
					std::cout << "[PluginManager] No loaders defined, using defaults." << std::endl;
				}
				if (config["pipeline"]["builders"]) {
					builder_names = config["pipeline"]["builders"].as<std::vector<std::string>>();
				} else {
					builder_names = default_builders;
					std::cout << "[PluginManager] No builders defined, using defaults." << std::endl;
				}
				if (config["pipeline"]["generators"]) {
					generator_names = config["pipeline"]["generators"].as<std::vector<std::string>>();
				} else {
					generator_names = default_generators;
					std::cout << "[PluginManager] No generators defined, using defaults." << std::endl;
				}
			} else {
				loader_names = default_loaders;
				builder_names = default_builders;
				generator_names = default_generators;
				std::cout << "[PluginManager] No pipeline defined, using default." << std::endl;
			}

			for (const auto &name : loader_names) {
				auto plugin = find_loader(name);
				if (!plugin)
					throw std::runtime_error("Loader not found: " + name);

				plugin->set_debug_flag(verbose_);
				// if (name == "legacy_loader") plugin->configure(config);
				// else {
				if (config[name]) plugin->configure(config[name]);
				else plugin->configure(config);
				// }
				
				active_loaders_.push_back(plugin);
			}

			for (const auto &name : builder_names) {
				auto plugin = find_builder(name);
				if (!plugin)
					throw std::runtime_error("Builder not found: " + name);

				plugin->set_debug_flag(verbose_);
				// if (name == "legacy_builder") plugin->configure(config);
				// else {
				if (config[name]) plugin->configure(config[name]);
				else plugin->configure(config);
                // }

				active_builders_.push_back(plugin);
			}

			for (const auto &name : generator_names) {
				auto plugin = find_generator(name);
				if (!plugin)
					throw std::runtime_error("Generator not found: " + name);

				plugin->set_debug_flag(verbose_);
				// if (name == "legacy_generator") plugin->configure(config);
				// else {
				if (config[name]) plugin->configure(config[name]);
				else plugin->configure(config);
                // }
				if (!NO_GENERATION){
					active_generators_.push_back(plugin);
				}
			}
		}

		/**
		 * @brief Executes the pipeline on the robot object
		 *
		 * @param robot_name name of the robot
		 *
		 * @return shared pointer to the robot
		 */
		std::shared_ptr<Robot> execute(std::string robot_name) {
			auto robot = std::make_shared<Robot>(robot_name);
			robot->config_yaml = config_yaml;

			if (verbose_)
				std::cout << "--- Starting Loaders ---" << std::endl;
			for (auto &loader : active_loaders_) {
				if (verbose_)
					std::cout << "Running Loader: " << loader->get_name() << std::endl;
				robot = loader->load(robot);
			}

			if (verbose_)
				std::cout << "--- Starting Builders ---" << std::endl;
			for (auto &builder : active_builders_) {
				if (verbose_)
					std::cout << "Running Builder: " << builder->get_name() << std::endl;
				builder->build(robot);
			}

			if (verbose_)
				std::cout << "--- Starting Generators ---" << std::endl;
			for (auto &gen : active_generators_) {
				if (verbose_)
					std::cout << "Running Generator: " << gen->get_name() << std::endl;
				gen->generate(robot);
			}

			return robot;
		}
	};

} // namespace thunder_ns

#endif