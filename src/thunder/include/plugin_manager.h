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

        // Sets verbosity for all plugins
        void set_verbose(bool v) { verbose_ = v; }

        // Prints all registered plugins from the global registry.
        void print_available_plugins(bool verbose) const {
            std::cout << "Available Thunder plugins" << std::endl;
            print_plugin_group("Loaders", LOADERS, verbose);
            print_plugin_group("Builders", POPULATORS, verbose);
            print_plugin_group("Generators", GENERATORS, verbose);
        }

        // Clears current pipeline and sets up plugins based on YAML config
        void configure_pipeline(const YAML::Node &config) {
            active_loaders_.clear();
            active_builders_.clear();
            active_generators_.clear();

            std::vector<std::string> loader_names;
            std::vector<std::string> builder_names;
            std::vector<std::string> generator_names;
            bool legacy_mode = false;

            if (config["pipeline"]) {
                loader_names = config["pipeline"]["loaders"].as<std::vector<std::string>>();
                builder_names = config["pipeline"]["builders"].as<std::vector<std::string>>();
                generator_names = config["pipeline"]["generators"].as<std::vector<std::string>>();
            } else {
                legacy_mode = true;
                loader_names = {"legacy_loader"};
                builder_names = {"legacy_builder"};
                generator_names = {"legacy_generator"};
                std::cout << "[PluginManager] No pipeline defined, using Legacy mode." << std::endl;
            }

            for (const auto &name : loader_names) {
                auto plugin = find_loader(name);
                if (!plugin)
                    throw std::runtime_error("Loader not found: " + name);

                plugin->set_debug_flag(verbose_);
                plugin->configure(legacy_mode ? config : config[name]);

                active_loaders_.push_back(plugin);
            }

            for (const auto &name : builder_names) {
                auto plugin = find_builder(name);
                if (!plugin)
                    throw std::runtime_error("Builder not found: " + name);

                plugin->set_debug_flag(verbose_);
                plugin->configure(legacy_mode ? config : config[name]);

                active_builders_.push_back(plugin);
            }

            for (const auto &name : generator_names) {
                auto plugin = find_generator(name);
                if (!plugin)
                    throw std::runtime_error("Generator not found: " + name);

                plugin->set_debug_flag(verbose_);
                plugin->configure(legacy_mode ? config : config[name]);

                active_generators_.push_back(plugin);
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