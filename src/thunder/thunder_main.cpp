/*
Command line interface for Thunder, it can generate code for robots
*/

#include <iostream>
#include <string>
#include <filesystem>
#include <stdexcept>
#include <chrono>

#include <yaml-cpp/yaml.h>

#include <argparse/argparse.hpp>

#include "plugin_interfaces.h"
#include "plugin_registry.h"


using namespace thunder_ns;
using namespace std::chrono;

using std::cout;
using std::endl;

bool COPY_GEN_FLAG = false; 		// used to copy generated files into thunder_robot project
bool COPY_GEN_CHRONO_FLAG = false; 	// used to copy generated files into thunder_robot_chrono project
bool GEN_PYTHON_FLAG = false; 		// used to generate python binding
bool GEN_CASADI = false;			// used to generate casadi functions
#define MU_JACOB 0.0
#define VERSION "0.8.19-plugin"

template <typename PluginType>
void print_plugin_group(const std::string& title, const std::map<std::string, std::shared_ptr<PluginType>>& plugins, bool verbosity){
	cout << title << ":" << endl;
	if (plugins.empty()) {
		cout << "  (none)" << endl;
		return;
	}
	for (const auto& plugin : plugins) {
		// cout << "  - " << plugin.first << endl;
		cout << " - '" << plugin.first << "' ";
		if(verbosity) 
			plugin.second->print_info();
		cout<<endl;
	}
}

void print_available_plugins(bool verbosity) {
	cout << "Available Thunder plugins" << endl;
	print_plugin_group("Loaders", LOADERS, verbosity);
	print_plugin_group("Builders", POPULATORS, verbosity);
	print_plugin_group("Generators", GENERATORS, verbosity);
}

// --- paths and files (default) --- //
std::string robot_name = "robot";
std::string path_robot = "../robots/";
std::string config_file = path_robot + robot_name + "/robot.yaml";
std::string robot_name_gen = robot_name + "_gen";


int main(int argc, char* argv[]){
	// --- Variables --- //
	int nj;

	// ----------------------------- //
	// ---------- CONSOLE ---------- //
	// ----------------------------- //

	// - defining cli - //
	argparse::ArgumentParser thunder_cli("thunder", VERSION);
	argparse::ArgumentParser gen_command("gen", VERSION);
	gen_command.add_description("Generate library from yaml configuration file");
	gen_command.add_argument("config_file")
		.help("Robot configuration file (.yaml)");
	gen_command.add_argument("-n", "--name")
		.help("Robot name");
	gen_command.add_argument("-v", "--verbose").help("Enable verbose output")
	.default_value(false)
	.implicit_value(true);

	argparse::ArgumentParser plugin_command("plugin", VERSION);
	plugin_command.add_description("Plugin management commands");

	argparse::ArgumentParser plugin_list_command("list", VERSION);
	plugin_list_command.add_description("List available plugins");
	plugin_list_command.add_argument("-v", "--verbose").help("Print additional plugins info")
	.default_value(false)
	.implicit_value(true);

	plugin_command.add_subparser(plugin_list_command);
	
	thunder_cli.add_subparser(gen_command);
	thunder_cli.add_subparser(plugin_command);

	// - parsing - //
	try {
		thunder_cli.parse_args(argc, argv);
	} catch (const std::runtime_error &err) {
		std::cout << err.what() << std::endl;
		std::cout << thunder_cli;
		return 0;
	}

	if (thunder_cli.is_subcommand_used("plugin")) {
		if (plugin_command.is_subcommand_used("list")) {
			print_available_plugins(plugin_list_command.get<bool>("--verbose"));
		} else {
			std::cout << plugin_command;
		}
		return 0;
	}

	if (!thunder_cli.is_subcommand_used("gen")) {
		std::cout << thunder_cli;
		return 0;
	}

	// - Get arguments - //
	config_file = gen_command.get<std::string>("config_file");

	// - check config_file and name - //
	int index_yaml = config_file.find_last_of(".yaml");
	if (index_yaml > 0){
		int index_path = config_file.find_last_of("/");
		if (index_path == std::string::npos){ // no occurrence
			path_robot = "./";
			robot_name = config_file.substr(0, index_yaml+1-5); // 5 stands for ".yaml"
		}else{
			path_robot = config_file.substr(0, index_path+1);
			robot_name = config_file.substr(index_path+1, index_yaml-index_path-5); // 5 stands for ".yaml"
		}
	}else{
		std::cout << "Invalid config file." << std::endl;
		return 0;
	}
	// if --name is used, robot_name is overwritten
	if (gen_command.is_used("--name")){
		robot_name = gen_command.get<std::string>("--name");
	}

	// Set name and paths
	cout<<"Robot name: "<<robot_name<<endl;
	// cout<<"robot_path: "<<path_robot<<endl;
	auto time_start = high_resolution_clock::now();
	robot_name_gen = robot_name + "_gen";


    // PARSE YAML 
    YAML::Node config_node = YAML::LoadFile(config_file);

    auto loaders = config_node["pipeline"]["loaders"].as<std::vector<std::string>>();
    auto builders = config_node["pipeline"]["builders"].as<std::vector<std::string>>();
    auto generators = config_node["pipeline"]["generators"].as<std::vector<std::string>>();

    // execute loaders
	auto robot_ptr = std::make_shared<Robot>();

	auto verbosity = gen_command.get<bool>("--verbose");

    for(auto loader : loaders){
		cout << "Executing Loader: " << loader << endl;
        std::shared_ptr<BaseLoader> loader_plugin = find_loader(loader);

		loader_plugin->set_debug_flag(verbosity);
        loader_plugin->configure(config_node[loader]);

        robot_ptr = loader_plugin->load(robot_ptr);
    }

    // execute builders
    for(auto builder : builders){
		cout << "Executing Builder: " << builder << endl;
        std::shared_ptr<BaseBuilder> builder_plugin = find_builder(builder);
		
		builder_plugin->set_debug_flag(verbosity);
        builder_plugin->configure(config_node[builder]);

        builder_plugin->execute(robot_ptr);
    }

    // execute generators
    for(auto generator : generators){
		cout << "Executing generator: " << generator << endl;
        
		std::shared_ptr<BaseGenerator> generator_plugin = find_generator(generator);
		generator_plugin->set_debug_flag(verbosity);
        generator_plugin->configure(config_node[generator]);
        
		generator_plugin->generate(robot_ptr);
    }

	// // thunder_robot path
	// std::string COPY_PREFIX;
	// if (currentPath.filename() == "build") { // last directory name
	// 	COPY_PREFIX = currentPath/"../../../";
	// } else {
	// 	COPY_PREFIX = "/home/thunder_dev/thunder_dynamics/";
	// }
	// std::string PATH_COPY_H = COPY_PREFIX + "src/thunder_robot/library/";
	// std::string PATH_COPY_CPP = COPY_PREFIX + "src/thunder_robot/src/";
	// std::string PATH_COPY_YAML = COPY_PREFIX + "src/thunder_robot/robots/";
	// std::string PATH_COPY_CHRONO_H = COPY_PREFIX + "src/thunder_robot_chrono/library/";
	// std::string PATH_COPY_CHRONO_CPP = COPY_PREFIX + "src/thunder_robot_chrono/src/";
	// std::string PATH_COPY_CHRONO_YAML = COPY_PREFIX + "src/thunder_robot_chrono/robots/";
	
	// // --- copy generated files in thunder_robot project --- //
	// if(COPY_GEN_FLAG){
	// 	copy_to(robot_name, absolutePath, PATH_COPY_YAML, PATH_COPY_YAML, PATH_COPY_H, PATH_COPY_CPP);
	// 	std::cout << "Copied to thunder_robot!" << std::endl;
	// }

	// // --- copy generated files in thunder_robot project --- //
	// if(COPY_GEN_CHRONO_FLAG){
	// 	copy_to(robot_name, absolutePath, PATH_COPY_CHRONO_YAML, PATH_COPY_CHRONO_YAML, PATH_COPY_CHRONO_H, PATH_COPY_CHRONO_CPP);
	// 	std::cout << "Copied to thunder_robot_chrono!" << std::endl;
	// }

	// --- elapsed time --- //
	auto time_stop = high_resolution_clock::now();
	auto duration = duration_cast<milliseconds>(time_stop - time_start);
	std::cout<<"done in "<<((double)duration.count())<<" ms!"<<endl; 

	return 1;
}
