#include "plugins/generators/ros_control_generator.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>


namespace fs = std::filesystem;

namespace thunder_ns {
namespace {

// The controller source is not written here: it is a template in data/templates/ros_control_generator,
// installed with thunder, where <ROBOT>, <PACKAGE>, <NDOF>, <NUM_NODES>, <NPAR>, <JOINT_NAMES> are replaced.
const std::string DEFAULT_TEMPLATE_PATH = "/usr/share/thunder_dynamics/templates/ros_control_generator";

// template file -> file in the package ("<PACKAGE>" in the destination is replaced too)
const std::vector<std::pair<std::string, std::string>> TEMPLATE_FILES = {
    {"CMakeLists.txt", "CMakeLists.txt"},
    {"package.xml", "package.xml"},
    {"plugin.xml", "<PACKAGE>.xml"},
    {"include/controller.hpp", "include/<PACKAGE>/controller.hpp"},
    {"include/mact.hpp", "include/<PACKAGE>/mact.hpp"},
    {"include/trajectory.hpp", "include/<PACKAGE>/trajectory.hpp"},
    {"include/filter.hpp", "include/<PACKAGE>/filter.hpp"},
    {"src/controller.cpp", "src/controller.cpp"},
    {"test/test_algorithm.cpp", "test/test_algorithm.cpp"},
};

// Runtime parameters of the controller that can be given in ros_control_generator.parameters.
// The required ones have no default in the controller: they depend on the robot.
enum class Kind { VECTOR_NDOF, VECTOR_NODES, VECTOR_10, DOUBLE, BOOL, STRING };
struct ParameterSpec {
    Kind kind;
    bool required;
};
const std::map<std::string, ParameterSpec> CONTROLLER_PARAMETERS = {
    {"k_p", {Kind::VECTOR_NDOF, true}},
    {"k_v", {Kind::VECTOR_NDOF, true}},
    {"robot_adds_gravity", {Kind::BOOL, true}},
    {"limits.max_acceleration", {Kind::VECTOR_NDOF, true}},
    {"limits.max_torque_rate", {Kind::DOUBLE, true}},
    {"adaptation.enabled", {Kind::BOOL, false}},
    {"adaptation.gamma", {Kind::DOUBLE, false}},
    {"adaptation.torque_source", {Kind::STRING, false}},
    {"adaptation.R_t", {Kind::VECTOR_NDOF, false}},
    {"adaptation.R_p_link", {Kind::VECTOR_NODES, false}},
    {"adaptation.R_p_param", {Kind::VECTOR_10, false}},
    {"filters.dq_cutoff_hz", {Kind::DOUBLE, false}},
    {"filters.ddq_cutoff_hz", {Kind::DOUBLE, false}},
};

struct GeneratorConfig {
    bool copy_gen = false;
    std::string template_path = DEFAULT_TEMPLATE_PATH;
    std::vector<std::string> joints;    // empty: from the urdf_loader property jointsUrdfName
    YAML::Node parameters;
};

GeneratorConfig parse_config(const YAML::Node& config) {
    GeneratorConfig result;
    if (config["copy_gen"]) result.copy_gen = config["copy_gen"].as<bool>();
    if (config["template_path"]) result.template_path = config["template_path"].as<std::string>();
    if (config["joints"]) result.joints = config["joints"].as<std::vector<std::string>>();
    if (!config["parameters"] || !config["parameters"].IsMap()) {
        throw std::runtime_error("ros_control_generator.parameters is required (k_p, k_v, robot_adds_gravity, limits).");
    }
    result.parameters = config["parameters"];
    return result;
}

void write_file(const fs::path& path, const std::string& content) {
    fs::create_directories(path.parent_path());
    std::ofstream output(path);
    if (!output) throw std::runtime_error("Unable to write generated file: " + path.string());
    output << content;
}

std::string read_file(const fs::path& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Unable to read template " + path.string() + ". Is thunder installed (make install)? The folder can be set with ros_control_generator.template_path.");
    std::stringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

void copy_generated_file(const fs::path& source, const fs::path& destination) {
    if (!fs::exists(source)) {
        throw std::runtime_error("Missing " + source.string() +
            ". ros_control_generator requires robot_generator to run first with gen_robot: true.");
    }
    fs::create_directories(destination.parent_path());
    fs::copy_file(source, destination, fs::copy_options::overwrite_existing);
}

bool is_valid_ros_package_name(const std::string& name) {
    return !name.empty() && std::islower(static_cast<unsigned char>(name.front())) &&
           std::all_of(name.begin(), name.end(), [](unsigned char character) {
               return std::islower(character) || std::isdigit(character) || character == '_';
           });
}

// The model quantities the controller uses: Yr, Y, reg_G (no explicit arguments, ndof x npar)
// and the runtime parameters it sets: q, dq, ddq, dqr, ddqr (ndof), par_REG (10 per node).
void check_model(const std::map<std::string, Function>& functions, const std::map<std::string, Parameter>& parameters,
                 int ndof, int num_nodes) {
    const std::string hint = " The controller needs reg_builder (dynamic regressors) in the pipeline.";
    const std::map<std::string, int> sizes = {{"q", ndof}, {"dq", ndof}, {"ddq", ndof}, {"dqr", ndof}, {"ddqr", ndof},
                                              {"par_REG", 10 * num_nodes}};
    for (const auto& [name, size] : sizes) {
        if (!parameters.count(name)) throw std::runtime_error("ros_control_generator: the robot has no '" + name + "'." + hint);
        const Parameter& parameter = parameters.at(name);
        if (parameter.size() != size || parameter.symb_size() != size) {
            throw std::runtime_error("ros_control_generator: '" + name + "' must have " + std::to_string(size) +
                                     " runtime (symbolic) entries, it has " + std::to_string(parameter.symb_size()) + ".");
        }
    }
    for (const std::string name : {"Yr", "Y", "reg_G"}) {
        if (!functions.count(name)) {
            throw std::runtime_error("ros_control_generator: the robot has no function '" + name + "' (or it is ignored in robot_generator)." + hint);
        }
        const Function& function = functions.at(name);
        const std::vector<long> out = function.get_out_size();
        if (!function.explicit_args.empty() || out.size() != 2 || out[0] != ndof || out[1] != 10 * num_nodes) {
            throw std::runtime_error("ros_control_generator: function '" + name + "' must have no explicit arguments and size " +
                                     std::to_string(ndof) + " x " + std::to_string(10 * num_nodes) + ".");
        }
    }
}

// URDF joint of each entry of q: given in the configuration, or the moving nodes of urdf_loader's jointsUrdfName.
std::vector<std::string> joint_names(const GeneratorConfig& settings, Robot& robot, int ndof) {
    std::vector<std::string> joints = settings.joints;
    if (joints.empty()) {
        if (!robot.properties.count("jointsUrdfName")) {
            throw std::runtime_error("ros_control_generator.joints is required: only urdf_loader gives the URDF joint names.");
        }
        const auto urdf_names = robot.get<std::vector<std::string>>("jointsUrdfName");
        const auto dimensions = robot.get<std::vector<int>>("jointsDimension");
        for (std::size_t node = 0; node < dimensions.size(); ++node) {
            if (dimensions[node] > 1) throw std::runtime_error("ros_control_generator: node " + std::to_string(node) + " has more than one joint variable; give ros_control_generator.joints.");
            if (dimensions[node] == 1) joints.push_back(urdf_names[node]);
        }
    }
    if (static_cast<int>(joints.size()) != ndof) {
        throw std::runtime_error("ros_control_generator: " + std::to_string(joints.size()) + " joint names for " + std::to_string(ndof) + " joints.");
    }
    for (const auto& joint : joints) {
        if (joint.empty()) throw std::runtime_error("ros_control_generator: empty joint name.");
    }
    return joints;
}

// parameters: {a: {b: 1}} -> {"a.b": node}
void flatten(const YAML::Node& node, const std::string& prefix, std::map<std::string, YAML::Node>& out) {
    for (const auto& entry : node) {
        const std::string key = prefix + entry.first.as<std::string>();
        if (entry.second.IsMap()) flatten(entry.second, key + ".", out);
        else out[key] = entry.second;
    }
}

void check_parameters(const YAML::Node& parameters, int ndof, int num_nodes) {
    std::map<std::string, YAML::Node> given;
    flatten(parameters, "", given);
    for (const auto& [name, value] : given) {
        if (!CONTROLLER_PARAMETERS.count(name)) {
            throw std::runtime_error("ros_control_generator.parameters: unknown parameter '" + name + "'.");
        }
        const Kind kind = CONTROLLER_PARAMETERS.at(name).kind;
        try {
            int size = -1;
            if (kind == Kind::VECTOR_NDOF) size = ndof;
            if (kind == Kind::VECTOR_NODES) size = num_nodes;
            if (kind == Kind::VECTOR_10) size = 10;
            if (size >= 0) {
                const auto values = value.as<std::vector<double>>();
                if (static_cast<int>(values.size()) != size) {
                    throw std::runtime_error("it has " + std::to_string(values.size()) + " entries, expected " + std::to_string(size));
                }
            }
            if (kind == Kind::DOUBLE) value.as<double>();
            if (kind == Kind::BOOL) value.as<bool>();
            if (kind == Kind::STRING) value.as<std::string>();
        } catch (const std::exception& error) {
            throw std::runtime_error("ros_control_generator.parameters." + name + ": " + error.what());
        }
    }
    for (const auto& [name, spec] : CONTROLLER_PARAMETERS) {
        if (spec.required && !given.count(name)) {
            throw std::runtime_error("ros_control_generator.parameters." + name + " is required.");
        }
    }
}

// Parameter file of the controller, to load with the controller manager.
std::string parameter_file(const std::string& package_name, const std::string& robot_name,
                           const std::vector<std::string>& joints, const YAML::Node& parameters,
                           const std::vector<std::string>& nodes) {
    // type: in the controller manager's section (parameters loaded into the controller manager)
    // and in the controller's own (Jazzy spawner --param-file)
    YAML::Node controller;
    controller["type"] = package_name + "/MactController";
    controller["joints"] = joints;
    for (const auto& entry : parameters) controller[entry.first.as<std::string>()] = YAML::Clone(entry.second);

    YAML::Node root;
    root["controller_manager"]["ros__parameters"][package_name]["type"] = package_name + "/MactController";
    root[package_name]["ros__parameters"] = controller;

    YAML::Emitter emitter;
    emitter.SetSeqFormat(YAML::Flow);
    emitter << root;

    std::string node_list = boost::algorithm::join(nodes, ", ");
    return "# Generated by Thunder Dynamics (ros_control_generator) for robot " + robot_name + ".\n"
           "# Parameters of the controller '" + package_name + "'; see docs/plugins/ros_control_plugin.md.\n"
           "# par_REG has 10 parameters per node, and adaptation.R_p_link one gain per node, in this order:\n"
           "#   " + node_list + "\n" +
           emitter.c_str() + "\n";
}

std::string replace_placeholders(std::string text, const std::map<std::string, std::string>& values) {
    for (const auto& [placeholder, value] : values) replace_all(text, placeholder, value);
    return text;
}

}  // namespace

void ROSControlGenerator::generate(const std::shared_ptr<Robot> robot) {
    const GeneratorConfig settings = parse_config(config_);
    const std::string robot_name = robot->robotName;
    const std::string package_name = boost::algorithm::to_lower_copy(robot_name) + "_controller";
    std::cout << "Generating ros2_control package '" << package_name << "' for robot '" << robot_name << "'...\n";
    if (!is_valid_ros_package_name(package_name)) {
        throw std::runtime_error("ROS 2 package names must be lowercase letters, digits, and underscores. Robot name '" +
                                 robot_name + "' cannot produce a standard package name.");
    }

    // - check the model and the configuration - //
    const int ndof = robot->get<int>("ndof");
    const int num_nodes = robot->get<int>("numJoints");
    const std::map<std::string, Function> functions = generated_functions(*robot);
    check_model(functions, robot->parameters, ndof, num_nodes);
    const std::vector<std::string> joints = joint_names(settings, *robot, ndof);
    check_parameters(settings.parameters, ndof, num_nodes);

    // - copy the robot library written by robot_generator - //
    const fs::path output_root = fs::current_path() / (robot_name + "_generatedFiles");
    const fs::path package_root = output_root / package_name;
    const fs::path include_dir = package_root / "include" / package_name;
    if (fs::exists(output_root / "pyproject.toml")) {
        throw std::runtime_error("ros_control_generator cannot use a Python-enabled robot_generator output. Set robot_generator.gen_python to false.");
    }
    copy_generated_file(output_root / ("thunder_" + robot_name + ".h"), include_dir / ("thunder_" + robot_name + ".h"));
    copy_generated_file(output_root / (robot_name + "_gen.h"), include_dir / (robot_name + "_gen.h"));
    copy_generated_file(output_root / ("thunder_" + robot_name + ".cpp"), package_root / "src" / ("thunder_" + robot_name + ".cpp"));
    copy_generated_file(output_root / (robot_name + "_gen.cpp"), package_root / "src" / (robot_name + "_gen.cpp"));
    copy_generated_file(output_root / (robot_name + "_conf.yaml"), package_root / "config" / (robot_name + "_conf.yaml"));
    copy_generated_file(output_root / (robot_name + "_par.yaml"), package_root / "config" / (robot_name + "_par.yaml"));

    // - controller from the templates - //
    std::string joint_list;
    for (const auto& joint : joints) joint_list += (joint_list.empty() ? "\"" : ", \"") + joint + "\"";
    const std::map<std::string, std::string> values = {
        {"<PACKAGE>", package_name},
        {"<ROBOT>", robot_name},
        {"<NDOF>", std::to_string(ndof)},
        {"<NUM_NODES>", std::to_string(num_nodes)},
        {"<NPAR>", std::to_string(10 * num_nodes)},
        {"<JOINT_NAMES>", joint_list},
    };
    for (const auto& [source, destination] : TEMPLATE_FILES) {
        const std::string text = read_file(fs::path(settings.template_path) / source);
        write_file(package_root / replace_placeholders(destination, values), replace_placeholders(text, values));
    }

    // - parameters - //
    const auto nodes = robot->get<std::vector<std::string>>("jointsName");
    write_file(package_root / "config" / (package_name + ".yaml"),
               parameter_file(package_name, robot_name, joints, settings.parameters, nodes));

    if (settings.copy_gen && fs::exists("/home/thunder_dev/thunder_dynamics")) {
        const fs::path destination = fs::path("/home/thunder_dev/thunder_dynamics") / "src" / "thunder_ros_control_test" / "src" / package_name;
        fs::create_directories(destination.parent_path());
        fs::copy(package_root, destination, fs::copy_options::recursive | fs::copy_options::overwrite_existing);
    }
    debug_log("Generated ros2_control package " + package_name, VERB_INFO);
}

}  // namespace thunder_ns
