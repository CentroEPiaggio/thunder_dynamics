// Joint limits added by the loaders (load_limits): q_min, q_max, dq_max, ddq_max, tau_max, dtau_max.

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "plugin_manager.h"

using namespace thunder_ns;
namespace fs = std::filesystem;

#ifdef THUNDER_SOURCE_DIR
namespace {

const double INF = std::numeric_limits<double>::infinity();

// revolute with <limit>, continuous with <limit> (position ignored), continuous without <limit>
const char* URDF = R"(<?xml version="1.0"?>
<robot name="limits">
  <link name="base"/><link name="l1"/><link name="l2"/><link name="l3"/>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="2" velocity="3" effort="4"/></joint>
  <joint name="j2" type="continuous"><parent link="l1"/><child link="l2"/><axis xyz="0 0 1"/>
    <limit velocity="5" effort="6"/></joint>
  <joint name="j3" type="continuous"><parent link="l2"/><child link="l3"/><axis xyz="0 0 1"/></joint>
</robot>
)";

fs::path temp_dir() {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const fs::path dir = fs::temp_directory_path() / ("thunder_limits_" + std::to_string(stamp));
    fs::create_directories(dir);
    return dir;
}

std::shared_ptr<Robot> run(const YAML::Node& config) {
    PluginManager manager;
    manager.set_verbose(false);
    manager.configure_pipeline(config, 0);
    return manager.execute("limits");
}

YAML::Node urdf_config(const fs::path& dir) {
    std::ofstream(dir / "limits.urdf") << URDF;
    YAML::Node config;
    config["pipeline"]["loaders"] = std::vector<std::string>{"urdf_loader"};
    config["urdf_loader"]["urdf_path"] = (dir / "limits.urdf").string();
    return config;
}

std::vector<double> values(const std::shared_ptr<Robot>& robot, const std::string& name) {
    return robot->parameters.at(name).num.get_elements();
}

void expect_error(const YAML::Node& config, const std::string& expected) {
    try {
        run(config);
        ADD_FAILURE() << "no error, expected one about: " << expected;
    } catch (const std::runtime_error& error) {
        EXPECT_NE(std::string(error.what()).find(expected), std::string::npos) << error.what();
    }
}

}  // namespace

TEST(JointLimits, FromTheUrdf) {
    const fs::path dir = temp_dir();
    const auto robot = run(urdf_config(dir));
    EXPECT_EQ(values(robot, "q_min"), (std::vector<double>{-1, -INF, -INF}));
    EXPECT_EQ(values(robot, "q_max"), (std::vector<double>{2, INF, INF}));
    EXPECT_EQ(values(robot, "dq_max"), (std::vector<double>{3, 5, INF}));
    EXPECT_EQ(values(robot, "tau_max"), (std::vector<double>{4, 6, INF}));
    EXPECT_EQ(values(robot, "ddq_max"), (std::vector<double>{INF, INF, INF}));
    EXPECT_EQ(values(robot, "dtau_max"), (std::vector<double>{INF, INF, INF}));
    for (const char* name : {"q_min", "q_max", "dq_max", "ddq_max", "tau_max", "dtau_max"}) {
        EXPECT_EQ(robot->parameters.at(name).symb_size(), 3) << name << " must be symbolic";
    }
    fs::remove_all(dir);
}

TEST(JointLimits, ConfigurationOverwritesTheUrdf) {
    const fs::path dir = temp_dir();
    YAML::Node config = urdf_config(dir);
    config["urdf_loader"]["limits"]["ddq_max"] = std::vector<double>{1, 2, 3};
    config["urdf_loader"]["limits"]["tau_max"] = std::vector<double>{10, 10, 10};
    const auto robot = run(config);
    EXPECT_EQ(values(robot, "ddq_max"), (std::vector<double>{1, 2, 3}));
    EXPECT_EQ(values(robot, "tau_max"), (std::vector<double>{10, 10, 10}));
    EXPECT_EQ(values(robot, "dq_max"), (std::vector<double>{3, 5, INF}));    // not given: from the URDF
    fs::remove_all(dir);
}

// RRR/testRRR.yaml (kin_loader, dyn_loader), loaders only
YAML::Node rrr_loaders() {
    YAML::Node config = YAML::LoadFile((fs::path(THUNDER_SOURCE_DIR) / "src/thunder/tests/fixtures/RRR/testRRR.yaml").string());
    config["pipeline"]["builders"] = std::vector<std::string>{};
    config["pipeline"]["generators"] = std::vector<std::string>{};
    return config;
}

TEST(JointLimits, KinLoaderHasNoLimits) {
    const auto robot = run(rrr_loaders());
    const int ndof = robot->get<int>("ndof");
    EXPECT_EQ(values(robot, "q_min"), std::vector<double>(ndof, -INF));
    for (const char* name : {"q_max", "dq_max", "ddq_max", "tau_max", "dtau_max"}) {
        EXPECT_EQ(values(robot, name), std::vector<double>(ndof, INF)) << name;
    }
}

TEST(JointLimits, DhLoaderFromTheConfiguration) {
    YAML::Node config = YAML::Load(R"(
pipeline:
  loaders: [dh_loader]
  builders: []
  generators: []
dh_loader:
  num_joints: 2
  joints_type: [R, R]
  DH:
    symb: [0,0,0,0, 0,0,0,0]
    value: [0, 0, 1, 0,  1, 0, 0, 0]
  limits:
    q_max: [1.5, .inf]
)");
    const auto robot = run(config);
    EXPECT_EQ(values(robot, "q_max"), (std::vector<double>{1.5, INF}));
    EXPECT_EQ(values(robot, "q_min"), (std::vector<double>{-INF, -INF}));
}

// Checked with kin_loader (3 joints), which lets the error through: urdf_loader catches the errors of its load(),
// prints them and returns the robot unfinished (its existing behavior), so the pipeline stops later on another error.
TEST(JointLimits, Errors) {
    YAML::Node config = rrr_loaders();
    config["kin_loader"]["limits"]["ddq_max"] = std::vector<double>{1, 2};
    expect_error(config, "limit 'ddq_max' has 2 values, the robot has 3 joints");

    config = rrr_loaders();
    config["kin_loader"]["limits"]["acc_max"] = std::vector<double>{1, 2, 3};
    expect_error(config, "unknown limit 'acc_max'");

    config = rrr_loaders();
    config["kin_loader"]["limits"]["dq_max"] = std::vector<double>{1, -2, 3};
    expect_error(config, "limit 'dq_max' must be non-negative");

    config = rrr_loaders();
    config["kin_loader"]["limits"]["q_min"] = std::vector<double>{3, 0, 0};
    config["kin_loader"]["limits"]["q_max"] = std::vector<double>{1, 1, 1};
    expect_error(config, "q_min > q_max for joint 0");
}

TEST(JointLimits, InfinityInTheGeneratedCodeAndInTheParameterFile) {
    const fs::path dir = temp_dir();
    const auto robot = run(urdf_config(dir));
    // C++ initializer of the generated class
    EXPECT_EQ(robot->parameters.at("dq_max").get_value_str(), "{3, 5, INFINITY}");
    EXPECT_EQ(robot->parameters.at("q_min").get_value_str(), "{-1, -INFINITY, -INFINITY}");
    // parameter file round trip
    const std::string file = (dir / "par.yaml").string();
    ASSERT_EQ(robot->save_par(file, {"q_min", "dq_max"}), 1);
    const auto loaded = run(urdf_config(dir));
    loaded->parameters.at("dq_max").num = casadi::DM::zeros(3, 1);
    loaded->parameters.at("q_min").num = casadi::DM::zeros(3, 1);
    ASSERT_EQ(loaded->load_par(file, {"q_min", "dq_max"}), 1);
    EXPECT_EQ(values(loaded, "dq_max"), (std::vector<double>{3, 5, INF}));
    EXPECT_EQ(values(loaded, "q_min"), (std::vector<double>{-1, -INF, -INF}));
    fs::remove_all(dir);
}
#endif
