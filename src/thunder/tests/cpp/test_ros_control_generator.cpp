#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "plugin_manager.h"

using namespace thunder_ns;
namespace fs = std::filesystem;

#ifdef THUNDER_SOURCE_DIR
namespace {

struct CwdGuard {
    explicit CwdGuard(fs::path path) : previous(fs::current_path()) { fs::current_path(std::move(path)); }
    ~CwdGuard() { fs::current_path(previous); }
    fs::path previous;
};

const fs::path FIXTURE = fs::path(THUNDER_SOURCE_DIR) / "src/thunder/tests/fixtures/urdf/serial_chain_ros_control.yaml";
const fs::path TEMPLATES = fs::path(THUNDER_SOURCE_DIR) / "src/thunder/data/templates/ros_control_generator";

// The fixture, with the templates of this repository and the URDF path made absolute (the test runs elsewhere).
YAML::Node fixture() {
    YAML::Node config = YAML::LoadFile(FIXTURE.string());
    config["urdf_loader"]["urdf_path"] = (FIXTURE.parent_path() / "serial_chain.urdf").string();
    config["ros_control_generator"]["template_path"] = TEMPLATES.string();
    return config;
}

// Runs the pipeline in a fresh temporary folder; returns the folder (removed by the caller).
fs::path run(const YAML::Node& config) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const fs::path root = fs::temp_directory_path() / ("thunder_ros_control_" + std::to_string(stamp));
    fs::create_directories(root);
    CwdGuard guard(root);
    PluginManager manager;
    manager.set_verbose(false);
    manager.configure_pipeline(config, 0);
    try {
        manager.execute("chain");
    } catch (...) {
        fs::remove_all(root);
        throw;
    }
    return root;
}

// The pipeline must stop with an error whose message contains `expected`.
void expect_error(const YAML::Node& config, const std::string& expected) {
    try {
        fs::remove_all(run(config));
        ADD_FAILURE() << "no error, expected one about: " << expected;
    } catch (const std::runtime_error& error) {
        EXPECT_NE(std::string(error.what()).find(expected), std::string::npos) << error.what();
    }
}

std::string read(const fs::path& path) {
    std::ifstream file(path);
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

}  // namespace

TEST(RosControlGenerator, WritesThePackage) {
    YAML::Node config = fixture();
    config["ros_control_generator"]["gen_test"] = true;
    const fs::path root = run(config);
    const fs::path package = root / "chain_generatedFiles" / "chain_controller";

    for (const char* file : {"CMakeLists.txt", "package.xml", "chain_controller.xml", "src/controller.cpp",
                             "test/test_algorithm.cpp", "include/chain_controller/controller.hpp",
                             "include/chain_controller/mact.hpp", "include/chain_controller/trajectory.hpp",
                             "include/chain_controller/filter.hpp", "config/chain_controller.yaml"}) {
        EXPECT_TRUE(fs::exists(package / file)) << "missing " << file;
    }
    // the robot is not in the controller package: it links the library package thunder_chain
    EXPECT_FALSE(fs::exists(package / "src/chain_gen.cpp"));
    const fs::path library = root / "chain_generatedFiles" / "thunder_chain";
    for (const char* file : {"CMakeLists.txt", "package.xml", "src/thunder_chain.cpp", "src/chain_gen.cpp",
                             "include/thunder_chain/thunder_chain.h", "include/thunder_chain/chain_gen.h",
                             "config/chain_conf.yaml", "config/chain_par.yaml"}) {
        EXPECT_TRUE(fs::exists(library / file)) << "missing thunder_chain/" << file;
    }
    EXPECT_NE(read(package / "CMakeLists.txt").find("thunder_chain::thunder_chain"), std::string::npos);
    EXPECT_NE(read(package / "package.xml").find("<depend>thunder_chain</depend>"), std::string::npos);
    EXPECT_NE(read(package / "include/chain_controller/controller.hpp").find("#include <thunder_chain/thunder_chain.h>"), std::string::npos);

    // every placeholder replaced
    for (const auto& entry : fs::recursive_directory_iterator(package)) {
        if (!entry.is_regular_file()) continue;
        const std::string text = read(entry.path());
        for (const char* placeholder : {"<ROBOT>", "<PACKAGE>", "<LIBRARY>", "<NDOF>", "<NUM_NODES>", "<NPAR>", "<JOINT_NAMES>"}) {
            EXPECT_EQ(text.find(placeholder), std::string::npos) << placeholder << " left in " << entry.path();
        }
    }

    // sizes and URDF joint names (from urdf_loader's jointsUrdfName)
    const std::string header = read(package / "include/chain_controller/controller.hpp");
    EXPECT_NE(header.find("kNdof = 2;"), std::string::npos);
    EXPECT_NE(header.find("kNumNodes = 4;"), std::string::npos);
    EXPECT_NE(header.find("kNpar = 40;"), std::string::npos);
    EXPECT_NE(header.find("{\"joint1\", \"joint2\"}"), std::string::npos);

    // parameter file
    const YAML::Node parameters = YAML::LoadFile((package / "config/chain_controller.yaml").string());
    EXPECT_EQ(parameters["controller_manager"]["ros__parameters"]["chain_controller"]["type"].as<std::string>(),
              "chain_controller/MactController");
    const YAML::Node controller = parameters["chain_controller"]["ros__parameters"];
    EXPECT_EQ(controller["type"].as<std::string>(), "chain_controller/MactController");
    EXPECT_EQ(controller["joints"].as<std::vector<std::string>>(), (std::vector<std::string>{"joint1", "joint2"}));
    EXPECT_EQ(controller["k_p"].as<std::vector<double>>().size(), 2u);
    // integers in the configuration are written as reals: a ROS parameter declared double rejects integers
    EXPECT_EQ(controller["k_p"][0].Scalar(), "10.0");
    // the limits are parameters of the robot class, not of the controller
    EXPECT_FALSE(controller["limits"]);
    EXPECT_NE(read(library / "include/thunder_chain/thunder_chain.h").find("get_ddq_max()"), std::string::npos);
    EXPECT_EQ(controller["adaptation"]["R_p_link"].as<std::vector<double>>().size(), 4u);

    fs::remove_all(root);
}

TEST(RosControlGenerator, TestOnlyWithGenTest) {
    const fs::path root = run(fixture());    // gen_test: false by default
    EXPECT_FALSE(fs::exists(root / "chain_generatedFiles/chain_controller/test/test_algorithm.cpp"));
    EXPECT_TRUE(fs::exists(root / "chain_generatedFiles/chain_controller/src/controller.cpp"));
    fs::remove_all(root);
}

TEST(RosControlGenerator, RequiresTheRobotParameters) {
    YAML::Node config = fixture();
    config["ros_control_generator"]["parameters"].remove("k_p");
    expect_error(config, "parameters.k_p is required");
}

TEST(RosControlGenerator, RejectsWrongSizes) {
    YAML::Node config = fixture();
    config["ros_control_generator"]["parameters"]["k_v"] = std::vector<double>{1.0, 1.0, 1.0};
    expect_error(config, "parameters.k_v: it has 3 entries, expected 2");
}

TEST(RosControlGenerator, RejectsUnknownParameters) {
    YAML::Node config = fixture();
    config["ros_control_generator"]["parameters"]["k_d"] = std::vector<double>{1.0, 1.0};
    expect_error(config, "unknown parameter 'k_d'");
}

TEST(RosControlGenerator, NeedsTheRegressors) {
    YAML::Node config = fixture();
    config["pipeline"]["builders"] = std::vector<std::string>{"kin_builder", "dyn_builder"};
    expect_error(config, "needs reg_builder");
}

TEST(RosControlGenerator, ExplicitJointsWin) {
    YAML::Node config = fixture();
    config["ros_control_generator"]["joints"] = std::vector<std::string>{"a", "b"};
    const fs::path root = run(config);
    const std::string header = read(root / "chain_generatedFiles/chain_controller/include/chain_controller/controller.hpp");
    EXPECT_NE(header.find("{\"a\", \"b\"}"), std::string::npos);
    fs::remove_all(root);
}
#endif
