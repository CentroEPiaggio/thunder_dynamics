#include <filesystem>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include "plugin_manager.h"

using namespace thunder_ns;
namespace fs = std::filesystem;

TEST(ThunderPipeline, NoGenerationBuildsRobot) {
#ifndef THUNDER_SOURCE_DIR
    GTEST_SKIP() << "THUNDER_SOURCE_DIR not defined.";
#endif

    const fs::path config_path = fs::path(THUNDER_SOURCE_DIR) / "src/thunder/tests/fixtures/RRR/testRRR.yaml";
    YAML::Node config = YAML::LoadFile(config_path.string());

    PluginManager manager;
    manager.set_verbose(false);
    manager.configure_pipeline(config, 1);

    auto robot = manager.execute("test_rrr_no_generation");
    ASSERT_NE(robot, nullptr) << "Robot pointer is null";
    EXPECT_EQ(robot->get<int>("numJoints"), 3) << "Expected three joints";

    const auto joints_type = robot->get<std::vector<std::string>>("jointsType");
    EXPECT_EQ(joints_type.size(), 3U) << "Expected three joint types";

    EXPECT_GT(robot->properties.count("jointsName"), 0U) << "Missing jointsName property";
    EXPECT_GT(robot->parameters.count("par_KIN"), 0U) << "Missing par_KIN parameter";
    EXPECT_GT(robot->parameters.count("par_DYN"), 0U) << "Missing par_DYN parameter";
    EXPECT_GT(robot->functions.count("M"), 0U) << "Missing mass matrix function";
    EXPECT_GT(robot->functions.count("C"), 0U) << "Missing coriolis function";
    EXPECT_GT(robot->functions.count("G"), 0U) << "Missing gravity function";
}
