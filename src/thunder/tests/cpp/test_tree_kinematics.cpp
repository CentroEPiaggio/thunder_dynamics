#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include "plugin_manager.h"

using namespace thunder_ns;
namespace fs = std::filesystem;

static std::shared_ptr<Robot> load_robot_from_urdf(const fs::path& urdf_path, const std::string& robot_name) {
    YAML::Node config;
    config["pipeline"]["loaders"] = std::vector<std::string>{"urdf_loader"};
    config["pipeline"]["builders"] = std::vector<std::string>{"kin_builder"};
    config["pipeline"]["generators"] = std::vector<std::string>{};
    config["urdf_loader"]["urdf_path"] = urdf_path.string();
    config["urdf_loader"]["base_link"] = "base_link";

    PluginManager manager;
    manager.set_verbose(false);
    manager.configure_pipeline(config, 1);
    return manager.execute(robot_name);
}

static int index_of(const std::vector<std::string>& names, const std::string& target) {
    for (int i = 0; i < static_cast<int>(names.size()); ++i) {
        if (names[i] == target) {
            return i;
        }
    }
    return -1;
}

static void check_tree_robot(const std::shared_ptr<Robot>& robot) {
    const auto names = robot->get<std::vector<std::string>>("jointsName");
    const auto parents = robot->get<std::vector<int>>("jointsParent");
    const auto available = robot->get<std::vector<bool>>("jointsAvailable");

    const int base = index_of(names, "base_link");
    const int left = index_of(names, "left_link");
    const int right = index_of(names, "right_link");
    const int left_tip = index_of(names, "left_tip");
    const int right_tip = index_of(names, "right_tip");
    const int ghost = index_of(names, "base_link_1");

    EXPECT_EQ(base, 0) << "tree robot should start at base_link";
    ASSERT_TRUE(left >= 0 && right >= 0 && left_tip >= 0 && right_tip >= 0) << "missing tree links";
    ASSERT_GE(ghost, 0) << "tree robot should contain a branch ghost node";

    EXPECT_EQ(parents[left], base) << "left_link should be attached to base_link";
    EXPECT_EQ(parents[right], base) << "right_link should be attached to base_link";
    EXPECT_EQ(parents[left_tip], left) << "left_tip should be attached to left_link";
    EXPECT_EQ(parents[right_tip], right) << "right_tip should be attached to right_link";
    EXPECT_EQ(parents[ghost], base) << "ghost node should reuse base_link parent";

    EXPECT_FALSE(available[ghost]) << "ghost node should not be available for alias functions";
    EXPECT_EQ(robot->functions.count("T_w_left_tip"), 1U) << "missing left_tip world transform";
    EXPECT_EQ(robot->functions.count("T_w_right_tip"), 1U) << "missing right_tip world transform";
    EXPECT_EQ(robot->functions.count("T_w_left_link"), 0U) << "intermediate left_link should not have an alias transform";
    EXPECT_EQ(robot->functions.count("T_w_right_link"), 0U) << "intermediate right_link should not have an alias transform";
    EXPECT_EQ(robot->functions.count("T_w_base_link_1"), 0U) << "ghost node should not have alias transform";
}

static void check_serial_robot(const std::shared_ptr<Robot>& robot) {
    const auto names = robot->get<std::vector<std::string>>("jointsName");
    const auto parents = robot->get<std::vector<int>>("jointsParent");

    const int base = index_of(names, "base_link");
    const int link1 = index_of(names, "link1");
    const int link2 = index_of(names, "link2");
    const int tip = index_of(names, "tool0");

    EXPECT_EQ(base, 0) << "serial robot should start at base_link";
    ASSERT_TRUE(link1 >= 0 && link2 >= 0 && tip >= 0) << "missing serial links";
    EXPECT_LT(index_of(names, "base_link_1"), 0) << "serial chain should not contain a branch ghost node";

    EXPECT_EQ(parents[link1], base) << "link1 should be attached to base_link";
    EXPECT_EQ(parents[link2], link1) << "link2 should be attached to link1";
    EXPECT_EQ(parents[tip], link2) << "tool0 should be attached to link2";
    EXPECT_EQ(robot->functions.count("T_w_tool0"), 1U) << "missing tool0 world transform";
}

TEST(ThunderPipeline, TreeAndSerialKinematicsConsistency) {
#ifndef THUNDER_SOURCE_DIR
    GTEST_SKIP() << "THUNDER_SOURCE_DIR not defined.";
#endif

    try {
        const fs::path fixtures = fs::path(THUNDER_SOURCE_DIR) / "src/thunder/tests/fixtures/urdf";

        auto tree_robot = load_robot_from_urdf(fixtures / "tree_branch.urdf", "tree_branch");
        ASSERT_NE(tree_robot, nullptr) << "tree robot pointer is null";
        EXPECT_GE(tree_robot->get<int>("numJoints"), 5) << "tree robot should expose the full branch structure";
        check_tree_robot(tree_robot);

        auto serial_robot = load_robot_from_urdf(fixtures / "serial_chain.urdf", "serial_chain");
        ASSERT_NE(serial_robot, nullptr) << "serial robot pointer is null";
        EXPECT_GE(serial_robot->get<int>("numJoints"), 4) << "serial robot should expose the full chain structure";
        check_serial_robot(serial_robot);
    } catch (const std::exception& e) {
        ADD_FAILURE() << "Unexpected exception: " << e.what();
    }
}
