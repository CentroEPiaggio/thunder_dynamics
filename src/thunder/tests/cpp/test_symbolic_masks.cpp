#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "plugin_manager.h"

using namespace thunder_ns;
namespace fs = std::filesystem;

// Symbolic masks of urdf_loader, fixtures/franka/symbolic_mask_test.yaml on franka_symbolic.urdf.
// YAML masks are keyed by node name and win over the URDF tags. In the URDF, <symbolic_kinematics> inside a
// <joint> sets the frame of the node of its parent link (the frame of a node is the origin of the joint that
// leaves its link), <symbolic_dynamics> inside a <link> the inertial parameters of that link.
TEST(UrdfLoader, SymbolicMasks) {
#ifndef THUNDER_SOURCE_DIR
    GTEST_SKIP() << "THUNDER_SOURCE_DIR not defined.";
#else
    const fs::path config_path = fs::path(THUNDER_SOURCE_DIR) / "src/thunder/tests/fixtures/franka/symbolic_mask_test.yaml";

    PluginManager manager;
    manager.set_verbose(false);
    manager.configure_pipeline(config_path.string(), 1);
    auto robot = manager.execute("symbolic_mask_test");
    ASSERT_NE(robot, nullptr);

    const int numJoints = robot->get<int>("numJoints");
    const auto jointsName = robot->get<std::vector<std::string>>("jointsName");
    std::unordered_map<std::string, int> nodeIndex;
    for (int i = 0; i < static_cast<int>(jointsName.size()); ++i) nodeIndex[jointsName[i]] = i;

    const auto& parDYN = robot->parameters.at("par_DYN");
    ASSERT_EQ(static_cast<int>(parDYN.is_symbolic.size()), 10 * numJoints) << "par_DYN mask size";

    // par_KIN of a node is built from KIN_<node>_xyz and KIN_<node>_rpy: their masks are the KIN mask
    auto kin_mask = [&](const std::string& node) {
        std::vector<short> mask = robot->parameters.at("KIN_" + node + "_xyz").is_symbolic;
        const auto& rpy = robot->parameters.at("KIN_" + node + "_rpy").is_symbolic;
        mask.insert(mask.end(), rpy.begin(), rpy.end());
        return mask;
    };
    auto dyn_mask = [&](int i) {
        return std::vector<short>(parDYN.is_symbolic.begin() + 10 * i, parDYN.is_symbolic.begin() + 10 * i + 10);
    };

    struct Expected {
        std::string node;
        std::vector<short> kin, dyn;
    };
    const std::vector<Expected> expected = {
        // YAML override
        {"panda_link0", {0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0,0}},
        {"panda_link1", {1,1,1,1,1,1}, {1,1,1,1,1,1,1,1,1,1}},
        {"panda_link2", {1,0,1,0,1,0}, {1,0,1,0,1,0,1,0,1,0}},
        // default (no tag)
        {"panda_link3", {0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0,0}},
        {"panda_link4", {0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0,0}},
        // URDF tags: kinematics from the joint leaving the link, dynamics from the link
        {"panda_link5", {0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0,0}},  // panda_joint6 tag + panda_link5 tag
        {"panda_link6", {1,1,1,1,1,1}, {1,1,1,1,1,1,1,1,1,1}},  // panda_joint7 tag + panda_link6 tag
        {"panda_link7", {1,0,1,0,1,0}, {1,0,1,0,1,0,1,0,1,0}},  // panda_joint8 tag + panda_link7 tag
    };

    for (const auto& e : expected) {
        SCOPED_TRACE(e.node);
        ASSERT_EQ(nodeIndex.count(e.node), 1U) << "node not found";
        EXPECT_EQ(kin_mask(e.node), e.kin) << "KIN mask";
        EXPECT_EQ(dyn_mask(nodeIndex[e.node]), e.dyn) << "DYN mask";
    }
#endif
}
