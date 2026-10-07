#include <gtest/gtest.h>

#include <string>

#include <casadi/casadi.hpp>
#include <yaml-cpp/yaml.h>

#include "robot_test_helpers.h"

using thunder_test::Dynamics;
using thunder_test::computePinocchioDynamics;
using thunder_test::computeThunderDynamics;

static void expectSame(const std::string& name, const casadi::DM& thunder, const casadi::DM& pinocchio, double tolerance) {
    ASSERT_EQ(thunder.size1(), pinocchio.size1()) << name << " rows";
    ASSERT_EQ(thunder.size2(), pinocchio.size2()) << name << " cols";
    const casadi::DM diff = casadi::DM::reshape(thunder - pinocchio, -1, 1);
    EXPECT_LT(static_cast<double>(casadi::DM::norm_inf(diff)), tolerance) << name << " differs from pinocchio";
}

// Franka (7 R + fixed + 1 P) against pinocchio, for each dyn_builder dynamics_method
class RobotComparison : public ::testing::TestWithParam<std::string> {};

TEST_P(RobotComparison, FrankaPinocchio) {
#ifdef THUNDER_SOURCE_DIR
    const std::string sourceRoot = THUNDER_SOURCE_DIR;
#else
    GTEST_SKIP() << "THUNDER_SOURCE_DIR is not defined";
#endif

    const std::string fixtureRoot = sourceRoot + "/src/thunder/tests/fixtures/franka";
    const std::string yamlPath = fixtureRoot + "/franka_urdf.yaml";
    const std::string urdfPath = fixtureRoot + "/franka_finger.urdf";

    YAML::Node config = YAML::LoadFile(yamlPath);
    config["urdf_loader"]["urdf_path"] = urdfPath;
    config["dyn_builder"]["dynamics_method"] = GetParam();

    thunder_ns::PluginManager manager;
    manager.set_verbose(false);
    manager.configure_pipeline(config, 1);
    auto robot = manager.execute("franka_fixture");
    ASSERT_NE(robot, nullptr);

    const int ndof = robot->get<int>("ndof");
    ASSERT_GT(ndof, 0);

    const casadi::DM q = 4 * casadi::DM::rand(ndof, 1) - 2;
    const casadi::DM dq = 4 * casadi::DM::rand(ndof, 1) - 2;
    robot->set("q", q);
    robot->set("dq", dq);
    const Dynamics thunder = computeThunderDynamics(robot);

    Dynamics pinocchio;
    const int helperCode = computePinocchioDynamics(urdfPath, q, dq, pinocchio);
    if (helperCode == 1) {
        GTEST_SKIP() << "Pinocchio is not available";
    }
    ASSERT_EQ(helperCode, 0) << "Pinocchio helper failed";

    const double tolerance = 1e-9;
    expectSame("M", thunder.M, pinocchio.M, tolerance);
    expectSame("C", thunder.C, pinocchio.C, tolerance);
    expectSame("Cdq", thunder.Cdq, pinocchio.Cdq, tolerance);
    expectSame("G", thunder.G, pinocchio.G, tolerance);
}

INSTANTIATE_TEST_SUITE_P(DynamicsMethod, RobotComparison, ::testing::Values("lagrange", "rnea"));
