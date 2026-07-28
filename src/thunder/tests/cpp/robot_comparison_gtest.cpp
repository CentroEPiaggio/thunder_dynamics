#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include <casadi/casadi.hpp>
#include <yaml-cpp/yaml.h>

#include "robot_test_helpers.h"

using thunder_test::computePinocchioTau;
using thunder_test::computeThunderTau;
using thunder_test::setSymbolicParameterDirect;

static casadi::DM randomColumnVector(int n) {
    return casadi::DM::rand(n, 1);
}

TEST(RobotComparison, FrankaUrdfRNEA) {
#ifdef THUNDER_SOURCE_DIR
    const std::string sourceRoot = THUNDER_SOURCE_DIR;
#else
    GTEST_SKIP() << "THUNDER_SOURCE_DIR is not defined";
#endif

    const std::string fixtureRoot = sourceRoot + "/src/thunder/tests/fixtures/franka";
    const std::string yamlPath = fixtureRoot + "/franka_urdf.yaml";
    const std::string urdfPath = fixtureRoot + "/franka.urdf";

    YAML::Node config = YAML::LoadFile(yamlPath);
    config["urdf_loader"]["urdf_path"] = urdfPath;

    thunder_ns::PluginManager manager;
    manager.set_verbose(false);
    manager.configure_pipeline(config, 1);
    auto robot = manager.execute("franka_fixture");
    ASSERT_NE(robot, nullptr);

    const int ndof = robot->get<int>("ndof");
    ASSERT_GT(ndof, 0);

    int stateSize = ndof;
    if (robot->parameters.count("q") > 0) {
        stateSize = std::max(stateSize, robot->parameters.at("q").symb_size());
    }
    stateSize = std::max(stateSize, 1);

    casadi::DM q = randomColumnVector(stateSize);
    casadi::DM dq = randomColumnVector(stateSize);
    casadi::DM ddq = randomColumnVector(stateSize);

    // Prefer direct symbolic writes to avoid warnings from partial vector updates.
    if (!setSymbolicParameterDirect(robot, "q", q)) {
        robot->set("q", q);
    }
    if (!setSymbolicParameterDirect(robot, "dqr", dq)) {
        robot->set("dqr", dq);
    }
    if (!setSymbolicParameterDirect(robot, "ddqr", ddq)) {
        robot->set("ddqr", ddq);
    }

    casadi::DM tauThunder = computeThunderTau(robot);

    casadi::DM tauPinocchio;
    const bool pinocchioOk = computePinocchioTau(urdfPath, q, dq, ddq, tauPinocchio);
    if (!pinocchioOk) {
        GTEST_SKIP() << "Pinocchio helper failed or pinocchio is unavailable";
    }

    const int compareSize = std::min(tauThunder.numel(), tauPinocchio.numel());
    ASSERT_GT(compareSize, 0);

    const double tolerance = 1e-2;
    for (int i = 0; i < compareSize; ++i) {
        const double thunderValue = static_cast<double>(tauThunder(i));
        const double pinocchioValue = static_cast<double>(tauPinocchio(i));
        EXPECT_NEAR(thunderValue, pinocchioValue, tolerance)
            << "Mismatch at torque component " << i;
    }
}

