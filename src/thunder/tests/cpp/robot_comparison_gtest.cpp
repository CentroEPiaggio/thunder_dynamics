#include <gtest/gtest.h>

#include <string>
#include <tuple>

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

// Franka (7 R + fixed + 1 P) fixture with the given builder options
static std::shared_ptr<thunder_ns::Robot> loadFranka(const std::string& fixtureRoot, const std::string& dynamicsMethod,
                                                     const std::string& CMethod, const std::string& regressorMethod) {
    YAML::Node config = YAML::LoadFile(fixtureRoot + "/franka_urdf.yaml");
    config["urdf_loader"]["urdf_path"] = fixtureRoot + "/franka_finger.urdf";
    config["dyn_builder"]["dynamics_method"] = dynamicsMethod;
    config["dyn_builder"]["C_method"] = CMethod;
    config["reg_builder"]["regressor_method"] = regressorMethod;

    thunder_ns::PluginManager manager;
    manager.set_verbose(false);
    manager.configure_pipeline(config, 1);
    return manager.execute("franka_fixture");
}

// M, C, Cdq, G against pinocchio, for each dyn_builder dynamics_method and C_method
class RobotComparison : public ::testing::TestWithParam<std::tuple<std::string, std::string>> {};

TEST_P(RobotComparison, FrankaPinocchio) {
#ifdef THUNDER_SOURCE_DIR
    const std::string fixtureRoot = std::string(THUNDER_SOURCE_DIR) + "/src/thunder/tests/fixtures/franka";
#else
    GTEST_SKIP() << "THUNDER_SOURCE_DIR is not defined";
#endif

    auto robot = loadFranka(fixtureRoot, std::get<0>(GetParam()), std::get<1>(GetParam()), "rnea");
    ASSERT_NE(robot, nullptr);

    const int ndof = robot->get<int>("ndof");
    ASSERT_GT(ndof, 0);

    const casadi::DM q = 4 * casadi::DM::rand(ndof, 1) - 2;
    const casadi::DM dq = 4 * casadi::DM::rand(ndof, 1) - 2;
    robot->set("q", q);
    robot->set("dq", dq);
    const Dynamics thunder = computeThunderDynamics(robot);

    Dynamics pinocchio;
    const int helperCode = computePinocchioDynamics(fixtureRoot + "/franka_finger.urdf", q, dq, pinocchio);
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

INSTANTIATE_TEST_SUITE_P(DynamicsMethod, RobotComparison,
                         ::testing::Combine(::testing::Values("rnea", "crba", "lagrange"), ::testing::Values("rnea", "christoffel")));

// Regressors times par_REG against pinocchio: Y p = M ddq + C dq + G, Yr p = M ddqr + C dqr + G
class RegressorComparison : public ::testing::TestWithParam<std::string> {};

TEST_P(RegressorComparison, FrankaPinocchio) {
#ifdef THUNDER_SOURCE_DIR
    const std::string fixtureRoot = std::string(THUNDER_SOURCE_DIR) + "/src/thunder/tests/fixtures/franka";
#else
    GTEST_SKIP() << "THUNDER_SOURCE_DIR is not defined";
#endif

    auto robot = loadFranka(fixtureRoot, "rnea", "rnea", GetParam());
    ASSERT_NE(robot, nullptr);

    const int ndof = robot->get<int>("ndof");
    ASSERT_GT(ndof, 0);

    const casadi::DM q = 4 * casadi::DM::rand(ndof, 1) - 2;
    const casadi::DM dq = 4 * casadi::DM::rand(ndof, 1) - 2;
    const casadi::DM ddq = 4 * casadi::DM::rand(ndof, 1) - 2;
    const casadi::DM dqr = 4 * casadi::DM::rand(ndof, 1) - 2;
    const casadi::DM ddqr = 4 * casadi::DM::rand(ndof, 1) - 2;
    robot->set("q", q);
    robot->set("dq", dq);
    robot->set("ddq", ddq);
    robot->set("dqr", dqr);
    robot->set("ddqr", ddqr);
    const casadi::DM par = robot->get("par_REG");

    Dynamics pinocchio;
    const int helperCode = computePinocchioDynamics(fixtureRoot + "/franka_finger.urdf", q, dq, pinocchio);
    if (helperCode == 1) {
        GTEST_SKIP() << "Pinocchio is not available";
    }
    ASSERT_EQ(helperCode, 0) << "Pinocchio helper failed";

    const double tolerance = 1e-9;
    expectSame("Y par_REG", mtimes(robot->get("Y"), par), mtimes(pinocchio.M, ddq) + pinocchio.Cdq + pinocchio.G, tolerance);
    expectSame("Yr par_REG", mtimes(robot->get("Yr"), par), mtimes(pinocchio.M, ddqr) + mtimes(pinocchio.C, dqr) + pinocchio.G, tolerance);
    expectSame("reg_M par_REG", mtimes(robot->get("reg_M"), par), mtimes(pinocchio.M, ddqr), tolerance);
    expectSame("reg_C par_REG", mtimes(robot->get("reg_C"), par), mtimes(pinocchio.C, dqr), tolerance);
    expectSame("reg_G par_REG", mtimes(robot->get("reg_G"), par), pinocchio.G, tolerance);
}

INSTANTIATE_TEST_SUITE_P(RegressorMethod, RegressorComparison, ::testing::Values("rnea", "lagrange"));
