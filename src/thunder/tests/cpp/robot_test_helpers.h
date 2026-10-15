#pragma once

#include <algorithm>
#include <cstdio>
#include <sys/wait.h>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <casadi/casadi.hpp>

#include "plugin_manager.h"
#include "robot.h"

namespace thunder_test {

static std::shared_ptr<thunder_ns::Robot> loadRobotFromYaml(const std::string& yamlPath) {
    thunder_ns::PluginManager manager;
    manager.set_verbose(false);
    // Tests do not need generated artifacts, so skip generators.
    manager.configure_pipeline(yamlPath, 1);
    std::filesystem::path p(yamlPath);
    return manager.execute(p.stem().string());
}

// Dynamics terms of tau = M ddq + C dq + G, with Cdq = C dq
struct Dynamics {
    casadi::DM M, C, Cdq, G;
};

// Thunder dynamics at the q, dq currently set in the robot.
static Dynamics computeThunderDynamics(const std::shared_ptr<thunder_ns::Robot>& robot) {
    return {robot->get("M"), robot->get("C"), robot->get("Cdq"), robot->get("G")};
}

// Runs the python helper on the URDF with the given options and vectors, reads the printed values.
// Returns the helper exit code: 0 ok, 1 pinocchio not available, other values are errors.
static int runPinocchioHelper(const std::string& urdfPath,
                              const std::string& options,
                              const std::vector<const casadi::DM*>& vectors,
                              std::vector<double>& values) {
    std::ostringstream command;
    command.precision(17);
    command << "python3 " << PINOCCHIO_HELPER_SCRIPT << " \"" << urdfPath << "\" " << options;
    for (const casadi::DM* v : vectors) {
        for (int i = 0; i < v->numel(); ++i) command << " " << static_cast<double>((*v)(i));
    }

    FILE* pipe = popen(command.str().c_str(), "r");
    if (!pipe) return -1;
    values.clear();
    double scalar = 0.0;
    while (fscanf(pipe, "%lf", &scalar) == 1) values.push_back(scalar);
    const int status = pclose(pipe);
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

// Pinocchio dynamics from the python helper.
// Returns the helper exit code: 0 ok, 1 pinocchio not available, other values are errors.
static int computePinocchioDynamics(const std::string& urdfPath,
                                    const casadi::DM& q,
                                    const casadi::DM& dq,
                                    Dynamics& out) {
    std::vector<double> values;
    const int exitCode = runPinocchioHelper(urdfPath, "", {&q, &dq}, values);
    if (exitCode != 0) return exitCode;

    // M and C column-major, then Cdq and G
    const int n = q.numel();
    if (static_cast<int>(values.size()) != 2 * n * n + 2 * n) return -1;
    auto block = [&](int start, int rows, int cols) {
        return casadi::DM::reshape(casadi::DM(std::vector<double>(values.begin() + start, values.begin() + start + rows * cols)), rows, cols);
    };
    out = {block(0, n, n), block(n * n, n, n), block(2 * n * n, n, 1), block(2 * n * n + n, n, 1)};
    return 0;
}

// Pose in the world and Jacobian ([linear velocity of the origin; angular velocity], world axes) of a frame
struct Frame {
    casadi::DM T, J;
};

// Pinocchio frames of the given URDF links from the python helper.
// Returns the helper exit code: 0 ok, 1 pinocchio not available, other values are errors.
static int computePinocchioFrames(const std::string& urdfPath,
                                  const casadi::DM& q,
                                  const std::vector<std::string>& links,
                                  std::vector<Frame>& out) {
    std::string options = "--frames ";
    for (size_t k = 0; k < links.size(); ++k) options += (k ? "," : "") + links[k];
    std::vector<double> values;
    const int exitCode = runPinocchioHelper(urdfPath, options, {&q}, values);
    if (exitCode != 0) return exitCode;

    // for each link: T (4x4) and J (6 x n), column-major
    const int n = q.numel();
    const int perFrame = 16 + 6 * n;
    if (values.size() != links.size() * perFrame) return -1;
    auto block = [&](int start, int rows, int cols) {
        return casadi::DM::reshape(casadi::DM(std::vector<double>(values.begin() + start, values.begin() + start + rows * cols)), rows, cols);
    };
    out.clear();
    for (size_t k = 0; k < links.size(); ++k) {
        const int start = k * perFrame;
        out.push_back({block(start, 4, 4), block(start + 16, 6, n)});
    }
    return 0;
}

}  // namespace thunder_test
