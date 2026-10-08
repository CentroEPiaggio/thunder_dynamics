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

// Pinocchio dynamics from the python helper.
// Returns the helper exit code: 0 ok, 1 pinocchio not available, other values are errors.
static int computePinocchioDynamics(const std::string& urdfPath,
                                    const casadi::DM& q,
                                    const casadi::DM& dq,
                                    Dynamics& out) {
    std::ostringstream command;
    command.precision(17);
    command << "python3 " << PINOCCHIO_HELPER_SCRIPT << " \"" << urdfPath << "\"";
    for (const casadi::DM* v : {&q, &dq}) {
        for (int i = 0; i < v->numel(); ++i) command << " " << static_cast<double>((*v)(i));
    }

    FILE* pipe = popen(command.str().c_str(), "r");
    if (!pipe) return -1;
    std::vector<double> values;
    double scalar = 0.0;
    while (fscanf(pipe, "%lf", &scalar) == 1) values.push_back(scalar);
    const int status = pclose(pipe);
    const int exitCode = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
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

}  // namespace thunder_test
