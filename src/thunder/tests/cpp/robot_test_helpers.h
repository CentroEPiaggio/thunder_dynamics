#pragma once

#include <algorithm>
#include <cstdio>
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

static bool setSymbolicParameterDirect(const std::shared_ptr<thunder_ns::Robot>& robot,
                                       const std::string& parameterName,
                                       const casadi::DM& symbolicValues) {
    if (!robot || robot->parameters.count(parameterName) == 0) {
        return false;
    }

    auto& parameter = robot->parameters.at(parameterName);
    if (parameter.symb_size() != symbolicValues.numel()) {
        return false;
    }

    int symbolicIndex = 0;
    for (int i = 0; i < parameter.size(); ++i) {
        if (parameter.is_symbolic[i]) {
            parameter.num(i) = symbolicValues(symbolicIndex++);
        }
    }
    return true;
}

static bool computePinocchioTau(const std::string& urdfPath,
                                const casadi::DM& q,
                                const casadi::DM& dq,
                                const casadi::DM& ddq,
                                casadi::DM& tauOut) {
    std::ostringstream command;
    command << "python3 " << PINOCCHIO_HELPER_SCRIPT << " \"" << urdfPath << "\"";

    auto appendValues = [&](const casadi::DM& value) {
        for (int i = 0; i < value.numel(); ++i) {
            command << " " << static_cast<double>(value(i));
        }
    };

    appendValues(q);
    appendValues(dq);
    appendValues(ddq);

    FILE* pipe = popen(command.str().c_str(), "r");
    if (!pipe) {
        return false;
    }

    std::vector<double> values;
    double scalar = 0.0;
    while (fscanf(pipe, "%lf", &scalar) == 1) {
        values.push_back(scalar);
    }

    const int returnCode = pclose(pipe);
    if (returnCode != 0 || values.empty()) {
        return false;
    }

    tauOut = casadi::DM(values);
    return true;
}

static casadi::DM computeThunderTau(const std::shared_ptr<thunder_ns::Robot>& robot) {
    casadi::DM M = robot->get("M");
    casadi::DM C = robot->get("C");
    casadi::DM G = robot->get("G");
    casadi::DM dq = robot->get("dqr");
    casadi::DM ddq = robot->get("ddqr");

    auto ensureColumn = [](casadi::DM& value) {
        if (value.size1() == 1 && value.size2() > 1) {
            value = value.T();
        }
    };

    ensureColumn(dq);
    ensureColumn(ddq);
    ensureColumn(G);

    return M * ddq + C * dq + G;
}

}  // namespace thunder_test
