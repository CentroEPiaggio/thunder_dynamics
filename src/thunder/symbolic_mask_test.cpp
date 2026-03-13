#include <iostream>
#include <filesystem>

#include "plugin_manager.h"

using namespace thunder_ns;

int main() {
    // NOTE: This test assumes the repository root is available via THUNDER_SOURCE_DIR.
    // The CMake build defines this macro for the test executable.
#ifndef THUNDER_SOURCE_DIR
    std::cerr << "THUNDER_SOURCE_DIR not defined. Please build using CMake." << std::endl;
    return 1;
#endif

    const std::string config_file = std::string(THUNDER_SOURCE_DIR) + "/src/thunder_robot_test/robots/symbolic_mask_test.yaml";

    PluginManager manager;
    manager.set_verbose(false);
    manager.configure_pipeline(config_file, 1); // no generation

    auto robot = manager.execute("symbolic_mask_test");

    const int numJoints = robot->get<int>("numJoints");

    const auto &parKIN = robot->parameters["par_KIN"];
    const auto &parDYN = robot->parameters["par_DYN"];

    bool ok = true;

    if ((int)parKIN.is_symbolic.size() != 6 * numJoints) {
        std::cerr << "Unexpected par_KIN size: " << parKIN.is_symbolic.size() << " (expected " << 6 * numJoints << ")" << std::endl;
        ok = false;
    }
    if ((int)parDYN.is_symbolic.size() < 10 * 2) {
        std::cerr << "Unexpected par_DYN size: " << parDYN.is_symbolic.size() << " (expected at least " << 10 * 2 << ")" << std::endl;
        ok = false;
    }

    // Expect first link to be numeric, second link fully symbolic, third link mixed
    const std::vector<short> expectedKIN0 = {0, 0, 0, 0, 0, 0};
    const std::vector<short> expectedKIN1 = {1, 1, 1, 1, 1, 1};
    const std::vector<short> expectedKIN2 = {1, 0, 1, 0, 1, 0};

    const std::vector<short> expectedDYN0 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    const std::vector<short> expectedDYN1 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    const std::vector<short> expectedDYN2 = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0};

    for (int i = 0; i < 6 && i < (int)parKIN.is_symbolic.size(); ++i) {
        if (parKIN.is_symbolic[i] != expectedKIN0[i]) ok = false;
    }
    for (int i = 0; i < 6 && (6 + i) < (int)parKIN.is_symbolic.size(); ++i) {
        if (parKIN.is_symbolic[6 + i] != expectedKIN1[i]) ok = false;
    }
    for (int i = 0; i < 6 && (12 + i) < (int)parKIN.is_symbolic.size(); ++i) {
        if (parKIN.is_symbolic[12 + i] != expectedKIN2[i]) ok = false;
    }

    for (int i = 0; i < 10 && i < (int)parDYN.is_symbolic.size(); ++i) {
        if (parDYN.is_symbolic[i] != expectedDYN0[i]) ok = false;
    }
    for (int i = 0; i < 10 && (10 + i) < (int)parDYN.is_symbolic.size(); ++i) {
        if (parDYN.is_symbolic[10 + i] != expectedDYN1[i]) ok = false;
    }
    for (int i = 0; i < 10 && (20 + i) < (int)parDYN.is_symbolic.size(); ++i) {
        if (parDYN.is_symbolic[20 + i] != expectedDYN2[i]) ok = false;
    }

    if (!ok) {
        std::cerr << "[FAIL] Symbolic mask test failed." << std::endl;
        std::cerr << "par_KIN is_symbolic: ";
        for (auto v : parKIN.is_symbolic) std::cerr << v;
        std::cerr << std::endl;
        std::cerr << "par_DYN is_symbolic: ";
        for (auto v : parDYN.is_symbolic) std::cerr << v;
        std::cerr << std::endl;
        return 1;
    }

    std::cout << "[PASS] Symbolic mask test passed." << std::endl;
    return 0;
}
