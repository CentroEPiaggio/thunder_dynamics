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

    std::cout << "[TEST] Using config: " << config_file << std::endl;

    PluginManager manager;
    manager.set_verbose(false);
    manager.configure_pipeline(config_file, 1); // no generation

    auto robot = manager.execute("symbolic_mask_test");

    const int numJoints = robot->get<int>("numJoints");

    const auto jointsName = robot->get<vector<string>>("jointsName");
    std::unordered_map<std::string, int> jointIndex;
    for (int i = 0; i < (int)jointsName.size(); ++i) {
        jointIndex[jointsName[i]] = i;
    }

    const auto &parKIN = robot->parameters["par_KIN"];
    const auto &parDYN = robot->parameters["par_DYN"];

    // DEBUG - print joint names and their current masks
    std::cout << "-- jointName -> KIN mask (first 6 bits) --" << std::endl;
    for (int i = 0; i < (int)jointsName.size(); ++i) {
        std::cout << i << ": " << jointsName[i] << " -> ";
        for (int j = 0; j < 6; ++j) {
            std::cout << parKIN.is_symbolic[i * 6 + j];
        }
        std::cout << " | DYN: ";
        for (int j = 0; j < 10; ++j) {
            std::cout << parDYN.is_symbolic[i * 10 + j];
        }
        std::cout << std::endl;
    }
    std::cout << "---------------------------------------" << std::endl;

    bool ok = true;

    if ((int)parKIN.is_symbolic.size() != 6 * numJoints) {
        std::cerr << "Unexpected par_KIN size: " << parKIN.is_symbolic.size() << " (expected " << 6 * numJoints << ")" << std::endl;
        ok = false;
    }
    if ((int)parDYN.is_symbolic.size() < 10 * 2) {
        std::cerr << "Unexpected par_DYN size: " << parDYN.is_symbolic.size() << " (expected at least " << 10 * 2 << ")" << std::endl;
        ok = false;
    }

    auto check_mask = [&](const std::string &link_name, const std::vector<short> &expected_kin, const std::vector<short> &expected_dyn) {
        if (!jointIndex.count(link_name)) {
            std::cerr << "[ERROR] Link name not found: " << link_name << std::endl;
            return false;
        }
        int idx = jointIndex[link_name];
        int baseKIN = idx * 6;
        int baseDYN = idx * 10;

        for (int i = 0; i < (int)expected_kin.size(); ++i) {
            if (parKIN.is_symbolic[baseKIN + i] != expected_kin[i]) {
                std::cerr << "Mismatch KIN for " << link_name << " at " << i << " (got "
                          << parKIN.is_symbolic[baseKIN + i] << ", expected " << expected_kin[i] << ")\n";
                return false;
            }
        }
        for (int i = 0; i < (int)expected_dyn.size(); ++i) {
            if (parDYN.is_symbolic[baseDYN + i] != expected_dyn[i]) {
                std::cerr << "Mismatch DYN for " << link_name << " at " << i << " (got "
                          << parDYN.is_symbolic[baseDYN + i] << ", expected " << expected_dyn[i] << ")\n";
                return false;
            }
        }
        return true;
    };

    ok &= check_mask("panda_link0", {0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0,0}); // YAML override
    ok &= check_mask("panda_link1", {1,1,1,1,1,1}, {1,1,1,1,1,1,1,1,1,1}); // YAML override
    ok &= check_mask("panda_link2", {1,0,1,0,1,0}, {1,0,1,0,1,0,1,0,1,0}); // YAML override

    ok &= check_mask("panda_link3", {0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0,0}); // default (no tag)
    ok &= check_mask("panda_link4", {0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0,0}); // default (no tag)

    ok &= check_mask("panda_link5", {0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0,0}); // URDF joint tag + link tag
    ok &= check_mask("panda_link6", {1,1,1,1,1,1}, {1,1,1,1,1,1,1,1,1,1}); // URDF joint tag + link tag
    ok &= check_mask("panda_link7", {1,0,1,0,1,0}, {1,0,1,0,1,0,1,0,1,0}); // URDF joint tag + link tag

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
