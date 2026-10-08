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

    const std::string config_file = std::string(THUNDER_SOURCE_DIR) + "/src/thunder/tests/fixtures/franka/symbolic_mask_test.yaml";

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

    // par_KIN is a function of the frame parameters of each node, KIN_<link>_xyz and KIN_<link>_rpy: their masks are the KIN mask
    auto kin_mask = [&](const std::string &link_name) {
        std::vector<short> mask = robot->parameters.at("KIN_" + link_name + "_xyz").is_symbolic;
        const auto &rpy = robot->parameters.at("KIN_" + link_name + "_rpy").is_symbolic;
        mask.insert(mask.end(), rpy.begin(), rpy.end());
        return mask;
    };
    const auto &parDYN = robot->parameters.at("par_DYN");

    // DEBUG - print joint names and their current masks
    std::cout << "-- jointName -> KIN mask | DYN mask --" << std::endl;
    for (int i = 0; i < (int)jointsName.size(); ++i) {
        std::cout << i << ": " << jointsName[i] << " -> ";
        for (short v : kin_mask(jointsName[i])) std::cout << v;
        std::cout << " | DYN: ";
        for (int j = 0; j < 10; ++j) {
            std::cout << parDYN.is_symbolic[i * 10 + j];
        }
        std::cout << std::endl;
    }
    std::cout << "---------------------------------------" << std::endl;

    bool ok = true;

    if ((int)parDYN.is_symbolic.size() != 10 * numJoints) {
        std::cerr << "Unexpected par_DYN size: " << parDYN.is_symbolic.size() << " (expected " << 10 * numJoints << ")" << std::endl;
        ok = false;
    }

    auto check_mask = [&](const std::string &link_name, const std::vector<short> &expected_kin, const std::vector<short> &expected_dyn) {
        if (!jointIndex.count(link_name)) {
            std::cerr << "[ERROR] Link name not found: " << link_name << std::endl;
            return false;
        }
        int baseDYN = jointIndex[link_name] * 10;
        const std::vector<short> kin = kin_mask(link_name);

        for (int i = 0; i < (int)expected_kin.size(); ++i) {
            if (kin[i] != expected_kin[i]) {
                std::cerr << "Mismatch KIN for " << link_name << " at " << i << " (got "
                          << kin[i] << ", expected " << expected_kin[i] << ")\n";
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

    // URDF tags: kinematics from the joint leaving the link (its origin is the frame of the node), dynamics from the link
    ok &= check_mask("panda_link5", {0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0,0}); // panda_joint6 tag + panda_link5 tag
    ok &= check_mask("panda_link6", {1,1,1,1,1,1}, {1,1,1,1,1,1,1,1,1,1}); // panda_joint7 tag + panda_link6 tag
    ok &= check_mask("panda_link7", {1,0,1,0,1,0}, {1,0,1,0,1,0,1,0,1,0}); // panda_joint8 tag + panda_link7 tag

    if (!ok) {
        std::cerr << "[FAIL] Symbolic mask test failed." << std::endl;
        std::cerr << "par_DYN is_symbolic: ";
        for (auto v : parDYN.is_symbolic) std::cerr << v;
        std::cerr << std::endl;
        return 1;
    }

    std::cout << "[PASS] Symbolic mask test passed." << std::endl;
    return 0;
}
