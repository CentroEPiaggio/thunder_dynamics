#include <chrono>
#include <filesystem>
#include <string>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "plugin_manager.h"

using namespace thunder_ns;
namespace fs = std::filesystem;

struct CwdGuard {
    explicit CwdGuard(fs::path path) : previous(fs::current_path()) {
        fs::current_path(std::move(path));
    }

    ~CwdGuard() {
        fs::current_path(previous);
    }

    fs::path previous;
};

TEST(ThunderPipeline, GeneratorSmokeCreatesExpectedFiles) {
#ifndef THUNDER_SOURCE_DIR
    GTEST_SKIP() << "THUNDER_SOURCE_DIR not defined.";
#else
    const fs::path config_path = fs::path(THUNDER_SOURCE_DIR) / "src/thunder/robots/testRRR.yaml";
    YAML::Node config = YAML::LoadFile(config_path.string());

    config["robot_generator"]["gen_casadi"] = false;
    config["robot_generator"]["gen_python"] = false;
    config["robot_generator"]["gen_robot"] = true;
    config["robot_generator"]["copy_gen"] = false;

    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const fs::path temp_root = fs::temp_directory_path() / ("thunder_gen_smoke_" + std::to_string(stamp));
    fs::create_directories(temp_root);

    try {
        {
            CwdGuard guard(temp_root);

            PluginManager manager;
            manager.set_verbose(false);
            manager.configure_pipeline(config, 0);

            auto robot = manager.execute("gen_smoke_robot");
            ASSERT_NE(robot, nullptr) << "Robot pointer is null";

            const fs::path out_dir = temp_root / "gen_smoke_robot_generatedFiles";
            EXPECT_TRUE(fs::exists(out_dir)) << "Generated directory missing";
            EXPECT_TRUE(fs::exists(out_dir / "gen_smoke_robot_gen.cpp")) << "Missing generated CasADi C++ file";
            EXPECT_TRUE(fs::exists(out_dir / "gen_smoke_robot_gen.h")) << "Missing generated CasADi header";
            EXPECT_TRUE(fs::exists(out_dir / "thunder_gen_smoke_robot.cpp")) << "Missing generated robot C++ file";
            EXPECT_TRUE(fs::exists(out_dir / "thunder_gen_smoke_robot.h")) << "Missing generated robot header";
            EXPECT_TRUE(fs::exists(out_dir / "gen_smoke_robot_conf.yaml")) << "Missing generated config file";
            EXPECT_TRUE(fs::exists(out_dir / "gen_smoke_robot_par.yaml")) << "Missing generated parameter file";
        }
    } catch (const std::exception& e) {
        ADD_FAILURE() << "Unexpected exception: " << e.what();
    }

    fs::remove_all(temp_root);
#endif
}
