\
#include <benchmark/benchmark.h>
#include <Eigen/Dense>
#include "../../robots/franka/FrankaNuovo9_generatedFiles/thunder_FrankaNuovo9.h\" // Adjust path if necessary

// Helper function to create random Eigen vectors
Eigen::VectorXd createRandomVector(int size) {
    return Eigen::VectorXd::Random(size);
}

// Benchmark fixture for setting up the robot and inputs
class RobotBenchmark : public benchmark::Fixture {
public:
    thunder_FrankaNuovo9 robot;
    Eigen::VectorXd q, dq, dqr, ddqr, w;

    void SetUp(const ::benchmark::State& state) override {
        q = createRandomVector(robot.n_joints);
        dq = createRandomVector(robot.n_joints);
        dqr = createRandomVector(robot.n_joints);
        ddqr = createRandomVector(robot.n_joints);
        w = createRandomVector(6); // External wrench size

        // Load default parameters (assuming a default config/parameter file exists or is needed)
        // robot.load_conf(\"path/to/your/FrankaNuovo9_conf.yaml\"); // Example: Load configuration if needed
        // robot.load_par_REG(\"path/to/your/FrankaNuovo9_par_REG.yaml\"); // Example: Load parameters if needed

        robot.setArguments(q, dq, dqr, ddqr);
        robot.set_w(w); // Set external wrench if needed for specific benchmarks like JTw
    }

    void TearDown(const ::benchmark::State& state) override {
        // Clean up if necessary
    }
};

// --- Benchmark Functions ---

BENCHMARK_F(RobotBenchmark, BM_GetM)(benchmark::State& state) {
    for (auto _ : state) {
        benchmark::DoNotOptimize(robot.get_M());
    }
}

BENCHMARK_F(RobotBenchmark, BM_GetC)(benchmark::State& state) {
    for (auto _ : state) {
        benchmark::DoNotOptimize(robot.get_C());
    }
}

BENCHMARK_F(RobotBenchmark, BM_GetG)(benchmark::State& state) {
    for (auto _ : state) {
        benchmark::DoNotOptimize(robot.get_G());
    }
}

BENCHMARK_F(RobotBenchmark, BM_GetYr)(benchmark::State& state) {
    for (auto _ : state) {
        benchmark::DoNotOptimize(robot.get_Yr());
    }
}

BENCHMARK_F(RobotBenchmark, BM_GetJee)(benchmark::State& state) {
    for (auto _ : state) {
        benchmark::DoNotOptimize(robot.get_J_ee());
    }
}

BENCHMARK_F(RobotBenchmark, BM_GetJeeDot)(benchmark::State& state) {
    for (auto _ : state) {
        benchmark::DoNotOptimize(robot.get_J_ee_dot());
    }
}

BENCHMARK_F(RobotBenchmark, BM_GetT0ee)(benchmark::State& state) {
    for (auto _ : state) {
        benchmark::DoNotOptimize(robot.get_T_0_ee());
    }
}

// Add more benchmarks for other functions as needed...
// Example: Regressor components
BENCHMARK_F(RobotBenchmark, BM_GetRegM)(benchmark::State& state) {
    for (auto _ : state) {
        benchmark::DoNotOptimize(robot.get_reg_M());
    }
}

BENCHMARK_F(RobotBenchmark, BM_GetRegC)(benchmark::State& state) {
    for (auto _ : state) {
        benchmark::DoNotOptimize(robot.get_reg_C());
    }
}

BENCHMARK_F(RobotBenchmark, BM_GetRegG)(benchmark::State& state) {
    for (auto _ : state) {
        benchmark::DoNotOptimize(robot.get_reg_G());
    }
}

// BENCHMARK_F(RobotBenchmark, BM_GetRegJTw)(benchmark::State& state) {
//     for (auto _ : state) {
//         benchmark::DoNotOptimize(robot.get_reg_JTw());
//     }
// }


// --- Main ---
BENCHMARK_MAIN();
