# Using the Generated C++ Library

Once you have generated the robot-specific library using `thunder gen` (see [Getting Started](./getting_started.md)), you can integrate it into your C++ projects.

## Prerequisites

*   Generated `<robot_name>_generatedFiles` directory.
*   A C++ project environment (e.g., using CMake).
*   Eigen library installed and accessible to your project.

## Building the Generated Library

Before using the library, you need to compile it.

1.  Navigate to the generated build directory:
    ```bash
    cd <path_to>/<robot_name>_generatedFiles/
    mkdir -p build && cd build
    ```
2.  Run CMake and Make:
    ```bash
    cmake ..
    make -j$(nproc)
    ```
This will compile the source files (`<robot_name>_gen.cpp`, `thunder_<robot_name>.cpp`) and create a static library (`libthunder_<robot_name>.a`) or shared library (`libthunder_<robot_name>.so`) inside the `build` directory.

## Integrating with CMake

In your project's `CMakeLists.txt`, you need to:

1.  **Find Eigen**: Ensure Eigen is found.
    ```cmake
    find_package(Eigen3 REQUIRED)
    ```
2.  **Add the Generated Library Directory**: Tell CMake where to find the header and library file.
    ```cmake
    # Path to the <robot_name>_generatedFiles directory
    set(THUNDER_ROBOT_DIR "/path/to/<robot_name>_generatedFiles")

    include_directories(${THUNDER_ROBOT_DIR}) # For headers
    link_directories(${THUNDER_ROBOT_DIR}/build) # For the library file
    ```
    *Alternatively, you can use `find_library` or install the library to a standard location.*
3.  **Link Your Executable/Library**: Link against the generated library and Eigen.
    ```cmake
    add_executable(my_robot_app main.cpp)

    target_link_libraries(my_robot_app
        PRIVATE
        thunder_<robot_name> # Name of the generated library
        Eigen3::Eigen
    )
    ```

## Basic C++ Code Example

Here's how to use the `thunder_<robot_name>` class in your C++ code:

```c++
#include "thunder_<robot_name>.h" // Include the main header
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

int main() {
    // 1. Create an instance of the robot class
    thunder_<robot_name> robot;

    // Get the number of joints
    int n_joints = robot.get_njoints();
    std::cout << "Robot DoF: " << n_joints << std::endl;

    // 2. Initialize input vectors (adjust sizes based on n_joints)
    Eigen::VectorXd q(n_joints);
    Eigen::VectorXd dq(n_joints);
    Eigen::VectorXd dq_r(n_joints);    // Reference velocity (for regressor)
    Eigen::VectorXd ddq_r(n_joints); // Reference acceleration (for regressor)

    // Set some example joint values (e.g., zero position, small velocities)
    q.setZero();
    dq.setConstant(0.1);
    dq_r.setZero();
    ddq_r.setZero();

    // 3. Set the input arguments for calculations
    // This typically needs to be called before computing dynamics/kinematics
    // if q, dq, etc. change.
    robot.setArguments(q, dq, dq_r, ddq_r);

    // 4. Load Parameters (Optional but common)
    // Option A: Load from the generated configuration file
    // Assumes the file is accessible from the execution path
    robot.load_conf("<path_to>/<robot_name>_generatedFiles/<robot_name>_conf.yaml");


    // Option B: Set parameters programmatically (especially for symbolic ones)
    // Example: Set the mass of the first link (if it was declared symbolic)
    // Note: Parameter names match the YAML structure. Check thunder_<robot_name>.h
    // or the generated _conf.yaml for exact names.
    // std::vector<double> link1_mass = {5.0}; // New mass value
    // robot.set_inertial_link1_mass(link1_mass); // Specific setter

    // Or set all dynamic parameters from a vector (order matters!)
    // Eigen::VectorXd dyn_params = robot.get_par_DYN(); // Get current DYN params
    // dyn_params(0) = 5.0; // Modify the first parameter (e.g., link1 mass)
    // robot.set_par_DYN(dyn_params); // Set the modified vector


    // 5. Compute Kinematics and Dynamics
    std::cout << "--- Computing Kinematics & Dynamics ---" << std::endl;

    // End-effector Homogeneous Transformation Matrix (T_0_ee)
    Eigen::MatrixXd T_0_ee = robot.get_T_0_ee();
    std::cout << "T_0_ee: " << T_0_ee << std::endl;

    // End-effector Basic Jacobian (J_ee)
    Eigen::MatrixXd J_ee = robot.get_J_ee();
    std::cout << "J_ee: " << J_ee << std::endl;

    // Mass Matrix (M)
    Eigen::MatrixXd M = robot.get_M();
    std::cout << "M: " << M << std::endl;

    // Coriolis/Centrifugal Matrix (C)
    Eigen::MatrixXd C = robot.get_C();
    std::cout << "C: " << C << std::endl;

    // Gravity Vector (G)
    Eigen::VectorXd G = robot.get_G();
    std::cout << "G: " << G << std::endl;

    // Regressor Matrix (Yr) - Requires dq_r, ddq_r to be set
    Eigen::MatrixXd Yr = robot.get_Yr();
    std::cout << "Yr: " << Yr << std::endl;

    // Access other generated functions as needed (check API Reference)
    // e.g., robot.get_J_ee_dot(), robot.get_reg_M(), etc.

    return 0;
}
```

## Key Concepts

*   **Include Header**: Always include `thunder_<robot_name>.h`.
*   **Instantiate**: Create an object of the `thunder_<robot_name>` class.
*   **`setArguments`**: Crucial method to update the internal state (q, dq, dq_r, ddq_r) used for subsequent calculations. Call this whenever the robot's state changes.
*   **Parameter Loading**:
    *   `load_conf()`: Loads all parameters (numeric and initial symbolic values) from the `_conf.yaml` file.
    *   `load_par_REG()`: Loads the standard inertial parameters from the `_par_REG.yaml` file.
*   **Parameter Setting (Runtime)**:
    *   `set_par_DYN()` / `get_par_DYN()`: Set/get all *symbolic* dynamic parameters (inertial, friction, etc.) as a single Eigen vector. The order corresponds to how they were declared symbolic in the YAML.
    *   `set_par_REG()` / `get_par_REG()`: Set/get all *symbolic* regressor parameters (standard 10 inertial params per link) as a single Eigen vector.
    *   `set_<parameter_group>_<link_name>_<parameter_name>(std::vector<double>)`: Specific setters for individual symbolic parameters (e.g., `set_inertial_link1_mass({value})`). Check the header file for available setters. These only work for parameters marked `symb: 1` in the YAML.
*   **Getter Functions**: Methods like `get_M()`, `get_C()`, `get_G()`, `get_T_0_ee()`, `get_J_ee()`, `get_Yr()` return the computed quantities based on the current state set by `setArguments` and the loaded/set parameters. Refer to the [API Reference](./api_reference.md) for a list of available functions.
