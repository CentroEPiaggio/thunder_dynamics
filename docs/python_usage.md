# Using the Generated Python Bindings

Thunder Dynamics can generate Python bindings for the robot-specific library, allowing you to leverage the efficient C++ computations directly within Python scripts.

## Prerequisites

*   Thunder Dynamics installed (see [Installation Guide](./installation.md)).
*   `pybind11` installed (`sudo apt install pybind11-dev` or `pip install pybind11`).
*   Eigen library development files installed (`sudo apt install libeigen3-dev`).
*   A robot configuration YAML file (see [Configuration Guide](./configuration.md)).

## 1. Generating the Bindings

Use the `thunder gen` command with the `--python` (or `-p`) flag:

```bash
# Example: Generate C++ library AND Python bindings for franka.yaml
# Name the module 'FrankaPanda'
./bin/thunder gen --python -n FrankaPanda robots/franka/franka.yaml
```

This creates the `<robot_name>_generatedFiles` directory containing the C++ sources, the `CMakeLists.txt` configured for Python bindings.

## 2. Building the Python Module

You need to compile the C++ code and the pybind11 wrapper into a Python extension module (`.so` file on Linux).

1.  Navigate to the generated build directory:
    ```bash
    cd <path_to>/<robot_name>_generatedFiles/
    mkdir -p build && cd build
    ```
2.  Run CMake and Make:
    ```bash
    # CMake will detect the Python binding request
    cmake ..
    make -j$(nproc)
    ```
This will create a Python extension module file in the `build` directory. The filename typically looks like `thunder_<robot_name>_py.<python-version-abi-tag>.so` (e.g., `thunder_FrankaPanda_py.cpython-38-x86_64-linux-gnu.so`).

## 3. Using the Module in Python

You can now import and use the generated module in your Python scripts.

```python
import numpy as np
import sys
import os

# --- Option 1: Add the build directory to Python's path ---
# Adjust the path to your actual build directory
build_dir = "/path/to/<robot_name>_generatedFiles/build"
if build_dir not in sys.path:
    sys.path.append(build_dir)

# --- Option 2: Copy the .so file ---
# Alternatively, copy the generated .so file from the build directory
# to the same directory as your Python script, or to a location
# already in your Python path (like site-packages).

# Import the class from the module
# The module name is 'thunder_<robot_name>_py'
from thunder_<robot_name>_py import thunder_<robot_name>

# 1. Create an instance of the robot class
robot = thunder_<robot_name>()

# Get the number of joints
n_joints = robot.get_njoints()
print(f"Robot DoF: {n_joints}")

# 2. Initialize input NumPy arrays
q = np.zeros(n_joints)
dq = np.random.rand(n_joints) * 0.1
# Reference vectors (only needed if using regressors like Yr)
dq_r = np.zeros(n_joints)
ddq_r = np.zeros(n_joints)

# 3. Set the input arguments
# Unlike C++, setters are often individual in Python bindings
robot.set_q(q)
robot.set_dq(dq)
robot.set_dq_r(dq_r)
robot.set_ddq_r(ddq_r)

# 4. Load Parameters (Optional)
# Assumes the file is accessible
conf_file_path = "/path/to/<robot_name>_generatedFiles/<robot_name>_conf.yaml"
try:
    robot.load_conf(conf_file_path)
    print(f"Loaded parameters from {conf_file_path}")
except Exception as e:
    print(f"Error loading conf file: {e}")
    # robot.load_par_REG(...) # Alternative

# Set parameters programmatically (for symbolic ones)
# Example: Set mass of link 1 (if symbolic)
# Note: Parameter names match YAML. Check generated _conf.yaml.
# The Python binding expects a list or NumPy array.
# robot.set_inertial_link1_mass([5.0])

# Or set all dynamic parameters from a NumPy array
# dyn_params = robot.get_par_DYN() # Get current DYN params as NumPy array
# dyn_params[0] = 5.0 # Modify the first parameter
# robot.set_par_DYN(dyn_params) # Set the modified array

# 5. Compute Kinematics and Dynamics
print("
--- Computing Kinematics & Dynamics ---")

# Getters return NumPy arrays
T_0_ee = robot.get_T_0_ee()
print(f"T_0_ee:
{T_0_ee}")

J_ee = robot.get_J_ee()
print(f"J_ee:
{J_ee}")

M = robot.get_M()
print(f"M:
{M}")

C = robot.get_C()
print(f"C:
{C}")

G = robot.get_G()
print(f"G:
{G}")

Yr = robot.get_Yr()
print(f"Yr:
{Yr}")

# Access other functions as needed
# e.g., robot.get_J_ee_dot(), robot.get_reg_M(), etc.

```

## Key Concepts

*   **Generate with `--python`**: Essential flag for `thunder gen`.
*   **Build**: Compile the C++ code and bindings using CMake/Make in the `_generatedFiles/build` directory.
*   **Import**:
    *   Make the `.so` file accessible (add path or copy file).
    *   Import the `thunder_<robot_name>` class from the `thunder_<robot_name>_py` module.
*   **NumPy Integration**: Inputs are standard NumPy arrays. Getter methods return NumPy arrays.
*   **Setters**: Individual setters like `set_q()`, `set_dq()` are typically used instead of a single `setArguments` method.
*   **Parameter Loading/Setting**: Works similarly to C++, using `load_conf()`, `load_par_REG()`, `set_par_DYN()`, `set_par_REG()`, and specific setters like `set_inertial_link1_mass()`. Input for setters should be lists or NumPy arrays.
*   **Getters**: Call methods like `get_M()`, `get_T_0_ee()` to retrieve computed quantities as NumPy arrays.
