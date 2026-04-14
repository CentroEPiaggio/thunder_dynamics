#  thunder_<ROBOT> Generated Library

This is a robot library generated for <ROBOT> using thunder.
It contains the generated C++ code and Python bindings for the <ROBOT> robot model.

## C++ Compilation

You can compile this code as a standalone C++ library to include in your own projects.

### Dependencies

*   **Eigen3**
*   **yaml-cpp** (or install via: `sudo apt install libyaml-cpp-dev`)

### Building and install the library
```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc) && sudo make install
```

By default, cmake will install in `/usr/local`, you can customize the install directory using `-DCMAKE_INSTALL_PREFIX`

## Python Library (Recommended)

This project is configured as a Python package using `scikit-build-core` and `pybind11`, making it easy to install with `pip` or `uv`.

### Installation Requirements
- Python 3.8+
- CMake >= 3.15
- C++ Compiler (GCC/Clang)
- Ninja (optional, but **recommended for faster builds**)

### Fast Installation with `uv` (Recommended ⚡)

[uv](https://github.com/astral-sh/uv) is the fastest Python package installer. Install with uv:

```bash
# Install with uv (fastest option)
uv pip install -e .

# Or install the wheel directly (no compilation)
uv pip install --no-build-isolation .
```

### Installation with `pip`

Standard `pip` installation works too:

```bash
# For faster compilation, install Ninja first
pip install ninja

# Then install the package
pip install -e .

# Or pre-compiled wheels (if available)
pip install .
```

### Performance Tips for Faster Compilation

1. **Install `ninja` - 30-50% faster builds:**
   ```bash
   pip install ninja
   # or
   sudo apt install ninja-build
   ```

2. **Use system yaml-cpp** (avoids rebuilding from source):
   ```bash
   sudo apt install libyaml-cpp-dev
   ```

3. **Parallel compilation** is enabled by default - uses all CPU cores

4. **For development mode** with incremental builds:
   ```bash
   pip install -e . --no-build-isolation
   # Only rebuilds changed files
   ```

### Build Time Expectations
- **First build:** 2-5 minutes (depends on robot complexity and hardware)
- **Subsequent builds:** 30 seconds - 2 minutes (only rebuilds changed files)
- **With all optimizations:** can be 40% faster

## Python Usage

Once installed:

```python
import numpy as np
from thunder_<ROBOT>_py import thunder_<ROBOT>

# Create robot instance
robot = thunder_<ROBOT>()

# Load configuration
robot.load_conf("path/to/<ROBOT>_conf.yaml")

# Set joint state
robot.set_q(np.zeros(robot.get_int("numJoints")))
robot.set_dq(np.zeros(robot.get_int("numJoints")))

# Get robot properties
T = robot.get_T_w_ee()       # End-effector transform
J = robot.get_J_ee()          # Jacobian
M = robot.get_M()             # Mass matrix
C = robot.get_C()             # Coriolis/centrifugal forces
G = robot.get_G()             # Gravity vector
Yr = robot.get_Yr()           # Regression matrix
```

## Virtual Environment (Optional but Recommended)

For isolated Python environments:

```bash
# Using uv
uv venv .venv
source .venv/bin/activate

# Using pip/venv
python -m venv .venv
source .venv/bin/activate

# Then install as above
uv pip install -e .
```

## Troubleshooting

### Build Fails with "yaml-cpp not found"
```bash
# Install system yaml-cpp
sudo apt install libyaml-cpp-dev
```

### Slow Compilation
- Install Ninja: `pip install ninja` or `sudo apt install ninja-build`
- Check CPU usage during build - if low, you have system resource limits

### Import Error: `ModuleNotFoundError`
```bash
# Make sure you're in the right directory or installed with -e
pip install -e .

# Or add to PYTHONPATH
export PYTHONPATH=$PWD:$PYTHONPATH
```


