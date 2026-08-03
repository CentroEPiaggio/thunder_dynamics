# ⚡ Thunder Dynamics (v1.0.0)

Thunder Dynamics is a modular C++ and Python framework for generating fast, optimized code for robot kinematics and dynamics computations. It uses [CasADi](https://web.casadi.org/) for symbolic differentiation and code generation, providing high-performance C++ libraries and Python bindings for control and simulation. It can be easily expanded and customized through a plugin architecture.

Robots can be defined from URDF, DH parameters, or manually using an internal joint-link representation.

---

##  Basic Usage

### Running Thunder CLI
Generate your robot library using a single configuration file:
```bash
thunder gen [-n <robot_name>] <path>/<robot>.yaml
```
This generates a `<robot>_generatedFiles/` directory containing:
- `<robot>_gen.h` is the C-generated library from CasADi associated with the source file `<robot>_gen.cpp`.
- `thunder_<robot>.h`, `thunder_<robot>.cpp` is the wrapper class for the generated files.
- `<robot>_conf` is a copy of the configuration file used to generate the robot.
- `<robot>_par` is the parameters' file that can be used to load parameters from the class `thunder_<robot>`.

For example:
```bash
thunder gen robots/RRR/RRR.yaml
```


### 🤖  Configuration (YAML)
Thunder uses YAML files to define the execution pipeline and plugin parameters.

Example configuration loading a robot from URDF, computing kinematics & dynamics, and generating a C++ library with Python bindings:

```yaml
pipeline:
  loaders: ["urdf_loader"]
  builders: ["kin_builder", "dyn_builder"]
  generators: ["robot_generator"]

urdf_loader:
  urdf_path: "robots/panda/panda.urdf"
  base_link: "panda_link0"
  ee_link: "panda_link8"

robot_generator:
  gen_robot: true
  gen_python: true
```

Example configurations can be found in the `robots/` directory.
Each of the elements in the `pipeline` section is a plugin that will be executed in the specified order. To get a list of all available plugins:
```bash
thunder plugin list --verbose
```



## 📚 Documentation
- 📐 [Architecture Overview](docs/architecture.md)
- ⚙️ [Configuration Guide](docs/configuration.md)
- 🧩 [Plugin Development Guide](docs/plugins.md)
---

## Using Generated Libraries

### In C++
```cpp
#include "thunder_<robot>.h"
#include <eigen3/Eigen/Dense>

int main() {
    Eigen::VectorXd q, dq, dq_r;
    // Instantiate generated robot wrapper
    thunder_<robot> my_robot;
    my_robot.set_q(q);
    my_robot.set_dq(dq)
    my_robot.set_dqr(dq_r);

    int numjoints = my_robot.ndof;

    // Compute kinematic and dynamic quantities
    Eigen::MatrixXd T  = my_robot.get_T_0_ee(); // End-effector transformation matrix
    Eigen::MatrixXd J  = my_robot.get_J_ee();    // End-effector Jacobian
    Eigen::MatrixXd M  = my_robot.get_M();       // Mass matrix
    Eigen::MatrixXd C  = my_robot.get_C();       // Coriolis matrix
    Eigen::MatrixXd G  = my_robot.get_G();       // Gravity vector
    Eigen::MatrixXd Yr = my_robot.get_Yr();      // Dynamic regressor matrix
}
```

### In Python
If `gen_python: true` is set in the configuration, build the generated Python module:
```bash
cd <robot>_generatedFiles
pip install .
```
Then import and use in Python:
```python
import numpy as np
from thunder_<robot>_py import thunder_<robot>

robot = thunder_<robot>()
robot.set_q(np.zeros(robot.get_numJoints()))
robot.set_dq(np.random.rand(robot.get_numJoints()))

T = robot.get_T_0_ee()
J = robot.get_J_ee()
M = robot.get_M()
C = robot.get_C()
G = robot.get_G()
Yr = robot.get_Yr()

help(robot)
```

---

## Plugin Architecture

Thunder features a 3-stage pipeline architecture:
1. **Loaders**: Initialize the robot model (from URDF, DH parameters, etc.).
2. **Builders**: Perform symbolic computations (kinematics, dynamics, regressors).
3. **Generators**: Output C++ source code, Python bindings, or parameters.

Plugins can be written in **C++** or directly in **Python** (prefixed with `py.`).

### Built-in Plugins4
To inspect all available C++ plugins:
```bash
thunder plugin list --verbose
```

| Type | Plugin Name | Description |
| :--- | :--- | :--- |
| **Loader** | `urdf_loader` | Load robot model from URDF file |
| **Loader** | `dh_loader` | Build robot structure from DH parameters |
| **Loader** | `kin_loader` | Initialize basic kinematic structures |
| **Loader** | `dyn_loader` | Load dynamic inertia parameters |
| **Loader** | `soft_loader` | Load elastic joint structures  |
| **Builder** | `kin_builder` | Build FK, Jacobians, and transformations |
| **Builder** | `dyn_builder` | Build M, C, G matrices and dynamic derivatives |
| **Builder** | `reg_builder` | Build kinematic and dynamic regressors |
| **Builder** | `soft_builder` | Build elastic joint dynamics |
| **Generator** | `robot_generator` | Generate standalone C++ library and Python bindings |

---

## Symbolic Parameters & URDF Masking

Parameters can be configured as symbolic or numeric.

When using `urdf_loader`, enable symbolic kinematic or dynamic parameters globally or per link:

```yaml
symbolic_kinematics:
  base_link: [0, 0, 0, 0, 0, 0] # symbolic flag can be expressed in a compact way
  link1:                        # or explicitly
    xyz: [1, 1, 1]
    rpy: [0, 0, 0]

symbolic_dynamics:
  base_link: [1,0,1,0,1,1,1,0,0,0]
  link1:
    mass: 1
    com: [0, 1, 0]
    inertia: [1, 1, 1, 0, 0, 0]
```

The same masks can also be embedded directly in the URDF (used if YAML does not override):

```xml
<link name="base_link">
  <!-- Values can be 0/1 or true/false -->
  <symbolic_kinematics xyz="1 1 1" rpy="0 0 0" />
  <symbolic_dynamics mass="1" com="0 1 0" inertia="1 1 1 0 0 0" />
</link>
```

---

## 📦 Installation & Setup

### Docker (Recommended)
1. Open repository in **VS Code**.
2. Run command **Dev Containers: Reopen in Container**.



---

## 📝 Citation & Research

If you use Thunder in your research, please cite our paper:

```bibtex
@Article{baracca_2025_thunder,
    AUTHOR = {Baracca, Marco and Simonini, Giorgio and Tolomei, Simone and De Santis, Yuri and Rosa Brusin, Paolo and Angeli, Stefano and Gabiccini, Marco and Bicchi, Antonio and Salaris, Paolo},
    TITLE = {Thunder Dynamics: A C++ Tool for Adaptive Control of Serial Manipulators},
    JOURNAL = {Robotics},
    VOLUME = {14},
    YEAR = {2025},
    NUMBER = {9},
    ARTICLE-NUMBER = {126},
    URL = {https://www.mdpi.com/2218-6581/14/9/126},
    ISSN = {2218-6581},
    DOI = {10.3390/robotics14090126}
}
```

### Sponsors
![DARKO Project](docs/assets/darko_logo.png)  
[DARKO Project](https://darko-project.eu/)
