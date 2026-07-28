# Architecture Overview


## Core Concepts

### The `Robot` Class
The `Robot` class is the central data structure in Thunder. It acts as a container for:
- **Properties**: Static information about the robot (e.g., number of joints, joint types).
- **Parameters**: Symbolic and numerical values (e.g., kinematic parameters, masses, inertia tensors).
- **Functions**: Symbolic expressions (`casadi::SX`) in the parameters (e.g., dynamical matrices, forward kinematics matrices.)

Plugins interact with the `Robot` object to read Properties and Parameters, perform calculations, and register new functions.

### The Pipeline
Thunder executes a sequence of plugins organized into a pipeline consisting of three main stages:

1. **Loaders**: Responsible for initializing the `Robot` object. They load data from external sources like URDF files, DH table, or manual configurations. They populate the `Robot` object with Properties and Parameters.
2. **Builders**: Perform symbolic computations. They read properties/parameters from the `Robot` and register new symbolic functions (e.g., Forward Kinematics, Jacobians, Mass Matrix).
3. **Generators**: Treat the `Robot` object as read-only and generate output files, such as C++ source code, Python bindings, or parameter files.

---

## Execution Flow

The `PluginManager` orchestrates pipeline execution:
1. **Configuration**: Parses a YAML configuration file to determine plugins and their parameter blocks.
2. **Initialization**: Instantiates and configures loaded plugins.
3. **Execution**:
   - `Loaders` run sequentially to build the initial `Robot` model.
   - `Builders` run to populate the `Robot` with symbolic functions.
   - `Generators` run to produce the output C++ libraries and Python bindings.

---

## Python & Multi-Language Support


Thunder supports writing plugins in Python, via the `thunder_core` Python package (nanobind bindings for `Robot` and utilities, plus pure-Python modules for plugin base classes and pipeline execution) which enables the `py.` prefix in YAML pipeline configurations.

### `thunder_core` package structure

```
thunder_core                  # Python package
├── _bindings                 # Compiled C++ nanobind module (Robot, PluginManager, utils)
├── plugins                   # ABC base classes: BaseLoader, BaseBuilder, BaseGenerator
└── pipeline                  # Config class and run_pipeline() for pipeline execution
```

Everything from `_bindings` is re-exported at the top level, so `thunder_core.Robot`, `thunder_core.PluginManager`, etc. work directly.

The Python module file (e.g., `my_plugins.py`) should be placed **in the same directory as the YAML config file**. Standard `PYTHONPATH` and `sys.path` rules also apply.


Thunder supports plugins written directly in Python alongside C++ plugins:
- **`py.` Prefix**: Plugins specified as `"py:module.ClassName"` in YAML are dynamically loaded at runtime.
- **C++ Proxies**: `PyLoaderProxy`, `PyBuilderProxy`, and `PyGeneratorProxy` bridge C++ pipeline execution to Python using `nanobind`.
- **CasADi Interop**: Symbolic CasADi expressions seamlessly cross the C++/Python boundary via SWIG pointer extraction without copy overhead.


### Running the pipeline from Python

Use `thunder_core.pipeline` to run a pipeline from Python code or Jupyter notebooks:

```python
from thunder_core.pipeline import Config, run_pipeline

# Quick one-liner from a YAML file
robot = run_pipeline("path/to/robot.yaml", robot_name="my_robot")

# Or from a dict
robot = run_pipeline({
    "pipeline": {
        "loaders": ["kin_loader"],
        "builders": ["kin_builder"],
        "generators": []
    },
    "kin_loader": { "num_joints": 2, "joints_type": ["R", "R"] }
}, robot_name="RR")

# Or use Config directly for more control
cfg = Config("path/to/robot.yaml")
print(cfg)
robot = cfg.execute(robot_name="RR", verbose=True)
```

### Available Robot API in Python

```python
import thunder_core

robot = thunder_core.Robot("my_robot")

# Properties (typed getters/setters — C++ templates mapped to explicit methods)
robot.add_property_int("ndof", 3)
robot.add_property_double("mass", 10.5)
robot.add_property_string("name", "arm")
robot.add_property_vector_double("lengths", [1.0, 1.0, 1.0])
ndof  = robot.get_int("ndof")
name  = robot.get_string("name")

# Symbolic variables and parameters (CasADi SX/DM pass seamlessly)
q = casadi.SX.sym("q", 3)
robot.add_variable("q", q, [0.0, 0.0, 0.0], [1, 1, 1])

# Symbolic model access
q_sym = robot.get_model("q")       # returns casadi.SX

# Numeric evaluation
q_val = robot.get_value("q")       # returns casadi.DM

# Set parameter values
robot.set("q", casadi.DM([1.0, 2.0, 3.0]))

# Add symbolic functions
robot.add_function("sin_q", casadi.sin(q), ["q"], "Sine of q")

# I/O
robot.save_par("params.yaml")
robot.load_par("params.yaml")

# Direct map access
print(robot.properties)    # dict-like: {name: Property, ...}
print(robot.parameters)    # dict-like: {name: Parameter, ...}
print(robot.functions)     # dict-like: {name: Function, ...}

# Utility functions
thunder_core.hat(v)        # skew-symmetric matrix
thunder_core.R_x(angle)    # rotation about X
thunder_core.R_y(angle)    # rotation about Y
thunder_core.R_z(angle)    # rotation about Z
```