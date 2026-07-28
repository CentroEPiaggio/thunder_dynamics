# Configuration Guide

Thunder Dynamics uses YAML files to define the execution pipeline and provide parameters for plugins.

---

## Pipeline Configuration

The `pipeline` section defines which plugins are executed and in what order.

```yaml
pipeline:
  loaders: ["urdf_loader"]
  builders: ["kin_builder", "dyn_builder", "py.my_plugins.CustomBuilder"]
  generators: ["robot_generator"]
```

If the `pipeline` section is omitted, Thunder uses a default set of plugins.

---

## Plugin-Specific Configuration

Each plugin can have its own configuration block matching the plugin name used in the pipeline.

```yaml
urdf_loader:
  urdf_path: "robots/panda/panda.urdf"
  base_link: "panda_link0"
  ee_link: "panda_link8"

robot_generator:
  gen_python: true
  gen_casadi: true
```

### Python Plugins in Config
For Python plugins, use the full `py.module.ClassName` identifier as the configuration key:

```yaml
"py.my_plugins.CustomBuilder":
  function_name: "custom_torque"
  scale_factor: 1.5
```

---

## Global Configuration & Merging

Any top-level key in the YAML file that is *not* a plugin name is considered a global configuration. The `PluginManager` merges global parameters into each plugin's specific configuration.

> [!NOTE]
> Plugin-specific keys take precedence over global keys.

```yaml
# Global configuration
gravity:
  value: [0, 0, -9.81]

pipeline:
  loaders: ["urdf_loader"]
  builders: ["dyn_builder"]

dyn_builder:
  damping_factor: 0.1
```

---

## Symbolic URDF Masking

When using `urdf_loader`, parameters can be declared as symbolic (adaptable at runtime) globally or per link:

```yaml
symbolic_kinematics:
  base_link: [0, 0, 0, 0, 0, 0]
  link1:
    xyz: [1, 1, 1]
    rpy: [0, 0, 0]

symbolic_dynamics:
  base_link: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  link1:
    mass: 1
    com: [0, 1, 0]
    inertia: [1, 1, 1, 0, 0, 0]
```

---

## Complete Example YAML

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
  gen_python: true
```
