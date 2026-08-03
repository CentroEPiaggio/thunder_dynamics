# Plugin Development Guide

Thunder Dynamics supports writing custom plugins in **C++** or **Python**. This guide explains how to create and integrate both types.

---

## Plugin Interface Types

All plugins inherit from a core plugin base class:

| Plugin Type | Base Class (C++) | Base Class (Python) | Interface Method |
| :--- | :--- | :--- | :--- |
| **Loader** | `BaseLoader` | `thunder_core.plugins.BaseLoader` | `load(robot)` |
| **Builder** | `BaseBuilder` | `thunder_core.plugins.BaseBuilder` | `build(robot)` |
| **Generator** | `BaseGenerator` | `thunder_core.plugins.BaseGenerator` | `generate(robot)` |

---

## 🛠️ Writing C++ Plugins

### 1. Header Implementation

```cpp
#ifndef MY_BUILDER_H
#define MY_BUILDER_H

#include <yaml-cpp/yaml.h>

#include "../plugin_interfaces.h"
#include "../../library/robot.h"

namespace thunder_ns {

    class MyBuilder : public BaseBuilder {
    public:
        MyBuilder() : BaseBuilder("my_builder", "Custom C++ builder plugin.") {}

        int configure(const YAML::Node& config) override {
            if (config["my_param"]) {
                double val = config["my_param"].as<double>();
            }
            return 1;
        }

        void build(std::shared_ptr<Robot> robot) override {
            auto q = robot->get_model("q");
            casadi::SX expr = q(0) * q(0);
            robot->add_function("my_custom_func", expr, {"q"}, "Squared first joint angle");
        }
    };

} // namespace thunder_ns

#endif
```

### 2. Registration

Add your plugin to `src/thunder/include/plugin_registry.h`:

```cpp
#include "plugins/builders/my_builder.h"

inline const std::map<std::string, std::shared_ptr<BaseBuilder>> POPULATORS = {
    // ... existing builders
    {"my_builder", std::make_shared<MyBuilder>()},
};
```

---

## 🐍 Writing Python Plugins

Python plugins do **not** require manual C++ compilation or registry modification. Place the `.py` file in your working directory or Python path and reference it in YAML with `py.module.ClassName`.

### Implementation

```python
import casadi
from thunder_core.plugins import BaseBuilder

class CustomBuilder(BaseBuilder):
    def build(self, robot):
        # Fetch symbolic variable from C++ Robot instance
        q = robot.get_model("q")
        
        # Build symbolic CasADi expression
        expr = casadi.cos(q) + casadi.sin(q)**2
        
        # Register function on Robot instance
        robot.add_function("custom_expr", expr, ["q"], "Custom Python symbolic expression")
```

### Optional Pydantic Config Validation

Define a `ConfigModel` nested class to automatically validate YAML inputs:

```python
from pydantic import BaseModel
from thunder_core.plugins import BaseBuilder
import casadi

class CustomBuilder(BaseBuilder):
    class ConfigModel(BaseModel):
        function_name: str = "custom_expr"
        scale: float = 1.0

    def build(self, robot):
        q = robot.get_model("q")
        expr = self.config.scale * casadi.cos(q)
        robot.add_function(self.config.function_name, expr, ["q"], "Validated config function")
```

### Usage in YAML

```yaml
pipeline:
  builders: ["py.my_module.CustomBuilder"]

"py.my_module.CustomBuilder":
  function_name: "my_torque"
  scale: 2.0
```




### Notes on Python plugins

- **CasADi interop**: CasADi's Python objects (`casadi.SX`, `casadi.DM`, `casadi.Function`) are converted automatically at the C++/Python boundary using SWIG pointer extraction. No serialization overhead.
- **Shared Robot object**: Python plugins receive the same `Robot` instance as C++ plugins. Modifications (adding functions, setting parameters) persist across the pipeline.
- **Dynamic attributes**: You can set arbitrary Python attributes on the Robot object (e.g., `robot.my_data = [1,2,3]`). These are visible to subsequent Python plugins but not to C++ plugins. Use `robot.add_property_*()` or `robot.add_parameter()` if C++ needs access.
- **Error handling**: Python exceptions are caught and re-raised as C++ `std::runtime_error` with the full Python traceback.
- **Config**: The YAML config block for a Python plugin is passed to `configure()` as a Python dict (or validated via pydantic if `ConfigModel` is defined). The key in the YAML must match the full `py:module.Class` name.
