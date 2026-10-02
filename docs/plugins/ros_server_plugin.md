# ROS Server Generator Plugin

`ros_server_generator` creates a standard ROS 2 C++ package around the C++ robot produced by `robot_generator`. It must run after `robot_generator`; it moves that generator's C++ sources into the ROS package, so there is a single authoritative copy of the generated robot code.

```yaml
pipeline:
  loaders: [...]
  builders: [...]
  generators: [robot_generator, ros_server_generator]

robot_generator:
  gen_robot: true     # needed by ros_server_generator

ros_server_generator: # plugin configuration
  frequency: 1000     # frequency of the server node
  real_time: false    # enable node RT features
  copy_gen: true      # copy package into thunder_ros_test
  inputs:             # topics where inputs are taken
    q: /robot/q
    dq: /robot/dq
  services: [M, q]    # exposed services (default all)
  topics: [M, q]      # exposed topics (default no one)
```

The package is written to `<robot>_generatedFiles/<robot>_server`. `copy_gen:
true` also copies it to `src/thunder_ros_test/src/<robot>_server`, ready for a
colcon build. Lowercase is enforced in the robot name to be a valid ROS 2 package-name prefix:
lowercase letters, digits, and underscores. The generated package contains
`src/`, `include/`, `msg/`, `srv/`, `launch/`, `config/`, `CMakeLists.txt`, and `package.xml`.

## Configuration

`frequency` is the node update frequency in Hz and defaults to `1000`.
`real_time` defaults to `false`. When enabled, the node requests `SCHED_FIFO`
priority `25`; it logs a warning and continues normally if the process lacks permission. Incoming topics always use `realtime_tools::RealtimeBuffer`, even when real-time scheduling is disabled.

`inputs` maps a runtime Thunder parameter to the topic used to update it:

```yaml
inputs:
  q: /robot/q
```

Input messages are float32[] and the message is in the package folder. Runtime parameters are normally vectors. If a matrix parameter is added in a future model, its data is also accepted as a flattened row-major array. A parameter must have symbolic entries in the configuration yaml file to appear in `inputs`, `services`, or `topics`.

`services` is a list of function and parameter names. When omitted, every function and every runtime parameter is exposed. An explicitly empty list creates no services. `topics` is a list of function and parameter names and defaults to no publishers. Unknown, duplicate, or non-runtime parameter names are rejected during generation.

Generated service and topic names are relative to the current ROS namespace (the root namespace by default):

| Selected item | Service(s) | Topic |
| --- | --- | --- |
| Function `M` | `get_M` (`GetVale`) | `M` (`Float32Array`) |
| Parameter `q` | `get_q` (`GetValue`), `set_q` (`SetValue`) | `q` (`Float32Array`) |

A function and parameter get service uses the reusable `GetValue.srv`:

```srv
float32[] input
---
float32[] value
```

The request contains the function's explicit arguments (if any), concatenated in the order declared by Thunder. Most Thunder functions have no explicit arguments; their `input` field must therefore be an empty array (`[]`). Regular Thunder function dependencies are robot parameters and are therefore set through their parameter services or `inputs` topics. Function output, parameter values, and topic messages are all flattened in row-major order. Callers know the dimensions from the robot model; the interfaces intentionally do not transmit shape metadata.

Parameter set services use these reusable definitions:

```srv
# SetValue.srv
float32[] value
---
bool success
string message
```

## Build and run

For direct ROS integration testing, open `src/thunder_ros_test` as a VS Code devcontainer. It uses ROS 2 Humble and provides a colcon workspace. After a package has been copied there:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch <robot>_server <robot>_server.launch.py
```

The generated Python launch file starts `<robot>_server_node` with the node name `<robot>_server`.

The test workspace also includes `rrr_server_test`, a small manual client for the RRR example. It sets `q`, reads `q`, then calls `M` with an empty GetValue request. See `src/thunder_ros_test/README.md` for the command.
