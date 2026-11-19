# Thunder ros2_control test workspace (Jazzy)

A ROS 2 Jazzy colcon workspace for the controller packages of `ros_control_generator` ([docs](../../docs/plugins/ros_control_plugin.md)). `copy_gen: true` in the generator configuration copies `<robot>_controller` and the robot library `thunder_<robot>` to `src/` (when Thunder runs in its dev container).

Open this folder in VS Code with **Dev Containers: Reopen in Container**, or without VS Code:

```bash
docker build -t thunder_ros_control_test:jazzy .devcontainer
docker run -d --name thunder_ros_control_test --network=host \
  -v "$PWD":/workspaces/thunder_ros_control_test thunder_ros_control_test:jazzy sleep infinity
docker exec -it thunder_ros_control_test bash
```

The image has `ros2_control`, `ros2_controllers`, `realtime_tools`, `gz_ros2_control` and `ros_gz` (about 3.5 GB). New shells source ROS and the workspace overlay.

## Build and unit tests

```bash
colcon build
colcon test --packages-select franka_controller && colcon test-result --verbose   # with gen_test: true
```

## Gazebo test of the Franka controller

`bringup/` is not a package: it holds the test robot, the launch file and a check script.

| file | what |
| --- | --- |
| `make_urdf.py` | test URDF from a URDF: chain base..tip, no visual/collision, fixed to the world, `ros2_control` block (effort command; position, velocity, effort state), `gz_ros2_control` plugin |
| `franka_gz.urdf` | `src/thunder/robots/debug/franka.urdf` from `panda_link0` to `panda_link8` (the chain of `src/thunder/robots/debug/franka_ros_control.yaml`), initial pose `[0, -0.785, 0, -2.356, 0, 1.571, 0.785]` |
| `controller_manager.yaml` | 1 kHz, `joint_state_broadcaster` |
| `gz_overrides.yaml` | `robot_adds_gravity: false`: `gz_ros2_control` applies the command as it is |
| `gz.launch.py` | headless Gazebo, `robot_state_publisher`, spawn, `joint_state_broadcaster`, the controller with the generated parameters and the overrides |
| `check_controller.py` | rejected trajectory, one point, two points, `header.stamp` now / past (jump) / ending in the past, adaptation on; prints the tracking errors. Uses simulation time for the stamps |

`franka_gz.urdf` was made from the repository root with

```bash
python3 src/thunder_ros_control_test/bringup/make_urdf.py src/thunder/robots/debug/franka.urdf \
  src/thunder_ros_control_test/bringup/franka_gz.urdf --base panda_link0 --tip panda_link8 \
  --initial 0 -0.785 0 -2.356 0 1.571 0.785 \
  --controllers /workspaces/thunder_ros_control_test/bringup/controller_manager.yaml
```

Run, in the container, in a ROS domain of its own: the controller manager hands the controller the last `/robot_description` it received, also from another node on the host network (seen once, the controller then failed to configure).

```bash
export ROS_DOMAIN_ID=73        # any domain no other ROS node uses, in both shells
ros2 launch bringup/gz.launch.py urdf:=/workspaces/thunder_ros_control_test/bringup/franka_gz.urdf
python3 bringup/check_controller.py franka_controller     # second shell
```

The URDF gives every joint `<dynamics damping="1.0"/>`, which Gazebo applies and the Thunder model does not have: the tracking error during motion is about `damping * velocity / k_p` per joint.

The container does not request real-time capabilities; timing is not meaningful here.
