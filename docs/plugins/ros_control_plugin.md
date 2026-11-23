# ROS Control Generator Plugin

`ros_control_generator` creates a `ros2_control` controller package (ROS 2 Humble and Jazzy, the same package for both) around the C++ robot produced by `robot_generator`: a MACT joint controller (Modified Adaptive Computed Torque) with the robot model linked in, for any robot whose `ros2_control` hardware interface has an `effort` command. It must run after `robot_generator` and needs the dynamic regressors of `reg_builder`.

```yaml
pipeline:
  loaders: [urdf_loader]
  builders: [kin_builder, dyn_builder, reg_builder]   # reg_builder: Yr, Y, reg_G, par_REG
  generators: [robot_generator, ros_control_generator]

robot_generator:
  gen_robot: true     # needed by ros_control_generator

ros_control_generator:
  copy_gen: false     # also copy the package into src/thunder_ros_control_test/src
  # joints: [...]     # URDF joint of each entry of q; not needed with urdf_loader
  parameters:         # defaults of the controller's ROS parameters (below)
    k_p: [30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 4.0]
    k_v: [5.0, 5.0, 5.0, 5.0, 1.0, 1.0, 0.4]
    robot_adds_gravity: true
```

The joint limits are not configured here: they are parameters of the robot class (see [Joint limits](#joint-limits)).

A complete example is `src/thunder/robots/debug/franka_ros_control.yaml`. The package is written to `<robot>_generatedFiles/<robot>_controller`; the robot name must give a valid ROS 2 package name (lowercase letters, digits, underscores).

## The controller

The control law and the update law, with `e = q_d - q`:

```
tau        = Yr(q, dq, dq_d, ddq_d) pi_hat + K_v e_dot + K_p e
pi_hat_dot = R_p ( Yr^T e_dot + gamma Y(q, dq, ddq)^T R_t (tau_applied - Y pi_hat) )
```

`pi_hat` is the estimate of `par_REG` (10 parameters per Thunder node, see [`reg_builder`](reg_builder_plugin.md#par_reg)); it starts from the generated `par_REG` at every activation. `K_p`, `K_v`, `R_t`, `R_p` are diagonal. The update law runs only when adaptation is enabled.

One cycle of `update()`, in this order:

1. read `q`, `dq` (and the measured torque if the update law uses it);
2. filter `dq`; `ddq` is the filtered difference of the raw `dq`;
3. sample the reference `q_d, dq_d, ddq_d` from the active trajectory;
4. `Yr` from the generated library; the law;
5. the update law: `Y` is evaluated only when adaptation is on and `gamma > 0`;
6. the command: if the robot adds its own gravity, `G = reg_G(q) par_REG_0` with the generated parameters is removed, so that the robot applies `Yr pi_hat + (G_robot - G)`; then saturation at `tau_max` and rate limiting at `dtau_max`; write.

The algorithm is in plain headers without ROS (`mact.hpp`, `trajectory.hpp`, `filter.hpp`), tested by `test/test_algorithm.cpp` (quintic, trajectory sampling, limit check, filter, MACT terms; it does not use the robot model, so it is the same for every robot), written in the package with `gen_test: true` and run with `colcon test`. Nothing in `update()` allocates or logs; trajectories are handed over with `realtime_tools::RealtimeBuffer`, topics are published with `realtime_tools::RealtimePublisher`.

### Interfaces

| | name | type |
| --- | --- | --- |
| command | `<joint>/effort` | |
| state | `<joint>/position`, `<joint>/velocity` | |
| state | `<joint>/effort`, only with `adaptation.torque_source: measured` and `gamma > 0` | |
| subscription | `~/joint_trajectory` | `trajectory_msgs/JointTrajectory` |
| service | `~/set_adaptation` | `std_srvs/SetBool` |
| publisher | `~/controller_state` | `control_msgs/JointTrajectoryControllerState` (reference, feedback, error, output effort) |
| publisher | `~/par_REG` | `std_msgs/Float64MultiArray`, `pi_hat` |

### Trajectories

A `JointTrajectory` with one point is a final state (`positions`, optional `velocities`, `accelerations`) reached at `time_from_start`; with more points, each segment is a quintic polynomial that matches position, velocity and acceleration at both ends. Missing velocities and accelerations are zero.

- The trajectory replaces the current one when the control loop takes it. Its first segment starts from the reference the controller is following then, so the reference is continuous.
- `header.stamp`, as in `joint_trajectory_controller`: zero means the times count from the cycle that takes the trajectory; otherwise they count from the stamp, on the controller manager's clock (simulation time in Gazebo). With the current time as stamp, the first segment is shorter by the delivery delay. A stamp in the future does not delay the start: the first segment is longer.
- A trajectory that ends in the past is rejected. If only its first point is past, it is accepted with a warning and the reference jumps onto the trajectory at the current time. A first point that becomes past between reception and the cycle that takes it (a few ms) gives the same jump without the warning.
- A trajectory received while the controller is not running is rejected.
- It must name all the joints, in any order; at most 256 points; `time_from_start` positive and increasing.
- Every segment is sampled (100 samples) against `q_min`, `q_max`, `dq_max` and `ddq_max` of the robot class, except a first segment that will not run (first point past). A trajectory that exceeds them is rejected with a warning and the current one continues. The first segment is checked from the last reference and time reported by the loop, a few cycles before the loop takes the trajectory: it will start from a slightly later reference and be slightly shorter. The loop also clamps `q_d` to the position limits.
- After the last point the reference holds its position at rest. At activation the reference is the measured position at rest.

## Configuration

| Key | Default | Meaning |
| --- | --- | --- |
| `parameters` | required | Values of the controller's ROS parameters written in `config/<robot>_controller.yaml`. Unknown names, wrong sizes and missing required ones are errors at generation |
| `joints` | from `urdf_loader` | URDF joint of each entry of `q`. With `urdf_loader` they are the nodes of the property `jointsUrdfName` that move; with other loaders this list is required |
| `copy_gen` | `false` | Also copy the package into `src/thunder_ros_control_test/src` (inside the Thunder dev container) |
| `gen_test` | `false` | Write `test/test_algorithm.cpp`, the unit test of the algorithm headers. With `false`, a test left by an earlier generation is removed |
| `template_path` | `/usr/share/thunder_dynamics/templates/ros_control_generator` | Folder of the templates, installed with thunder (`make install`) |

### Controller parameters

The required ones depend on the robot and have no default in the controller; the others default to "do nothing" (no adaptation, no filtering). Sizes: `n` joints, `N` Thunder nodes (the generated YAML lists them in order).

| Parameter | Required | Size | Meaning |
| --- | --- | --- | --- |
| `joints` | | `n` | Joint names; default from generation |
| `k_p`, `k_v` | yes | `n` | Diagonal gains. They are not scaled by the mass matrix |
| `robot_adds_gravity` | yes | bool | The hardware adds its own gravity to the effort command (libfranka, Flexiv): remove the model's |
| `adaptation.enabled` | | bool | Value at every activation; switched by `~/set_adaptation`. Default `false` |
| `adaptation.gamma` | | scalar | Weight of the prediction-error term. Default `0` |
| `adaptation.torque_source` | | `measured` / `commanded` | `tau_applied`: the effort state, or the command of the previous cycle plus the removed `G`. Default `measured` |
| `adaptation.R_t` | | `n` | Default ones |
| `adaptation.R_p_link`, `adaptation.R_p_param` | | `N`, 10 | `R_p = R_p_link[node] R_p_param[k]`, `k` over `[m, m c_x, m c_y, m c_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]`. Default zeros |
| `filters.dq_cutoff_hz`, `filters.ddq_cutoff_hz` | | scalar | First-order low-pass [Hz]; `<= 0` means no filtering. Default `0` |

The rate of the loop is the controller manager's `update_rate`; the loop uses the measured period.

### Joint limits

The controller uses the limits of the robot class, the parameters `q_min`, `q_max`, `dq_max`, `ddq_max`, `tau_max`, `dtau_max` added by the loaders (see [`urdf_loader`](urdf_loader_plugin.md#joint-limits)): from the URDF `<limit>` with `urdf_loader`, from the `limits` key of the loader configuration otherwise. A limit not given is infinite: no limit. They are read from the robot object where they are used (in every cycle and when a trajectory arrives), not copied at configuration, since they are runtime parameters that can be changed; nothing changes them while the controller runs yet. The controller does not read the URDF.

### ROS 2 distribution

The same generated package builds on Humble and Jazzy (tested with ros2_control 2.51.0 and 4.50.0). The only difference in the code is how a state interface is read: `get_optional()` from ros2_control 4.27 (Jazzy), `get_value()` before; `CMakeLists.txt` chooses from the version of `hardware_interface` (`THUNDER_GET_OPTIONAL`). Parameters are declared only if not declared yet, since Humble declares the ones in the YAML automatically.

### Robot-specific setup

The controller does nothing specific to a robot beyond the configuration above. Settings of the robot itself have to be configured manually.

An end effector or payload with unknown inertia: keep its link symbolic in `urdf_loader` (`symbolic_dynamics`) and give its node a non-zero `adaptation.R_p_link`.

## Generated package

```
<robot>_controller/
  CMakeLists.txt, package.xml, <robot>_controller.xml     # pluginlib: <robot>_controller/MactController
  include/<robot>_controller/  controller.hpp, mact.hpp, trajectory.hpp, filter.hpp
  src/  controller.cpp
  test/ test_algorithm.cpp                                  # only with gen_test: true
  config/ <robot>_controller.yaml
thunder_<robot>/                                            # the robot library, linked by the controller
```

The robot is written as its own package `thunder_<robot>` (a shared library), the same one `ros_server_generator` writes and links; see the [ROS Server docs](ros_server_plugin.md) for its content. `copy_gen` copies both packages.

`config/<robot>_controller.yaml` has the controller type in the controller manager's section and in the controller's own, so it works loaded into the controller manager or with `spawner <robot>_controller -p <file>`. Sizes (`n`, `N`, size of `par_REG`) and the joint names are compile-time constants of the package: the code uses fixed-size Eigen types.

## Build and run

`src/thunder_ros_control_test` is a Jazzy workspace with its dev container and a Gazebo test of the generated Franka controller, also run on Humble; see its [README](../../src/thunder_ros_control_test/README.md).

## Notes

- `tau_applied` must be the full joint torque, gravity included, since it is compared with `Y pi_hat`. `commanded` is the torque written in the previous cycle (after saturation and rate limit) plus, when `robot_adds_gravity` is set, `reg_G(q) par_REG_0`: the gravity the robot adds, approximated with the nominal parameters (their difference biases the estimate). `measured` is the effort state as the driver reports it, which must then be the full joint torque. Checked on 2026-11-23: FR3 `franka_hardware` reports `tau_J`, the link-side torque sensors (libfranka: `tau_J_d`, the desired one, is the one "without gravity"); Flexiv reports the RDK `tau`, "measured joint torques" (0 for a joint without torque sensor); UR reports `actual_current_as_torque` (what it includes not verified), or currents in A with the driver parameter `use_currents_as_efforts`, which are not torques. A hardware interface whose effort state echoes the gravity-free command cannot be used with `measured`.
- With `adaptation.torque_source: measured` the update law needs a meaningful effort state. The controller does not change its configuration, but warns once per joint and activation (from a 1 s timer, outside the loop) when the effort state has been exactly 0 for 100 consecutive cycles while the command was not: the hardware interface probably does not update it. This happens in simulation with `gz_ros2_control` 0.7.x (Humble), which never updates it (`gz_system.cpp`, `read()`, the line is commented out, ign-physics#124); there adaptation diverges, use `commanded`. `gz_ros2_control` 1.2 (Jazzy) reports the transmitted joint torque.

## Not yet

- `control_msgs/FollowJointTrajectory` action.
- Model services from the controller, like `ros_server_generator`.
- Cartesian reference.
- First real-time cycle: the generated getters use `thread_local` buffers; their first use in the control thread happens in the first `update()`. Whether that allocates is not measured yet.
