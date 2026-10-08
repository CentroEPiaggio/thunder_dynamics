# Kinematics Loader Plugin

`kin_loader` loads the structure of the robot from the YAML file, as a tree of frames written by hand. It adds the robot structure, the joint variables and the kinematic parameters `par_KIN`. `kin_builder` turns them into transforms and Jacobians. To load from a URDF or from DH parameters, use [`urdf_loader`](urdf_loader_plugin.md) or [`dh_loader`](dh_loader_plugin.md) instead.

```yaml
pipeline:
  loaders: [kin_loader, dyn_loader]
  builders: [kin_builder, dyn_builder]
  generators: [robot_generator]

kin_loader:
  kinematics:
    base:                       # one entry per node, in tree order
      parent: world
      joint_type: FIXED
      xyzrpy: [0, 0, 0, 0, 0, 0]
    link0:
      parent: base
      joint_type: R
      xyzrpy: [0, 0, 0.333, 0, 0, 0]
    link1:
      parent: link0
      joint_type: R
      symb: [0, 0, 0, 1, 0, 0]  # roll of this frame stays a symbolic parameter
      xyzrpy: [0, 0, 0, -1.5708, 0, 0]
    EE:
      parent: link1
      joint_type: FIXED
      xyzrpy: [0, 0, 0.107, 0, 0, 0]
      derivatives: true         # also J_EE_dot, J_EE_ddot, J_EE_pinv
```

## The tree

The robot is a tree of **nodes**. Node `i` is a body (link) together with the joint that follows it:

- `T_i = X_i(par_KIN) T_JOINT_<type>(q_i, axis_i)` goes from frame `parent(i)` to frame `i`. `X_i` is the fixed frame of the joint (`xyz`, `rpy`), `T_JOINT` its motion.
- Frame `i` is the frame after joint `i`. `T_w_<name>` and `J_<name>` refer to it.
- The body of node `i` is attached to frame `parent(i)`: its inertial parameters (`dyn_loader`) are expressed there. A body attached to the world (`parent: world`) does not move.
- The `friction` of node `i` (`dyn_loader`) belongs to joint `i`, the joint after the body, not to the joint that moves it. This is temporary, until the link-joint structure is reorganised.

So the first node, with `parent: world`, is usually a fixed base frame, and the last node a fixed end-effector frame (`EE` above).

## Configuration

`kinematics` lists the nodes in order. Every node must come after its parent. The key is the node name. Each node takes:

| Key | Default | Meaning |
| --- | --- | --- |
| `parent` | `world` | Name of the parent node, or `world` |
| `joint_type` | `FIXED` | `R`, `P`, `FIXED`, `R_SEA`, `P_SEA`, or any type registered by another plugin (see [Joint types](../joints.md)) |
| `axis` | `[0, 0, 1]` | Axis of the joint, in frame `i` |
| `dimension` | from `joint_type` | Number of joint variables. Needed only for types other than the built-in ones (1 for `R`, `P`, `R_SEA`, `P_SEA`, 0 for `FIXED`) |
| `xyzrpy` | | Frame of the joint, `[x, y, z, roll, pitch, yaw]` |
| `xyz` + `rpy` | | The same, split in two vectors |
| `xyz` + `ypr` | | Translation and `[yaw, pitch, roll]`, converted to `rpy` |
| `symb` | `[0, 0, 0, 0, 0, 0]` | Which of the 6 frame entries stay symbolic (see below) |
| `available` | `false` | Add `T_w_<name>` and `J_<name>` for this frame |
| `derivatives` | `false` | Also add `J_<name>_dot`, `J_<name>_ddot`, `J_<name>_pinv`. Implies `available` |

Each node needs either `xyzrpy` or `xyz` with `rpy` or `ypr`.

**Angles.** `rpy = [r, p, y]` gives `R = R_x(r) R_y(p) R_z(y)`: rotations about the current x, then y, then z axis. This is not the URDF convention. `ypr = [y, p, r]` gives `R = R_z(y) R_y(p) R_x(r)`, which is the URDF `rpy` written in reverse order.

## Symbolic and numeric parameters

Every parameter in thunder has a mask (`symb`) with one flag per entry:

- `1`: the entry stays a symbol. It is an input of the generated functions and can be changed at run time (`set_<parameter>()` in the generated class), e.g. for identification or for a tool that changes.
- `0`: the entry is replaced by its value when the functions are built. The generated code is smaller and faster, because CasADi folds the constants. For frames it also removes the sines and cosines of the angles.

Keep entries numeric unless they really change at run time.

## What it adds to the robot

| Kind | Name | Description |
| --- | --- | --- |
| Properties | `numJoints`, `ndof` | Number of nodes, number of joint variables |
| | `jointsName`, `jointsType`, `jointsParent`, `jointsAxis`, `jointsDimension` | Per node: name, joint type, parent index (`-1` for the world), axis, dimension |
| | `jointsAvailable`, `jointsDerivatives` | Per node: the `available` and `derivatives` flags |
| Variables | `q`, `dq`, `ddq`, `d3q`, `d4q` | Joint position and its derivatives, `ndof x 1`, symbolic |
| Parameters | `KIN_<name>_xyzrpy`, or `KIN_<name>_xyz` with `KIN_<name>_rpy` / `KIN_<name>_ypr` | Frame of each node, as written in the file |
| Function | `par_KIN` | The frames of all nodes as `[x, y, z, r, p, y]`, `6 numJoints x 1`, the input of `kin_builder` |
