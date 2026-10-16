# Soft Loader Plugin

`soft_loader` adds series elastic joints: the joint types `R_SEA` and `P_SEA`, the motor variables and the parameters of the elastic coupling and of the motors. `soft_builder` builds the elastic model from them. Run it after the loader of the structure, which must use the `_SEA` types for the elastic joints.

```yaml
pipeline:
  loaders: [kin_loader, dyn_loader, soft_loader]
  builders: [kin_builder, dyn_builder, soft_builder]

kin_loader:
  kinematics:
    joint1:
      parent: world
      joint_type: R_SEA         # elastic joint
      xyzrpy: [0, 0, 1, 0, 0, 0]
    ...

soft_loader:
  K_order: 1                    # stiffness terms per joint
  D_order: 0                    # coupling damping terms per joint
  Dm_order: 1                   # motor damping terms per joint
  joints:
    joint1:                     # one entry per elastic joint, by node name
      K: [10.0]
      D: []
      Dm: [0.1]
      Mm: 1.0
      K_symb: [0]
      D_symb: []
      Dm_symb: [0]
      Mm_symb: 0
```

## Joint types

`R_SEA` and `P_SEA` have the same kinematics as `R` and `P` (`T_JOINT` and `S_JOINT`, see [Joint types](../joints.md)). The link side of the joint is the joint variable `q`; the motor side is the new variable `x`. Each elastic joint has one motor.

## Configuration

`K_order`, `D_order` and `Dm_order` are required when the robot has at least one elastic joint. Each order is the number of terms of that model, see [`soft_builder`](soft_builder_plugin.md).

`joints` has one entry per elastic joint, keyed by its node name; the order does not matter, a missing entry is an error. The parameters and `x` follow the order of the elastic joints in the tree. Each entry holds:

| Key | Size | Meaning |
| --- | --- | --- |
| `K` | `K_order` | Stiffness coefficients |
| `D` | `D_order` | Damping coefficients of the coupling |
| `Dm` | `Dm_order` | Damping coefficients of the motor |
| `Mm` | 1 | Motor inertia |
| `K_symb`, `D_symb`, `Dm_symb`, `Mm_symb` | as above | Masks, default all `0` (see [masks](kin_loader_plugin.md#symbolic-and-numeric-parameters)) |

## What it adds to the robot

| Kind | Name | Description |
| --- | --- | --- |
| Joint types | `T_JOINT_R_SEA`, `T_JOINT_P_SEA`, `S_JOINT_R_SEA`, `S_JOINT_P_SEA` | As `R` and `P` |
| Properties | `numSoftJoints` | Number of elastic joints |
| | `isSoftJoint` | Per joint variable, `1` if the joint is elastic |
| | `K_order`, `D_order`, `Dm_order` | Orders of the models |
| Variables | `x`, `dx`, `ddx` | Motor position, velocity, acceleration, `numSoftJoints x 1` |
| | `ddxr` | Reference motor acceleration |
| Parameters | `par_K`, `par_D`, `par_Dm` | Coefficients, `K_order`, `D_order`, `Dm_order` per elastic joint |
| | `par_Mm` | Motor inertias, one per elastic joint |
