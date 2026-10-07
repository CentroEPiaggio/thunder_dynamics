# Joint Types

A joint type is defined by two template functions stored in the robot. Builders find them through the `joint_type` string of each joint, so adding a joint type means adding its templates; no builder code changes.

| Template | Size | Needed by | Meaning |
| --- | --- | --- | --- |
| `T_JOINT_<type>(q_joint, axis)` | 4 x 4 | `kin_builder` (always) | Transform added by the joint motion |
| `S_JOINT_<type>(q_joint, axis)` | 6 x dim | `dyn_builder` with RNEA (optional) | Motion subspace of the joint |

Both templates take two explicit arguments:

- `q_joint`: the joint variables, `dim x 1`. `dim` is the joint dimension: 1 for R and P, 0 for FIXED.
- `axis`: the `axis` of the joint, `3 x 1`, from the URDF or from the `axis` key of `kin_loader` (default `[0, 0, 1]`).

## T_JOINT: kinematics

`kin_builder` builds the transform of joint `i` from its parent frame as

```
T_i = X_i(par_KIN) * T_JOINT_<type>(q_i, axis_i)
```

where `X_i` is the fixed frame (`xyz`, `rpy`) of the joint and `T_JOINT` is the motion of the joint in that frame (the identity at `q_joint = 0` for the built-in types).

## S_JOINT: motion subspace

`S_JOINT` gives, for each joint variable, the velocity of the joint frame per unit joint velocity, as a twist `[linear; angular]` in the joint frame (the frame after the motion). As everywhere in thunder (Jacobians, spatial vectors), the linear part comes first:

```
S = vee(T_JOINT^-1 * dT_JOINT/dq_joint)       one column per joint variable
```

For example, a revolute joint has `S = [0; axis/|axis|]` and a prismatic joint has `S = [axis; 0]`. The RNEA dynamics (`dynamics_method: rnea`, `C_method: rnea`) uses only `S`. When `S` depends on `q_joint`, its time derivative is handled automatically.

`S_JOINT` is optional. If it is missing, `dyn_builder` derives it from `T_JOINT` with the formula above. The result is exact, but CasADi does not simplify trigonometric identities, so a derived `S` can be a q-dependent expression that is numerically constant. That makes the generated code larger: on a test tree with non-unit revolute axes, about 25% more instructions for `M` and almost twice as many for the Christoffel `C`. So define `S_JOINT` whenever you know it in closed form.

## Existing joint types

| Type | Defined in | `T_JOINT` | `S_JOINT` | dim |
| --- | --- | --- | --- | --- |
| `R` | `kin_builder` | rotation `R_aa(axis, q)` (axis normalised) | `[0; axis/\|axis\|]` | 1 |
| `P` | `kin_builder` | translation `axis * q` | `[axis; 0]` | 1 |
| `FIXED` | `kin_builder` | identity | none (dim 0) | 0 |
| `R_SEA` | `soft_loader` | as `R` | as `R` | 1 |
| `P_SEA` | `soft_loader` | as `P` | as `P` | 1 |

The `_SEA` types are also recognised by `soft_loader` and `soft_builder`, which add the elastic model.

## Adding a joint type

1. **Register the templates** before `kin_builder` runs, in a loader or in a builder listed before it, with the same signature as the existing ones. Example, a helical (screw) joint `H` with pitch `h` (metres per radian):

   ```cpp
   SX q_joint = SX::sym("q_joint");
   SX axis = SX::sym("axis", 3, 1);
   FunArg q_joint_arg("q_joint", q_joint), axis_arg("axis", axis);
   SX a = axis / SX::norm_2(axis);
   const double h = 0.01;

   SX T = SX::eye(4);
   T(casadi::Slice(0,3), casadi::Slice(0,3)) = R_aa(axis, q_joint);
   T(casadi::Slice(0,3), 3) = h * a * q_joint;
   robot->add_function("T_JOINT_H", T, {}, "Helical joint", {q_joint_arg, axis_arg});

   SX S = SX::vertcat({h * a, a});		// [linear; angular]
   robot->add_function("S_JOINT_H", S, {}, "Motion subspace of the helical joint", {q_joint_arg, axis_arg});
   ```

   From a Python plugin the same calls are `robot.add_function(name, expr, [], descr, [FunArg("q_joint", q_joint), FunArg("axis", axis)])`.

2. **Use it in the configuration.** `kin_loader` knows the dimension only of the built-in types, so give it explicitly:

   ```yaml
   kin_loader:
     kinematics:
       joint3:
         parent: joint2
         joint_type: H
         dimension: 1
         axis: [0, 0, 1]
         xyz: [0, 0, 0.1]
         rpy: [0, 0, 0]
   ```

   `urdf_loader` maps only the URDF types revolute, continuous, prismatic and fixed.

3. **Check it.** With `dyn_builder` the two methods must agree: build once with `dynamics_method: lagrange` and once with `rnea` and compare `M`, `C` and `G`. Lagrange uses only `T_JOINT`; RNEA uses `S_JOINT`. A wrong `S_JOINT` shows up as a mismatch. To check `S_JOINT` itself, compare it with the derived one by temporarily removing it.
