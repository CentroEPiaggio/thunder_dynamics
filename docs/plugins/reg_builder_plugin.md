# Regressors Builder Plugin

`reg_builder` builds the regressors: matrices that multiply the robot parameters linearly. The main ones are the dynamic regressors,

```
M(q) ddqr + C(q, dq) dqr + G(q) = Yr(q, dq, dqr, ddqr) par_REG
```

It adds the variables `dqr`, `ddqr` (reference velocity and acceleration) and `w` (wrench). Run it after `dyn_builder`.

```yaml
pipeline:
  builders: [kin_builder, dyn_builder, reg_builder]

reg_builder:                  # plugin configuration, optional
  regressor_method: rnea      # Yr, Y, reg_M, reg_C, reg_G: "rnea" (default) or "lagrange"
```

## Dynamic regressors

`par_REG` holds 10 parameters per link: `[m, m CoM_x, m CoM_y, m CoM_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]`. The inertia is about the link frame origin. `dyn_builder` sets its value from `par_DYN` (`dyn2reg`). Links attached to the world give zero columns.

| Function | Arguments | Regressor of |
| --- | --- | --- |
| `Yr` | `q, dq, dqr, ddqr, par_KIN, par_gravity` | `M ddqr + C dqr + G` (Slotine-Li) |
| `Y` | `q, dq, ddq, par_KIN, par_gravity` | `M ddq + C dq + G` |
| `reg_M` | `q, ddqr, par_KIN` | `M ddqr` |
| `reg_C` | `q, dq, dqr, par_KIN` | `C dqr` |
| `reg_G` | `q, par_KIN, par_gravity` | `G` |

`regressor_method` selects how they are built. Both give the same matrices; `thunder_robot_comparison_gtest` checks both against Pinocchio.

| `regressor_method` | How it works |
| --- | --- |
| `rnea` (default) | The modified RNEA of `dyn_builder` written with the `par_REG` parameters, so it is linear in them; each regressor is its Jacobian with respect to `par_REG`. |
| `lagrange` | Per link, from the centre-of-mass Jacobians and Christoffel symbols of each parameter's contribution to `M`. |

`rnea` is much smaller. Franka with a prismatic finger (8 dof, symbolic parameters): `Yr` has 457k CasADi instructions with `lagrange` and 15k with `rnea`, and the pipeline builds in 16 s against 2.6 s.

## Other regressors

These do not depend on `regressor_method`. They are added only when the corresponding parameters have symbolic entries.

| Function | Arguments | Regressor of |
| --- | --- | --- |
| `reg_Jdq_<joint>`, `reg_JTw_<joint>` | `q, dq, par_KIN` / `q, w, par_KIN` | `J dq` and `Jᵀ w` w.r.t. the kinematic parameters, for each available joint |
| `reg_dl` | `dq` | Link friction `dl`, if `Dl_order > 0` |
| `reg_k`, `reg_d`, `reg_dm`, `reg_Mm` | `q, x` / `dq, dx` / `dx` / `ddx` | Elastic joints: coupling stiffness, coupling damping, motor friction, motor inertia |
