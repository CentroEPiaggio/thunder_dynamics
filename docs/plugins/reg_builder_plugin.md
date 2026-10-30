# Regressors Builder Plugin

`reg_builder` builds the regressors: matrices that multiply the robot parameters linearly. The main ones are the dynamic regressors,

```
M(q) ddqr + C(q, dq) dqr + G(q) = Yr(q, dq, dqr, ddqr) par_REG
```

It adds the variable `w` (wrench). It needs the kinematics (`kin_builder`). `dyn_builder` is optional: without it `reg_builder` still builds the dynamic regressors and adds `par_REG`, `tau` and `tau_r` itself, e.g. for a Slotine-Li controller where the whole dynamics is `Yr par_REG`. When both are used, run it after `dyn_builder`.

```yaml
pipeline:
  builders: [kin_builder, dyn_builder, reg_builder]   # or [kin_builder, reg_builder]

reg_builder:                  # plugin configuration, optional
  regressor_method: rnea      # Yr, Y, reg_M, reg_C, reg_G: "rnea" (default) or "lagrange"
  par_REG: [...]              # initial value of par_REG, 10 per link (optional)
```

## Dynamic regressors

They need `par_gravity`, `dqr` and `ddqr`, added by `dyn_loader` or `urdf_loader`. Without them `reg_builder` skips this whole part (dynamic regressors, `par_REG`, `tau`, `tau_r`) with a warning and builds only the other regressors.

| Function | Arguments | Regressor of |
| --- | --- | --- |
| `Yr` | `q, dq, dqr, ddqr, par_KIN, par_gravity` | `M ddqr + C dqr + G` (Slotine-Li) |
| `Y` | `q, dq, ddq, par_KIN, par_gravity` | `M ddq + C dq + G` |
| `reg_M` | `q, ddqr, par_KIN` | `M ddqr` |
| `reg_C` | `q, dq, dqr, par_KIN` | `C dqr` |
| `reg_G` | `q, par_KIN, par_gravity` | `G` |

Links attached to the world give zero columns. `regressor_method` selects how they are built. Both give the same matrices; `thunder_robot_comparison_gtest` checks both against Pinocchio.

| `regressor_method` | How it works |
| --- | --- |
| `rnea` (default) | The modified RNEA of `dyn_builder` written with the `par_REG` parameters, so it is linear in them; each regressor is its Jacobian with respect to `par_REG`. |
| `lagrange` | Per link, from the centre-of-mass Jacobians and Christoffel symbols of each parameter's contribution to `M`. |

`rnea` is much smaller. Franka with a prismatic finger (8 dof, symbolic parameters): `Yr` has 457k CasADi instructions with `lagrange` and 15k with `rnea`, and the pipeline builds in 16 s against 2.6 s.

### `par_REG`

`par_REG` holds 10 parameters per link, `[m, m CoM_x, m CoM_y, m CoM_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]`, with the inertia about the link frame origin. All entries are symbolic. `dyn_builder` creates it; without `dyn_builder`, `reg_builder` creates it. Its initial value, in order of priority:

1. `par_REG` in the `reg_builder` configuration (10 values per link, checked);
2. `dyn2reg(par_DYN)`, if `par_DYN` exists;
3. zero.

`par_REG` only stores the parameters, to set, print, save and load them: it is independent of `par_DYN`, and changing it does not change `M`, `C`, `G`, `tau`. This keeps the two sets separate (the conversion `reg2dyn` divides by the masses). To drive the dynamics with `par_REG`, set `par_DYN` from `reg2dyn` (added by `dyn_builder`).

### `tau` and `tau_r`

If `dyn_builder` did not add them, `reg_builder` adds `tau = M ddq + C dq + G` and `tau_r = M ddqr + C dqr + G` with the same one-pass RNEA, but on `par_REG` instead of `par_DYN`:

| Function | Arguments |
| --- | --- |
| `tau` | `q, dq, ddq, par_KIN, par_gravity, par_REG` |
| `tau_r` | `q, dq, dqr, ddqr, par_KIN, par_gravity, par_REG` |

So `tau_r = Yr par_REG`. With `dyn_builder` in the pipeline, `tau` and `tau_r` depend on `par_DYN` instead (see [`dyn_builder`](dyn_builder_plugin.md#functions)).

## Other regressors

These do not depend on `regressor_method`. They are added only when the corresponding parameters have symbolic entries.

| Function | Arguments | Regressor of |
| --- | --- | --- |
| `reg_Jdq_<joint>`, `reg_JTw_<joint>` | `q, dq, par_KIN` / `q, w, par_KIN` | `J dq` and `Jᵀ w` w.r.t. the kinematic parameters, for each available joint |
| `reg_dl` | `dq` | Link friction `dl`, if `Dl_order > 0`. Needs `dl` from `dyn_builder`, skipped with a warning otherwise |
| `reg_k`, `reg_d`, `reg_dm`, `reg_Mm` | `q, x` / `dq, dx` / `dx` / `ddx` | Elastic joints: coupling stiffness, coupling damping, motor friction, motor inertia. Need the functions of `soft_builder`, skipped with a warning otherwise |
