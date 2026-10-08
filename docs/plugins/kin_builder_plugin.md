# Kinematics Builder Plugin

`kin_builder` builds the kinematics of the robot: the transform of every frame, its Jacobian, and, for the frames that ask for it, the time derivatives and the damped pseudo-inverse of the Jacobian. It also registers the built-in joint types. It reads the kinematic structure loaded by `kin_loader`, `urdf_loader` or `dh_loader`, and must run before `dyn_builder` and `reg_builder`, which use its frames and joint types.

```yaml
pipeline:
  loaders: [kin_loader, dyn_loader]
  builders: [kin_builder, dyn_builder]
  generators: [robot_generator]
```

`kin_builder` has no configuration keys.

## What it does

1. **Joint types.** It registers `T_JOINT_<type>` and `S_JOINT_<type>` for the built-in types `R`, `P` and `FIXED`. For a joint type that defines only `T_JOINT_<type>` (registered by another plugin), it derives `S_JOINT_<type>` and registers it, so that every later plugin finds it. See [Joint types](../joints.md).
2. **Transforms.** `T_i = X_i(par_KIN) T_JOINT_<type>(q_i, axis_i)` from frame `parent(i)` to frame `i`, where `X_i` is the fixed frame of the joint (`xyz`, `rpy`). `T_w_i` is the product along the tree from the world frame.
3. **Jacobians** of every frame, from the motion subspaces of the joints that move it (below).
4. **Jacobian derivatives and pseudo-inverse**, only for the frames with `derivatives: true`.

## Functions

| Function | Arguments | Description |
| --- | --- | --- |
| `T_JOINT_<type>`, `S_JOINT_<type>` | `q_joint, axis` (explicit) | Joint templates, see [Joint types](../joints.md) |
| `T_<i>` | `q, par_KIN` | Transform from frame `parent(i)` to frame `i` |
| `T_w_<i>`, `T_w_<name>` | `q, par_KIN` | Transform from the world frame to frame `i` |
| `J_<i>`, `J_<name>` | `q, par_KIN` | Jacobian of frame `i`, 6 x ndof |
| `J_<name>_dot` | `q, dq, par_KIN` | Time derivative of the Jacobian |
| `J_<name>_ddot` | `q, dq, ddq, par_KIN` | Second time derivative of the Jacobian |
| `J_<name>_pinv` | `q, par_KIN` | Damped pseudo-inverse of the Jacobian, ndof x 6 |

`<i>` is the index of the frame, `<name>` its name. The functions with `<name>` exist only for the frames marked `available` (`T_w_<name>`, `J_<name>`) or `derivatives` (all of them, and `derivatives` implies `available`). With `kin_loader` these are the `available` and `derivatives` keys of each joint. `urdf_loader` marks the end-effector frames, and `dh_loader` the last frame.

As everywhere in thunder, only the symbolic entries of `par_KIN` are arguments of the generated functions. The numeric ones are constants in the expressions.

With `robot_generator`, each function becomes a `get_<name>()` method of the generated class, e.g. `get_T_w_EE()` and `get_J_EE_dot()`.

## Conventions

- **Tree.** Joint `i` moves frame `i` with respect to frame `parent(i)` (`-1` is the world frame). Every joint must come after its parent.
- **Jacobian.** `J_i` maps `dq` to `[v; w]`: `v` is the linear velocity of the origin of frame `i`, `w` the angular velocity of the frame. Both are in world coordinates. The linear part comes first, as in the motion subspaces and the spatial vectors of `dyn_builder`.
- **Pseudo-inverse.** `J_<name>_pinv = Jᵀ (J Jᵀ + μ I)⁻¹` with `μ = 0.02` (`MU` in `kin_builder.cpp`).

## How the Jacobians are computed

Let `p_j` and `R_j` be the origin and orientation of frame `j` (from `T_w_j`) and `S_j` the motion subspace of joint `j` (`S_JOINT_<type>`, 6 x dim, in frame `j`). In world coordinates the subspace is `[L_j; z_j] = [R_j S_j,lin; R_j S_j,ang]`: `L_j` is the velocity of `p_j` and `z_j` the angular velocity per unit `dq_j`. For a revolute joint `L_j = 0` and `z_j` is the axis.

**Jacobian.** Each joint `j` that moves frame `i` (frame `i` itself or one of its ancestors) gives the columns of its own `dq_j`:

```
J_i(:, q_j) = [ L_j + z_j x (p_i - p_j) ;  z_j ]
```

The other columns are zero. This is the geometric Jacobian, so no derivative of the transforms is needed.

**Time derivative.** With `w_j` the angular velocity of frame `j` and `v_j` the velocity of its origin, both computed recursively from the root,

```
w_j = w_parent + z_j dq_j
v_j = v_parent + w_parent x (p_j - p_parent) + L_j dq_j
d/dt [L_j; z_j] = [w_j x L_j; w_j x z_j] + R_j dS_j/dt
```

and the derivative of each column is

```
dJ_i(:, q_j) = [ dL_j + dz_j x (p_i - p_j) + z_j x (v_i - v_j) ;  dz_j ]
```

`dS_j/dt` is computed with `jtimes` and is zero for joints with a constant subspace (`R`, `P`). For joints whose `S` depends on `q` (e.g. a universal joint), it makes the result exact.

**Second derivative.** `J_<name>_ddot` is `jtimes` of `J_<name>_dot` with respect to `q` and `dq`: `∂J̇/∂q dq + ∂J̇/∂dq ddq`.

**Pseudo-inverse.** `Jᵀ (J Jᵀ + μ I)⁻¹` equals `(Jᵀ J + μ I)⁻¹ Jᵀ`, so the smaller of the two systems is solved: 6 x 6, or ndof x ndof when ndof < 6. Both matrices are positive definite for `μ > 0`, so an LDLᵀ factorisation without pivoting is used. It needs no square roots and gives smaller code than `SX::inv`.

### Cost

These formulas replaced the automatic differentiation of the transforms (`jacobian` of the origin and `dR/dq Rᵀ` for the angular part, `jtimes` of `J` for the derivatives). The values are the same to machine precision. Franka with a prismatic finger (8 dof, last frame), CasADi instructions (with `cse`), and the time of the generated code with numeric kinematics (gcc `-O3`, one core):

| | symbolic `par_KIN` | numeric `par_KIN` | time, numeric |
| --- | --- | --- | --- |
| `J_<i>` | 4030 → 1190 | 1466 → 412 | 141 → 70 ns |
| `J_<name>_dot` | 8806 → 1532 | 3954 → 672 | 286 → 94 ns |
| `J_<name>_ddot` | 21063 → 3498 | 11373 → 1961 | 1036 → 198 ns |
| `J_<name>_pinv` | 5687 → 2126 | 3061 → 1307 | 332 → 155 ns |

`dyn_builder` builds the centre-of-mass Jacobians `J_cm_<i>` (used by `dynamics_method: lagrange`) from these frame Jacobians, which reduced their total size from 7.8k to 3.3k instructions on the same robot.

With symbolic `par_KIN`, the sines and cosines of the frame angles are evaluated at every call: on the robot above, `T_w` takes about 460 ns with symbolic `par_KIN` and 60 ns with numeric `par_KIN`. Keep `par_KIN` numeric unless its entries really change at run time.

Each generated function is evaluated on its own, so calling `get_T_w_EE()`, `get_J_EE()` and `get_J_EE_dot()` computes the chain three times. A CasADi problem built from the symbolic expressions (`robot->get_model(...)`) shares them automatically.
