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

## Mathematics

Notation: `p_i` and `R_i` are the origin and orientation of frame `i` in the world (from `T_w_i`), `[a]` is the skew matrix with `[a] b = a x b`, and `dq_j` is the part of `dq` that belongs to joint `j` (`dim` entries). Every joint must come after its parent, so one pass from the root reaches each frame after its parent.

### Transforms

```
T_i   = X_i(par_KIN) T_JOINT(q_i, axis_i)        frame parent(i) -> frame i, q_i: variables of joint i
T_w_i = T_w_parent(i) T_i                        (T_w_parent = identity for the world)
```

`X_i` is built from `[x, y, z, r, p, y]` as `R = R_x(r) R_y(p) R_z(y)` and the translation `[x, y, z]`.

### Motion subspace

The motion subspace of joint `j` is its twist per unit joint velocity, in the frame after the joint (frame `j`), `[linear; angular]`. With `T = T_JOINT(q_j, axis_j)`:

```
S_j = vee(T^-1 dT/dq_j)      one column per joint variable:   T^-1 dT/dq = [ [w]  v ;  0  0 ]  ->  [v; w]
```

For the built-in types it is given in closed form: `R` is `[0; axis/|axis|]`, `P` is `[axis; 0]`. For a type that defines only `T_JOINT`, `kin_builder` evaluates the formula above symbolically and registers the result as `S_JOINT_<type>`.

In world coordinates the subspace of joint `j` is

```
[L_j; z_j] = [R_j S_j,lin;  R_j S_j,ang]      (6 x dim)
```

`z_j` is the angular velocity of frame `j` per unit `dq_j` and `L_j` the velocity of its origin `p_j`. For a revolute joint `L_j = 0` and `z_j` is the axis in the world; for a prismatic one `z_j = 0` and `L_j` is the axis.

### Jacobian

Joint `j` moves frame `i` when `j = i` or `j` is an ancestor of `i`. Moving `dq_j` alone, the whole subtree of `j` moves rigidly with frame `j`: it turns with `w = z_j dq_j` about `p_j` and translates with `L_j dq_j`. The origin of frame `i` is a point of that subtree, so its velocity is `L_j dq_j + w x (p_i - p_j)`. Summing over the joints that move frame `i`:

```
[v_i; w_i] = J_i dq,      J_i(:, q_j) = [ L_j + z_j x (p_i - p_j) ;  z_j ]      (j = i or an ancestor of i)
```

and the other columns are zero. This is the geometric Jacobian: it needs only the frames and the subspaces, no derivative of the transforms.

### Time derivative of the Jacobian

The velocities of the frames are propagated from the root (`w`, `v` are zero for the world):

```
w_j = w_parent + z_j dq_j
v_j = v_parent + w_parent x (p_j - p_parent) + L_j dq_j
```

The second line is the velocity of the point `p_j` of the parent body, plus the sliding of the joint. Since `dR_j/dt = [w_j] R_j`, the subspace in the world changes as

```
d/dt [L_j; z_j] = [ w_j x L_j ;  w_j x z_j ] + [ R_j dS_j,lin/dt ;  R_j dS_j,ang/dt ]
```

`dS_j/dt = dS_j/dq dq` is computed with `jtimes`: it is structurally zero for a constant subspace (`R`, `P`), and exact for joints whose `S` depends on `q`. Differentiating the columns of `J_i` with `d/dt (p_i - p_j) = v_i - v_j`:

```
dJ_i(:, q_j) = [ dL_j + dz_j x (p_i - p_j) + z_j x (v_i - v_j) ;  dz_j ]
```

### Second time derivative

`J_<name>_dot` depends on `q` and `dq`, so

```
ddJ = d/dt dJ = ∂dJ/∂q dq + ∂dJ/∂dq ddq
```

computed with two `jtimes` of the expression above.

### Damped pseudo-inverse

`J_<name>_pinv = Jᵀ (J Jᵀ + μ I_6)⁻¹`. From `Jᵀ (J Jᵀ + μ I_6) = (Jᵀ J + μ I_n) Jᵀ`, with `n = ndof`,

```
Jᵀ (J Jᵀ + μ I_6)⁻¹ = (Jᵀ J + μ I_n)⁻¹ Jᵀ
```

so the smaller system is solved: `(J Jᵀ + μ I_6) X = J`, `pinv = Xᵀ`, or `(Jᵀ J + μ I_n) X = Jᵀ` when `n < 6`. Both matrices are symmetric and positive definite for `μ > 0`, so they are factorised as `A = L D Lᵀ` (`L` unit lower triangular, `D` diagonal, all pivots `≥ μ`) without pivoting:

```
D_j  = A_jj - Σ_{m<j} L_jm² D_m
L_ij = (A_ij - Σ_{m<j} L_im L_jm D_m) / D_j          i > j
```

followed by forward substitution with `L`, division by `D` and back substitution with `Lᵀ`. Unlike `SX::inv` (a QR factorisation) it needs no square roots and gives smaller code.

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
