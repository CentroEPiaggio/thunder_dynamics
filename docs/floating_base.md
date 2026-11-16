# Floating base

A robot whose root moves freely in the world (legged, wheeled-legged, humanoid, drone): declare it with [`floating_loader`](plugins/floating_loader_plugin.md) after the structure loader. The builders then include the base motion in every kinematic and dynamic quantity.

```yaml
pipeline:
  loaders: [urdf_loader, floating_loader]
  builders: [kin_builder, dyn_builder, reg_builder]
  generators: [robot_generator]
floating_loader:
  base: base_link        # optional, the root of the structure (the only one that can float for now)
  twist: world           # optional, axes of the base twist and its derivatives: world (default) or body
```

The URDF needs no change: no `world` link and no floating joint, the same file used by Gazebo or Pinocchio (`JointModelFreeFlyer`). A URDF that has them (`world` + `<joint type="floating">`) is refused with an error: set `base_link` to the child of the floating joint, so the chain starts at the base, or remove both (see the [URDF loader](plugins/urdf_loader_plugin.md#from-urdf-links-to-thunder-nodes)).

## Not a joint

The base is not a joint (base and world are disconnected): it is a state of the structure. Frame -1, the still world where the recursions start, becomes the base frame, with a pose and a twist as variables. The joints stay as they are, so only the root of the recursions changes. The model is the same as Pinocchio's free-flyer (6 velocities, 7 configuration numbers).

## Variables and conventions

| Variable | Size | Meaning |
| --- | --- | --- |
| `pose_base` | 7 | `[p; quat]`: position of the base origin and orientation, world frame. Quaternion **x y z w** (Hamilton, as ROS, Pinocchio, Eigen `coeffs()`), **unit**: the caller normalises |
| `twist_base` | 6 | `[v; w]`: velocity of the base origin and angular velocity, in the axes chosen by `twist` (below) |
| `acc_base` | 6 | time derivative of the `twist_base` components |
| `jerk_base` | 6 | time derivative of `acc_base` (second time derivatives) |
| `twist_base_r`, `acc_base_r` | 6 | reference twist and acceleration (Slotine-Li `tau_r`, `Yr`), as `dqr`, `ddqr`, same axes |

`twist` selects the axes of the base velocity and of its derivatives (one switch, read only by the floating-base code):

| | `twist: world` (default) | `twist: body` |
| --- | --- | --- |
| `twist_base` | `[dp; w]` in world axes (iDynTree MIXED, Pinocchio `LOCAL_WORLD_ALIGNED`, Drake) | `[R^T dp; R^T w]` in base axes (Pinocchio free-flyer, Featherstone) |
| `acc_base` | classical `[ddp; dw]` | derivative of the body components (not the classical acceleration: `R^T ddp = dv_b + w_b x v_b`) |
| base rows of `tau`, `G`, `Cdq` | base wrench at the base origin, world axes | base wrench at the base origin, base axes (e.g. thrust and torques of a drone) |
| `M` | depends on the base orientation | depends on the joints only |
| matches directly | Gazebo, PhysX, estimators (world velocity), classical-acceleration tasks | IMU gyro, `nav_msgs/Odometry` twist, Pinocchio |

The frame Jacobians `J_<frame>` keep the thunder convention in both cases: they give `[v; w]` of the frame origin in **world axes** (with `twist: body`, the same as Pinocchio's `LOCAL_WORLD_ALIGNED` frame Jacobian).

Generalized vectors are stacked base first: `u = [twist_base; dq]`, `du = [acc_base; ddq]`, size `nv = 6 + ndof`. All outputs follow: `M`, `C` are `nv x nv`, `Cdq`, `G`, `tau` have `nv` rows (the first 6 are the base wrench at the base origin, the unactuated part), `J_<frame>` is `6 x nv`, the regressors have `nv` rows and include the 10 parameters of the base body. `q`, `dq`, `ddq` are the joints only, as without floating base. The generated functions take each base variable right before its joint counterpart, e.g. `M(pose_base, q, ...)`, `tau(pose_base, q, twist_base, dq, acc_base, ddq, ...)`.

## What changes in the builders

In the formulas the base is a 6-dof block at the root of the tree, with motion subspace `S_b` in base coordinates: the base velocity in base coordinates is `S_b twist_base`, with `S_b = I` for `body` and `blkdiag(R^T, R^T)` for `world`. The base rows of every output are `S_b^T` times the force on frame -1.

- `kin_builder`: `T_w_i = T(pose_base) T_0i`. In world coordinates the base subspace is `Sw_b = blkdiag(R, R) S_b` (`I` for `world`, `blkdiag(R, R)` for `body`), so it enters the same formulas as the joints: the base columns of every Jacobian are `[Sw_b_lin + (p_b - p_i)^ Sw_b_ang; Sw_b_ang]`, i.e. `[I, (p_b - p_i)^; 0, I]` (world) or `[R, (p_b - p_i)^ R; 0, R]` (body). `J_dot` uses `d/dt Sw_b` (zero for `world`, `blkdiag(w^ R, w^ R)` for `body`) as the joint columns do.
- `dyn_builder`, RNEA: the recursion starts from the base instead of the still world, in base coordinates: `v_root = S_b u_b`, `vr_root = S_b ur_b`, `ar_root = S_b dur_b + bias - [R^T g; 0]`, where `bias = d/dt(S_b) ur_b + v_root x vr_root` are the velocity terms of a joint (`dS/dt dqr + v x S dqr`):
  - `body`: `bias = v_root x vr_root`, zero in the plain RNEA (`vr = v`);
  - `world`: with `d/dt R^T = -w_b^ R^T`, `d/dt(S_b) ur_b = -[w_b x vr_b; w_b x wr_b]` cancels the angular terms of `v_root x vr_root = [w_b x vr_b + v_b x wr_b; w_b x wr_b]`, so `bias = [v_b x wr_b; 0]` (`_b`: in base coordinates).

  The bodies on frame -1 (the base body) now load the base: the base rows of `tau` are `S_b^T f_root`.
- `dyn_builder`, CRBA: `M_bb = S_b^T Ic_root S_b`, with `Ic_root` the composite inertia of the whole tree; `M_bj = S_b^T F_j`, with `F_j` the composite force of joint column `j` carried to frame -1; `G_b = S_b^T [-m R^T g; -h x R^T g]`, the total weight (`m`, `h`: mass and first moment of the whole robot, in base coordinates).
- `dyn_builder`, `J_cm_<i>`: the bodies on frame -1 move with the Jacobian of frame -1, `[Sw_b, 0]` at the base origin.
- Time derivatives (`*_dot`, `*_ddot`, `J_ddot`): jtimes along the motion with the configuration `z = [pose_base; q]` moving as `dz = [v; 1/2 [w; 0] (x) quat; dq]` (world) or `dz = [R v; 1/2 quat (x) [w; 0]; dq]` (body). Exact for any state.
- `reg_builder`: same RNEA, so `Yr par_REG = tau_r`. The base state is not a parameter of the kinematic regressors.

The two conventions are related by `T = blkdiag(R^T, R^T, I)` (world to body twist): `M_w = T^T M_b T`, `C_w = T^T (C_b T + M_b dT/dt)`, `G_w = T^T G_b`. `dM/dt - 2C` is skew in both.

## Refused options

The angular velocity is not the time derivative of any coordinates (it is a quasi-velocity), so the formulas that assume `dq = d/dt q` miss the `w x (I w)` terms of the base (for one free rigid body `dM/dq = 0` and Christoffel gives `C = 0`). With a floating base:

- `dyn_builder`: `dynamics_method: lagrange`, `C_method: christoffel`, `compute_C_std` raise an error; `C_method: auto` uses `rnea`;
- `reg_builder`: `regressor_method: lagrange` raises an error;
- `Base_to_L0` must be the identity (the base pose is the state).

`compute_J_cm` is valid: the centre-of-mass Jacobians are built from the frame Jacobians, not from derivatives with respect to coordinates. The link friction `dl` stays on the joints (`ndof` rows).

## Implementation

The helpers for the builders are in `namespace floating` of [`floating_loader.h`](../src/thunder/include/plugins/loaders/floating_loader.h), next to the plugin that declares the variables. Without `floating_loader` the base has no dofs and every helper returns the fixed-base quantity, so a builder has one code path:

| Helper | Floating base | Fixed base |
| --- | --- | --- |
| `velocity`, `acceleration`, `jerk`, `ref_velocity`, `ref_acceleration` | `[twist_base; dq]`, `[acc_base; ddq]`, ... | `dq`, `ddq`, ... |
| `nv` | `6 + ndof` | `ndof` |
| `split(u)` | `{u_b, u_j}` | `{empty, u}` |
| `stack_rows`, `stack_cols`, `stack_blocks` | `[base; joints]`, `[base, joints]`, `[bb, bj; bj^T, jj]` | the joint part |
| `add_base_args(list)` | each base variable before its joint counterpart: `{q, dq, par_KIN}` gives `{pose_base, q, twist_base, dq, par_KIN}` | `list` |
| `R_base`, `T_base` | from `pose_base` | identity |
| `S_world`, `dS_world`, `twist_world` | `Sw_b`, `d/dt Sw_b`, base twist in world axes | no columns, zero twist |
| `motion(m)`, `force(f)`, `bias(v, vr)` | `S_b m`, `S_b^T f`, velocity terms of the base acceleration | zero, no rows, zero |
| `time_derivative(e)` | along `[pose_base; q]`, `u`, `du` (above) | along `q`, `dq`, `ddq` |

The `world`/`body` switch is read only inside these helpers. `S_b` is applied in closed form (`motion`, `force`, `bias`), never as a matrix product, so the generated code has no `R R^T` and no products with identity blocks.

## Tests

`tests/python/test_floating_base.py` compares with Pinocchio's free-flyer, on `fixtures/floating/biped_tree.urdf` (base and two legs, two branches on the base) and `fixtures/franka/franka_finger.urdf`, at random states (base orientation included), with `twist: world` and `twist: body`:

- `M`, `G`, `Cdq`, `C` (element by element with `computeCoriolisMatrix`), `tau`, `tau_r`, `Y par_REG`, `Yr par_REG`;
- `T_w_i`, `J_i` of every node and `J_<name>_dot` of the named frames;
- `J_cm_<i>` of every link, the base body included, through the Jacobian of the centre of mass of the whole robot (`jacobianCenterOfMass`);
- `C u = Cdq`, `dM/dt - 2C` skew, `M_dot` and `G_dot` against finite differences along the exact motion;
- the refused options raise errors.

The Pinocchio velocity is the body twist: identical for `body`, converted with `T` above for `world`.
