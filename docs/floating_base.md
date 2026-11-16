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

Generalized vectors are stacked base first: `u = [twist_base; dq]`, `du = [acc_base; ddq]`, size `nv = 6 + ndof`. All outputs follow: `M`, `C` are `nv x nv`, `Cdq`, `G`, `tau` have `nv` rows (the first 6 are the base wrench at the base origin, the unactuated part), `J_<frame>` is `6 x nv`, the regressors have `nv` rows and include the 10 parameters of the base body. `q`, `dq`, `ddq` are the joints only, as without floating base.

## What changes in the builders

- `kin_builder`: `T_w_i = T(pose_base) T_0i`. The base is one more block at the root, with motion subspace `Sw_b` in world coordinates (`I` for `world`, `blkdiag(R, R)` for `body`), so it enters the same formulas as the joints: the base columns of every Jacobian are `[Sw_b_lin + (p_b - p_i)^ Sw_b_ang; Sw_b_ang]`, i.e. `[I, (p_b - p_i)^; 0, I]` (world) or `[R, (p_b - p_i)^ R; 0, R]` (body). `J_dot` uses `d/dt Sw_b` (zero for world, `blkdiag(w^ R, w^ R)` for body) as the joint columns do.
- `dyn_builder` (RNEA, CRBA): the recursions start from the base instead of the still world, in base coordinates (`X = blkdiag(R^T, R^T)`):
  - body: `v_root = twist`, `vr_root = twist_r`, `ar_root = acc_r + v_root x vr_root - [R^T g; 0]` (the base block has `S = I` in base coordinates, and `v x S dqr` is its self term, as for a joint);
  - world: `v_root = X twist`, `vr_root = X twist_r`, `ar_root = X acc_r + [v_b x w_r,b; 0] - [R^T g; 0]` (the rotation of the frame and the self term of the base block sum to the cross product);
  - the bodies on frame -1 (the base body) now load the base: the base rows are `f_root` (body) or `blkdiag(R, R) f_root` (world).
  CRBA: the base block of `M` is the composite inertia of the whole tree, `M_bj` the composite forces carried to the base, the base rows of `G` the total weight `[-m g; -m (c - p) x g]` (rotated to world axes for `world`).
- Time derivatives (`*_dot`, `*_ddot`, `J_ddot`): jtimes along the motion with the configuration `z = [pose_base; q]` moving as `dz = [v; 1/2 [w; 0] (x) quat; dq]` (world) or `dz = [R v; 1/2 quat (x) [w; 0]; dq]` (body). Exact for any state.
- `reg_builder`: same RNEA, so `Yr par_REG = tau_r`. The base state is not a parameter of the kinematic regressors.

The two conventions are related by `T = blkdiag(R^T, R^T, I)` (world to body twist): `M_w = T^T M_b T`, `C_w = T^T (C_b T + M_b dT/dt)`, `G_w = T^T G_b`. `dM/dt - 2C` is skew in both.

## Refused options

The angular velocity is not the time derivative of any coordinates (it is a quasi-velocity), so the formulas that assume `dq = d/dt q` miss the `w x (I w)` terms of the base (for one free rigid body `dM/dq = 0` and Christoffel gives `C = 0`). With a floating base:

- `dyn_builder`: `dynamics_method: lagrange`, `C_method: christoffel`, `compute_C_std`, `compute_J_cm` raise an error; `C_method: auto` uses `rnea`;
- `reg_builder`: `regressor_method: lagrange` raises an error;
- `Base_to_L0` must be the identity (the base pose is the state).

The link friction `dl` stays on the joints (`ndof` rows).

## Tests

`tests/python/test_floating_base.py` compares with Pinocchio's free-flyer, on `fixtures/floating/biped_tree.urdf` (base and two legs, two branches on the base) and `fixtures/franka/franka_finger.urdf`, at random states (base orientation included), with `twist: world` and `twist: body`:

- `M`, `G`, `Cdq`, `C` (element by element with `computeCoriolisMatrix`), `tau`, `tau_r`, `Y par_REG`, `Yr par_REG`;
- `T_w_i`, `J_i` of every node and `J_<name>_dot` of the named frames;
- `C u = Cdq`, `dM/dt - 2C` skew, `M_dot` and `G_dot` against finite differences along the exact motion;
- the refused options raise errors.

The Pinocchio velocity is the body twist: identical for `body`, converted with `T` above for `world`.
