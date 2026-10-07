# Dynamics Builder Plugin

`dyn_builder` builds the joint-space dynamics of the robot,

```
tau = M(q) ddq + C(q, dq) dq + G(q) + dl(dq)
```

and adds them to the robot as symbolic functions. It also adds their time derivatives and the conversions between dynamic and regressor parameters. It needs the kinematics: run it after `kin_builder`, with the dynamic parameters loaded by `dyn_loader` or `urdf_loader`.

```yaml
pipeline:
  loaders: [kin_loader, dyn_loader]
  builders: [kin_builder, dyn_builder]
  generators: [robot_generator]

dyn_builder:                  # plugin configuration, all keys optional
  dynamics_method: rnea       # M, Cdq, G: "rnea" (default), "crba" or "lagrange"
  C_method: rnea              # C: "rnea" (default) or "christoffel"
  compute_C_std: false        # also add C_std (default false)
  compute_J_cm: false         # also add J_cm_<i> (default false)
```

As with any plugin, these keys can also be written at the top level of the file, outside the `dyn_builder` block.

## Configuration

### `dynamics_method`: M, Cdq, G

| `dynamics_method` | How it works |
| --- | --- |
| `lagrange` | Euler-Lagrange with the centre-of-mass Jacobians: `M = Σ m_i Jc_iᵀ Jc_i + Jω_iᵀ R_i I_i R_iᵀ Jω_i`, `G = -Σ m_i Jc_iᵀ g`, `Cdq = dM/dt dq - ½ ∂(dqᵀ M dq)/∂q`. |
| `rnea` (default) | Recursive Newton-Euler on the kinematic tree, in spatial (6D) form and local frames. `M` is the Jacobian of `rnea(q, 0, ddq, 0)` with respect to `ddq`. `Cdq = rnea(q, dq, 0, 0)` and `G = rnea(q, 0, 0, g)`. |
| `crba` | `M` from the composite rigid body algorithm, `Cdq` and `G` as `rnea`. |

### `C_method`: the matrix C

| `C_method` | How it works |
| --- | --- |
| `rnea` (default) | Modified RNEA (Niemeyer-Slotine). The RNEA is run with a reference velocity `dqr`, so that `tau = M ddqr + C(q, dq) dqr + G`, and `C = ∂tau/∂dqr`. |
| `christoffel` | Christoffel symbols of the registered `M`, so it depends on `dynamics_method`. |

Both give the same matrix, with `dM/dt - 2C` skew-symmetric. The two keys are independent: any combination gives the same `M`, `C`, `Cdq` and `G`. `thunder_robot_comparison_gtest` checks all six combinations against Pinocchio. They differ only in the size of the generated expressions, and so in build and evaluation time.

Franka with a prismatic finger (8 dof, symbolic parameters), in CasADi instructions:

| | `lagrange` | `rnea` | `crba` |
| --- | --- | --- | --- |
| `M` | 13k | 7.1k | 6.6k |
| `Cdq` | 66k | 2.9k | 2.9k |
| `G` | 2.5k | 1.2k | 1.2k |

| | `christoffel` on `lagrange` M | `christoffel` on `rnea` M | `C_method: rnea` |
| --- | --- | --- | --- |
| `C` | 109k | 31k | 17k |
| `C_dot` | 296k | 69k | 53k |

`crba` gives a smaller `M` on serial chains (R7: -35%, franka: -7%) but can give a larger one on trees (+67% on a test tree with non-unit axes).

On small robots (3 to 5 dof) `christoffel` on the `rnea` M is somewhat smaller than `C_method: rnea`, but both are a few hundred to a few thousand instructions.

`Cdq` equals `C dq` but is computed directly, so it is much cheaper when the matrix itself is not needed.

### Other options

- `compute_C_std` adds `C_std`, the Christoffel matrix built element by element from `M`. Its values equal `C` but it is far more expensive, so it is off by default and meant for comparisons.
- `compute_J_cm` adds the centre-of-mass Jacobians `J_cm_<i>` (6 x ndof, linear velocity of the centre of mass on top of angular velocity). The `lagrange` method builds these Jacobians for `M` and `G` whether or not this option is set. The option only decides whether they are added as functions.

### Requirements of RNEA and CRBA

- Every joint must come after its parent in the joint list.
- Each joint type must define `T_JOINT_<type>`, which `kin_builder` needs anyway, and preferably `S_JOINT_<type>`. See [Joint types](../joints.md) for how joints are defined and how to add new ones. All built-in types (`R`, `P`, `FIXED`, `R_SEA`, `P_SEA`) define both.

## Functions

Functions added to the robot, with their arguments:

| Function | Arguments | Description |
| --- | --- | --- |
| `M` | `q, par_KIN, par_DYN` | Mass matrix |
| `C` | `q, dq, par_KIN, par_DYN` | Coriolis matrix, `dM/dt - 2C` skew-symmetric |
| `Cdq` | `q, dq, par_KIN, par_DYN` | Coriolis and centrifugal vector `C dq` |
| `G` | `q, par_KIN, par_gravity, par_DYN` | Gravity vector |
| `C_std` | `q, dq, par_KIN, par_DYN` | Element-wise Christoffel `C`, only with `compute_C_std: true` |
| `J_cm_<i>` | `q, par_KIN, par_DYN` | Jacobian of the centre of mass of link `i`, only with `compute_J_cm: true` |
| `dl` | `dq, par_Dl` | Link friction vector, only if `Dl_order > 0` |
| `Dl1` ... `Dl<order>` | `par_Dl` | Link friction matrices of each order, only if `Dl_order > 0` |
| `M_dot`, `M_ddot` | `q, dq, (ddq,) par_KIN, par_DYN` | First and second time derivatives of `M` |
| `C_dot`, `C_ddot` | `q, dq, ddq, (d3q,) par_KIN, par_DYN` | First and second time derivatives of `C` |
| `G_dot`, `G_ddot` | `q, dq, (ddq,) par_KIN, par_gravity, par_DYN` | First and second time derivatives of `G` |
| `dyn2reg`, `reg2dyn` | `par_DYN` / `par_REG` | Conversions between dynamic and regressor parameters. `par_REG` is set from `dyn2reg`. |

With `robot_generator`, each function becomes a `get_<name>()` method of the generated class, e.g. `get_M()` and `get_Cdq()`.

## Conventions

- **Dynamic parameters.** `par_DYN` holds 10 parameters per link: `[m, CoM_x, CoM_y, CoM_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]`. The centre of mass and the inertia (about the centre of mass) are expressed in the link frame.
- **Tree convention.** Joint `i` moves frame `i` (`T_w_<i>`) with respect to frame `parent(i)`. Link `i` is the body rigidly attached to frame `parent(i)`. Links attached to the world (`parent = -1`) do not move and do not load any joint. This is the same convention the URDF loader uses: node `i` is a URDF link together with its outgoing joint.
- **Gravity.** `par_gravity` is expressed in the world frame, e.g. `[0, 0, -9.81]`. `G` is the torque that balances gravity.
- **Link friction.** It uses `Dl_order` and `par_Dl` from `dyn_loader`. Odd orders use `dq^k`, even orders use `|dq| dq^(k-1)`.
