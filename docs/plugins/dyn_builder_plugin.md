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
  dynamics_method: auto       # M, Cdq, G: "auto" (default), "rnea", "crba" or "lagrange"
  C_method: auto              # C: "auto" (default), "rnea" or "christoffel"
  compute_C_std: false        # also add C_std (default false)
  compute_J_cm: false         # also add J_cm_<i> (default false)
```

As with any plugin, these keys can also be written at the top level of the file, outside the `dyn_builder` block.

## Configuration

### `dynamics_method`: M, Cdq, G

| `dynamics_method` | How it works |
| --- | --- |
| `lagrange` | Euler-Lagrange with the centre-of-mass Jacobians: `M = Σ m_i Jc_iᵀ Jc_i + Jω_iᵀ R_i I_i R_iᵀ Jω_i`, `G = -Σ m_i Jc_iᵀ g`, `Cdq = dM/dt dq - ½ ∂(dqᵀ M dq)/∂q`. |
| `auto` (default) | `M` and `G` each from the cheaper of `rnea` and `crba`, `Cdq` as `rnea`. |
| `rnea` | Recursive Newton-Euler on the kinematic tree, in spatial (6D) form and local frames. `M` is the Jacobian of `rnea(q, 0, ddq, 0)` with respect to `ddq`. `Cdq = rnea(q, dq, 0, 0)` and `G = rnea(q, 0, 0, g)`. |
| `crba` | Composite rigid body algorithm: each joint sees the bodies it carries as one rigid body. `M` from their composite inertias, `G` from their composite mass and first moment. `Cdq` as `rnea` (velocity terms have no composite form). |

### `C_method`: the matrix C

| `C_method` | How it works |
| --- | --- |
| `auto` (default) | The cheaper of `rnea` and `christoffel`. |
| `rnea` | Modified RNEA (Niemeyer-Slotine). The RNEA is run with a reference velocity `dqr`, so that `tau = M ddqr + C(q, dq) dqr + G`, and `C = ∂tau/∂dqr`. |
| `christoffel` | Christoffel symbols of the registered `M`, so it depends on `dynamics_method`. |

Both give the same matrix, with `dM/dt - 2C` skew-symmetric. The two keys are independent: any combination gives the same `M`, `C`, `Cdq` and `G`. `thunder_robot_comparison_gtest` checks all twelve combinations against Pinocchio. They differ only in the size of the generated expressions, and so in build and evaluation time.

`auto` measures "cheaper" as the number of CasADi instructions of each candidate (built with `cse`, as `add_function` does) and logs its choice, e.g. `auto: M by crba (rnea 7141, crba 6622 instructions)`. On the robots tested it picks `crba` for `M` and `G`, `christoffel` for `C` up to 7 dof and `rnea` for `C` on franka (8 dof). It costs little build time: up to 0.6 s more on franka, and less time on smaller robots, whose cheaper expressions make the derivatives faster to build.

Franka with a prismatic finger (8 dof, symbolic parameters), in CasADi instructions:

| | `lagrange` | `rnea` | `crba` |
| --- | --- | --- | --- |
| `M` | 13k | 7.1k | 6.6k |
| `Cdq` | 66k | 2.9k | 2.9k |
| `G` | 2.5k | 1.2k | 0.9k |

| | `christoffel` on `lagrange` M | `christoffel` on `rnea` M | `C_method: rnea` |
| --- | --- | --- | --- |
| `C` | 109k | 31k | 17k |
| `C_dot` | 296k | 69k | 53k |

`crba` gives a smaller `M` on serial chains (R7: -35%, franka: -7%) but can give a larger one on trees (+67% on a test tree with non-unit axes). Its `G` is smaller on every robot tested (-20% to -45%).

On small robots (3 to 5 dof) `christoffel` on the `rnea` M is somewhat smaller than `C_method: rnea`, but both are a few hundred to a few thousand instructions.

`Cdq` equals `C dq` but is computed directly, so it is much cheaper when the matrix itself is not needed.

### Other options

- `compute_C_std` adds `C_std`, the Christoffel matrix built element by element from `M`. Its values equal `C` but it is far more expensive, so it is off by default and meant for comparisons.
- `compute_J_cm` adds the centre-of-mass Jacobians `J_cm_<i>` (6 x ndof, linear velocity of the centre of mass on top of angular velocity). They are the frame Jacobians of `kin_builder` moved to the centre of mass. The `lagrange` method builds these Jacobians for `M` and `G` whether or not this option is set. The option only decides whether they are added as functions.

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
- **Link friction.** It uses `Dl_order` and `par_Dl` from `dyn_loader`. Odd orders use `dq^k`, even orders use `|dq| dq^(k-1)`. The `friction` of a node's `dynamics` entry belongs to the joint of that node (the joint after its body), not to the joint that moves the body. This is temporary, until the link-joint structure is reorganised.

## Mathematics

### Notation

Spatial vectors are 6D with the linear part first, as in Pinocchio and in the Jacobians of `kin_builder` (Featherstone's book puts the angular part first; the formulas do not depend on the order):

- motion `m = [v; w]`: velocity of the frame origin and angular velocity;
- force `f = [f; n]`: force and moment about the frame origin.

Each vector is expressed in a body frame. With `[a]` the skew matrix of `a`, the cross products are

```
m1 x m2  = [ w1 x v2 + v1 x w2 ;  w1 x w2 ]                (motion on motion)
m x* f   = [ w x f ;  w x n + v x f ]                      (motion on force)
```

For node `i` with parent `p`, `R`, `r` are the orientation and origin of frame `i` in frame `p` (from `T_i`). Velocities go from the parent to the child, forces from the child to the parent:

```
X_i m   = [ Rᵀ (v - r x w) ;  Rᵀ w ]                       (motion, frame p -> frame i)
X_iᵀ f  = [ R f ;  R n + r x (R f) ]                       (force, frame i -> frame p)
```

**Bodies.** A node's body is attached to the frame of its parent (see [the tree](kin_loader_plugin.md#the-tree)). `B(k)` is the set of bodies attached to frame `k`.

**Inertial parameters.** `par_DYN` gives, per body, `[m, c, I_c]`: mass, centre of mass and inertia about the centre of mass, in the frame the body is attached to. The algorithms use the regressor form `par_REG = [m, h, I_O]` (`dyn2reg`), which enters the dynamics linearly:

```
h   = m c                                                   first moment
I_O = I_c + m [c]ᵀ [c] = I_c + m (cᵀc 1 - c cᵀ)            inertia about the frame origin (parallel axis)
```

The spatial inertia and its product with a motion vector are

```
I = [ m 1    -[h] ;          I m = [ m v + w x h ;
      [h]    I_O  ]                  I_O w + h x v ]
```

### RNEA (`dynamics_method: rnea`)

Recursive Newton-Euler on the tree, in the local frames. Forward pass, from the root, for each frame `k` with parent `p` and joint velocity `dq_k`:

```
v_k = X_k v_p + S_k dq_k
a_k = X_k a_p + S_k ddq_k + dS_k/dt dq_k + v_k x (S_k dq_k)
```

with `v_p = 0` and `a_p = [-g; 0]` for the world: gravity enters as an upward acceleration of the base, so the result includes `G`. `dS_k/dt` (`jtimes`) is zero for constant subspaces (`R`, `P`).

Backward pass, from the leaves: `f_k` is the force transmitted by joint `k`, from the bodies on frame `k` and the subtrees of its child frames `c`,

```
f_k   = Σ_{i in B(k)} ( I_i a_k + v_k x* I_i v_k )  +  Σ_c X_cᵀ f_c
tau_k = S_kᵀ f_k
```

Then `tau = M ddq + C dq + G`, and

```
M   = ∂ rnea(q, 0, ddq, 0) / ∂ddq        Cdq = rnea(q, dq, 0, 0)        G = rnea(q, 0, 0, g)
```

### Modified RNEA for C (`C_method: rnea`)

The RNEA gives `C dq`, not the matrix `C`, and the matrix is not unique: any `C'` with `C' dq = C dq` gives the same torques. Controllers (passivity, Slotine-Li, observers) need the `C` with `dM/dt - 2C` skew-symmetric. Following Niemeyer and Slotine, the RNEA is run with the real velocity `dq` and a reference velocity `dqr`:

```
tau(q, dq, dqr, ddqr) = M ddqr + C(q, dq) dqr + G
```

`dqr` enters linearly, so `C = ∂tau/∂dqr` (with `ddqr = 0`, `g = 0`), one RNEA pass and one `jacobian`. Forward pass:

```
v_k  = X_k v_p  + S_k dq_k                                           real velocity
vr_k = X_k vr_p + S_k dqr_k                                          reference velocity
ar_k = X_k ar_p + S_k ddqr_k + dS_k/dt dqr_k + v_k x (S_k dqr_k)     reference acceleration
```

Backward pass, with the body term split between `v` and `vr`:

```
f_k = Σ_{i in B(k)} ( I_i ar_k + B_i(v_k) vr_k )  +  Σ_c X_cᵀ f_c,        tau_k = S_kᵀ f_k

B_i(v) = 1/2 [ (v x*) I_i  +  (I_i v) x̄  -  I_i (v x) ],                 (f x̄) u := u x* f
```

Why this split:

- `B(v) v = v x* I v`, the usual gyroscopic and centrifugal wrench, so with `dqr = dq` this is the RNEA.
- For a rigid body, `dI/dt = (v x*) I - I (v x)`, so `dI/dt - 2 B = -(I v) x̄`, which is skew-symmetric because `uᵀ (u x* f) = 0`.
- With `M = Σ J_iᵀ I_i J_i` and `C = Σ J_iᵀ (I_i dJ_i + B_i J_i)` (`J_i` the Jacobian of body `i`), `dM/dt - 2C = Σ (dJ_iᵀ I_i J_i - J_iᵀ I_i dJ_i) + Σ J_iᵀ (dI_i/dt - 2 B_i) J_i`, a sum of skew-symmetric terms.

This `C` is the one of the Christoffel symbols of the first kind (Echeandia and Wensing): it equals `christoffel` and Pinocchio's `computeCoriolisMatrix` to machine precision. For one body about its centre of mass, with angular velocities `w` and `wr`, the moment part of `B(v) vr` is `1/2 [ w x (I_c wr) + wr x (I_c w) - I_c (w x wr) ]`, which is `w x I_c w` at `wr = w`; the naive `w x (I_c wr)` gives the same torques but not a skew-symmetric `dM/dt - 2C`.

### CRBA (`dynamics_method: crba`)

Each joint carries the bodies after it as one rigid body. The composite inertia of frame `k`, in frame `k`, is

```
Ic_k = Σ_{i in B(k)} I_i  +  Σ_c X_cᵀ Ic_c X_c
```

and the mass matrix follows from the force that an acceleration of joint `k` needs, moved up to each ancestor joint `j`:

```
M_kk = S_kᵀ Ic_k S_k
F = Ic_k S_k
for each step c -> j = parent(c) on the path from k to the root:   F <- X_cᵀ F,   M_jk = M_kjᵀ = S_jᵀ F
```

Gravity needs only the composite mass and first moment (4 numbers per frame instead of the 6x6 `Ic`):

```
mc_k = Σ_{i in B(k)} m_i + Σ_c mc_c
hc_k = Σ_{i in B(k)} h_i + Σ_c ( R_c hc_c + mc_c r_c )
G_k  = S_kᵀ [ mc_k a_k ;  hc_k x a_k ],          a_k = -g expressed in frame k
```

`Cdq` comes from the RNEA (velocity terms have no composite form).

### Lagrange (`dynamics_method: lagrange`)

From the centre-of-mass Jacobians `J_cm_i = [Jv_i; Jw_i]` of the bodies. Body `i` moves with frame `p = parent(i)`; its centre of mass is at `r_i = R_p c_i` from the origin of `p`, so from the frame Jacobian `J_p = [Jv_p; Jw_p]` of `kin_builder`:

```
Jw_i = Jw_p,      Jv_i = Jv_p - [r_i] Jw_p                      (v_cm = v_o + w x r)

M   = Σ_i  m_i Jv_iᵀ Jv_i + Jw_iᵀ R_p I_c,i R_pᵀ Jw_i
G   = - Σ_i m_i Jv_iᵀ g
Cdq = dM/dt dq - 1/2 ∂(dqᵀ M dq)/∂q
```

### Christoffel C (`C_method: christoffel`)

From the registered `M`, with the Christoffel symbols of the first kind:

```
C_hj = 1/2 Σ_k ( ∂M_hj/∂q_k + ∂M_hk/∂q_j - ∂M_jk/∂q_h ) dq_k
```

computed in matrix form from `jacobian(M, q)` (`C_std` builds the same matrix element by element).

### Choice of `auto`

`auto` builds both candidates and keeps the one with fewer CasADi instructions (`Function(..., {"cse": true}).n_instructions()`), the same count as the generated code: `M` and `G` from `rnea` or `crba`, `C` from `rnea` or `christoffel`.

### Time derivatives

With `jtimes` (forward-mode directional derivatives) of the expressions above:

```
dM  = ∂M/∂q dq                              ddM = ∂dM/∂q dq + ∂dM/∂dq ddq
dC  = ∂C/∂q dq + ∂C/∂dq ddq                 ddC = ∂dC/∂q dq + ∂dC/∂dq ddq + ∂dC/∂ddq d3q
dG  = ∂G/∂q dq                              ddG = ∂dG/∂q dq + ∂dG/∂dq ddq
```

### Link friction

For each joint variable `i`, with the `Dl_order` coefficients `Dl_i,k` of `par_Dl`:

```
dl_i = Σ_{k=1..Dl_order} Dl_i,k f_k(dq_i),       f_k(v) = v^k (k odd),   f_k(v) = |v| v^(k-1) (k even)
```

`Dl<k>` is the diagonal matrix of the coefficients of order `k`.

### References

- G. Niemeyer, J.-J. E. Slotine, "Performance in Adaptive Manipulator Control", IJRR 10(2), 1991.
- S. Echeandia, P. M. Wensing, "Numerical Methods to Compute the Coriolis Matrix and Christoffel Symbols for Rigid-Body Systems", J. Comput. Nonlinear Dynam. 16(9), 2021.
- R. Featherstone, "Rigid Body Dynamics Algorithms", Springer 2008.
