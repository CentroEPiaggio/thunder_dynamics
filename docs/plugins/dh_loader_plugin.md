# DH Loader Plugin

`dh_loader` loads a serial chain from a table of Denavit-Hartenberg parameters. It replaces `kin_loader`; use it with `dyn_loader` for the inertial parameters.

```yaml
pipeline:
  loaders: [dh_loader, dyn_loader]
  builders: [kin_builder, dyn_builder]
  generators: [robot_generator]

dh_loader:
  joints_type: [R, R, R]
  DH:                           # one row [a, alpha, d, theta] per joint
    value: [0,   0,      0.3, 0,
            0,   1.5708, 0,   0,
            0.4, 0,      0,   0]
    symb: [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0]
  Base_to_L0:                   # optional, frame of the base in the world
    xyz: [0, 0, 0]
    rpy: [0, 0, 0]
  Ln_to_EE:                     # optional, end-effector in the last DH frame
    name: EE
    xyzrpy: [0, 0, 0.1, 0, 0, 0]
```

## Convention

The table uses the **modified** (Craig) convention. Row `k`, `[a, alpha, d, theta]`, gives the frame of joint `k` in the frame of joint `k-1`:

```
X_k = Rot_x(alpha) Trans_x(a) Rot_z(theta) Trans_z(d)
```

The joint then moves along or about the `z` axis of that frame: a revolute joint adds `q_k` to `theta`, a prismatic joint adds it to `d`.

The chain becomes a [thunder tree](kin_loader_plugin.md#the-tree) with `n + 3` nodes:

| Node | Name | Joint | Frame | Body (`dynamics` entry) |
| --- | --- | --- | --- | --- |
| 0 | `Base_to_L0.name`, default `base` | `FIXED`, parent world | `Base_to_L0` | none (on the world) |
| `k + 1` | `link<k>` (prefix from `DH.link_names`) | `joints_type[k]` | DH row `k` | link `k` |
| `n + 1` | `link<n>` | `FIXED` | identity: the last DH frame | link `n`, moved by the last joint |
| `n + 2` | `Ln_to_EE.name`, default `ee` | `FIXED` | `Ln_to_EE` | what is added after the arm (e.g. a tool) |

So the `dynamics` entries follow the Craig numbering: `link<k>` is the link moved by joint `k`, in the frame of joint `k`, and `link0` is the base, which does not move. The body of each node is attached to the frame of its parent (see [the tree](kin_loader_plugin.md#the-tree)); the node `link<n>` exists so that the last link has its own entry and the end-effector stays free for what is added after it. A `friction` entry belongs to the joint of its node: `link<k>` holds the friction of joint `k + 1`, not of joint `k` that moves link `k`. This is temporary, until the link-joint structure is reorganised.

The last node is marked `available` and `derivatives`, so it gets `T_w_<name>`, `J_<name>`, `J_<name>_dot`, `J_<name>_ddot` and `J_<name>_pinv`.

## Configuration

| Key | Default | Meaning |
| --- | --- | --- |
| `joints_type` | required | Type of each DH joint: `R`, `P`, `R_SEA`, `P_SEA` or `FIXED` (a `FIXED` row keeps its frame and has no joint variable) |
| `num_joints` | | Optional check: must equal the length of `joints_type` |
| `DH.value` | required | The table, 4 values per joint, `[a, alpha, d, theta]` |
| `DH.symb` | all `0` | Mask of `DH.value` |
| `DH.link_names` | `link` | Prefix of the node names |
| `Base_to_L0` | identity | Frame of the base in the world, with optional `name` |
| `Ln_to_EE` | identity | Frame of the end-effector in the last DH frame (the frame of `link<n>`), with optional `name` |

`Base_to_L0` and `Ln_to_EE` are written as in `kin_loader`: `xyzrpy`, or `xyz` with `rpy` or `ypr`, plus an optional 6-element `symb` mask (see [angles](kin_loader_plugin.md#configuration) and [masks](kin_loader_plugin.md#symbolic-and-numeric-parameters)).

## What it adds to the robot

The same properties, variables and `par_KIN` as [`kin_loader`](kin_loader_plugin.md#what-it-adds-to-the-robot), with all joint axes `[0, 0, 1]`. The parameters are `par_DHtable` (the table), `Wxyzrpy` or `Wxyz` with `Wrpy` / `Wypr` (base) and `Lnxyzrpy` or `Lnxyz` with `Lnrpy` / `Lnypr` (end-effector). `par_world2L0` and `par_Ln2EE` hold the base and end-effector frames as `[x, y, z, r, p, y]`.
