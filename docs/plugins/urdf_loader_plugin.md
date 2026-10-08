# URDF Loader Plugin

`urdf_loader` loads the robot from a URDF file: the structure, the frames and the inertial parameters. It replaces `kin_loader` and `dyn_loader`.

```yaml
pipeline:
  loaders: [urdf_loader]
  builders: [kin_builder, dyn_builder]
  generators: [robot_generator]

urdf_loader:
  urdf_path: franka.urdf        # relative to this YAML file
  base_link: panda_link0
  ee_link: panda_hand
  gravity:
    value: [0, 0, -9.81]
  symbolic_kinematics: false    # all frames numeric (default)
  symbolic_dynamics:
    default: 0
    panda_hand: 1               # the hand inertia stays symbolic, e.g. to identify a tool
```

## From URDF links to thunder nodes

Each URDF link of the selected chain becomes a node of the [thunder tree](kin_loader_plugin.md#the-tree), with the same name. Node `<link>` holds the body of `<link>` and the URDF joint that leaves it towards the rest of the chain:

- the frame `X_i` of the node is the `<origin>` of that joint, its type and axis are those of the joint;
- frame `<link>` (`T_w_<link>`) is therefore the frame after that joint, i.e. the frame of the next link;
- a link without children in the chain (or `ee_link`) becomes a `FIXED` node with an identity frame. These terminal nodes are marked `available` and `derivatives`, so they get `T_w_<link>`, `J_<link>`, `J_<link>_dot`, `J_<link>_ddot` and `J_<link>_pinv`.

When a link has more than one child joint in the chain (a tree), the first branch continues from the node `<link>` and every other branch starts from a node `<link>_<k>`, a frame with no mass attached to the same parent.

URDF joint types: `revolute` and `continuous` become `R`, `prismatic` becomes `P`, `fixed` becomes `FIXED`. Other types (`floating`, `planar`) are not supported yet: the loader stops with an error. Joint limits are not read.

The inertial parameters of `<link>` (`<inertial>`: mass, `<origin>` of the centre of mass, inertia) are expressed in the frame of `<link>`, which is frame `parent(i)`, as `dyn_builder` expects. The inertia is taken about the centre of mass, rotated from the `<inertial><origin>` frame to the link frame.

## Configuration

| Key | Default | Meaning |
| --- | --- | --- |
| `urdf_path` | required | URDF file. A relative path is resolved from the directory of the YAML file, or from the working directory when the configuration is not read from a file |
| `base_link` | URDF root | First link of the chain |
| `ee_link` | none | Last link of the chain. The chain is the path from `base_link` to `ee_link`. Without it, every link of the URDF is loaded |
| `gravity.value`, `gravity.symb` | `[0, 0, 0]`, `[0, 0, 0]` | Gravity in the world frame and its mask |
| `Base_to_L0` | identity | Frame of the base in the world, applied to the root node |
| `Ln_to_EE` | identity | Extra frame applied after every terminal node, e.g. a tool |
| `kinematics.<link>` | from the URDF | Override of the frame of a node |
| `symbolic_kinematics` | `false` | Which frame entries stay symbolic |
| `symbolic_dynamics` | `false` | Which inertial parameters stay symbolic |
| `par_KIN_symb`, `par_DYN_symb` | none | Full masks of `par_KIN` (6 per node) and `par_DYN` (10 per node), overriding the two keys above |

`Base_to_L0`, `Ln_to_EE` and the entries of `kinematics` are frames written as in `kin_loader`: `xyzrpy`, or `xyz` (or `tr`) with `rpy` or `ypr`, plus an optional 6-element `symb` mask. Angles follow the [thunder convention](kin_loader_plugin.md#configuration) (`rpy` gives `R_x R_y R_z`); the frames read from the URDF are converted to it.

### Symbolic masks

Masks decide which parameters stay symbols in the generated code (see [symbolic and numeric parameters](kin_loader_plugin.md#symbolic-and-numeric-parameters)). Both default to numeric. They can be set at three levels; the more specific wins:

1. **Global**: `symbolic_kinematics: true` or `symbolic_dynamics: true`, or `default: 1` inside the map.
2. **Per link**, in the YAML:

   ```yaml
   symbolic_kinematics:
     default: 0
     panda_link3: 1               # all 6 entries
     panda_link4:
       xyz: [1, 1, 1]
       rpy: [0, 0, 0]
   symbolic_dynamics:
     panda_hand: [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
     panda_link7:
       mass: 1
       com: [1, 1, 1]
       inertia: [0, 0, 0, 0, 0, 0]
   ```

   A kinematic mask under `<link>` applies to the frame of node `<link>`, i.e. to the origin of the joint that leaves `<link>`.

3. **In the URDF**, used when the YAML does not set that link. Values can be `0`/`1` or `true`/`false`:

   ```xml
   <joint name="panda_joint4" type="revolute">
     ...
     <symbolic_kinematics xyz="1 1 1" rpy="0 0 0" />   <!-- origin of this joint -->
   </joint>
   <link name="panda_hand">
     ...
     <symbolic_dynamics mass="1" com="1 1 1" inertia="0 0 0 0 0 0" />
   </link>
   ```

   Both tags also accept child elements (`<xyz>1 1 1</xyz>`) or a flat list of 6 or 10 values.

## What it adds to the robot

The same properties, variables and `par_KIN` as [`kin_loader`](kin_loader_plugin.md#what-it-adds-to-the-robot), and the same `STD_PAR_LINK`, `par_gravity`, `par_DYN` and `par_REG` as [`dyn_loader`](dyn_loader_plugin.md#what-it-adds-to-the-robot). The frame parameters are named `KIN_<link>_xyz`, `KIN_<link>_rpy` (or as written in `kinematics`), `world2L0_*` and `Ln2EE_*`. It also adds the functions `par_world2L0` and `par_Ln2EE`, the two extra frames as `[x, y, z, r, p, y]`.
