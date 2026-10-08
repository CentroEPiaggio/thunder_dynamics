# Dynamics Loader Plugin

`dyn_loader` loads the dynamic parameters from the YAML file: the inertial parameters of each body, gravity and the link friction. Run it after the loader of the structure (`kin_loader` or `dh_loader`), because it reads the node names and the number of joints. `urdf_loader` loads these parameters itself, so it does not need `dyn_loader`.

```yaml
pipeline:
  loaders: [kin_loader, dyn_loader]
  builders: [kin_builder, dyn_builder]

dyn_loader:
  gravity:
    value: [0, 0, -9.81]
    symb: [0, 0, 0]
  Dl_order: 2                   # link friction order, 0 (default) for none
  dynamics:
    link0:                      # node name, as in kin_loader
      inertial:
        mass: 0.63
        CoM: [-0.041, 0, 0.050]
        I: [0.00315, 0, 0.00015, 0.00388, 0, 0.004285]   # Ixx Ixy Ixz Iyy Iyz Izz
        symb: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
      friction:
        Dl: [0.1, 0.01]
        symb: [1, 1]
```

## Configuration

| Key | Default | Meaning |
| --- | --- | --- |
| `gravity.value` | `[0, 0, 0]` | Gravity in the world frame |
| `gravity.symb` | `[0, 0, 0]` | Mask of `gravity.value` |
| `Dl_order` | `0` | Order of the link friction model, see [`dyn_builder`](dyn_builder_plugin.md) |
| `dynamics.<node>` | zeros | Parameters of the body of node `<node>` |

For each node, `inertial` holds:

| Key | Meaning |
| --- | --- |
| `mass` | Mass |
| `CoM`, or `CoM_x`, `CoM_y`, `CoM_z` | Centre of mass |
| `I`, or `Ixx`, `Ixy`, `Ixz`, `Iyy`, `Iyz`, `Izz` | Inertia about the centre of mass. `I` is `[Ixx, Ixy, Ixz, Iyy, Iyz, Izz]` |
| `symb` | Mask of the 10 parameters `[m, CoM_x, CoM_y, CoM_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]`, default all `0` |

The body of node `i` is attached to frame `parent(i)`, and its centre of mass and inertia are expressed in that frame (see [the tree](kin_loader_plugin.md#the-tree)). Nodes not listed under `dynamics` get zero parameters.

`friction`, read only when `Dl_order > 0`, holds `Dl`, the `Dl_order` coefficients per joint variable of the node, and its mask `symb`. It belongs to the joint of the node (the joint after the body), not to the joint that moves the body. This is temporary, until the link-joint structure is reorganised.

Masks work as in every loader: `1` keeps the entry symbolic, `0` makes it a constant (see [symbolic and numeric parameters](kin_loader_plugin.md#symbolic-and-numeric-parameters)).

## What it adds to the robot

| Kind | Name | Description |
| --- | --- | --- |
| Properties | `STD_PAR_LINK` | Parameters per body, 10 |
| | `Dl_order` | Order of the link friction |
| Parameters | `par_gravity` | Gravity in the world frame, 3 x 1 |
| | `par_DYN` | Inertial parameters, 10 per node, `[m, CoM, I]` as above |
| | `par_REG` | The same parameters in regressor form (`[m, m CoM, I about the frame origin]`), all symbolic. `dyn_builder` sets its value from `par_DYN` |
| | `par_Dl` | Link friction coefficients, `Dl_order` per joint variable, only if `Dl_order > 0` |
