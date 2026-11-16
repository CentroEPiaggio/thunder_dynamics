# Floating Loader Plugin

`floating_loader` declares that the root of the structure is a floating base. It must come after the structure loader (`urdf_loader`, `kin_loader`, `dh_loader`, ...). Conventions, outputs and restrictions: [Floating base](../floating_base.md).

```yaml
pipeline:
  loaders: [urdf_loader, floating_loader]
floating_loader:
  base: base_link
  twist: body
```

## Configuration

| Key | Default | Meaning |
| --- | --- | --- |
| `base` | the root node | Name of the floating body. Only the root of the structure can float for now; another name is an error |
| `twist` | `world` | Axes of `twist_base`, `acc_base` and the references: `world` (world-aligned, classical acceleration) or `body` (base axes, as Pinocchio). See [Floating base](../floating_base.md#variables-and-conventions) |

## What it adds to the robot

- Variables `pose_base` (7, default `[0 0 0 0 0 0 1]`), `twist_base`, `acc_base`, `jerk_base`, `twist_base_r`, `acc_base_r` (6 each, default zero).
- Properties `floating` (`true`), `floating_base` (the name) and `floating_twist` (`world` or `body`).

Errors: no structure loaded before it, the base declared twice, `base` not the root, `twist` not `world` or `body`, `Base_to_L0` not the identity.
