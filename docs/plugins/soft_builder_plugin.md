# Soft Builder Plugin

`soft_builder` builds the model of the series elastic joints loaded by [`soft_loader`](soft_loader_plugin.md): the elastic coupling between link and motor, the damping of the coupling and of the motors, and the motor inertia. It has no configuration keys and does nothing on a robot without elastic joints.

```yaml
pipeline:
  loaders: [kin_loader, dyn_loader, soft_loader]
  builders: [kin_builder, dyn_builder, soft_builder]
```

## Model

For elastic joint `i`, with link variable `q_i` (the joint variable of that joint) and motor variable `x_i`:

```
k_i  = Σ_{n=1..K_order}  K_i,n (x_i - q_i)^(2n-1)              orders 1, 3, 5, ...
d_i  = Σ_{n=1..D_order}  D_i,n  f_n(dx_i - dq_i)
dm_i = Σ_{n=1..Dm_order} Dm_i,n f_n(dx_i)

f_n(v) = v^n          for odd n
f_n(v) = |v| v^(n-1)  for even n
```

The damping terms follow the same convention as the link friction of `dyn_builder`: odd orders are odd powers, even orders use `|v|`, so every term has the sign of `v`. The coefficients are the entries of `par_K`, `par_D` and `par_Dm`, in the order written in `soft_loader`.

## Functions

| Function | Arguments | Description |
| --- | --- | --- |
| `k` | `q, x, par_K` | Elastic coupling, `numSoftJoints x 1` |
| `K1`, `K3`, ... | `par_K` | Diagonal matrix of the coefficients of each order |
| `d` | `dq, dx, par_D` | Damping of the coupling |
| `D1`, `D2`, ... | `par_D` | Diagonal matrix of the coefficients of each order |
| `dm` | `dx, par_Dm` | Motor damping |
| `Dm1`, `Dm2`, ... | `par_Dm` | Diagonal matrix of the coefficients of each order |
| `Mm` | `par_Mm` | Motor inertia matrix, diagonal |

`k`, `d`, `dm` and their matrices exist only when the corresponding order is greater than 0.
