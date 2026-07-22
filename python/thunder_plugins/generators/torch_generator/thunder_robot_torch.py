"""
PyTorch batched wrapper for Thunder robot dynamics.

Mirrors the C++ thunder_robot template with batched PyTorch tensors.
Provides state and parameter management for vectorized dynamics computation.

Author: Thunder Dynamics Team (adapted from legacy thunder_robot.py)
"""

from __future__ import annotations

from typing import Dict, Mapping, Optional, Sequence, Union

import torch

TensorLike = Union[torch.Tensor, Sequence[float], Sequence[int]]


def _as_tensor(
    x: TensorLike, device: torch.device, dtype: torch.dtype
) -> torch.Tensor:
    """Convert input to tensor on specified device/dtype."""
    if isinstance(x, torch.Tensor):
        return x.to(device=device, dtype=dtype)
    return torch.as_tensor(x, device=device, dtype=dtype)


def _expand_to_batch(
    x: TensorLike, batch_size: int, dim: int, device: torch.device, dtype: torch.dtype
) -> torch.Tensor:
    """
    Expand 1D vector to batch of (B, dim) or validate existing batch tensor.

    Accepts:
    - (dim,) -> expand to (B, dim)
    - (B, dim) -> returned as-is (moved to device/dtype)

    A clone is deliberately made after ``expand``: ``expand`` creates a view
    with shared storage, so without it changing one simulated robot would also
    change every other robot in the batch.
    """
    t = _as_tensor(x, device, dtype)
    if t.ndim == 1:
        if t.shape[0] != dim:
            raise ValueError(
                f"Expected vector of shape ({dim},), got {tuple(t.shape)}"
            )
        return t.unsqueeze(0).expand(batch_size, dim).clone()
    if t.ndim == 2:
        if t.shape != (batch_size, dim):
            raise ValueError(
                f"Expected tensor of shape ({batch_size}, {dim}), got {tuple(t.shape)}"
            )
        return t
    raise ValueError(f"Unsupported tensor rank {t.ndim}, expected 1 or 2")


def hat(v: torch.Tensor) -> torch.Tensor:
    """
    Skew-symmetric matrix (hat operator) for vectors v of shape (..., 3).

    Returns:
        Tensor of shape (..., 3, 3) containing skew-symmetric matrices.
    """
    if v.shape[-1] != 3:
        raise ValueError(f"hat expects last dimension=3, got {v.shape}")
    zeros = torch.zeros_like(v[..., 0])
    vx, vy, vz = v[..., 0], v[..., 1], v[..., 2]
    out = torch.stack([
        torch.stack([zeros, -vz, vy], dim=-1),
        torch.stack([vz, zeros, -vx], dim=-1),
        torch.stack([-vy, vx, zeros], dim=-1),
    ], dim=-2)
    return out


class ThunderRobotTorch:
    """
    Batched PyTorch wrapper for Thunder robot dynamics.

    Provides batching, device/dtype handling, and generic parameter storage.
    The generated wrapper defines the robot-specific parameter inventory and its
    ``get_<parameter>()`` / ``set_<parameter>()`` accessors.

    Attributes:
        robotName: Robot identifier
        batch_size: Number of parallel simulations
        device: PyTorch device (cpu, cuda, etc.)
        dtype: PyTorch data type (float32, float64, etc.)
        n_joints: Optional compatibility metadata for the robot representation
        
    Constructor parameters:
        n_joints: Number of joints
        batch_size: Batch dimension size (default 1)
        device: torch.device or string (default "cpu")
        dtype: torch.dtype (default torch.double)
        parameter_sizes: Map {parameter_name: flattened size}
        parameter_defaults: Optional map {parameter_name: initial values}
        robotName: Optional robot identifier string
    """

    def __init__(
        self,
        n_joints: Optional[int] = None,
        batch_size: int = 1,
        device: Union[str, torch.device] = "cpu",
        dtype: torch.dtype = torch.double,
        parameter_sizes: Optional[Dict[str, int]] = None,
        parameter_defaults: Optional[Mapping[str, TensorLike]] = None,
        robotName: str = "",
    ) -> None:
        self.robotName = robotName
        self.batch_size = int(batch_size)
        self.device = torch.device(device)
        self.dtype = dtype

        # This inventory is created by torch generator from Robot.parameters.
        # Keep it as the source of truth since the base class cannot know what exists
        self.parameter_sizes: Dict[str, int] = {
            k: int(v)
            for k, v in (parameter_sizes or {}).items()
            if int(v) > 0
        }
        self.parameter_defaults = dict(parameter_defaults or {})

        #! TODO: We could probably pass somehow this directly from the parameter ndof or njoints
        self.n_joints = int(
            self.parameter_sizes.get("q", n_joints if n_joints is not None else 0)
        )

        self._allocate_parameters()

    def _alloc_tensor(self, name: str, dim: int) -> None:
        """Create the dynamically named tensor that generated functions read."""
        if dim <= 0:
            return
        setattr(
            self,
            name,
            torch.zeros((self.batch_size, int(dim)), device=self.device, dtype=self.dtype),
        )

    def _allocate_parameters(self) -> None:
        """Allocate zeros first, then replace them with model defaults if present."""
        for name, size in self.parameter_sizes.items():
            self._alloc_tensor(name, size)

        for name, value in self.parameter_defaults.items():
            if name not in self.parameter_sizes:
                raise ValueError(f"Default provided for unknown parameter '{name}'")
            self.set_parameter(name, value)

    def set_parameter(self, name: str, value: TensorLike) -> None:
        """Set a parameter with the size fixed when this wrapper was generated."""
        try:
            dim = self.parameter_sizes[name]
        except KeyError as exc:
            raise KeyError(f"Unknown robot parameter '{name}'") from exc
        setattr(
            self,
            name,
            _expand_to_batch(value, self.batch_size, dim, self.device, self.dtype),
        )

    def get_parameter(self, name: str) -> torch.Tensor:
        """Return a copy so callers cannot mutate the robot state accidentally."""
        #! TODO: might be useful to have the option to return a view
        if name not in self.parameter_sizes:
            raise KeyError(f"Unknown robot parameter '{name}'")
        return getattr(self, name).clone()

    def _prepare_explicit_input(
        self, value: TensorLike, size: int, name: str
    ) -> torch.Tensor:
        """Batch an argument passed to one generated function call.

        Explicit inputs (for example ``q_joint`` in a joint-transform helper)
        are not persistent robot parameters, but must have the same batch,
        dtype, and device as persistent inputs before CasADi operations run.
        """
        try:
            return _expand_to_batch(value, self.batch_size, size, self.device, self.dtype)
        except ValueError as exc:
            raise ValueError(f"Invalid explicit input '{name}': {exc}") from exc

    def list_parameters(self) -> tuple[str, ...]:
        """Return the generated parameter names in deterministic order."""
        return tuple(self.parameter_sizes)

    def to(self, device: Optional[torch.device] = None, dtype: Optional[torch.dtype] = None):
        """Move all tensors to specified device/dtype."""
        if device is not None:
            self.device = torch.device(device)
        if dtype is not None:
            self.dtype = dtype

        # Use instance storage instead of a hard-coded list
        # !TODO: Can't we use list of params?
        for attr_name, attr in vars(self).items():
            if isinstance(attr, torch.Tensor):
                setattr(self, attr_name, attr.to(device=self.device, dtype=self.dtype))

        return self

    def __repr__(self) -> str:
        return (
            f"ThunderRobotTorch("
            f"name={self.robotName}, "
            f"joints={self.n_joints}, "
            f"batch_size={self.batch_size}, "
            f"device={self.device}, "
            f"dtype={self.dtype})"
        )
