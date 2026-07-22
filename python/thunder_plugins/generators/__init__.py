"""Thunder generator plugins."""

from . import torch_generator
from .torch_generator import TorchGenerator

__all__ = ["torch_generator", "TorchGenerator"]