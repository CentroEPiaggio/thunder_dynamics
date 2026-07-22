"""
Thunder PyTorch Code Generator Plugin

Modern reimplementation of the legacy thunder_torch_gen.py as a Thunder plugin.
Generates batched PyTorch code from CasADi symbolic functions.
"""

from .torch_generator import TorchGenerator

__all__ = ["TorchGenerator"]
