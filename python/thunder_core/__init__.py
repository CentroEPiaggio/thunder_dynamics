"""thunder_core: Python interface for Thunder Dynamics."""

try:
    from thunder_core._bindings import (
        Property,
        Parameter,
        FunArg,
        Function,
        Robot,
        PluginManager,
        hat,
        vect,
        R_x,
        R_y,
        R_z,
        R_aa,
        get_transform_rpy,
        get_euler_rpy,
    )
except ModuleNotFoundError as exc:
    if exc.name != "thunder_core._bindings":
        raise

    from _bindings import (
        Property,
        Parameter,
        FunArg,
        Function,
        Robot,
        PluginManager,
        hat,
        vect,
        R_x,
        R_y,
        R_z,
        R_aa,
        get_transform_rpy,
        get_euler_rpy,
    )

from thunder_core import plugins, pipeline

__all__ = [
    # C++ bindings
    "Property",
    "Parameter",
    "FunArg",
    "Function",
    "Robot",
    "PluginManager",
    "hat",
    "vect",
    "R_x",
    "R_y",
    "R_z",
    "R_aa",
    "get_transform_rpy",
    "get_euler_rpy",
    # Submodules
    "plugins",
    "pipeline",
]
