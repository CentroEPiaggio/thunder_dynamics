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

    # _bindings built outside the package (e.g. ctest, PYTHONPATH=build:python): registered under
    # its package name too, which the C++ Python plugin proxies import
    import sys as _sys
    import _bindings as _bindings_module

    _sys.modules["thunder_core._bindings"] = _bindings_module
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
