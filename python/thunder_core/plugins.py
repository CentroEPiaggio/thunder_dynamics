"""
Base classes for Thunder Python plugins.

All Python plugins MUST inherit from the appropriate base class
(BaseLoader, BaseBuilder, or BaseGenerator). These are abstract —
you must implement the required methods.

Plugins can optionally define a ``ConfigModel`` class attribute
(a pydantic BaseModel subclass) to validate the YAML config dict
received in ``configure()``. When present, ``self.config`` will be
the validated pydantic model instance instead of a raw dict.

Example::

    from pydantic import BaseModel
    from thunder_core.plugins import BaseBuilder

    class MyBuilder(BaseBuilder):
        class ConfigModel(BaseModel):
            function_name: str = "cos_q"
            scale: float = 1.0

        def build(self, robot):
            # self.config is a validated ConfigModel instance
            q = robot.get_model("q")
            import casadi
            robot.add_function(
                self.config.function_name,
                self.config.scale * casadi.cos(q),
                ["q"],
            )
"""

import hashlib
import importlib
import importlib.util
import sys
from abc import ABC, abstractmethod
from pathlib import Path


class _PluginBase(ABC):
    """Common base for all Thunder plugin types.

    Handles optional pydantic config validation via the ``ConfigModel``
    class attribute.
    """

    ConfigModel = None  # Override with a pydantic BaseModel subclass

    def configure(self, config: dict) -> None:
        """Called with the plugin's YAML config section as a Python dict.

        If the plugin defines a ``ConfigModel`` (pydantic BaseModel),
        the dict is validated and ``self.config`` is set to the model
        instance. Otherwise ``self.config`` is the raw dict.
        """
        if self.ConfigModel is not None:
            self.config = self.ConfigModel(**config)
        else:
            self.config = config


class BaseLoader(_PluginBase):
    """Base class for Python loader plugins.

    Loaders initialize the Robot with properties and parameters
    from configuration data (URDF, DH tables, custom formats, etc.).
    """

    @abstractmethod
    def load(self, robot) -> None:
        """Populate the robot with initial data.

        Args:
            robot: thunder_core.Robot instance (shared with C++ pipeline)
        """
        ...


class BaseBuilder(_PluginBase):
    """Base class for Python builder plugins.

    Builders perform symbolic computation (e.g. kinematics, dynamics)
    and add functions to the Robot using CasADi expressions.
    """

    @abstractmethod
    def build(self, robot) -> None:
        """Build symbolic functions on the robot.

        Args:
            robot: thunder_core.Robot instance (shared with C++ pipeline)
        """
        ...


class BaseGenerator(_PluginBase):
    """Base class for Python generator plugins.

    Generators read the completed Robot model and produce output
    (code files, visualizations, exports, etc.).
    """

    @abstractmethod
    def generate(self, robot) -> None:
        """Generate output from the robot model.

        Args:
            robot: thunder_core.Robot instance (read-only recommended)
        """
        ...


_KINDS = {"loader": BaseLoader, "builder": BaseBuilder, "generator": BaseGenerator}


def load_plugin(name, base_dir=None, kind=None):
    """Return the plugin class named ``"module.Class"`` (or ``"py.module.Class"``).

    If ``<base_dir>/<module>.py`` exists (``<base_dir>/pkg/mod.py`` for ``pkg.mod``), the file is
    executed again at every call, so an edited plugin is picked up without restarting Python. It is
    registered in ``sys.modules`` under a private name derived from its path: the same file always
    reuses the same entry, and two files with the same name in different directories do not collide.
    Otherwise the module is imported by name with the normal Python rules (installed packages,
    ``PYTHONPATH``, ``sys.path``).

    Args:
        name: ``"module.Class"``, with an optional ``"py."`` prefix as written in the YAML.
        base_dir: directory to look for the module file in; default: current working directory.
        kind: required base class, as a class or as ``"loader"``, ``"builder"``, ``"generator"``;
            default: any plugin base class.

    Raises:
        TypeError: if the class does not inherit from the required base class.
    """
    if name.startswith("py."):
        name = name[3:]
    module_name, _, class_name = name.rpartition(".")
    if not module_name:
        raise ValueError(f"Invalid plugin name '{name}': expected 'module.Class'")
    base = _KINDS[kind] if isinstance(kind, str) else (kind or _PluginBase)

    *pkg, mod = module_name.split(".")
    path = Path(base_dir if base_dir is not None else Path.cwd()).joinpath(*pkg, mod + ".py").resolve()
    module = _exec_file(path, module_name) if path.is_file() else importlib.import_module(module_name)

    cls = getattr(module, class_name, None)
    if cls is None:
        raise AttributeError(f"Class '{class_name}' not found in '{module.__file__}'")
    if not (isinstance(cls, type) and issubclass(cls, base)):
        raise TypeError(f"Plugin '{name}' must inherit from thunder_core.plugins.{base.__name__}")
    return cls


def _exec_file(path, module_name):
    """Execute the file as a module named after its path, replacing an earlier load of the same file."""
    digest = hashlib.sha1(str(path).encode()).hexdigest()[:10]
    unique_name = f"_thunder_plugin_{digest}_{module_name.replace('.', '_')}"
    spec = importlib.util.spec_from_file_location(unique_name, path)
    module = importlib.util.module_from_spec(spec)

    # registered before executing, as a normal import does (needed e.g. by dataclasses);
    # on error the previous version is put back
    previous = sys.modules.get(unique_name)
    sys.modules[unique_name] = module
    # the directory is on sys.path only while the file runs, for imports of its sibling files
    folder = str(path.parent)
    added = folder not in sys.path
    if added:
        sys.path.append(folder)
    try:
        # compiled from the source, not from __pycache__: the bytecode cache is checked only by
        # mtime and size, and would miss a quick edit that keeps the size
        exec(compile(path.read_bytes(), str(path), "exec"), module.__dict__)
    except BaseException:
        if previous is not None:
            sys.modules[unique_name] = previous
        else:
            sys.modules.pop(unique_name, None)
        raise
    finally:
        if added:
            sys.path.remove(folder)
    return module
