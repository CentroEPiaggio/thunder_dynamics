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

from abc import ABC, abstractmethod


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
