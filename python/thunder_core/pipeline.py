"""
Thunder pipeline configuration and execution from Python.

Allows running the full Thunder plugin pipeline from Python code or notebooks,
using either a YAML file or a Python dict as configuration.

Example::

    from thunder_core.pipeline import Config

    # From YAML file
    robot = Config("path/to/robot.yaml").execute(robot_name="my_robot")

    # From dict
    robot = Config({
        "pipeline": {
            "loaders": ["kin_loader"],
            "builders": ["kin_builder"],
            "generators": []
        },
        "kin_loader": {
            "num_joints": 2,
            "joints_type": ["R", "R"],
        }
    }).execute(robot_name="RR")

    # The Config can be inspected before running it
    cfg = Config("path/to/robot.yaml")
    print(cfg)
    robot = cfg.execute(robot_name="RR")
"""

import os

import yaml
import thunder_core


class Config:
    """Thunder pipeline configuration.

    Can be created from a YAML file path or a Python dict.
    Relative paths in the configuration (e.g. ``urdf_path``, modules of ``py.`` plugins)
    are resolved from the directory of the YAML file, or from the current working
    directory for a dict.

    Args:
        config: Either a file path (str) to a YAML file, or a dict with
                the pipeline configuration.
    """

    def __init__(self, config):
        if isinstance(config, str):
            self.yaml_path = os.path.abspath(config)
            with open(self.yaml_path) as f:
                self.data = yaml.safe_load(f)
        elif isinstance(config, dict):
            self.data = config
            self.yaml_path = None
        else:
            raise TypeError(f"Config expects str (file path) or dict, got {type(config).__name__}")

    def execute(self, robot_name="robot", no_generation=True, verbose=False):
        """Run the pipeline and return the Robot.

        Args:
            robot_name: Name for the robot instance.
            no_generation: If True (default), skip generator plugins.
                Useful for interactive/notebook use where you don't want
                code generation, just the symbolic model.
            verbose: Print pipeline progress.

        Returns:
            thunder_core.Robot with all loaded data, parameters, and functions.
        """
        pm = thunder_core.PluginManager()
        pm.set_verbose(verbose)
        # a file keeps its path (config_path), a dict is passed as it is, keeping the order of its keys
        source = self.yaml_path if self.yaml_path is not None else self.data
        pm.configure_pipeline(source, int(no_generation))
        return pm.execute(robot_name)

    def __repr__(self):
        src = self.yaml_path or "dict"
        plugins = self.data.get("pipeline", {})
        return (
            f"Config({src}, "
            f"loaders={plugins.get('loaders', [])}, "
            f"builders={plugins.get('builders', [])}, "
            f"generators={plugins.get('generators', [])})"
        )
