"""
Thunder pipeline configuration and execution from Python.

Allows running the full Thunder plugin pipeline from Python code or notebooks,
using either a YAML file or a Python dict as configuration.

Example::

    from thunder_core.pipeline import Config, run_pipeline

    # From YAML file
    robot = run_pipeline("path/to/robot.yaml", robot_name="my_robot")

    # From dict
    robot = run_pipeline({
        "pipeline": {
            "loaders": ["kin_loader"],
            "builders": ["kin_builder"],
            "generators": []
        },
        "kin_loader": {
            "num_joints": 2,
            "joints_type": ["R", "R"],
        }
    }, robot_name="RR")

    # Or use Config directly for more control
    cfg = Config("path/to/robot.yaml")
    print(cfg)
    robot = cfg.execute(robot_name="RR")
"""

import os
import tempfile

import yaml
import thunder_core


class Config:
    """Thunder pipeline configuration.

    Can be created from a YAML file path or a Python dict.
    Handles conversion to a temporary YAML file for the C++ PluginManager.

    Args:
        config: Either a file path (str) to a YAML file, or a dict with
                the pipeline configuration.
    """

    def __init__(self, config):
        if isinstance(config, str):
            self.yaml_path = os.path.abspath(config)
            with open(self.yaml_path) as f:
                self.data = yaml.safe_load(f)
            self._temp_file = None
        elif isinstance(config, dict):
            self.data = config
            self.yaml_path = None
            self._temp_file = None
        else:
            raise TypeError(f"Config expects str (file path) or dict, got {type(config).__name__}")

    def _ensure_yaml_file(self):
        """Create a temporary YAML file from dict config if needed."""
        if self.yaml_path is not None:
            return self.yaml_path

        if self._temp_file is None:
            self._temp_file = tempfile.NamedTemporaryFile(
                mode='w', suffix='.yaml', delete=False, prefix='thunder_'
            )
            # Order of elements in params is significant to loaders such as kin_loader.
            # PyYAML sorts mappings by default, which differs from the dict.
            # This is a problem when a configuration is supplied as a Python dict, so we disable sorting here.
            yaml.dump(self.data, self._temp_file, default_flow_style=False, sort_keys=False)
            self._temp_file.close()
            self.yaml_path = self._temp_file.name

        return self.yaml_path

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
        pm.configure_pipeline(self._ensure_yaml_file(), int(no_generation))
        return pm.execute(robot_name)

    def __del__(self):
        """Clean up temporary YAML file."""
        if self._temp_file is not None:
            try:
                os.unlink(self._temp_file.name)
            except OSError:
                pass

    def __repr__(self):
        src = self.yaml_path or "dict"
        plugins = self.data.get("pipeline", {})
        return (
            f"Config({src}, "
            f"loaders={plugins.get('loaders', [])}, "
            f"builders={plugins.get('builders', [])}, "
            f"generators={plugins.get('generators', [])})"
        )
