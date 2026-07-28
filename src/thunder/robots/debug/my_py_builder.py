"""
Example Python builder plugin for Thunder.

This plugin adds a custom function (cosine of joint angles) to the robot model.
It demonstrates how to:
  - Inherit from BaseBuilder (mandatory)
  - Optionally define a pydantic ConfigModel for config validation
  - Access robot properties and symbolic variables
  - Use CasADi Python API for symbolic computation
  - Add new functions to the Robot from Python
"""
import casadi
from pydantic import BaseModel
from thunder_core.plugins import BaseBuilder


class MyCosBuilder(BaseBuilder):
    """Adds cos(q) as a new function to the robot."""

    class ConfigModel(BaseModel):
        function_name: str = "cos_q"

    def build(self, robot):
        num_joints = robot.get_int("numJoints")
        print(f"  [MyCosBuilder] Robot '{robot.robotName}' has {num_joints} joints")

        q = robot.get_model("q")
        print(f"  [MyCosBuilder] q = {q}")

        expr = casadi.cos(q)

        robot.add_function(self.config.function_name, expr, ["q"], "Cosine of joint angles")
        print(f"  [MyCosBuilder] Added function '{self.config.function_name}'")
