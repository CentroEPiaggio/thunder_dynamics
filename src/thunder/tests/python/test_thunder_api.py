import contextlib
import os
import tempfile
import unittest
from pathlib import Path

import yaml

import thunder_core
from thunder_core.plugins import BaseBuilder
from thunder_core.pipeline import Config


@contextlib.contextmanager
def temporary_cwd(path: str):
    previous = os.getcwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(previous)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[4]


class DummyBuilder(BaseBuilder):
    def build(self, robot):
        robot.add_property_string("unit_test_marker", "ok")


class ThunderApiTests(unittest.TestCase):
    def test_robot_api_roundtrip(self):
        robot = thunder_core.Robot("unit_robot")
        robot.add_property_int("ndof", 2)
        robot.add_property_double("mass", 1.5)
        robot.add_property_vector_string("joints", ["a", "b"])

        self.assertEqual(robot.get_int("ndof"), 2)
        self.assertEqual(robot.get_double("mass"), 1.5)
        self.assertEqual(robot.get_vector_string("joints"), ["a", "b"])

    def test_python_plugin_base_class(self):
        builder = DummyBuilder()
        builder.configure({"flag": True})
        robot = thunder_core.Robot("plugin_robot")

        builder.build(robot)

        self.assertEqual(builder.config["flag"], True)
        self.assertEqual(robot.get_string("unit_test_marker"), "ok")

    def test_pipeline_without_generation(self):
        config_path = repo_root() / "src" / "thunder" / "tests" / "fixtures" / "RRR" / "testRRR.yaml"
        robot = Config(str(config_path)).execute(
            robot_name="py_pipeline_no_generation",
            no_generation=True,
        )

        self.assertEqual(robot.get_int("numJoints"), 3)
        self.assertIn("M", robot.functions)
        self.assertIn("C", robot.functions)
        self.assertIn("G", robot.functions)

    def test_dict_config_preserves_kinematic_frame_order(self):
        config = Config({
            "pipeline": {"loaders": ["kin_loader"]},
            "kin_loader": {
                "num_joints": 2,
                "joints_type": ["R", "R"],
                "kinematics": {
                    "base_joint": {"xyzrpy": [0, 0, 0, 0, 0, 0]},
                    "tool_joint": {"xyzrpy": [1, 0, 0, 0, 0, 0]},
                },
            },
        })
        temporary_yaml = config._ensure_yaml_file()
        with open(temporary_yaml) as handle:
            written = yaml.safe_load(handle)

        self.assertEqual(
            list(written["kin_loader"]["kinematics"]),
            ["base_joint", "tool_joint"],
        )

    def test_generator_smoke(self):
        config_path = repo_root() / "src" / "thunder" / "tests" / "fixtures" / "RRR" / "testRRR.yaml"
        with config_path.open() as handle:
            data = yaml.safe_load(handle)

        data["robot_generator"]["gen_casadi"] = False
        data["robot_generator"]["gen_python"] = False
        data["robot_generator"]["gen_robot"] = True
        data["robot_generator"]["copy_gen"] = False

        with tempfile.TemporaryDirectory() as tmpdir:
            with temporary_cwd(tmpdir):
                robot = Config(data).execute(
                    robot_name="py_gen_smoke",
                    no_generation=False,
                )

                out_dir = Path(tmpdir) / "py_gen_smoke_generatedFiles"
                self.assertIsNotNone(robot)
                self.assertTrue(out_dir.exists())
                self.assertTrue((out_dir / "py_gen_smoke_gen.cpp").exists())
                self.assertTrue((out_dir / "py_gen_smoke_gen.h").exists())
                self.assertTrue((out_dir / "thunder_py_gen_smoke.cpp").exists())
                self.assertTrue((out_dir / "thunder_py_gen_smoke.h").exists())
                self.assertTrue((out_dir / "py_gen_smoke_conf.yaml").exists())
                self.assertTrue((out_dir / "py_gen_smoke_par.yaml").exists())


if __name__ == "__main__":
    unittest.main()
