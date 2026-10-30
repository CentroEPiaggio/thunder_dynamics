# Thunder Tests

Everything the automatic tests use is in this folder: sources in `cpp/` and `python/`, robots in `fixtures/`.

## Run

The tests are built with the CMake option `WITH_TESTS` and run with CTest:

```sh
cd src/thunder/build
cmake .. -DWITH_TESTS=ON
cmake --build . -j$(nproc)
ctest --output-on-failure
```

`ctest -R <name>` runs only the tests whose name matches `<name>`, e.g. `ctest -R comparison`.

The tests find their fixtures from the repository layout: CMake defines `THUNDER_SOURCE_DIR` as the repository root (`src/thunder/../..`), and the tests read `src/thunder/tests/fixtures/...` from there. They have to be built from this repository.

## Registered tests

| CTest name | Source | What it checks |
| --- | --- | --- |
| `thunder_no_generation_test` | `cpp/test_no_generation.cpp` | `RRR/testRRR.yaml` without generators: joints, `par_KIN`, `par_DYN`, `M`, `C`, `G` exist |
| `thunder_generator_smoke_test` | `cpp/test_generator_smoke.cpp` | `robot_generator` writes the expected files for `RRR/testRRR.yaml` |
| `thunder_tree_kinematics_test` | `cpp/test_tree_kinematics.cpp` | Nodes, parents, branch nodes and named frames of `urdf/tree_branch.urdf` and `urdf/serial_chain.urdf` |
| `thunder_symbolic_masks_test` | `cpp/test_symbolic_masks.cpp` | Symbolic masks of `urdf_loader`: YAML keys, URDF tags and defaults, on `franka/symbolic_mask_test.yaml` |
| `thunder_robot_comparison_gtest` | `cpp/robot_comparison_gtest.cpp` | Kinematics, dynamics and regressors of `franka/franka_finger.urdf` against Pinocchio, see below |
| `thunder_python_api_test` | `python/test_thunder_api.py` | Python API: robot round trip, Python plugins, pipeline from a file and from a dict, generator. Registered only with `BUILD_PYTHON_PLUGINS` (default `ON`) |

`cpp/robot_test_helpers.h` holds the shared helpers: loading a robot from a YAML file and calling the Pinocchio helper.

## Pinocchio comparison

`thunder_robot_comparison_gtest` compares thunder with Pinocchio on `franka/franka_finger.urdf`, at random `q`, `dq`, `ddq`:

- `T_w_<i>` and `J_<i>` of every node, and `T_w_<name>`, `J_<name>` of the available ones. Node `<link>` is compared with the frame of the next URDF link (the frame after the joint that leaves `<link>`), a terminal node with its own link;
- `M`, `C`, `Cdq`, `G`, `tau` and `tau_r`, for each combination of the `dyn_builder` options `dynamics_method` (`auto`, `rnea`, `crba`, `lagrange`) and `C_method` (`auto`, `rnea`, `christoffel`);
- the regressors times `par_REG` (`Y`, `Yr`, `reg_M`, `reg_C`, `reg_G`) for each `reg_builder` `regressor_method` (`rnea`, `lagrange`);
- `tau_r` and `Yr par_REG` built by `reg_builder` alone (pipeline without `dyn_builder`).

Pinocchio is called through `python/pinocchio_helper.py`. If Pinocchio is not available in the active Python environment, the test is skipped. Install it with `pip install --user pin`.

## Fixtures

| File | Used by |
| --- | --- |
| `RRR/testRRR.yaml` | `thunder_no_generation_test`, `thunder_generator_smoke_test`, `thunder_python_api_test` |
| `urdf/tree_branch.urdf`, `urdf/serial_chain.urdf` | `thunder_tree_kinematics_test` |
| `franka/franka_symbolic.urdf`, `franka/symbolic_mask_test.yaml` | `thunder_symbolic_masks_test` |
| `franka/franka_finger.urdf`, `franka/franka_urdf.yaml` | `thunder_robot_comparison_gtest` |
| `franka/franka.urdf` | none: the full franka, kept as the reference `franka_finger.urdf` was derived from |

`franka_finger.urdf` is the franka without `panda_ee` and the right finger, a single chain (7 R + fixed + 1 P) that thunder and Pinocchio load the same way. Its inertial parameters are not the real ones: every link has an anisotropic inertia in an `<inertial><origin>` frame rotated and moved from the link frame, so that the comparison also checks how `urdf_loader` rotates the inertia into the link frame (a diagonal inertia at the link origin would hide it).

## Adding a test

- C++: a gtest in `cpp/`, added in `src/thunder/CMakeLists.txt` inside `if(WITH_TESTS)` like the existing ones (`add_executable` linked to `library` and `gtest_main`, `THUNDER_SOURCE_DIR`, `BUILD_RPATH`, `add_test` and the `LD_LIBRARY_PATH` of `set_tests_properties`). Names follow `thunder_<topic>_test`.
- Python: a `test_*.py` file in `python/`, found by the `unittest discover` of `thunder_python_api_test`.
- Robots and configurations in `fixtures/`, read from `THUNDER_SOURCE_DIR`, never from `src/thunder/robots`.

## Manual programs

These are not run by CTest:

- `thunder_test` and `thunder_test_chrono` (`src/thunder/thunder_test*.cpp`, built with thunder): a debug print of the franka robots in `robots/debug/` and a timing benchmark on `robots/testRobots/R*.yaml`. Their paths are relative to `src/thunder/build`, run them from there.
- `src/thunder_robot_test`: a CMake project to try a generated robot library. `robot_generator` copies the generated files there with `copy_gen: true`, see the [robot_generator docs](../../../docs/plugins/robot_generator_plugin.md).
- `src/thunder_ros_test`: a ROS 2 workspace for the packages of `ros_server_generator` (`copy_gen: true`), see its [README](../../thunder_ros_test/README.md).
