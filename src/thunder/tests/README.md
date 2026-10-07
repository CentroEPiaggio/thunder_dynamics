# Thunder Tests

This folder contains the current test architecture.

## Google Test Integration

The CMake option below enables the C++ test suite:

```sh
cmake .. -DWITH_TESTS=ON
```

When enabled, these targets are built and registered in CTest:

- `thunder_no_generation_test`
- `thunder_generator_smoke_test`
- `thunder_tree_kinematics_test`
- `thunder_robot_comparison_gtest`

The test source and helpers are:

- `cpp/robot_comparison_gtest.cpp`
- `cpp/robot_test_helpers.h`
- `python/pinocchio_helper.py`
- `fixtures/franka/franka_urdf.yaml`
- `fixtures/franka/franka_finger.urdf` (franka without `panda_ee` and the right finger, a single chain)

## Run From CLI

```sh
cd src/thunder/build
cmake .. -DWITH_TESTS=ON
cmake --build . -j$(nproc)
ctest -V -R thunder_.*_test
```

## Pinocchio Dependency

`thunder_robot_comparison_gtest` compares M, C, Cdq and G against Pinocchio (via the Python helper),
once for each `dyn_builder` `dynamics_method` (`lagrange`, `rnea`).
If Pinocchio is not available in the active Python environment, the test is skipped.
Install it with `pip install --user pin`.
