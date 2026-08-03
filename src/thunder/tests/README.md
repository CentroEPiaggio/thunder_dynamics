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
- `fixtures/franka/franka.urdf`

## Run From CLI

```sh
cd src/thunder/build
cmake .. -DWITH_TESTS=ON
cmake --build . -j$(nproc)
ctest -V -R thunder_.*_test
```

## Pinocchio Dependency

The Google Test compares Thunder torques against Pinocchio (via the Python helper).
If Pinocchio is not available in the active Python environment, the test is skipped.
