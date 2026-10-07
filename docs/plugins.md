# Plugin Reference - ❗Work in progress
Description index of documentation for each existing plugin.

## Loaders

## Builders

[Dynamics Builder](plugins/dyn_builder_plugin.md): Builds `M`, `C`, `Cdq`, `G` (RNEA, CRBA or Lagrange), link friction, time derivatives and parameter conversions.

[Regressors Builder](plugins/reg_builder_plugin.md): Builds the dynamic regressors (`Yr`, `Y`, ...) and the kinematic, friction and elastic regressors.

Joint types used by the builders, and how to add new ones: [Joint types](joints.md).

## Generators

[ROS Server](plugins/ros_server_plugin.md): Creates a Ros 2 C++ package around the C++ robot produced by `robot_generator`.