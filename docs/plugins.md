# Plugin Reference

Documentation of each built-in plugin. Plugins run in the order of the `pipeline` section: loaders, then builders, then generators (see the [Configuration Guide](configuration.md)).

## Loaders

[Kinematics Loader](plugins/kin_loader_plugin.md) (`kin_loader`): Loads the tree of frames and joints written in the YAML file. Also defines the tree conventions and the symbolic masks used by all loaders.

[Dynamics Loader](plugins/dyn_loader_plugin.md) (`dyn_loader`): Loads the inertial parameters of the bodies, gravity and the link friction.

[URDF Loader](plugins/urdf_loader_plugin.md) (`urdf_loader`): Loads structure, frames and inertial parameters from a URDF, with symbolic masks per link.

[DH Loader](plugins/dh_loader_plugin.md) (`dh_loader`): Loads a serial chain from a modified Denavit-Hartenberg table.

[Soft Loader](plugins/soft_loader_plugin.md) (`soft_loader`): Adds the series elastic joint types, the motor variables and the elastic parameters.

## Builders

[Kinematics Builder](plugins/kin_builder_plugin.md) (`kin_builder`): Builds the transforms and Jacobians of the frames, the Jacobian derivatives and pseudo-inverse, and registers the built-in joint types.

[Dynamics Builder](plugins/dyn_builder_plugin.md) (`dyn_builder`): Builds `M`, `C`, `Cdq`, `G` (RNEA, CRBA or Lagrange), link friction, time derivatives and parameter conversions.

[Regressors Builder](plugins/reg_builder_plugin.md) (`reg_builder`): Builds the dynamic regressors (`Yr`, `Y`, ...) and the kinematic, friction and elastic regressors.

[Soft Builder](plugins/soft_builder_plugin.md) (`soft_builder`): Builds the elastic coupling, damping and motor inertia of the series elastic joints.

`example_builder` is a template for new builders, see the [Plugin Development Guide](plugin_guide.md#starting-from-example_builder).

Joint types used by the builders, and how to add new ones: [Joint types](joints.md).

## Generators

[Robot Generator](plugins/robot_generator_plugin.md) (`robot_generator`): Generates the C code of the functions, the `thunder_<robot>` C++ class, and optionally Python bindings and CasADi files.

[ROS Server](plugins/ros_server_plugin.md) (`ros_server_generator`): Creates a ROS 2 C++ package around the C++ robot produced by `robot_generator`.
