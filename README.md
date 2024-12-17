# Thunder - [thunder_dynamics](https://github.com/CentroEPiaggio/thunder_dynamics) - v0.4.8

The aim of `thunder_dynamics` is to generate code useful for robot's dynamics and control.

* `thunder`: we implemented some classes in OOP to generalize the approach to serial manipulator control, under the point of view of Denavit-Hartenberg parametrization.
* `thunder_robot`: it is a wrapper class for the generated files. Uses library generated by casadi and provide simple functions for the robot dynamics and adaptive control.
* [old] `genYAML`: we manipulate "original" Franka Emika Panda inertial parameters in `inertial.yaml` to create yaml files for Denavit-Hartenberg parametrization and then for the Regressor.
* [old] `generatedFiles`: it contains the files generated from 2 folders above.
* [old] `chrono_test`: it contains the files to compare execution-time of different algorithms.


## Requirements
The straightforward installation require the use of docker and dev-container extension for vscode. Otherwise it is possible to install everything in the host machine.

### Docker
* [docker](https://www.docker.com/get-started/) (see [installation](https://docs.docker.com/get-docker/)).
* [dev-container](https://code.visualstudio.com/docs/devcontainers/containers) for [vscode](https://code.visualstudio.com/).

### Host machine
* Linux
* C++ 17
* In order to use and/or modify implemented classes you have to install [casadi](https://github.com/casadi/casadi.git) (see [installation](#casadi---from-source)).
* In order to generate and manage yaml files you have to install [yaml-cpp](https://github.com/jbeder/yaml-cpp.git) (see [installation](#yaml-cpp---from-zip)).


## Usage
After the docker building, the software can be used with:

	cd bin
	./thunder gen <path>/<robot>.yaml <robot_name>

where `<path>` is the relative path from the `thunder` binary and the folder containing the .yaml configuration file, `<robot>` is the default robot name and `<robot_name>` is the optional name of the robot. The name will be used to create library files.

The file `<robot>.yaml` is a configuration file that contain all the information about the robot.
The DH table takes the trasformation in the order a, alpha, d, theta.
The inertial parameters are expressed in the DH frames with the same convention.
An example can be finded in the folder `robots/` for a 7 d.o.f. robot Franka Emika Panda, or a 3 d.o.f RRR manipulator, or a SEA RRR robot.


The framework will create a `<robot>_generatedFiles/` directory containing some files:
- `<robot>_gen.h` is the C-generated library from CasADi associated with the source file `<robot>_gen.cpp`.
- `thunder_<robot>.h`, `thunder_<robot>.cpp` is the wrapper class for the generated files.
- `<robot>_par` is the parameters' file that can be used to load parameters from the class `thunder_<robot>`.
- `<robot>_inertial_REG` is another parameter's file that have the classical parameters in which the system is linear to.

In order to use the framework you can write on your own C++ program:
```C++
#include "thunder_<robot>.h"
#include <eigen3/Eigen/Dense>

int main(){
	// init variables
	eigen::VectorXd q;
	eigen::VectorXd dq;
	eigen::VectorXd dq_r;
	eigen::VectorXd ddq_r;
	eigen::VectorXd params;
	// create robot instance
	thunder_<robot> my_robot;
	// set configuration
	my_robot.setArguments(q, dq, dq_r, ddq_r);
	// load parameters
	my_robot.load_conf("path/to/par/file"); // or load_par_REG()
	// or set it at runtime from vector
	my_robot.set_par_DYN(params); // or set_par_REG(), or set_par_<par>()

	// - compute standard quantities - //
	Eigen::MatrixXd T = my_robot.get_T_0_ee(); // end-effector kinematics
	Eigen::MatrixXd J = my_robot.get_J_ee(); // end-effector Jacobian matrix
	Eigen::MatrixXd M = my_robot.get_M(); // Mass matrix
	Eigen::MatrixXd C = my_robot.get_C(); // Coriolis matrix
	Eigen::MatrixXd G = my_robot.get_G(); // Gravity vector
	Eigen::MatrixXd Yr = my_robot.get_Yr(); // regressor matrix
	...
}
```

If you need to modify something in the library, you can compile it from source following the lasts instructions.
If you recompile the docker image the binary will be builded and updated in the thunder_dynamics/bin directory.
The library requires casadi and yaml-cpp that are already included in the docker image.

## Usage in python: 
This branch of Thunder can generate python bindings for the generated library. Simply add the `python` flag to the `thunder gen` command:

	cd bin
	./thunder gen <path>/<robot>.yaml <robot_name> python

This will generate a python wrapper for the generated library. To use it you need to build the module. Make sure to have installed pybind11

	pip install pybind11

To build the module:

	cd <robot>_generatedFiles
	mkdir -p build && cd build
	cmake .. 
	make

This will create a .so file that can be imported in python. Make it executable:

	chmod +x thuder_<robot>_py.<pyversion>.so

and use it in python:

```python
import numpy as np
import sys
sys.path.append("path/to/thunder_robot/generatedFiles/build") # Where the .so file is located. 
# Note: This is not needed if the .so file is in the same directory as the python script

from thunder_<robot>_py import thunder_<robot>

robot = thunder_<robot>()
robot.load_conf("path/to/robot_conf.yaml")

robot.set_q(np.zeros(robot.get_njoints))
robot.set_dq(np.random.rand(robot.get_njoints))

T = robot.get_T_0_ee()
J = robot.get_J_ee()
M = robot.get_M()
C = robot.get_C()
G = robot.get_G()
Yr = robot.get_Yr()
```

> [!NOTE]
> The generated bindings can be built on any sistem, simply install the runtime dependencies:

	sudo apt install libeigen3-dev pybind11-dev


## Code generation with casadi
Here you can find different classes implemented with casadi library to generalize serial manipulator control.
The main classes contained in `thunder` are:

* `Robot`: object that contain everything needed to the robot:
   - contain all the robot functions (kinematics, dynamics...)
   - functions can be dynamically added to the robot
   - all the functions added to the robot can be exported on the thunder_robot class with code generation
   - with the function `add_function()` is possible to add expressions to the internal robot functions
   - following modules permits to expand the robot functionalities by adding functions
* `kinematics`: contain standard kinematic functions:
   - T_0_i: return the transformation 0->Li
   - J_i: jacobian of the frame i
   - J_ee_dot: jacobian derivative
   - J_ee_pinv: jacobian pseudoinverse
* `dynamics`: contain standard dynamic functions:
   - M: mass matrix
   - C: coriolis matrix
   - G: gravity matrix
   - Dl: link friction
   - K: elastic actuator stiffness torque
   - D: elastic actuator coupling damping
   - Dm: elastic actuator motor friction
* `regressors`: contain the regressor formulation:
   - Yr: regressor matrix 
   - reg_M: regressor of $M\ddot{q}$
   - reg_C: regressor of $C\dot{q}$
   - reg_G: regressor of $G$
   - reg_K, reg_D, reg_Dl, reg_Dm: regressors of other dynamics
   - reg_Jdq: regressor of $\omega = J\dot{q}$
   - reg_JTw: regressor of $\tau = J^T w$

the content of the classes is then integrated in the C++ library `<robot>_gen.h/cpp` and the `thunder_<robot>` class provide a wrapper for the automatically generated code.

## Symbolic selectivity of parameters
Each parameter specified in the config file can be symbolic or not based on the `symb:` control boxes in the specific parameter.
For example, in the inertial parameters it is sufficient to write `symb: [1,1,1,1,1,1,1,1,1,1]` to enable the symbolic computation of the classical dynamics.
This functionality can be used to change some parameters in the compiled functions without the necessity of re-build the code., a features which is particulary important in applications like adaptive control, where parameters have to adapt at real time.

The parameters in the yaml file are something like:
```yaml

parameter:
  symb: [0,0,1] 		# only the third element of parameter is symbolic
  value: [1, 2, 3] 		# initial values of the parameter
...
}
```

then in the built code it is possible to write
```C++
thunder_<robot> myRobot;
myRobot.set_parameter(1); 	# this change the symbolic third element of parameter from 3 to 1
```

## Installation
Steps for installation:

### Docker:
clone the thunder_dynamics repository

	git clone https://github.com/CentroEPiaggio/thunder_dynamics.git

open the folder with vscode and build docker image with devcontainer extension.
If you don't want to use docker you have to follow the compilation from source instructions:

### casadi - from source
   
Before continue check the instructions of installation in the official [API site](https://casadi.sourceforge.net/api/html/d3/def/chapter2.html), [git repostery](https://github.com/casadi/casadi.git) and [website](https://web.casadi.org/).

That are following:

1. Clone the `main` repostery of `casadi`:
	```
	git clone https://github.com/casadi/casadi.git -b main casadi
	```

1. Set the environment variable `CMAKE_PREFIX_PATH` to inform CMake where the dependecies are located. For example if headers and libraries are installed under `$HOME/local/`, then type:
	```
	export CMAKE_PREFIX_PATH=$HOME/local/
	```

1. Make install
	```
	cd casadi && mkdir -p build && cd build
	cmake ..
	ccmake ..
	make
	make install
	```

### yaml-cpp - from source

Before continue follow installation instruction from repostery https://github.com/jbeder/yaml-cpp.

That are the following:

1. Download local clone .zip from https://github.com/jbeder/yaml-cpp and extract.
1. Navigate into the source directory and run:
	```
	mkdir -p build && cd build
	cmake ..
	cmake --build .
	make
	sudo make install
	```

### Thunder - from source

In the thunder folder exec:
```
mkdir -p build && cd build
cmake ..
make
```

then you can substitute the binary file with `setup_bin` in `.devcontainer/` folder and use the binary where you want. Remember that `neededFiles/` have to be in the same folder of `thunder`.
