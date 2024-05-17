# Thunder - [thunder_dynamics](https://github.com/CentroEPiaggio/thunder_dynamics)

The aim of `thunder_dynamics` is to generate code useful for robot's control.

* `thunder`: we implemented some classes in OOP to generalize the approach to serial manipulator control, under the point of view of Denavit-Hartenberg parametrization.
* `thunder_robot`: it is a wrapper class for the generated files. Uses library generated by casadi and provide simple functions for the robot dynamics.
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
After building the software can be used with:

	cd bin
	./thunder gen <path>/<robot>.yaml <robot_name>

where `<path>` is the relative path from the `thunder` binary and the folder containing the .yaml configuration file, `<robot>` is the default robot name and `<robot_name>` is the optional name of the robot. The name will be used to create library files.

The file `<robot>.yaml` is a configuration file that contain all the information about the robot.
The DH table takes the trasformation in the order a, alpha, d, theta.
The inertial parameters are expressed in the DH frames with the same convention.
An example can be finded in the folder `robots/` for a 7 d.o.f. robot Franka Emika Panda.


The framework will create a `generatedFiles/` directory containing some files:
- `<robot>_gen.h` is the C-generated library from CasADi associated with the source file `<robot>_gen.cpp`.
- `thunder_<robot>.h`, `thunder_<robot>.cpp` is the wrapper class for the generated files.
- `<robot>_inertial_REG` is the parameters' file that can be used to load parameters from the class `thunder_<robot>`.

In order to use the framework you can write on your own C++ program:
```C++
#include "thunder_<robot>.h"

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
	// set inertial param from vector
	my_robot.set_inertial_REG(params);
	// or load it from file
	my_robot.load_inertial_REG(".../<robot>_inertial_REG.yaml");
	// compute regressor
	eigen3::MatrixXd Y = my_robot.get_regressor();
	...
}
```

If you need to modify something in the library, you can compile it from source following the lasts instructions.
If you recompile the docker image the binary will be builded and updated in the thunder_dynamics/bin directory.
The library requires casadi and yaml-cpp that are already included in the docker image.


## Code generation with casadi
Here you can find different classes implemented with casadi library to generalize serial manipulator control.
The main classes contained in `thunder` are:

* `CasadiObj`: abstract object useful to:
   - convert casadi element to eigen element
   - obtain matrix
   - generate code
   - ...
* `RobKinBasic`: (derived from `CasadiObj`) useful for compute:
   - forward kinematic
   - jacobian.
* `RobKinAdv`: (derived from `RobKinBasic`) useful for compute:
   - derivative of jacobian
   - pseudo-inverve of jacobian
   - derivative of pseudo-inverve of jacobian
   - ... 
* `RobDyn`: (derived from `RobKinBasic`) useful for compute:
   - mass matrix
   - coriolis matrix
   - gravity matrix
* `RobReg`: (derived from `RobKinBasic`) useful for compute:
   - regressor 

the content of the classes is then integrated in the C++ library `<robot>_gen.h/cpp` and the `thunder_<robot>` class provide a wrapper for the automatically generated code.


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
