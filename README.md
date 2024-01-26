# Thunder - [thunder_dynamics](https://github.com/CentroEPiaggio/thunder_dynamics)

`Repository under development!!`

Here you can find different classes implemented with casadi library to generalize serial manipulator control, in particolar for the robot Franka Emika Panda.

* `thunder`: we implemented some classes in OOP to generalize the approach to serial manipulator control, under the point of view of Denavit-Hartenberg parametrization.
* `genYAML`: we manipulate "original" Franka Emika Panda inertial parameters in `inertial.yaml` to create yaml files for Denavit-Hartenberg parametrization and then for the Regressor.
* `generatedFiles`: it contains the files generated from 2 folders above.
* `chrono_test`: it contains the files to compare execution-time of different algorithms.

## Requirements
* If you want to use and/or modify implemented classes you have to install [casadi](https://github.com/casadi/casadi.git) (see [installation](#casadi---from-source)).
* If you want generate yaml file you have to install [yaml-cpp](https://github.com/jbeder/yaml-cpp.git) (see [installation](#yaml-cpp---from-zip)).

## Generation of code with casadi
The aim of `thunder` folder is to generate code useful for robot's control.

The main classes contained in `thunder` folder are:

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

## Installation
Steps for installation:

### Docker:
clone the thunder_dynamics repository

	git clone https://github.com/CentroEPiaggio/thunder_dynamics.git

open the folder with vscode and build docker image with devcontainer extension.
After building the software can be used with:

	cd bin
	./thunder gen path/robot.yaml

The framework will create 3 files: `robot_lib.h`, `robot_lib.cpp`, `thunder_robot.h`.
`robot_lib.h` is the C-generated library from CasADi associated with the source file `robot_lib.cpp`.
`thunder_robot.h` is the wrapper class for the generated files.
In order to use the framework you can write on your main C++ program:

```C++
#include "thunder_robot.h"

int main(){
	thunder_robot robot;
	robot.setArguments(q, dq, dq_r, ddq_r);
	robot.setParams(params);
	eigen3::MatrixXd Y = robot.get_regressor();
```

If you need to modify something in the library, you can compile it from source following the lasts instructions.
If you recompile the docker image the binary will be builded and updated with the modification in the thunder_dynamics directory.
The library requires casadi and yaml-cpp that are already included in the docker image.
If you don't want to use docker you have to follow the compilation from source:

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
	cd casadi; mkdir build; cd build
	cmake ..
	ccmake ..
	make
	make install
	```

### yaml-cpp - from source

Before continue follow installation instruction from repostery https://github.com/jbeder/yaml-cpp.

That are following:

1. Download local clone .zip from https://github.com/jbeder/yaml-cpp and extract.
1. Navigate into the source directory and run:
	```
	mkdir build; cd build
	cmake ..
	cmake --build .
	make
	sudo make install
	```

### Thunder - from source

In the thunder folder exec:
	```
	mkdir build; cd build
	cmake ..
	make
	```

then you can substitute the binary file or use it where you want.
