# Thunder - [thunder_dynamics](https://github.com/CentroEPiaggio/thunder_dynamics) - v0.8.14

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

**Thunder Dynamics** is a C++ framework for generating efficient, robot-specific C++ libraries and Python bindings for dynamics and control computations. It uses the Denavit-Hartenberg (DH) convention and leverages [CasADi](https://web.casadi.org/) for symbolic derivation and code generation.

Define your robot's kinematics and dynamics in a YAML file, and `thunder` generates optimized code for:
*   Kinematics (Forward Kinematics, Jacobians, etc.)
*   Dynamics (Mass Matrix, Coriolis, Gravity)
*   Regressor formulations for parameter identification / adaptive control
*   Custom user-defined functions

## Documentation

**For detailed information on installation, usage, configuration, and the API, please refer to the [Documentation](./docs/README.md).**

## Quick Installation (Recommended)

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/CentroEPiaggio/thunder_dynamics.git
    cd thunder_dynamics
    ```
2.  **Open in VS Code with Dev Containers:**
    *   Ensure you have [Docker](https://docs.docker.com/get-docker/) and the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) installed.
    *   Open the cloned folder in VS Code.
    *   Click "Reopen in Container" when prompted.
3.  **Ready!** The `thunder` executable will be built automatically inside the container and placed in the `bin/` directory.

For alternative installation methods, see the full [Installation Guide](./docs/installation.md).

## Quick Usage

1.  **Configure**: Define robot DH parameters and inertial properties in a `.yaml` file (see `robots/` for examples).
2.  **Generate**: Use the command-line tool:
    ```bash
    # C++ only
    thunder gen path/to/your_robot.yaml

    # Python bindings and a custom name
    thunder gen --python -n MyRobot path/to/your_robot.yaml
    ```
3.  **Use**: Integrate the generated library (`<robot_name>_generatedFiles/`) into your C++ or Python projects.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
