# Installation Guide

Thunder Dynamics can be set up using Docker (recommended for ease of use and dependency management) or by installing the required dependencies directly on your host machine.


## Option 1: Docker Installation (Recommended)

Using the provided Dev Container configuration is the simplest way to get started.

1.  **Install Prerequisites**:
    *   [Docker](https://docs.docker.com/get-docker/)
    *   [Visual Studio Code](https://code.visualstudio.com/)
    *   [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) for VS Code.
2.  **Clone the Repository**:
    ```bash
    git clone https://github.com/CentroEPiaggio/thunder_dynamics.git
    cd thunder_dynamics
    ```
3.  **Open in Dev Container**:
    *   Open the `thunder_dynamics` folder in VS Code.
    *   VS Code should prompt you to "Reopen in Container". Click it.
    *   Alternatively, open the command palette (Ctrl+Shift+P), type "Dev Containers: Reopen in Container", and select it.
4.  **Build**: The container includes all necessary dependencies and builds the `thunder` executable, placing it in the `bin/` directory. You can also manually build using the VS Code Task "makeThunder".

## Option 2: Host Machine Installation

If you prefer not to use Docker, follow these steps to install dependencies and build the project manually.

### Requirements

*   **Operating System**: Linux
*   **C++ Compiler**: C++17 compatible (e.g., GCC >= 7, Clang >= 5)
*   **CMake**: Version 3.10 or higher
*   **Eigen**: Version 3.3 or higher (`sudo apt install libeigen3-dev`)
*   **CasADi**: Required for the core `thunder` library.
*   **yaml-cpp**: Required for parsing configuration files.
*   **pybind11**: Required only if generating Python bindings (`sudo apt install pybind11-dev` or `pip install pybind11`).

### 1. Install Core Dependencies

Install Eigen, CMake, and a C++17 compiler using your system's package manager. Example for Debian/Ubuntu:
```bash
sudo apt update
sudo apt install build-essential cmake libeigen3-dev
```

### 2. Install CasADi (from source)

Refer to the official [CasADi installation guide](https://web.casadi.org/get/). A typical source build involves:

```bash
# Install CasADi dependencies (check their guide for specifics)
sudo apt install gcc g++ gfortran cmake liblapack-dev pkg-config

# Clone CasADi
git clone https://github.com/casadi/casadi.git -b main casadi_source
cd casadi_source

# Build and Install
mkdir build && cd build
cmake .. -DWITH_IPOPT=ON -DWITH_LAPACK=ON # Add other options as needed
make -j$(nproc)
sudo make install
```
*Note: Ensure CasADi is installed in a location where CMake can find it (e.g., `/usr/local`) or set `CMAKE_PREFIX_PATH`.*

### 3. Install yaml-cpp (from source)

Refer to the official [yaml-cpp repository](https://github.com/jbeder/yaml-cpp).

```bash
# Install yaml-cpp dependencies (usually just CMake and a C++ compiler)

# Clone yaml-cpp
git clone https://github.com/jbeder/yaml-cpp.git yaml-cpp_source
cd yaml-cpp_source

# Build and Install
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```
*Note: Ensure yaml-cpp is installed where CMake can find it.*

### 4. Build Thunder

Navigate to the `thunder` source directory and build:

```bash
cd /path/to/thunder_dynamics/src/thunder
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```
This will create the `thunder` executable inside the `build` directory. For system-wide installation (optional):
```bash
sudo make install
```
If not installing system-wide, you might need to copy the `thunder` executable and the `neededFiles` directory from `bin/` to a location in your `PATH` or call it using its full path. Remember that the `neededFiles` directory must reside alongside the `thunder` executable.

### 5. Install Python Dependencies (Optional)

If you plan to generate Python bindings:
```bash
sudo apt install pybind11-dev
# or using pip
pip install pybind11
```
