#!/usr/bin/bash

set -e

WORKSPACE_DIR="${1:-$HOME}"
BIN_DIR=$HOME/.local/bin
WS_DIR=$1

if [ ! -d "$BIN_DIR" ]; then
    mkdir -p $BIN_DIR
fi
cd $BIN_DIR

# Download bazel buildifier
wget -O buildifier https://github.com/bazelbuild/buildtools/releases/download/v7.3.1/buildifier-linux-amd64
chmod +x buildifier

# Download bazel lsp
wget -O starpls.tar.gz https://github.com/withered-magic/starpls/releases/download/v0.1.14/starpls-linux-amd64.tar.gz
tar -xf starpls.tar.gz && rm -rf starpls.tar.gz
chmod +x starpls

# TODO: the dependecies below will be handled by bazel
# # yaml-cpp installation
# cd $BIN_DIR
# git clone https://github.com/jbeder/yaml-cpp.git yaml_cpp
# cd yaml_cpp
# mkdir -p build
# cd build
# cmake ..
# cmake --build .
# make
# sudo make install

# # install CasADi
# cd $BIN_DIR
# git clone https://github.com/casadi/casadi.git casadi
# cd casadi
# mkdir -p build
# cd build
# cmake ..
# make
# sudo make install

# # Pybind11 installation (https://pybind11.readthedocs.io/en/stable/compiling.html#find-package-vs-add-subdirectory)
# cd $BIN_DIR
# git clone https://github.com/pybind/pybind11.git pybind11
# cd pybind11
# pip install pytest
# cmake -S . -B build
# cmake --build build -j 2
# cmake --install build

# # Thunder built from source


# finishing up
chmod 777 -R "$1"
cd "$1/.devcontainer"
touch .bash_history
