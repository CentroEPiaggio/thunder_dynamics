FROM mcr.microsoft.com/devcontainers/cpp:0-ubuntu-22.04

# ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="3.22.2"

# # Optionally install the cmake for vcpkg
# COPY ./reinstall-cmake.sh /tmp/

# RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
#         chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
#     fi \
#     && rm -f /tmp/reinstall-cmake.sh

# [Optional] Uncomment this section to install additional vcpkg ports.
# RUN su vscode -c "${VCPKG_ROOT}/vcpkg install <your-port-name-here>"

# [Optional] Uncomment this section to install additional packages.
# RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
#     && apt-get -y install --no-install-recommends <your-package-list-here>

# RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
# 	&& apt-get -y install --no-install-recommends cmake-curses-gui

# ARG CMAKE_VERSION=3.24.2

RUN	apt-get update && export DEBIAN_FRONTEND=noninteractive \
	&& apt-get install -y git \
	wget \
	cmake \
	cmake-curses-gui \
	&& rm -rf /var/lib/apt/lists/*

ENV HOME /home
WORKDIR ${HOME}

RUN git clone https://github.com/jbeder/yaml-cpp.git yaml_cpp
WORKDIR ${HOME}/yaml_cpp
RUN mkdir build \
	&& cd build \
	&& cmake .. \
	&& cmake --build . \
	&& make \
	&& make install

WORKDIR ${HOME}
RUN git clone https://github.com/casadi/casadi.git casadi
WORKDIR ${HOME}/casadi
RUN mkdir build \
	&& cd build \
	&& cmake .. \
	&& make \
	&& make install

# decidere se compilare alla compilazione del container o no, nel caso va montata la cartella thunder_dynamics


