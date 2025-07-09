FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

RUN	apt-get update && apt-get install -y \
	git \
	nano \
	wget \
	build-essential \
	cmake \
	cmake-curses-gui \
	gdb \
	libeigen3-dev \
	python3	\
	python3-pip \
	&& rm -rf /var/lib/apt/lists/*

# Create a new user
ARG USERNAME=thunder_dev
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && apt-get update \
    && apt-get install -y sudo \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

ENV HOME /home/${USERNAME} 

# Enable bash history so it can be mounted and become persistent
RUN SNIPPET="export PROMPT_COMMAND='history -a' && export HISTFILE=/commandhistory/.bash_history" \
    && mkdir /commandhistory \
    && touch /commandhistory/.bash_history \
    && chown -R $USERNAME /commandhistory \
    && echo "${SNIPPET}" >> "/home/$USERNAME/.bashrc"

# Install casadi and yaml-cpp
WORKDIR ${HOME}

# yaml-cpp installation
RUN git clone https://github.com/jbeder/yaml-cpp.git yaml_cpp
WORKDIR ${HOME}/yaml_cpp
RUN mkdir -p build && \
	cd build && \
	cmake .. && \
	cmake --build . && \
	make && \
	make install

# CasADi installation
WORKDIR ${HOME}
RUN git clone https://github.com/casadi/casadi.git casadi
WORKDIR ${HOME}/casadi
RUN mkdir -p build && \
	cd build && \
	cmake .. && \
	make && \
	make install

# Pybind11 installation (https://pybind11.readthedocs.io/en/stable/compiling.html#find-package-vs-add-subdirectory)
WORKDIR ${HOME}
RUN git clone https://github.com/pybind/pybind11.git pybind11
WORKDIR ${HOME}/pybind11
RUN pip install pytest && \
	cmake -S . -B build && \
	cmake --build build -j 2  && \
	cmake --install build

# Thunder built from sources
WORKDIR ${HOME}/..
RUN mkdir -p thunder_dynamics_tmp
COPY ../. ${HOME}/../thunder_dynamics_tmp
WORKDIR ${HOME}/../thunder_dynamics_tmp/src/thunder
RUN rm -r -f build && \
	mkdir -p build && \
	cd build && \
	cmake .. && \
	make && \
	make install

# # copy thunder binary and auxiliary files on the root folder, not possible here, made by postCreateCommand in devcontainer.json
# WORKDIR ${HOME}/thunder_dev/thunder_dynamics
# RUN mkdir -p bin && \
# 	mkdir -p bin/neededFiles && \
# 	cp ${HOME}/../thunder_dynamics_tmp/src/thunder/build/thunder bin/thunder && \
# 	cp ${HOME}/../thunder_dynamics_tmp/src/thunder_robot/library/thunder_robot.h bin/neededFiles/thunder_robot.h && \
# 	cp ${HOME}/../thunder_dynamics_tmp/src/thunder_robot/src/thunder_robot.cpp bin/neededFiles/thunder_robot.cpp

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER ${USERNAME}
WORKDIR ${HOME}

# Change entrypoint to source ~/.bashrc and start in ~
COPY ./.devcontainer/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh ; sudo chown ${USERNAME} /entrypoint.sh \
    && echo "cd "${HOME} >> /entrypoint.sh \
    && echo 'exec bash -i -c $@' >> /entrypoint.sh \
    && cat /entrypoint.sh

# Clean image
RUN sudo apt clean && sudo rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

