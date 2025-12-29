FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

RUN mkdir -p .local/bin

RUN	apt-get update 
RUN apt-get install --no-install-recommends --assume-yes \
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

# scripts setup
RUN mkdir -p ${HOME}/scripts
COPY .devcontainer/scripts/bootstrap.sh ${HOME}/scripts/bootstrap.sh

RUN chown -R ${USERNAME}:${USERNAME} ${HOME} \
    && chmod +x ${HOME}/scripts/bootstrap.sh

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

