FROM osrf/ros:humble-desktop

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ros-humble-realtime-tools \
    && rm -rf /var/lib/apt/lists/* \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    'if [ -f /workspaces/thunder_ros_test/install/setup.bash ]; then' \
    '    source /workspaces/thunder_ros_test/install/setup.bash' \
    'fi' >> /etc/bash.bashrc

USER ${USERNAME}
WORKDIR /workspaces/thunder_ros_test
