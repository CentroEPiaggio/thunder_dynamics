FROM ros:jazzy-ros-base

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# ros2_control, the standard controllers (joint_state_broadcaster, JTC as baseline),
# gz_ros2_control for closed-loop tests in simulation
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-realtime-tools \
    ros-jazzy-control-msgs \
    ros-jazzy-urdf \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# the base image already has user 'ubuntu' with UID 1000: reuse the UID for the bind mount
RUN (userdel -r ubuntu || true) \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

RUN printf '%s\n' \
    'source /opt/ros/jazzy/setup.bash' \
    'if [ -f /workspaces/thunder_ros_control_test/install/setup.bash ]; then' \
    '    source /workspaces/thunder_ros_control_test/install/setup.bash' \
    'fi' >> /etc/bash.bashrc

USER ${USERNAME}
WORKDIR /workspaces/thunder_ros_control_test
