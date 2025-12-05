# Set ROS distribution. 
ARG ROS_DISTRO=jazzy

# Use official image. 
FROM ros:${ROS_DISTRO}-ros-base

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN if id -u ${USER_UID} ; then userdel -f `id -un ${USER_UID}` ; fi

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && apt-get update \
    && apt-get install -y sudo \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# ROS2 installs. 
RUN apt update && \
    apt install -y software-properties-common && \
    apt install -y curl && \
    apt install -y wget just
RUN apt install -y ros-${ROS_DISTRO}-turtlesim
RUN apt install -y ros-${ROS_DISTRO}-rqt*
RUN apt install -y ros-${ROS_DISTRO}-ros-gz

# Install MESA graphics utility tools. 
RUN apt install -y mesa-utils
RUN apt install -y ffmpeg libsm6 libxext6
RUN apt install -y libgl1-mesa-dev libosmesa6-dev

# Display environment setup. 
ENV DISPLAY=host.docker.internal:0.0
ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Use Stable version of MESA. 
RUN add-apt-repository ppa:kisak/kisak-mesa && \
    apt update && \
    apt install -y mesa-utils && \
    apt install -y ffmpeg libsm6 libxext6 && \
    apt install -y libgl1-mesa-dev libosmesa6-dev && \
    apt upgrade -y

# Check versions. 
RUN apt update && \
    apt upgrade -y

# Source ROS2 for default terminal. 
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

ENV SHELL /bin/bash

USER $USERNAME
CMD ["/bin/bash"]