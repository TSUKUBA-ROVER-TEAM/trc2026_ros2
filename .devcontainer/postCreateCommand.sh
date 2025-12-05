#!/usr/bin/env bash

set -ex

USERNAME=vscode
RUNTIME_DIR="/home/$USERNAME/.xdg-runtime"
if [ ! -d "$RUNTIME_DIR" ]; then
    mkdir -p "$RUNTIME_DIR"
    chown -R $USERNAME:$USERNAME "$RUNTIME_DIR"
    chmod 0700 "$RUNTIME_DIR"
fi

echo "source /opt/ros/$ROS_DISTRO/setup.bash" >>~/.bashrc

if [ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
fi

sudo rosdep init
just deps

echo "Done!"