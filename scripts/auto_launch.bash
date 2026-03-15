#!/bin/bash

cd /home/trt/trc2026_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ubxtool -p CFG-RATE,100,1,1 -f /dev/gps
ubxtool -S 115200 -f /dev/gps
ros2 launch trc2026_bringup real.launch.yaml