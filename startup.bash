#!/bin/bash

source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
ros2 run velocity_mode velocity_mode & MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
