#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/uav/src/sysid-ROS2-offboard/install/setup.bash
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600 & ros2 run actuator_control actuator_control
