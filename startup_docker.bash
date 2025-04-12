#!/bin/bash

source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600 & ros2 run actuator_control actuator_control
