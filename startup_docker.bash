#!/bin/bash

source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
ros2 run actuator_control actuator_control & MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
