# Base image
FROM ros:humble-ros-base

# Install some Python dependencies
RUN apt update && apt install python3-genmsg python3-setuptools -y 

# Set up the XRCE-DDS agent
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
WORKDIR /Micro-XRCE-DDS-Agent/build
RUN cmake .. && make && make install && ldconfig /usr/local/lib/

# Add source code into workspace
ADD ./src/ /ros_ws/src/
ADD ./startup_docker.bash /ros_ws/startup.bash

# Build the ROS workspace
WORKDIR /ros_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Set entry point
CMD ["/ros_ws/startup.bash"]
