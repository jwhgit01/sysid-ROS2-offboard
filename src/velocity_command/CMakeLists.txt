cmake_minimum_required(VERSION 3.5)
project(velocity_command)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
# include_directories(
#   ${EIGEN3_INCLUDE_DIR}
# )

#################
# Setup targets #
#################

include_directories(include SYSTEM ${Eigen3_INCLUDE_DIRS})

# velocity_command
add_executable(velocity_command src/velocity_command.cpp)
ament_target_dependencies(velocity_command rclcpp px4_msgs)
install(TARGETS velocity_command DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export dependencies to downstream packages
ament_export_dependencies(ament_cmake rclcpp eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)

# Install header files
# install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

############
# Testing ##
############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
