# File: /Omega-Code/ros/robot_simulation/CMakeLists.txt

# Minimum CMake version required to build this package
cmake_minimum_required(VERSION 3.0.2)

# Project name
project(robot_simulation)

# Find required catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

# Declare catkin package
catkin_package()

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)
