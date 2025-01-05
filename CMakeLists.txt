cmake_minimum_required(VERSION 3.0.2)
project(project_package)

# Abhängige ROS-Pakete
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
)

# Erstelle das catkin-Paket
catkin_package()

# Inklusionsverzeichnisse
include_directories(
  ${catkin_INCLUDE_DIRS}
)

