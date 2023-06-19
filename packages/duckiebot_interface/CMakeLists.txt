cmake_minimum_required(VERSION 2.8.3)
project(duckiebot_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  # duckietown_msgs # Every duckietown packages should use this.
)


catkin_package()

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
