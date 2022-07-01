cmake_minimum_required(VERSION 2.8.3)
project(flight_controller_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  duckietown_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

