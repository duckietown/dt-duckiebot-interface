cmake_minimum_required(VERSION 2.8.3)
project(traffic_light)

find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  rgb_led
  roscpp
  rospy
)

catkin_python_setup()


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
