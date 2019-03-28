cmake_minimum_required(VERSION 2.8.3)
project(duckietown_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  tf_conversions
  tf
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  WheelsCmd.msg
  WheelsCmdStamped.msg
  BoolStamped.msg
  CoordinationSignal.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES duckietown_msgs
   CATKIN_DEPENDS
   geometry_msgs
   roscpp rospy
   sensor_msgs
   std_msgs
   message_runtime
   visualization_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
