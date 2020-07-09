cmake_minimum_required(VERSION 2.8.3)
project(led_emitter)


find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  rospy
  message_generation
  std_msgs
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Light_Adjustment.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetCustomLEDColor.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
  duckietown_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
