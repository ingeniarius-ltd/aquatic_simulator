cmake_minimum_required(VERSION 2.8.3)
project(aquatic_gazebo_sensor_plugins_msgs)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation)

add_message_files(
  FILES
  Beacon.msg
  Hydrophone.msg
)

add_service_files(
  FILES
  ChangeSensorState.srv
)


generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS geometry_msgs message_runtime
# DEPENDS
)
