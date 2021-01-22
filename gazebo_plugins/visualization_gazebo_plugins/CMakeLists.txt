cmake_minimum_required(VERSION 2.8.3)
project(visualization_gazebo_plugin)

# We need erb to process the .world.erb files.
find_program(ERB_EXE_PATH erb)
if(NOT ERB_EXE_PATH)
  message(FATAL_ERROR "Could not find the `erb` tool.  Try `sudo apt-get install ruby`")
endif()

set (CMAKE_AUTOMOC ON)
set (CMAKE_AUTOUIC ON)

find_package (Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)
find_package (Protobuf REQUIRED)
find_package (gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  vrx_gazebo
  message_generation
  wamv_gazebo
  wave_gazebo
  xacro
  gazebo_dev
  geographic_msgs
  std_msgs
  rospy
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS vrx_gazebo wamv_gazebo wave_gazebo xacro gazebo_dev geographic_msgs std_msgs message_runtime
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}") 

include_directories(include ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${QT_USE_FILE}
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
)

add_definitions(${QT_DEFINITIONS})

# Plugin for scoring the waypoint navigation challenge task.
add_library(waypoints_visualization_plugin src/waypoints_visualization_gazebo_plugin.cc)
target_link_libraries(waypoints_visualization_plugin
  ${catkin_LIBRARIES}
  scoring_plugin
  waypoint_markers)
add_dependencies(waypoints_visualization_plugin ${catkin_EXPORTED_TARGETS})
install(TARGETS waypoints_visualization_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
