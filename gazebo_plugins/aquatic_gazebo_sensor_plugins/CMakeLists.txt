cmake_minimum_required(VERSION 2.8.3)
project(aquatic_gazebo_sensor_plugins)

add_definitions(-std=c++11)

# set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  uuv_gazebo_plugins
  sensor_msgs
  image_transport
  geometry_msgs
  std_msgs
  roscpp
  tf
  gazebo_plugins
  aquatic_gazebo_sensor_plugins_msgs
  message_filters
  tf2_ros
  tf2
  laser_geometry)

find_package(Eigen3 REQUIRED)
find_package(OGRE REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

set(UUV_SENSOR_ROS_PLUGINS_LIST "")

# Add Gazebo custom protobuf messages
set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

message(STATUS "PROTOBUF_IMPORT_DIRS = " ${PROTOBUF_IMPORT_DIRS})

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

message(STATUS "GAZEBO_MSG_INCLUDE_DIRS = " ${GAZEBO_MSG_INCLUDE_DIRS})

file(GLOB msgs msgs/*.proto)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    uuv_gazebo_ros_beacon_plugin
    uuv_gazebo_ros_hydrophone_plugin
  CATKIN_DEPENDS
    gazebo_dev
    uuv_gazebo_plugins
    sensor_msgs
    geometry_msgs
    std_msgs
    roscpp
    tf
    gazebo_plugins
    aquatic_gazebo_sensor_plugins_msgs
    message_filters
    tf2_ros
    tf2
    image_transport
    laser_geometry
)

###########
## Build ##
###########

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR} # for generated messages
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_Paging_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(aquatic_gazebo_sensor_msgs SHARED ${PROTO_SRCS})
target_link_libraries(aquatic_gazebo_sensor_msgs ${PROTOBUF_LIBRARY} ${GAZEBO_LIBRARIES})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST aquatic_gazebo_sensor_msgs)

###############################################################################

add_library(uuv_gazebo_ros_hydrophone_plugin
  src/HydrophoneROSPlugin.cc
  src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_hydrophone_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_hydrophone_plugin aquatic_gazebo_sensor_msgs ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_hydrophone_plugin)

###############################################################################

add_library(uuv_gazebo_ros_beacon_plugin
  src/BeaconROSPlugin.cc
  src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc)
target_link_libraries(uuv_gazebo_ros_beacon_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(uuv_gazebo_ros_beacon_plugin aquatic_gazebo_sensor_msgs ${catkin_EXPORTED_TARGETS})
list(APPEND UUV_SENSOR_ROS_PLUGINS_LIST uuv_gazebo_ros_beacon_plugin)

#############
## Install ##
#############

install(
  TARGETS ${UUV_SENSOR_ROS_PLUGINS_LIST}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.pb.*"
  PATTERN "*~" EXCLUDE
)

# Install mesh files
install(DIRECTORY meshes urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(xacro REQUIRED)
  find_package(rosunit REQUIRED)
  foreach(T
      test/test_urdf_files.test
      )
     add_rostest(${T})
  endforeach()
endif()
