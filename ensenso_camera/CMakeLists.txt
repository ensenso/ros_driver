cmake_minimum_required(VERSION 2.8.3)
project(ensenso_camera)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")
endif()
if(WIN32)
  add_definitions("-DWIN32_LEAN_AND_MEAN")
endif()

find_package(Boost 1.58.0 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  cv_bridge
  diagnostic_msgs
  ensenso_camera_msgs
  image_geometry
  image_transport
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  roslint
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  geometry_msgs
  sensor_msgs
)

roslint_python(
  scripts/fit_primitive
  scripts/image_stream
  scripts/image_stream_mono
  scripts/pattern_marker
  scripts/request_data
  scripts/request_data_mono
  scripts/set_origin
  scripts/color_point_cloud
)
if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()

# Find the Ensenso SDK.
if(NOT $ENV{ENSENSO_INSTALL} STREQUAL "")
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "$ENV{ENSENSO_INSTALL}/cmake/")
  find_package(Ensenso REQUIRED)
else()
    message(FATAL_ERROR "Please install the Ensenso SDK")
endif()

catkin_package(
  CATKIN_DEPENDS visualization_msgs ensenso_camera_msgs diagnostic_msgs
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${ENSENSO_INCLUDE_DIRS}
)

# Nodelet.
add_library(
  ensenso_camera_nodelet
  src/nodelet.cpp
  src/nodelet_mono.cpp
  src/calibration_pattern.cpp
  src/camera.cpp
  src/conversion.cpp
  src/stereo_camera.cpp
  src/mono_camera.cpp
  src/image_utilities.cpp
  src/point_cloud_utilities.cpp
  src/pose_utilities.cpp
)
target_link_libraries(
  ensenso_camera_nodelet
  ${catkin_LIBRARIES}
  ${ENSENSO_LIBRARIES}
)
add_dependencies(ensenso_camera_nodelet ${catkin_EXPORTED_TARGETS})
add_dependencies(ensenso_camera_nodelet ${ensenso_camera_msgs_EXPORTED_TARGETS})

# Standalone node based on the nodelet. Used for stereo cameras.
add_executable(
  ensenso_camera_node
  src/ensenso_camera_node.cpp
)
target_link_libraries(
  ensenso_camera_node
  ${catkin_LIBRARIES}
)

# Standalone node based on the nodelet. Used for mono cameras.
add_executable(
  ensenso_camera_mono_node
  src/ensenso_camera_mono_node.cpp
)
target_link_libraries(
  ensenso_camera_mono_node
  ${catkin_LIBRARIES}
)

add_executable(
  texture_point_cloud
  src/texture_point_cloud.cpp
)
target_link_libraries(
  texture_point_cloud
  ${catkin_LIBRARIES}
)

##################
## Installation ##
##################

install(
  TARGETS ensenso_camera_nodelet ensenso_camera_node texture_point_cloud
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS
  scripts/image_stream
  scripts/pattern_marker
  scripts/set_origin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
