cmake_minimum_required(VERSION 3.5)

project(ensenso_camera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_definitions(ROS2)
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "jazzy")
    add_compile_definitions(ROS2_JAZZY)
  endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ensenso_camera_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4.0.0 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Find the Ensenso SDK.
if(NOT $ENV{ENSENSO_INSTALL} STREQUAL "")
  if(UNIX)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "$ENV{ENSENSO_INSTALL}/cmake/")
    find_package(Ensenso REQUIRED)
  elseif(WIN32)
    set(ENSENSO_INCLUDE_DIRS $ENV{ENSENSO_INSTALL}/development/c/include)
    set(ENSENSO_LIBRARIES $ENV{ENSENSO_INSTALL}/development/c/lib/nxLib64.lib)
  endif()
else()
    message(FATAL_ERROR "Please install the Ensenso SDK")
endif()

include_directories(
  include
  SYSTEM
  ${ENSENSO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#-----------------------------------------------------------------------------------------------------------------------
# Library ensenso_camera
#-----------------------------------------------------------------------------------------------------------------------
add_library(ensenso_camera SHARED
  src/calibration_pattern.cpp
  src/camera.cpp
  src/camera_node.cpp
  src/conversion.cpp
  src/image_utilities.cpp
  src/mono_camera.cpp
  src/mono_camera_node.cpp
  src/nxlib_initialize_finalize.cpp
  src/pose_utilities.cpp
  src/stereo_camera.cpp
  src/stereo_camera_node.cpp
  src/virtual_object_handler.cpp
)
target_compile_definitions(ensenso_camera
  PRIVATE "ENSENSO_CAMERA_BUILDING_DLL"
)
target_link_libraries(ensenso_camera
  ${ENSENSO_LIBRARIES}
)
ament_target_dependencies(ensenso_camera
  common_interfaces
  cv_bridge
  diagnostic_msgs
  ensenso_camera_msgs
  geometry_msgs
  image_geometry
  image_transport
  pcl_conversions
  pcl_ros
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

#-----------------------------------------------------------------------------------------------------------------------
# Components
#-----------------------------------------------------------------------------------------------------------------------
rclcpp_components_register_nodes(ensenso_camera "ensenso_camera::StereoCameraNode")
rclcpp_components_register_nodes(ensenso_camera "ensenso_camera::MonoCameraNode")

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")
set(node_plugins "${node_plugins}ensenso_camera::StereoCameraNode;$<TARGET_FILE:ensenso_camera>\n")
set(node_plugins "${node_plugins}ensenso_camera::MonoCameraNode;$<TARGET_FILE:ensenso_camera>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

#-----------------------------------------------------------------------------------------------------------------------
# ensenso_camera_mono_node
#-----------------------------------------------------------------------------------------------------------------------
add_executable(ensenso_camera_mono_node
  src/ensenso_camera_mono_node.cpp
)
target_link_libraries(ensenso_camera_mono_node
  ensenso_camera
)
ament_target_dependencies(ensenso_camera_mono_node
  rclcpp
  OpenCV
)

#-----------------------------------------------------------------------------------------------------------------------
# ensenso_camera_node
#-----------------------------------------------------------------------------------------------------------------------
add_executable(ensenso_camera_node
  src/ensenso_camera_node.cpp
)
target_link_libraries(ensenso_camera_node
  ensenso_camera
)
ament_target_dependencies(ensenso_camera_node
  rclcpp
  OpenCV
)

#-----------------------------------------------------------------------------------------------------------------------
# texture_point_cloud_node
#-----------------------------------------------------------------------------------------------------------------------
add_executable(texture_point_cloud
  src/texture_point_cloud.cpp
)
target_link_libraries(texture_point_cloud
  ${ENSENSO_LIBRARIES}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(texture_point_cloud
  common_interfaces
  cv_bridge
  diagnostic_msgs
  geometry_msgs
  image_geometry
  image_transport
  pcl_conversions
  pcl_ros
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  OpenCV
)

#-----------------------------------------------------------------------------------------------------------------------
# INSTALLATION
#-----------------------------------------------------------------------------------------------------------------------
install(TARGETS
  ensenso_camera
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ensenso_camera_mono_node
  ensenso_camera_node
  texture_point_cloud
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/calibrate_handeye
  scripts/color_point_cloud
  scripts/fit_primitive
  scripts/image_stream
  scripts/image_stream_mono
  scripts/pattern_marker
  scripts/request_data
  scripts/request_data_linked
  scripts/request_data_mono
  scripts/set_origin
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

#-----------------------------------------------------------------------------------------------------------------------
# FINALIZE PACKAGE
#-----------------------------------------------------------------------------------------------------------------------
ament_export_dependencies(ensenso_camera_msgs)

ament_package()
