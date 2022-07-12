cmake_minimum_required(VERSION 3.5)
project(ensenso_camera_msgs)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)

set(msg_files
  "msg/ImagePoint.msg"
  "msg/MonoCalibrationPattern.msg"
  "msg/NxLibException.msg"
  "msg/Parameter.msg"
  "msg/Primitive.msg"
  "msg/RegionOfInterest.msg"
  "msg/StereoCalibrationPattern.msg"
)

set(action_files
  "action/AccessTree.action"
  "action/CalibrateHandEye.action"
  "action/CalibrateWorkspace.action"
  "action/ExecuteCommand.action"
  "action/FitPrimitive.action"
  "action/GetParameter.action"
  "action/LocatePattern.action"
  "action/LocatePatternMono.action"
  "action/ProjectPattern.action"
  "action/RequestData.action"
  "action/RequestDataMono.action"
  "action/SetParameter.action"
  "action/TelecentricProjection.action"
  "action/TexturedPointCloud.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs stereo_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
