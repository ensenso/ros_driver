cmake_minimum_required(VERSION 2.8.3)
project(ensenso_camera_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  stereo_msgs
  rgbd
)

add_message_files(
  FILES
  CalibrationPattern.msg
  ImagePoint.msg
  NxLibException.msg
  Parameter.msg
  RegionOfInterest.msg
)

add_action_files(
  FILES
  AccessTree.action
  CalibrateHandEye.action
  CalibrateWorkspace.action
  ExecuteCommand.action
  GetParameter.action
  LocatePattern.action
  ProjectPattern.action
  RequestData.action
  SetParameter.action
)

add_service_files(
  FILES
  SetSettings.srv
  GetCameraData.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs geometry_msgs sensor_msgs std_msgs stereo_msgs rgbd
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs geometry_msgs sensor_msgs std_msgs stereo_msgs rgbd
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
