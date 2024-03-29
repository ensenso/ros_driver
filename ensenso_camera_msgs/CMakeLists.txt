cmake_minimum_required(VERSION 3.0.2)
project(ensenso_camera_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  stereo_msgs
)

add_message_files(
  FILES
  StereoCalibrationPattern.msg
  MonoCalibrationPattern.msg
  ImagePoint.msg
  NxLibException.msg
  Parameter.msg
  Primitive.msg
  RegionOfInterest.msg
)

add_action_files(
  FILES
  AccessTree.action
  CalibrateHandEye.action
  CalibrateWorkspace.action
  ExecuteCommand.action
  FitPrimitive.action
  GetParameter.action
  LocatePattern.action
  LocatePatternMono.action
  ProjectPattern.action
  TexturedPointCloud.action
  RequestData.action
  RequestDataMono.action
  SetParameter.action
  TelecentricProjection.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs geometry_msgs sensor_msgs std_msgs stereo_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs geometry_msgs sensor_msgs std_msgs stereo_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
