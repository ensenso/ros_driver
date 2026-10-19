cmake_minimum_required(VERSION 3.5)
project(ensenso_camera_msgs)

set(msg_names
  ImagePoint.msg
  MonoCalibrationPattern.msg
  NxLibException.msg
  Parameter.msg
  Primitive.msg
  RegionOfInterest.msg
  StereoCalibrationPattern.msg
)

set(action_names
  AccessTree.action
  CalibrateHandEye.action
  CalibrateWorkspace.action
  ExecuteCommand.action
  FitPrimitive.action
  GetParameter.action
  LocatePattern.action
  LocatePatternMono.action
  ProjectPattern.action
  RequestData.action
  RequestDataMono.action
  SetParameter.action
  TelecentricProjection.action
  TexturedPointCloud.action
)

if("$ENV{ROS_VERSION}" STREQUAL "2")
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

  # ROS2 has no 'time' primitive; use builtin_interfaces/Time instead.
  set(generated_base_dir ${CMAKE_CURRENT_BINARY_DIR}/ros2_interfaces)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS action/GetParameter.action)
  file(READ action/GetParameter.action get_parameter_action)
  string(REGEX REPLACE "(^|\n)time stamp" "\\1builtin_interfaces/Time stamp" get_parameter_action "${get_parameter_action}")
  file(WRITE ${generated_base_dir}/action/GetParameter.action "${get_parameter_action}")
  list(TRANSFORM msg_names PREPEND "msg/" OUTPUT_VARIABLE msg_files)
  list(TRANSFORM action_names PREPEND "action/" OUTPUT_VARIABLE action_files)
  list(TRANSFORM action_files REPLACE "^action/GetParameter.action$" "${generated_base_dir}:action/GetParameter.action")

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${action_files}
    DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs stereo_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)

  ament_package()
else()
  find_package(catkin REQUIRED COMPONENTS
    message_generation
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    stereo_msgs
  )

  add_message_files(FILES ${msg_names})
  add_action_files(FILES ${action_names})

  generate_messages(
    DEPENDENCIES
    actionlib_msgs geometry_msgs sensor_msgs std_msgs stereo_msgs
  )

  catkin_package(
    CATKIN_DEPENDS message_runtime actionlib_msgs geometry_msgs sensor_msgs std_msgs stereo_msgs
  )
endif()
