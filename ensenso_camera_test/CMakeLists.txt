cmake_minimum_required(VERSION 2.8.3)
project(ensenso_camera_test)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ensenso_camera
  ensenso_camera_msgs
  roslint
  rospy
  rostest
  tf
)

catkin_package(
  CATKIN_DEPENDS
  ensenso_camera_msgs
  tf
)

roslint_python()

if("$ENV{ENSENSO_SDK_VERSION}" STREQUAL "")
  message(WARNING "ENSENSO_SDK_VERSION not set. Assuming SDK version to be 3.2.489 and enabling S-series tests.")
  set(is_sdk_supporting_s_series true)
else()
  string(COMPARE GREATER "$ENV{ENSENSO_SDK_VERSION}" "3.1.996" is_sdk_supporting_s_series)
endif()

if(CATKIN_ENABLE_TESTING)
  add_rostest(test/colorized_point_cloud.test)
  add_rostest(test/fit_primitive.test)
  add_rostest(test/hand_eye_calibration.test)
  add_rostest(test/locate_pattern.test)
  add_rostest(test/locate_pattern_mono.test)
  add_rostest(test/locate_multiple_patterns.test)
  add_rostest(test/parameter.test)
  add_rostest(test/project_pattern.test)
  add_rostest(test/request_data_mono.test)
  add_rostest(test/request_data_stereo.test)
  add_rostest(test/telecentric_projection.test)
  add_rostest(test/texture_point_cloud.test)
  add_rostest(test/workspace_calibration.test)

  if(is_sdk_supporting_s_series)
    add_rostest(test/request_data_structured_light.test)
  endif()

  roslint_add_test()
endif ()
