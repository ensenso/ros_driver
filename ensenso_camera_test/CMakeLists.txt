cmake_minimum_required(VERSION 3.0.2)
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

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  ensenso_camera_msgs
  tf
)

# pep8 is wrong, E203 is not PEP 8 compliant.
set(ROSLINT_PYTHON_OPTS "--ignore=E203")

roslint_python()

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
  add_rostest(test/scale_disparity_map.test)
  add_rostest(test/telecentric_projection.test)
  add_rostest(test/texture_point_cloud.test)
  add_rostest(test/workspace_calibration.test)
  add_rostest(test/request_data_structured_light.test)

  roslint_add_test()
endif ()
