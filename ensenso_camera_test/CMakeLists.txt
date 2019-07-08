cmake_minimum_required(VERSION 2.8.3)
project(ensenso_camera_test)

find_package(catkin REQUIRED COMPONENTS
  ensenso_camera
  ensenso_camera_msgs
  roslint
  rospy
  tf
)

catkin_package(
  CATKIN_DEPENDS ensenso_camera_msgs
)

roslint_python()

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/hand_eye_calibration.test)
    add_rostest(test/fit_primitive.test)
    add_rostest(test/locate_pattern.test)
    add_rostest(test/locate_multiple_patterns.test)
    add_rostest(test/parameter.test)
    add_rostest(test/project_pattern.test)
    add_rostest(test/request_data.test)
    add_rostest(test/texture_point_cloud.test)
    add_rostest(test/workspace_calibration.test)
    add_rostest(test/colorized_point_cloud.test)

  # Mono camera tests
  add_rostest(test/request_data_mono.test)
  add_rostest(test/locate_pattern_mono.test)

  roslint_add_test()
endif()
