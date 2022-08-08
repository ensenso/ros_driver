cmake_minimum_required(VERSION 3.5)

project(ensenso_camera_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake_python REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(launch_testing_ament_cmake)
find_package(ensenso_camera)
find_package(ensenso_camera_msgs)
find_package(rclpy)
find_package(tf2_ros)
find_package(OpenCV REQUIRED)

if("$ENV{ENSENSO_SDK_VERSION}" STREQUAL "")
  message(WARNING "ENSENSO_SDK_VERSION not set. Assuming SDK version to be 3.3.1385 and enabling S-series tests.")
  set(is_sdk_supporting_s_series true)
else()
  string(COMPARE GREATER "$ENV{ENSENSO_SDK_VERSION}" "3.1.996" is_sdk_supporting_s_series)
endif()

if(BUILD_TESTING)
  set(launch_test_timeout 180)
  add_launch_test(test/colorized_point_cloud.test.py TIMEOUT ${launch_test_timeout})
  add_launch_test(test/fit_primitive.test.py TIMEOUT ${launch_test_timeout})
  add_launch_test(test/hand_eye_calibration.test.py TIMEOUT ${launch_test_timeout})
  add_launch_test(test/hztest.request_data_mono.test.py TIMEOUT ${launch_test_timeout})
  add_launch_test(test/hztest.request_data_stereo.test.py TIMEOUT ${launch_test_timeout})
  add_launch_test(test/hztest.texture_point_cloud.test.py TIMEOUT ${launch_test_timeout})
  add_launch_test(test/locate_multiple_patterns.test.py TIMEOUT ${launch_test_timeout})
  add_launch_test(test/locate_pattern.test.py TIMEOUT ${launch_test_timeout})
  add_launch_test(test/locate_pattern_mono.test.py TIMEOUT ${launch_test_timeout})
  add_launch_test(test/parameter.test.py TIMEOUT ${launch_test_timeout})
  add_launch_test(test/project_pattern.test.py TIMEOUT ${launch_test_timeout})
  add_launch_test(test/telecentric_projection.test.py TIMEOUT ${launch_test_timeout})
  add_launch_test(test/workspace_calibration.test.py TIMEOUT ${launch_test_timeout})

  if(is_sdk_supporting_s_series)
    add_launch_test(test/hztest.request_data_structured_light.test.py TIMEOUT ${launch_test_timeout})
  endif()

  # ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
