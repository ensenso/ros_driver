cmake_minimum_required(VERSION 3.5)

project(ensenso_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  stl_meshes
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  .
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.xacro"
)

ament_package()