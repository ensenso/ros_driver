cmake_minimum_required(VERSION 2.8.3)
project(ensenso_description)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

install(DIRECTORY stl_meshes
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY .
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.xacro")
