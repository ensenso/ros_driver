#!/bin/bash

# This script checks whether a ROS1 or ROS2 build is executed and prepares the
# repository correspondingly.

set -e

ignore_ros1_package () {
    # For rosdep to ignore the duplicate ROS1 package, we simply rename the
    # package manifest.
    echo "Ignoring ROS1 package $1"
    mv $1/package.xml $1/ignore_ros1_package.xml
    touch $1/COLCON_IGNORE
}

recognize_ros2_package () {
    echo "Recognizing ROS2 package $1"
    mv $1/ignore_ros2_package.xml $1/package.xml
    rm $1/CATKIN_IGNORE
}

copy_to_nonexistent_dir () {
    # Create directory if not existing. Copy files without overwriting.
    #   $1 target file (pattern)
    #   $2 destination directory

    echo "Creating directory: $2"
    mkdir -p $2

    echo "Copying files from: $1"
    cp -r --no-clobber $1 $2
}

if [[ $ROS_VERSION -eq "2" ]]; then
	echo "Preparing repository for ROS2"

    ignore_ros1_package "ensenso_camera"
    ignore_ros1_package "ensenso_camera_msgs"
    ignore_ros1_package "ensenso_camera_test"
    ignore_ros1_package "ensenso_description"
    ignore_ros1_package "ensenso_driver"

    recognize_ros2_package "ensenso_camera2"
    recognize_ros2_package "ensenso_camera_msgs2"
    recognize_ros2_package "ensenso_camera_test2"
    recognize_ros2_package "ensenso_description2"
    recognize_ros2_package "ensenso_driver2"

    # ensenso_camera - C++
    copy_to_nonexistent_dir "ensenso_camera/include/ensenso_camera/*.h" "ensenso_camera2/include/ensenso_camera"
    copy_to_nonexistent_dir "ensenso_camera/include/ensenso_camera/ros2/*" "ensenso_camera2/include/ensenso_camera/ros2"
    copy_to_nonexistent_dir "ensenso_camera/scripts/*" "ensenso_camera2/scripts"
    copy_to_nonexistent_dir "ensenso_camera/src/*.cpp" "ensenso_camera2/src"

    # ensenso_camera - Python
    copy_to_nonexistent_dir "ensenso_camera/src/ensenso_camera/*.py" "ensenso_camera2/ensenso_camera"

    # ensenso_camera_msgs
    copy_to_nonexistent_dir "ensenso_camera_msgs/action/*" "ensenso_camera_msgs2/action"
    copy_to_nonexistent_dir "ensenso_camera_msgs/msg/*" "ensenso_camera_msgs2/msg"

    # ensenso_camera_test
    copy_to_nonexistent_dir "ensenso_camera_test/data/*" "ensenso_camera_test2/data"
    copy_to_nonexistent_dir "ensenso_camera_test/src/ensenso_camera_test/*.py" "ensenso_camera_test2/ensenso_camera_test"

    # ensenso_description
    copy_to_nonexistent_dir "ensenso_description/stl_meshes/*" "ensenso_description2/stl_meshes"
    copy_to_nonexistent_dir "ensenso_description/*.xacro" "ensenso_description2"
fi
