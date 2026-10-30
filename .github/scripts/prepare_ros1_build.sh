#!/bin/bash

# This script checks whether a ROS1 or ROS2 build is executed and prepares the
# repository correspondingly.

set -e

ignore_ros2_package () {
    # For rosdep to ignore the duplicate ROS2 package, we simply rename the
    # package manifest.
    echo "Ignoring ROS2 package $1"
    mv $1/package.xml $1/ignore_ros2_package.xml
    ignore_file="$1/CATKIN_IGNORE"
    touch $ignore_file
    echo $1 > $ignore_file
}

recognize_ros1_package () {
    echo "Recognizing ROS1 package $1"
    mv $1/ignore_ros1_package.xml $1/package.xml
    rm $1/COLCON_IGNORE
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

if [[ $ROS_VERSION -eq "1" ]]; then
    echo "Preparing repository for ROS1"

    ignore_ros2_package "ensenso_camera2"
    ignore_ros2_package "ensenso_camera_msgs2"
    ignore_ros2_package "ensenso_camera_test2"
    ignore_ros2_package "ensenso_description2"
    ignore_ros2_package "ensenso_driver2"

    recognize_ros1_package "ensenso_camera"
    recognize_ros1_package "ensenso_camera_msgs"
    recognize_ros1_package "ensenso_camera_test"
    recognize_ros1_package "ensenso_description"
    recognize_ros1_package "ensenso_driver"

    # ensenso_camera - C++
    copy_to_nonexistent_dir "ensenso_camera2/include/ensenso_camera/*.h" "ensenso_camera/include/ensenso_camera"
    copy_to_nonexistent_dir "ensenso_camera2/include/ensenso_camera/ros2/*" "ensenso_camera/include/ensenso_camera/ros2"
    copy_to_nonexistent_dir "ensenso_camera2/scripts/*" "ensenso_camera/scripts"
    copy_to_nonexistent_dir "ensenso_camera2/src/*.cpp" "ensenso_camera/src"

    # ensenso_camera - Python
    copy_to_nonexistent_dir "ensenso_camera2/ensenso_camera/__init__.py" "ensenso_camera/src/ensenso_camera"
    copy_to_nonexistent_dir "ensenso_camera2/ensenso_camera/ros2.py" "ensenso_camera/src/ensenso_camera"

    # ensenso_camera_msgs
    copy_to_nonexistent_dir "ensenso_camera_msgs2/action/*" "ensenso_camera_msgs/action"
    copy_to_nonexistent_dir "ensenso_camera_msgs2/msg/*" "ensenso_camera_msgs/msg"

    # ensenso_camera_test
    copy_to_nonexistent_dir "ensenso_camera_test2/data/*" "ensenso_camera_test/data"
    copy_to_nonexistent_dir "ensenso_camera_test2/ensenso_camera_test/*.py" "ensenso_camera_test/src/ensenso_camera_test"
    rm ensenso_camera_test/src/ensenso_camera_test/ros2_point_cloud2.py
    rm ensenso_camera_test/src/ensenso_camera_test/ros2_testing_hz.py
    rm ensenso_camera_test/src/ensenso_camera_test/ros2_testing_launch.py

    # ensenso_description
    copy_to_nonexistent_dir "ensenso_description2/stl_meshes/*" "ensenso_description/stl_meshes"
    copy_to_nonexistent_dir "ensenso_description2/*.xacro" "ensenso_description"
fi
