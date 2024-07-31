#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash
source install/setup.bash

test_package="ensenso_camera_test"
test_folder="src/driver/${test_package}2/test"

cd $test_folder
tests=(*.py)
PS3="Which test do you want to run? "

select test in "${tests[@]}"
do
    python3 -m launch_testing.launch_test --package-name=$test_package $test
    break
done
