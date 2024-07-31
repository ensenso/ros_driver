#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash
source devel/setup.bash

test_package="ensenso_camera_test"
test_folder="src/driver/$test_package/test"

cd $test_folder
tests=(*.test)
PS3="Which test do you want to run? "

select test in "${tests[@]}"
do
    rostest $test_package $test
    break
done
